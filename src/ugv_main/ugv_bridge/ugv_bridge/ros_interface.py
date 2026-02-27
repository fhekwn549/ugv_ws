"""ROS 2 subscriptions, publishers, and Nav2 action client.

Each RosInterface instance manages one robot's topics,
optionally prefixed (e.g. "" → /odom, "ugv01" → /ugv01/odom).
"""

import math

from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
)
from rclpy.action import ActionClient

from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import LaserScan, JointState, Imu
from std_msgs.msg import Float32, Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from nav2_msgs.action import NavigateToPose

from .shared_state import RobotState, MapMeta
from .map_converter import occupancy_grid_to_png
from .db_writer import DbWriter


SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)

RELIABLE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

MAP_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


def _quat_to_yaw(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class RosInterface:
    """Manages ROS 2 subscriptions and publishers for a single robot."""

    def __init__(self, node: Node, robot_id: str, topic_prefix: str,
                 state: RobotState, db: DbWriter):
        self._node = node
        self._robot_id = robot_id
        self._prefix = ("/" + topic_prefix) if topic_prefix else ""
        self._state = state
        self._db = db
        self._nav_client: ActionClient | None = None
        self._nav_goal_handle = None

        self._setup_subscribers()
        self._setup_publishers()
        self._setup_nav2()

    def _topic(self, name: str) -> str:
        """Build full topic name with optional prefix."""
        return f"{self._prefix}/{name}"

    # -- subscribers --

    def _setup_subscribers(self):
        n = self._node

        n.create_subscription(
            Odometry, self._topic("odom"), self._on_odom, SENSOR_QOS)
        n.create_subscription(
            Float32, self._topic("voltage"), self._on_voltage, SENSOR_QOS)
        n.create_subscription(
            JointState, self._topic("joint_states"), self._on_joints, RELIABLE_QOS)
        n.create_subscription(
            LaserScan, self._topic("scan"), self._on_scan, SENSOR_QOS)
        n.create_subscription(
            Imu, self._topic("imu/data"), self._on_imu, SENSOR_QOS)
        n.create_subscription(
            OccupancyGrid, self._topic("map"), self._on_map, MAP_QOS)
        n.create_subscription(
            Path, self._topic("plan"), self._on_path, RELIABLE_QOS)

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        yaw = _quat_to_yaw(msg.pose.pose.orientation)
        lin = msg.twist.twist.linear.x
        ang = msg.twist.twist.angular.z
        self._state.update_odom(p.x, p.y, yaw, lin, ang)

    def _on_voltage(self, msg: Float32):
        self._state.update_voltage(msg.data)

    def _on_joints(self, msg: JointState):
        self._state.update_joints(list(msg.name), list(msg.position))

    def _on_scan(self, msg: LaserScan):
        self._state.update_scan(
            msg.angle_min, msg.angle_max, msg.angle_increment,
            msg.range_min, msg.range_max, list(msg.ranges),
        )

    def _on_imu(self, msg: Imu):
        yaw = _quat_to_yaw(msg.orientation)
        self._state.update_imu(yaw)

    def _on_map(self, msg: OccupancyGrid):
        info = msg.info
        meta = MapMeta(
            width=info.width,
            height=info.height,
            resolution=info.resolution,
            origin_x=info.origin.position.x,
            origin_y=info.origin.position.y,
            origin_yaw=_quat_to_yaw(info.origin.orientation),
        )
        png = occupancy_grid_to_png(info.width, info.height, list(msg.data))
        self._state.update_map(meta, png)
        self._db.log_event(self._robot_id, "info", "map", "Map updated")

    def _on_path(self, msg: Path):
        points = [
            {"x": round(p.pose.position.x, 4),
             "y": round(p.pose.position.y, 4)}
            for p in msg.poses
        ]
        self._state.update_path(points)

    # -- publishers --

    def _setup_publishers(self):
        n = self._node
        self._cmd_vel_pub = n.create_publisher(
            Twist, self._topic("cmd_vel"), 10)
        self._arm_pub = n.create_publisher(
            JointTrajectory, self._topic("arm_controller/joint_trajectory"), 10)
        self._gripper_pub = n.create_publisher(
            Float64, self._topic("roarm/gripper_cmd"), 10)
        self._initial_pose_pub = n.create_publisher(
            PoseWithCovarianceStamped, self._topic("initialpose"), MAP_QOS)

    def publish_cmd_vel(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self._cmd_vel_pub.publish(msg)

    def publish_arm(self, positions: list[float]):
        joint_names = [
            "arm_base_link_to_arm_link1",
            "arm_link1_to_arm_link2",
            "arm_link2_to_arm_link3",
            "arm_link3_to_arm_gripper_link",
        ]
        msg = JointTrajectory()
        msg.joint_names = joint_names
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.time_from_start = Duration(sec=0, nanosec=500_000_000)
        msg.points = [pt]
        self._arm_pub.publish(msg)

    def publish_gripper(self, value: float):
        msg = Float64()
        msg.data = float(value)
        self._gripper_pub.publish(msg)

    def publish_initial_pose(self, x: float, y: float, yaw: float):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        sin_h = math.sin(yaw / 2.0)
        cos_h = math.cos(yaw / 2.0)
        msg.pose.pose.orientation.z = sin_h
        msg.pose.pose.orientation.w = cos_h
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.068
        self._initial_pose_pub.publish(msg)
        self._db.log_event(
            self._robot_id, "info", "amcl",
            f"Initial pose set: ({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f}°)")

    # -- Nav2 action --

    def _setup_nav2(self):
        action_name = self._topic("navigate_to_pose")
        self._nav_client = ActionClient(
            self._node, NavigateToPose, action_name)

    def send_nav_goal(self, x: float, y: float, theta: float):
        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            self._node.get_logger().warn(
                f"[{self._robot_id}] Nav2 action server not available")
            self._db.log_event(
                self._robot_id, "warn", "nav2", "Action server not available")
            return False

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self._node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        sin_h = math.sin(theta / 2.0)
        cos_h = math.cos(theta / 2.0)
        goal.pose.pose.orientation.z = sin_h
        goal.pose.pose.orientation.w = cos_h

        self._state.update_nav("navigating",
                               goal_x=x, goal_y=y, goal_theta=theta)
        self._db.log_nav_start(self._robot_id, x, y, theta)
        self._db.log_event(
            self._robot_id, "info", "nav2",
            f"Goal sent: ({x:.2f}, {y:.2f}, {math.degrees(theta):.1f}°)")

        future = self._nav_client.send_goal_async(
            goal, feedback_callback=self._nav_feedback_cb)
        future.add_done_callback(self._nav_goal_response_cb)
        return True

    def cancel_nav(self):
        if self._nav_goal_handle is not None:
            self._nav_goal_handle.cancel_goal_async()
            self._state.update_nav("canceled")
            self._db.log_nav_end(self._robot_id, "canceled", 0.0)
            self._db.log_event(
                self._robot_id, "info", "nav2", "Navigation canceled")
            self._nav_goal_handle = None
            return True
        return False

    def _nav_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._state.update_nav("failed")
            self._db.log_nav_end(self._robot_id, "rejected", 0.0)
            self._db.log_event(
                self._robot_id, "warn", "nav2", "Goal rejected")
            return
        self._nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_cb)

    def _nav_feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self._state.update_nav(
            "navigating",
            feedback_distance=fb.distance_remaining,
        )

    def _nav_result_cb(self, future):
        result = future.result()
        status = result.status
        self._nav_goal_handle = None

        if status == 4:  # SUCCEEDED
            self._state.update_nav("succeeded")
            self._db.log_nav_end(self._robot_id, "succeeded", 0.0)
            self._db.log_event(
                self._robot_id, "info", "nav2", "Navigation succeeded")
        elif status == 5:  # CANCELED
            self._state.update_nav("canceled")
            self._db.log_nav_end(self._robot_id, "canceled", 0.0)
        elif status == 6:  # ABORTED
            self._state.update_nav("failed")
            self._db.log_nav_end(self._robot_id, "aborted", 0.0)
            self._db.log_event(
                self._robot_id, "error", "nav2", "Navigation aborted")
        else:
            self._state.update_nav("failed")
            self._db.log_nav_end(self._robot_id, f"status_{status}", 0.0)
