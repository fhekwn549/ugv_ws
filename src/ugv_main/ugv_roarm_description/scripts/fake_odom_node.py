#!/usr/bin/env python3
"""
Fake Odometry Node

/cmd_vel을 구독하여 2D 운동학 적분으로 /odom과 odom→base_footprint TF를 발행합니다.
Gazebo 없이 Nav2를 테스트하기 위한 경량 오도메트리 시뮬레이션입니다.

/initialpose (RViz "2D Pose Estimate")를 구독하여 위치를 재설정할 수 있습니다.

joint_states도 동일 타임스탬프로 발행하여 TF 동기화 문제(바퀴 진동)를 방지합니다.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from geometry_msgs.msg import Twist, TransformStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


# All non-fixed joints in ugv_roarm.xacro
WHEEL_JOINTS = [
    'left_up_wheel_link_joint',
    'left_down_wheel_link_joint',
    'right_up_wheel_link_joint',
    'right_down_wheel_link_joint',
]

ARM_JOINTS = [
    'arm_base_link_to_arm_link1',
    'arm_link1_to_arm_link2',
    'arm_link2_to_arm_link3',
    'arm_link3_to_arm_gripper_link',
]

ALL_JOINTS = WHEEL_JOINTS + ARM_JOINTS


class FakeOdomNode(Node):
    def __init__(self):
        super().__init__('fake_odom_node')

        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('update_rate', 50.0)

        self.x = self.get_parameter('initial_x').value
        self.y = self.get_parameter('initial_y').value
        self.yaw = self.get_parameter('initial_yaw').value

        self.vx = 0.0
        self.vyaw = 0.0
        self.last_cmd_time = self.get_clock().now()

        # cmd_vel timeout (stop if no command received)
        self.cmd_timeout = 0.5

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribers
        self.create_subscription(Twist, 'cmd_vel', self._cmd_vel_cb, 10)

        initialpose_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.create_subscription(
            PoseWithCovarianceStamped, 'initialpose',
            self._initialpose_cb, initialpose_qos)

        # Timer
        rate = self.get_parameter('update_rate').value
        self.create_timer(1.0 / rate, self._update)

        self.get_logger().info(
            f'Fake odom started at ({self.x:.2f}, {self.y:.2f}, {self.yaw:.2f})')

    def _cmd_vel_cb(self, msg: Twist):
        self.vx = msg.linear.x
        self.vyaw = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def _initialpose_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose
        self.x = p.position.x
        self.y = p.position.y
        # Extract yaw from quaternion
        q = p.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        self.vx = 0.0
        self.vyaw = 0.0
        self.get_logger().info(
            f'Pose reset to ({self.x:.2f}, {self.y:.2f}, {math.degrees(self.yaw):.1f}°)')

    def _update(self):
        now = self.get_clock().now()

        # cmd_vel timeout check
        dt_cmd = (now - self.last_cmd_time).nanoseconds * 1e-9
        if dt_cmd > self.cmd_timeout:
            self.vx = 0.0
            self.vyaw = 0.0

        # Integration (fixed timestep)
        dt = 1.0 / self.get_parameter('update_rate').value
        self.yaw += self.vyaw * dt
        # Normalize yaw to [-pi, pi]
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        self.x += self.vx * math.cos(self.yaw) * dt
        self.y += self.vx * math.sin(self.yaw) * dt

        # Quaternion from yaw
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)

        stamp = now.to_msg()

        # Publish TF: map → odom (identity) + odom → base_footprint
        # map → odom을 dynamic TF로 발행하여 costmap의 TF 체인 해석 문제 방지
        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = stamp
        map_to_odom.header.frame_id = 'map'
        map_to_odom.child_frame_id = 'odom'
        map_to_odom.transform.rotation.w = 1.0

        odom_to_base = TransformStamped()
        odom_to_base.header.stamp = stamp
        odom_to_base.header.frame_id = 'odom'
        odom_to_base.child_frame_id = 'base_footprint'
        odom_to_base.transform.translation.x = self.x
        odom_to_base.transform.translation.y = self.y
        odom_to_base.transform.rotation.z = qz
        odom_to_base.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform([map_to_odom, odom_to_base])

        # Publish /odom
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.vyaw
        self.odom_pub.publish(odom)

        # Publish /joint_states (same timestamp → no TF jitter)
        js = JointState()
        js.header.stamp = stamp
        js.name = ALL_JOINTS
        js.position = [0.0] * len(ALL_JOINTS)
        js.velocity = []
        js.effort = []
        self.joint_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = FakeOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
