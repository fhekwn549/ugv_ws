#!/usr/bin/env python3
"""
WSL ↔ RPi ROS 2 Topic Bridge via rosbridge (WebSocket)

RPi에서 rosbridge_server 실행 후, WSL에서 이 노드를 실행하면
RPi의 ROS 토픽을 WSL의 로컬 ROS 2 토픽으로 중계합니다.

TF 관련:
- /tf_static, /robot_description → 로컬 robot_state_publisher가 담당
- odom → base_footprint TF → 이 노드가 /odom 데이터에서 생성
- 바퀴 joint_states → 기본값(0.0)으로 자동 추가

Usage:
  ros2 run ugv_bringup rosbridge_relay --ros-args -p host:=192.168.0.71
"""

import logging
import math
import threading

# Suppress Twisted signal handler warning in non-main thread
logging.getLogger('twisted').setLevel(logging.CRITICAL)

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState, LaserScan, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float64
from trajectory_msgs.msg import JointTrajectory
from builtin_interfaces.msg import Time

import roslibpy

# Wheel joints not in RPi's /joint_states (continuous, no encoder feedback)
WHEEL_JOINTS = [
    'left_up_wheel_link_joint',
    'left_down_wheel_link_joint',
    'right_up_wheel_link_joint',
    'right_down_wheel_link_joint',
]

# NOTE: All URDF ↔ ESP32 angle conversions are handled by roarm_driver.
# This relay is a pure bridge — it passes values through without conversion.


class RosbridgeRelay(Node):
    def __init__(self):
        super().__init__('rosbridge_relay')

        self.declare_parameter('host', '192.168.0.71')
        self.declare_parameter('port', 9090)
        host = self.get_parameter('host').value
        port = self.get_parameter('port').value

        # --- RPi → WSL publishers ---
        self.pub_joint = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_scan = self.create_publisher(LaserScan, '/scan', 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.pub_imu = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.pub_voltage = self.create_publisher(Float32, '/voltage', 10)

        # --- WSL → RPi (subscribe locally, forward to RPi) ---
        self.sub_cmd_vel = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        self.sub_arm_traj = self.create_subscription(
            JointTrajectory, '/arm_controller/joint_trajectory',
            self._arm_traj_cb, 10)
        self.sub_gripper = self.create_subscription(
            Float64, '/roarm/gripper_cmd', self._gripper_cb, 10)

        # rosbridge client
        self.client = roslibpy.Ros(host=host, port=port)
        self.client.on_ready(self._on_connected)

        self.get_logger().info(f'Connecting to rosbridge at ws://{host}:{port} ...')

        # Run rosbridge client in background thread
        self._ws_thread = threading.Thread(target=self._run_client, daemon=True)
        self._ws_thread.start()

    def _run_client(self):
        try:
            self.client.run_forever()
        except Exception as e:
            self.get_logger().error(f'rosbridge connection error: {e}')

    def _on_connected(self):
        self.get_logger().info('Connected to rosbridge server!')

        # Subscribe to RPi topics
        roslibpy.Topic(self.client, '/joint_states', 'sensor_msgs/msg/JointState').subscribe(
            self._relay_joint_states)
        roslibpy.Topic(self.client, '/scan', 'sensor_msgs/msg/LaserScan').subscribe(
            self._relay_scan)
        roslibpy.Topic(self.client, '/odom', 'nav_msgs/msg/Odometry').subscribe(
            self._relay_odom)
        roslibpy.Topic(self.client, '/imu/data_raw', 'sensor_msgs/msg/Imu').subscribe(
            self._relay_imu)
        roslibpy.Topic(self.client, '/voltage', 'std_msgs/msg/Float32').subscribe(
            self._relay_voltage)

        # Publishers for forwarding WSL → RPi
        self._rpi_cmd_vel = roslibpy.Topic(
            self.client, '/cmd_vel', 'geometry_msgs/msg/Twist')
        self._rpi_arm_traj = roslibpy.Topic(
            self.client, '/arm_controller/joint_trajectory',
            'trajectory_msgs/msg/JointTrajectory')
        self._rpi_gripper = roslibpy.Topic(
            self.client, '/roarm/gripper_cmd', 'std_msgs/msg/Float64')

        self.get_logger().info('All topic bridges active.')

    # --- Helper ---
    def _now(self):
        """Current ROS time as builtin_interfaces/Time."""
        t = self.get_clock().now().to_msg()
        return t

    def _parse_header(self, header_dict):
        from std_msgs.msg import Header
        h = Header()
        h.stamp = self._now()
        h.frame_id = header_dict.get('frame_id', '')
        return h

    # --- RPi → WSL callbacks ---
    def _relay_joint_states(self, msg):
        m = JointState()
        m.header = self._parse_header(msg.get('header', {}))
        m.name = list(msg.get('name', []))
        m.position = [float(v) for v in msg.get('position', [])]
        m.velocity = [float(v) for v in msg.get('velocity', [])]
        m.effort = [float(v) for v in msg.get('effort', [])]

        # Add wheel joints with default position (0.0)
        for wj in WHEEL_JOINTS:
            if wj not in m.name:
                m.name.append(wj)
                m.position.append(0.0)

        self.pub_joint.publish(m)

    def _relay_scan(self, msg):
        m = LaserScan()
        m.header = self._parse_header(msg.get('header', {}))
        m.angle_min = float(msg.get('angle_min', 0.0))
        m.angle_max = float(msg.get('angle_max', 0.0))
        m.angle_increment = float(msg.get('angle_increment', 0.0))
        m.time_increment = float(msg.get('time_increment', 0.0))
        m.scan_time = float(msg.get('scan_time', 0.0))
        m.range_min = float(msg.get('range_min', 0.0))
        m.range_max = float(msg.get('range_max', 0.0))
        m.ranges = [float(v) if v is not None else float('inf') for v in msg.get('ranges', [])]
        m.intensities = [float(v) if v is not None else 0.0 for v in msg.get('intensities', [])]
        self.pub_scan.publish(m)

    def _relay_odom(self, msg):
        m = Odometry()
        m.header = self._parse_header(msg.get('header', {}))
        m.child_frame_id = msg.get('child_frame_id', '')
        pose = msg.get('pose', {}).get('pose', {})
        pos = pose.get('position', {})
        ori = pose.get('orientation', {})
        m.pose.pose.position.x = float(pos.get('x', 0.0))
        m.pose.pose.position.y = float(pos.get('y', 0.0))
        m.pose.pose.position.z = float(pos.get('z', 0.0))
        m.pose.pose.orientation.x = float(ori.get('x', 0.0))
        m.pose.pose.orientation.y = float(ori.get('y', 0.0))
        m.pose.pose.orientation.z = float(ori.get('z', 0.0))
        m.pose.pose.orientation.w = float(ori.get('w', 1.0))
        twist = msg.get('twist', {}).get('twist', {})
        lin = twist.get('linear', {})
        ang = twist.get('angular', {})
        m.twist.twist.linear.x = float(lin.get('x', 0.0))
        m.twist.twist.linear.y = float(lin.get('y', 0.0))
        m.twist.twist.linear.z = float(lin.get('z', 0.0))
        m.twist.twist.angular.x = float(ang.get('x', 0.0))
        m.twist.twist.angular.y = float(ang.get('y', 0.0))
        m.twist.twist.angular.z = float(ang.get('z', 0.0))
        self.pub_odom.publish(m)

    def _relay_imu(self, msg):
        m = Imu()
        m.header = self._parse_header(msg.get('header', {}))
        ori = msg.get('orientation', {})
        m.orientation.x = float(ori.get('x', 0.0))
        m.orientation.y = float(ori.get('y', 0.0))
        m.orientation.z = float(ori.get('z', 0.0))
        m.orientation.w = float(ori.get('w', 1.0))
        ang = msg.get('angular_velocity', {})
        m.angular_velocity.x = float(ang.get('x', 0.0))
        m.angular_velocity.y = float(ang.get('y', 0.0))
        m.angular_velocity.z = float(ang.get('z', 0.0))
        lin = msg.get('linear_acceleration', {})
        m.linear_acceleration.x = float(lin.get('x', 0.0))
        m.linear_acceleration.y = float(lin.get('y', 0.0))
        m.linear_acceleration.z = float(lin.get('z', 0.0))
        self.pub_imu.publish(m)

    def _relay_voltage(self, msg):
        m = Float32()
        m.data = float(msg.get('data', 0.0))
        self.pub_voltage.publish(m)

    # --- WSL → RPi callbacks ---
    def _cmd_vel_cb(self, msg):
        if not self.client.is_connected:
            return
        self._rpi_cmd_vel.publish(roslibpy.Message({
            'linear': {'x': msg.linear.x, 'y': msg.linear.y, 'z': msg.linear.z},
            'angular': {'x': msg.angular.x, 'y': msg.angular.y, 'z': msg.angular.z},
        }))

    def _arm_traj_cb(self, msg):
        """Forward arm trajectory WSL → RPi (pass-through, roarm_driver converts)."""
        if not self.client.is_connected:
            return
        points = []
        for pt in msg.points:
            points.append({
                'positions': list(pt.positions),
                'time_from_start': {
                    'sec': pt.time_from_start.sec,
                    'nanosec': pt.time_from_start.nanosec,
                },
            })
        self._rpi_arm_traj.publish(roslibpy.Message({
            'joint_names': list(msg.joint_names),
            'points': points,
        }))

    def _gripper_cb(self, msg):
        """Forward gripper command WSL → RPi (pass-through, roarm_driver converts)."""
        if not self.client.is_connected:
            return
        self._rpi_gripper.publish(roslibpy.Message({'data': msg.data}))

    def destroy_node(self):
        if self.client.is_connected:
            self.client.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RosbridgeRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
