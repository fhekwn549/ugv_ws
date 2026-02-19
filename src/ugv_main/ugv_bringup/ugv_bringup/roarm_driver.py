#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import serial
import json
import threading

# ROS joint name -> ESP32 T:102 field
JOINT_MAP = {
    'arm_base_link_to_arm_link1': 'base',
    'arm_link1_to_arm_link2': 'shoulder',
    'arm_link2_to_arm_link3': 'elbow',
    'arm_link3_to_arm_gripper_link': 'hand',
}

# T:105 response field -> ROS joint name
FEEDBACK_MAP = {
    'b': 'arm_base_link_to_arm_link1',
    's': 'arm_link1_to_arm_link2',
    'e': 'arm_link2_to_arm_link3',
    't': 'arm_link3_to_arm_gripper_link',
}


class RoarmDriver(Node):
    def __init__(self):
        super().__init__('roarm_driver')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('feedback_rate', 5.0)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        feedback_rate = self.get_parameter('feedback_rate').value

        try:
            self.ser = serial.Serial(port, baud, timeout=0.5)
            self.get_logger().info(f'Serial opened: {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise

        self.serial_lock = threading.Lock()

        # Subscribe: arm joint trajectory
        self.traj_sub = self.create_subscription(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            self.trajectory_callback,
            10,
        )

        # Subscribe: gripper command
        self.gripper_sub = self.create_subscription(
            Float64,
            '/roarm/gripper_cmd',
            self.gripper_callback,
            10,
        )

        # Publish: joint states
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Periodic feedback via T:105
        period = 1.0 / feedback_rate
        self.feedback_timer = self.create_timer(period, self.feedback_callback)

        # Enable torque
        self._serial_write({'T': 210, 'cmd': 1})

        self.get_logger().info('RoArm-M2 driver ready')

    def _serial_write(self, cmd_dict):
        with self.serial_lock:
            data = json.dumps(cmd_dict) + '\n'
            self.ser.write(data.encode())

    def _serial_query(self, cmd_dict):
        with self.serial_lock:
            self.ser.reset_input_buffer()
            data = json.dumps(cmd_dict) + '\n'
            self.ser.write(data.encode())
            # ESP32 responds: echo -> \r\n -> actual JSON response
            for _ in range(5):
                raw = self.ser.readline()
                if len(raw) == 0:
                    break  # timeout, no data at all
                line = raw.decode().strip()
                if not line:
                    continue  # skip blank lines (\r\n)
                try:
                    parsed = json.loads(line)
                    if parsed.get('T') == cmd_dict.get('T'):
                        continue  # skip echo
                    return parsed
                except (json.JSONDecodeError, UnicodeDecodeError):
                    pass
            return None

    def trajectory_callback(self, msg: JointTrajectory):
        if not msg.points:
            return

        point = msg.points[0]
        cmd = {'T': 102, 'spd': 0, 'acc': 10}

        for i, name in enumerate(msg.joint_names):
            if name in JOINT_MAP and i < len(point.positions):
                cmd[JOINT_MAP[name]] = round(point.positions[i], 4)

        self._serial_write(cmd)

    def gripper_callback(self, msg: Float64):
        cmd = {'T': 106, 'cmd': round(msg.data, 4), 'spd': 0, 'acc': 0}
        self._serial_write(cmd)

    def feedback_callback(self):
        resp = self._serial_query({'T': 105})
        if resp is None or 'T' not in resp:
            return

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()

        for field, joint_name in FEEDBACK_MAP.items():
            if field in resp:
                js.name.append(joint_name)
                js.position.append(float(resp[field]))

        if js.name:
            self.joint_state_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = RoarmDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.ser.close()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
