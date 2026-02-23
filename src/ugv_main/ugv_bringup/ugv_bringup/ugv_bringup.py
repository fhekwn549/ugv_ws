import serial
import json
import queue
import threading
import rclpy
from rclpy.node import Node
import logging
import time
import math
from std_msgs.msg import Header, Float32MultiArray, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import os

def is_jetson():
    result = any("ugv_jetson" in root for root, dirs, files in os.walk("/"))
    return result

if is_jetson():
    serial_port = '/dev/ttyTHS1'
else:
    serial_port = '/dev/ttyAMA0'

# Helper class for reading lines from a serial port
class ReadLine:
    def __init__(self, s):
        self.buf = bytearray()
        self.s = s

    def readline(self):
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i+1]
            self.buf = self.buf[i+1:]
            return r
        while True:
            i = max(1, min(512, self.s.in_waiting))
            data = self.s.read(i)
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i+1]
                self.buf[0:] = data[i+1:]
                return r
            else:
                self.buf.extend(data)

    def clear_buffer(self):
        self.s.reset_input_buffer()

# Base controller class for managing UART communication
class BaseController:
    def __init__(self, uart_dev_set, baud_set):
        self.logger = logging.getLogger('BaseController')
        self.ser = serial.Serial(uart_dev_set, baud_set, timeout=1)
        self.rl = ReadLine(self.ser)
        self.command_queue = queue.Queue()
        self.command_thread = threading.Thread(target=self.process_commands, daemon=True)
        self.command_thread.start()
        self.data_buffer = None
        # ESP32 T:1001 feedback format:
        #   L/R: left/right encoder, r/p/y: roll/pitch/yaw (degrees),
        #   temp: temperature, v: voltage (volts)
        self.base_data = {"T": 1001, "L": 0, "R": 0,
                          "r": 0.0, "p": 0.0, "y": 0.0,
                          "temp": 0.0, "v": 0.0}

    def feedback_data(self):
        try:
            line = self.rl.readline().decode('utf-8')
            self.data_buffer = json.loads(line)
            self.base_data = self.data_buffer
            return self.base_data
        except json.JSONDecodeError as e:
            self.logger.error(f"JSON decode error: {e} with line: {line}")
            self.rl.clear_buffer()
        except Exception as e:
            self.logger.error(f"[base_ctrl.feedback_data] unexpected error: {e}")
            self.rl.clear_buffer()

    def send_command(self, data):
        self.command_queue.put(data)

    def process_commands(self):
        while True:
            data = self.command_queue.get()
            self.ser.write((json.dumps(data) + '\n').encode("utf-8"))

    def base_json_ctrl(self, input_json):
        self.send_command(input_json)


def rpy_to_quaternion(roll, pitch, yaw):
    """Convert roll/pitch/yaw (radians) to quaternion (x, y, z, w)."""
    cr = math.cos(roll / 2)
    sr = math.sin(roll / 2)
    cp = math.cos(pitch / 2)
    sp = math.sin(pitch / 2)
    cy = math.cos(yaw / 2)
    sy = math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,  # x
        cr * sp * cy + sr * cp * sy,  # y
        cr * cp * sy - sr * sp * cy,  # z
        cr * cp * cy + sr * sp * sy,  # w
    )


class ugv_bringup(Node):
    def __init__(self):
        super().__init__('ugv_bringup')
        # Publishers
        self.imu_publisher_ = self.create_publisher(Imu, "imu/data", 100)
        self.odom_publisher_ = self.create_publisher(Float32MultiArray, "odom/odom_raw", 100)
        self.voltage_publisher_ = self.create_publisher(Float32, "voltage", 50)
        # Initialize serial
        self.base_controller = BaseController(serial_port, 115200)
        # Enable continuous feedback from ESP32
        self.get_logger().info('Enabling ESP32 feedback flow...')
        self.base_controller.send_command({"T": 131, "cmd": 1})
        # Timer to read feedback
        self.feedback_timer = self.create_timer(0.001, self.feedback_loop)

    def feedback_loop(self):
        self.base_controller.feedback_data()
        if self.base_controller.base_data.get("T") == 1001:
            self.publish_imu()
            self.publish_odom_raw()
            self.publish_voltage()

    def publish_imu(self):
        """Publish IMU orientation from ESP32 r/p/y (degrees) as quaternion."""
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_imu_link"
        data = self.base_controller.base_data

        # Convert degrees to radians
        roll = math.radians(float(data["r"]))
        pitch = math.radians(float(data["p"]))
        yaw = math.radians(float(data["y"]))

        qx, qy, qz, qw = rpy_to_quaternion(roll, pitch, yaw)
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        self.imu_publisher_.publish(msg)

    def publish_odom_raw(self):
        """Publish left/right encoder values for base_node odometry calculation."""
        data = self.base_controller.base_data
        array = [float(data["L"]) / 100.0, float(data["R"]) / 100.0]
        msg = Float32MultiArray(data=array)
        self.odom_publisher_.publish(msg)

    def publish_voltage(self):
        """Publish battery voltage (ESP32 sends volts directly)."""
        data = self.base_controller.base_data
        msg = Float32()
        msg.data = float(data["v"])
        self.voltage_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ugv_bringup()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
