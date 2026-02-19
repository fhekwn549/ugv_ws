#!/usr/bin/env python3
# encoding: utf-8
"""
Unified keyboard teleop: drive + arm + gripper simultaneous control.
Supports both Gazebo (ros2_control) and RViz (direct TF) modes.

Usage:
  ros2 run ugv_roarm_description teleop_all.py                    # Gazebo mode (default)
  ros2 run ugv_roarm_description teleop_all.py --ros-args -p mode:=rviz   # RViz mode

Key mappings (all active at the same time):
  [Drive]       w/s = forward/back, a/d = turn left/right
                q/e = increase/decrease drive speed
                Space = emergency stop
  [Arm]         1/2/3 = select joint 1~3
                i/k = selected joint +/- (incremental)
                u/j = increase/decrease step size
  [Gripper]     o/p = gripper +/- (incremental)
  Ctrl+C        quit
"""
import sys
import select
import termios
import tty
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, TransformStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

# ── Joint limits ──────────────────────────────────────────────
ARM_JOINTS = [
    'arm_base_link_to_arm_link1',
    'arm_link1_to_arm_link2',
    'arm_link2_to_arm_link3',
]
GRIPPER_JOINT = 'arm_link3_to_arm_gripper_link'

WHEEL_JOINTS = [
    'left_up_wheel_link_joint',
    'left_down_wheel_link_joint',
    'right_up_wheel_link_joint',
    'right_down_wheel_link_joint',
]

JOINT_LIMITS = {
    'arm_base_link_to_arm_link1': (-math.pi, math.pi),
    'arm_link1_to_arm_link2':    (-math.pi / 2, math.pi / 2),
    'arm_link2_to_arm_link3':    (-1.0, 2.95),
    'arm_link3_to_arm_gripper_link': (0.0, 1.5),
}

# ── Per-model configurations ─────────────────────────────────
# Arm joints are identical across models
_ARM_DYNAMIC = [
    ('arm_base_link', 'arm_link1',
     (0.0100000008759151, 0, 0.123059270461044), (0, 0, 0),
     'arm_base_link_to_arm_link1', (0, 0, 1)),
    ('arm_link1', 'arm_link2',
     (0, 0, 0), (-1.5708, -1.5708, 0),
     'arm_link1_to_arm_link2', (0, 0, 1)),
    ('arm_link2', 'arm_link3',
     (0.236815132922094, 0.0300023995170449, 0), (0, 0, 1.5708),
     'arm_link2_to_arm_link3', (0, 0, 1)),
    ('arm_link3', 'arm_gripper_link',
     (0.002906, -0.21599, -0.00066683), (-1.5708, 0, -1.5708),
     'arm_link3_to_arm_gripper_link', (0, 0, 1)),
]

MODEL_CONFIGS = {
    'ugv_rover': {
        'wheel_radius': 0.0325,
        'wheel_separation': 0.1745,
        'fixed_joints': [
            ('base_footprint', 'base_link',
             (0.000460087791654118, 6.50856019576702e-10, 0.08), (0, 0, 0)),
            ('base_link', 'base_imu_link',
             (0, 0, 0), (0, 0, 0)),
            ('base_link', '3d_camera_link',
             (0.06531, 0, 0.021953), (0, 0, 0)),
            ('base_link', 'base_lidar_link',
             (0.0398145505519817, 0, 0.04), (0, 0, 1.5708)),
            ('base_link', 'arm_base_link',
             (-0.018892, 0, 0.04), (0, 0, 0)),
            ('arm_link3', 'arm_hand_tcp',
             (0.002, -0.2802, 0), (1.5708, 0, -1.5708)),
        ],
        'wheel_dynamic': [
            ('base_link', 'left_up_wheel_link',
             (0.0855, 0.08726, -0.041212), (1.5708, 0, 0),
             'left_up_wheel_link_joint', (0, 0, -1)),
            ('base_link', 'left_down_wheel_link',
             (-0.0854999999999981, 0.0872599990000038, -0.0412119912691592), (1.5708, 0, 0),
             'left_down_wheel_link_joint', (0, 0, -1)),
            ('base_link', 'right_up_wheel_link',
             (0.0854999999999989, -0.0872600009999962, -0.0412119912691588), (1.5708, 0, 0),
             'right_up_wheel_link_joint', (0, 0, -1)),
            ('base_link', 'right_down_wheel_link',
             (-0.0854999999999981, -0.0872600009999962, -0.0412119912691594), (1.5708, 0, 0),
             'right_down_wheel_link_joint', (0, 0, -1)),
        ],
    },
    'rasp_rover': {
        'wheel_radius': 0.035,
        'wheel_separation': 0.076,
        'fixed_joints': [
            ('base_footprint', 'base_link',
             (0.000460087791654118, 6.50856019576702e-10, 0.02825), (0, 0, 0)),
            ('base_link', 'base_imu_link',
             (0, 0, 0), (0, 0, 0)),
            ('base_link', 'base_lidar_link',
             (0.0316958390518911, -0.00101477740749894, 0.07468), (0, 0, 0)),
            ('base_link', 'arm_base_link',
             (-0.02571, -0.0010148, 0.07468), (0, 0, 0)),
            ('arm_link3', 'arm_hand_tcp',
             (0.002, -0.2802, 0), (1.5708, 0, -1.5708)),
        ],
        'wheel_dynamic': [
            ('base_link', 'left_up_wheel_link',
             (0.0494657123506755, 0.0380100014305094, 0.00674802802973084), (0, 0, 0),
             'left_up_wheel_link_joint', (0, -1, 0)),
            ('base_link', 'left_down_wheel_link',
             (-0.0475324306493252, 0.0380100014305063, 0.00674802802973084), (0, 0, 0),
             'left_down_wheel_link_joint', (0, -1, 0)),
            ('base_link', 'right_up_wheel_link',
             (0.0494657123506773, -0.0380100014304871, 0.00674802802973084), (0, 0, 0),
             'right_up_wheel_link_joint', (0, 1, 0)),
            ('base_link', 'right_down_wheel_link',
             (-0.0475324306493234, -0.0380100014304901, 0.00674802802973144), (0, 0, 0),
             'right_down_wheel_link_joint', (0, 1, 0)),
        ],
    },
}


def _load_model_config(model_name):
    """Return (wheel_radius, wheel_separation, fixed_joints, dynamic_joints)."""
    cfg = MODEL_CONFIGS[model_name]
    dynamic = _ARM_DYNAMIC + cfg['wheel_dynamic']
    return cfg['wheel_radius'], cfg['wheel_separation'], cfg['fixed_joints'], dynamic

BANNER_JOINT = """
=========================================
  Unified Teleop  (simultaneous control)
  Mode: {mode}  |  Arm: JOINT mode
=========================================
  [Drive]  w/s : forward / back
           a/d : turn left / right
           q/e : speed  +/-
         Space : emergency stop

  [Arm]  1/2/3 : select joint 1~3
           i/k : joint  +/-
           u/j : step size +/-
             m : switch to TCP mode

  [Gripper] o  : gripper +
            p  : gripper -

  Ctrl+C : quit
========================================="""

BANNER_TCP = """
=========================================
  Unified Teleop  (simultaneous control)
  Mode: {mode}  |  Arm: TCP mode
=========================================
  [Drive]  w/s : forward / back
           a/d : turn left / right
           q/e : speed  +/-
         Space : emergency stop

  [Arm]  1/2/3 : select axis X/Y/Z
           i/k : TCP  +/- (mm)
           u/j : step size +/-
             m : switch to Joint mode

  [Gripper] o  : gripper +
            p  : gripper -

  Ctrl+C : quit
========================================="""


# ── Quaternion helpers ────────────────────────────────────────
def quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def quat_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def make_tf(stamp, parent, child, xyz, quat):
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x = float(xyz[0])
    t.transform.translation.y = float(xyz[1])
    t.transform.translation.z = float(xyz[2])
    t.transform.rotation.x = float(quat[0])
    t.transform.rotation.y = float(quat[1])
    t.transform.rotation.z = float(quat[2])
    t.transform.rotation.w = float(quat[3])
    return t


def clamp(val, lo, hi):
    return max(lo, min(hi, val))


# ── IK/FK for roarm_m2 (ported from solver.hpp) ──────────────
L2A, L2B = 236.82, 30.00
L3A, L3B = 280.15, 1.73
L2 = math.sqrt(L2A**2 + L2B**2)
L3 = math.sqrt(L3A**2 + L3B**2)
T2RAD = math.atan2(L2B, L2A)
T3RAD = math.atan2(L3B, L3A)

TCP_AXIS_NAMES = ['X', 'Y', 'Z']


def fk_m2(base, shoulder, elbow):
    """FK: joint angles (rad) -> TCP position (mm). Ported from solver.hpp computePosbyJointRad (EEMode=0)."""
    aOut = L2 * math.cos(math.pi / 2 - (shoulder + T2RAD))
    bOut = L2 * math.sin(math.pi / 2 - (shoulder + T2RAD))
    cOut = L3 * math.cos(math.pi / 2 - (elbow + shoulder + T3RAD))
    dOut = L3 * math.sin(math.pi / 2 - (elbow + shoulder + T3RAD))
    r_ee = aOut + cOut
    z_ee = bOut + dOut
    x_ee = r_ee * math.cos(base)
    y_ee = r_ee * math.sin(base)
    return (x_ee, y_ee, z_ee)


def ik_m2(x, y, z):
    """IK: TCP position (mm) -> joint angles (rad). Ported from solver.hpp computeJointRadbyPos."""
    r = math.sqrt(x * x + y * y)
    base_angle = math.atan2(y, x)
    LA, LB = L2, L3
    if abs(z) < 1e-6:
        cos_psi = (LA * LA + r * r - LB * LB) / (2 * LA * r)
        cos_omega = (r * r + LB * LB - LA * LA) / (2 * r * LB)
        if abs(cos_psi) > 1.0 or abs(cos_omega) > 1.0:
            return None
        psi = math.acos(cos_psi) + T2RAD
        alpha = math.pi / 2.0 - psi
        omega = math.acos(cos_omega)
        beta = psi + omega - T3RAD
    else:
        L2C = r * r + z * z
        LC = math.sqrt(L2C)
        lam = math.atan2(z, r)
        cos_psi = (LA * LA + L2C - LB * LB) / (2 * LA * LC)
        cos_omega = (LB * LB + L2C - LA * LA) / (2 * LC * LB)
        if abs(cos_psi) > 1.0 or abs(cos_omega) > 1.0:
            return None
        psi = math.acos(cos_psi) + T2RAD
        alpha = math.pi / 2.0 - lam - psi
        omega = math.acos(cos_omega)
        beta = psi + omega - T3RAD
    if math.isnan(alpha) or math.isnan(beta):
        return None
    return (base_angle, alpha, beta)


class TeleopAll(Node):
    def __init__(self):
        super().__init__('teleop_all')

        # ── Mode parameter ────────────────────────────────────
        self.declare_parameter('mode', 'gazebo')
        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        if self.mode not in ('gazebo', 'rviz'):
            self.get_logger().error(
                f"Unknown mode '{self.mode}', use 'gazebo' or 'rviz'")
            raise ValueError(f"Unknown mode: {self.mode}")

        # ── Model parameter ───────────────────────────────────
        self.declare_parameter('model', 'rasp_rover')
        model_name = self.get_parameter('model').get_parameter_value().string_value
        if model_name not in MODEL_CONFIGS:
            self.get_logger().error(
                f"Unknown model '{model_name}', available: {list(MODEL_CONFIGS.keys())}")
            raise ValueError(f"Unknown model: {model_name}")
        self.wheel_radius, self.wheel_separation, \
            self.fixed_joints, self.dynamic_joints = _load_model_config(model_name)
        self.get_logger().info(f"Loaded model config: {model_name}")

        # ── Common state ──────────────────────────────────────
        self.joint_positions = {j: 0.0 for j in ARM_JOINTS}
        self.joint_positions[GRIPPER_JOINT] = 0.0
        for wj in WHEEL_JOINTS:
            self.joint_positions[wj] = 0.0
        self.drive_speed = 0.2
        self.turn_speed = 0.5
        self.selected_joint = 0
        self.arm_step = 0.05
        self.arm_control_mode = 'joint'  # 'joint' | 'tcp'
        self.selected_axis = 0           # 0=X, 1=Y, 2=Z
        self.tcp_step = 10.0             # mm
        self.settings = termios.tcgetattr(sys.stdin)

        # ── Publishers (both modes) ──────────────────────────
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)

        if self.mode == 'gazebo':
            self._init_gazebo()
        else:
            self._init_rviz()

    def _init_gazebo(self):
        self.arm_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 1)
        self.gripper_client = ActionClient(
            self, GripperCommand, '/gripper_controller/gripper_cmd')
        self.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, 10)

    def _init_rviz(self):
        # All TFs published directly — no joint_states needed
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.last_time = time.monotonic()

        # Publish fixed joints once as static TFs
        stamp = self.get_clock().now().to_msg()
        static_tfs = []
        for parent, child, xyz, rpy in self.fixed_joints:
            q = quat_from_rpy(rpy[0], rpy[1], rpy[2])
            static_tfs.append(make_tf(stamp, parent, child, xyz, q))
        self.static_tf_broadcaster.sendTransform(static_tfs)

    # ── Callbacks ─────────────────────────────────────────────
    def _joint_state_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name in self.joint_positions:
                self.joint_positions[name] = pos

    # ── Input ─────────────────────────────────────────────────
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.02)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    # ── Drive ─────────────────────────────────────────────────
    def publish_cmd_vel(self, linear: float, angular: float):
        t = Twist()
        t.linear.x = linear
        t.angular.z = angular
        self.cmd_vel_pub.publish(t)

    # ── Arm ───────────────────────────────────────────────────
    def move_arm_joint(self, direction: int):
        jname = ARM_JOINTS[self.selected_joint]
        lo, hi = JOINT_LIMITS[jname]
        current = self.joint_positions[jname]
        target = clamp(current + direction * self.arm_step, lo, hi)
        self.joint_positions[jname] = target

        if self.mode == 'gazebo':
            positions = [self.joint_positions[j] for j in ARM_JOINTS]
            traj = JointTrajectory()
            traj.joint_names = list(ARM_JOINTS)
            pt = JointTrajectoryPoint()
            pt.positions = positions
            pt.time_from_start = Duration(sec=0, nanosec=200_000_000)
            traj.points = [pt]
            self.arm_pub.publish(traj)

    # ── TCP move ──────────────────────────────────────────────
    def move_tcp(self, direction: int):
        base = self.joint_positions[ARM_JOINTS[0]]
        shoulder = self.joint_positions[ARM_JOINTS[1]]
        elbow = self.joint_positions[ARM_JOINTS[2]]
        x, y, z = fk_m2(base, shoulder, elbow)
        delta = direction * self.tcp_step
        if self.selected_axis == 0:
            x += delta
        elif self.selected_axis == 1:
            y += delta
        else:
            z += delta
        result = ik_m2(x, y, z)
        if result is None:
            return
        new_base, new_shoulder, new_elbow = result
        new_joints = {
            ARM_JOINTS[0]: new_base,
            ARM_JOINTS[1]: new_shoulder,
            ARM_JOINTS[2]: new_elbow,
        }
        for jname, val in new_joints.items():
            lo, hi = JOINT_LIMITS[jname]
            new_joints[jname] = clamp(val, lo, hi)
        for jname, val in new_joints.items():
            self.joint_positions[jname] = val
        if self.mode == 'gazebo':
            positions = [self.joint_positions[j] for j in ARM_JOINTS]
            traj = JointTrajectory()
            traj.joint_names = list(ARM_JOINTS)
            pt = JointTrajectoryPoint()
            pt.positions = positions
            pt.time_from_start = Duration(sec=0, nanosec=200_000_000)
            traj.points = [pt]
            self.arm_pub.publish(traj)

    # ── Gripper ───────────────────────────────────────────────
    def move_gripper(self, direction: int):
        lo, hi = JOINT_LIMITS[GRIPPER_JOINT]
        current = self.joint_positions[GRIPPER_JOINT]
        target = clamp(current + direction * self.arm_step, lo, hi)
        self.joint_positions[GRIPPER_JOINT] = target

        if self.mode == 'gazebo':
            if not self.gripper_client.wait_for_server(timeout_sec=0.5):
                self.get_logger().warn('Gripper action server not available')
                return
            goal = GripperCommand.Goal()
            goal.command.position = target
            goal.command.max_effort = 5.0
            self.gripper_client.send_goal_async(goal)

    # ── RViz: publish ALL TFs in one batch ────────────────────
    def publish_rviz_state(self, linear: float, angular: float):
        now = time.monotonic()
        dt = now - self.last_time
        self.last_time = now

        # Integrate odom
        self.odom_theta += angular * dt
        self.odom_x += linear * math.cos(self.odom_theta) * dt
        self.odom_y += linear * math.sin(self.odom_theta) * dt

        # Integrate wheel rotations
        v_left = (linear - angular * self.wheel_separation / 2.0) / self.wheel_radius
        v_right = (linear + angular * self.wheel_separation / 2.0) / self.wheel_radius
        self.joint_positions['left_up_wheel_link_joint'] += v_left * dt
        self.joint_positions['left_down_wheel_link_joint'] += v_left * dt
        self.joint_positions['right_up_wheel_link_joint'] += v_right * dt
        self.joint_positions['right_down_wheel_link_joint'] += v_right * dt

        stamp = self.get_clock().now().to_msg()
        tfs = []

        # odom → base_footprint
        odom_q = (0.0, 0.0,
                  math.sin(self.odom_theta / 2.0),
                  math.cos(self.odom_theta / 2.0))
        tfs.append(make_tf(
            stamp, 'odom', 'base_footprint',
            (self.odom_x, self.odom_y, 0.0), odom_q))

        # All dynamic joints (arm + gripper + wheels)
        for parent, child, xyz, rpy, jname, axis in self.dynamic_joints:
            angle = self.joint_positions.get(jname, 0.0)
            origin_q = quat_from_rpy(rpy[0], rpy[1], rpy[2])
            half = angle / 2.0
            s = math.sin(half)
            joint_q = (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(half))
            total_q = quat_multiply(origin_q, joint_q)
            tfs.append(make_tf(stamp, parent, child, xyz, total_q))

        # Publish all dynamic TFs in one call
        self.tf_broadcaster.sendTransform(tfs)

    # ── Status line ───────────────────────────────────────────
    def print_status(self):
        gpos = self.joint_positions[GRIPPER_JOINT]
        drive_str = f'Drive: spd={self.drive_speed:.2f} turn={self.turn_speed:.2f}'
        grip_str = f'Grip: {gpos:.2f}'
        if self.arm_control_mode == 'joint':
            jname = ARM_JOINTS[self.selected_joint]
            jpos = self.joint_positions[jname]
            arm_str = (f'Arm[Joint]: J{self.selected_joint+1}'
                       f'({jname.split("_to_")[1]}) '
                       f'pos={jpos:.2f} step={self.arm_step:.3f}')
        else:
            base = self.joint_positions[ARM_JOINTS[0]]
            shoulder = self.joint_positions[ARM_JOINTS[1]]
            elbow = self.joint_positions[ARM_JOINTS[2]]
            tx, ty, tz = fk_m2(base, shoulder, elbow)
            axis = TCP_AXIS_NAMES[self.selected_axis]
            arm_str = (f'Arm[TCP]: {axis} '
                       f'tcp=({tx:.1f}, {ty:.1f}, {tz:.1f})mm '
                       f'step={self.tcp_step:.1f}mm')
        sys.stdout.write(f'\r  {drive_str}  |  {arm_str}  |  {grip_str}  ')
        sys.stdout.flush()


def main():
    rclpy.init()
    node = TeleopAll()
    print(BANNER_JOINT.format(mode=node.mode.upper()))
    node.print_status()

    x = 0.0
    th = 0.0
    count = 0

    try:
        while True:
            key = node.get_key()

            # ── Drive keys ────────────────────────────────────
            if key == 'w':
                x, th, count = 1.0, 0.0, 0
            elif key == 's':
                x, th, count = -1.0, 0.0, 0
            elif key == 'a':
                x, th, count = 0.0, 1.0, 0
            elif key == 'd':
                x, th, count = 0.0, -1.0, 0
            elif key == 'q':
                node.drive_speed = min(node.drive_speed * 1.1, 1.0)
                node.turn_speed = min(node.turn_speed * 1.1, 2.0)
                node.print_status()
            elif key == 'e':
                node.drive_speed *= 0.9
                node.turn_speed *= 0.9
                node.print_status()
            elif key == ' ':
                x, th, count = 0.0, 0.0, 0

            # ── Arm mode toggle ───────────────────────────────
            elif key == 'm':
                if node.arm_control_mode == 'joint':
                    node.arm_control_mode = 'tcp'
                    banner = BANNER_TCP
                else:
                    node.arm_control_mode = 'joint'
                    banner = BANNER_JOINT
                print(banner.format(mode=node.mode.upper()))
                node.print_status()

            # ── Arm keys ──────────────────────────────────────
            elif key in ('1', '2', '3'):
                if node.arm_control_mode == 'joint':
                    node.selected_joint = int(key) - 1
                else:
                    node.selected_axis = int(key) - 1
                node.print_status()
            elif key == 'i':
                if node.arm_control_mode == 'joint':
                    node.move_arm_joint(+1)
                else:
                    node.move_tcp(+1)
                node.print_status()
            elif key == 'k':
                if node.arm_control_mode == 'joint':
                    node.move_arm_joint(-1)
                else:
                    node.move_tcp(-1)
                node.print_status()
            elif key == 'u':
                if node.arm_control_mode == 'joint':
                    node.arm_step = min(node.arm_step * 1.5, 0.5)
                else:
                    node.tcp_step = min(node.tcp_step * 1.5, 100.0)
                node.print_status()
            elif key == 'j':
                if node.arm_control_mode == 'joint':
                    node.arm_step = max(node.arm_step / 1.5, 0.005)
                else:
                    node.tcp_step = max(node.tcp_step / 1.5, 1.0)
                node.print_status()

            # ── Gripper keys ──────────────────────────────────
            elif key == 'o':
                node.move_gripper(+1)
                node.print_status()
            elif key == 'p':
                node.move_gripper(-1)
                node.print_status()

            # ── Quit ──────────────────────────────────────────
            elif key == '\x03':
                break

            # ── No key / unknown → gradual stop ───────────────
            else:
                count += 1
                if count > 20:
                    x, th = 0.0, 0.0

            # Always publish drive command
            node.publish_cmd_vel(
                node.drive_speed * x,
                node.turn_speed * th)

            # RViz mode: publish all TFs in one synchronized batch
            if node.mode == 'rviz':
                node.publish_rviz_state(
                    node.drive_speed * x,
                    node.turn_speed * th)

    except Exception as e:
        print(e)
    finally:
        node.publish_cmd_vel(0.0, 0.0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
