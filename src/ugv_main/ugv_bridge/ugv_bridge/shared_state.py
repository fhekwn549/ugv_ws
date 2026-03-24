"""Thread-safe shared state between ROS callbacks, MQTT, and FastAPI."""

import math
import threading
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class Pose:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass
class Velocity:
    linear: float = 0.0
    angular: float = 0.0


@dataclass
class NavStatus:
    status: str = "idle"  # idle | navigating | succeeded | failed | canceled
    goal_x: float = 0.0
    goal_y: float = 0.0
    goal_theta: float = 0.0
    feedback_distance: float = 0.0


@dataclass
class ScanData:
    angle_min: float = 0.0
    angle_max: float = 0.0
    angle_increment: float = 0.0
    range_min: float = 0.0
    range_max: float = 0.0
    ranges: list = field(default_factory=list)


@dataclass
class MapMeta:
    width: int = 0
    height: int = 0
    resolution: float = 0.05
    origin_x: float = 0.0
    origin_y: float = 0.0
    origin_yaw: float = 0.0


class RobotState:
    """Centralised robot state, guarded by a single lock."""

    def __init__(self):
        self._lock = threading.Lock()
        self.pose = Pose()
        self.velocity = Velocity()
        self.battery: float = 0.0
        self.imu_yaw: float = 0.0
        self.joints: dict[str, float] = {}
        self.scan = ScanData()
        self.nav = NavStatus()
        self.path: list[dict] = []
        self.map_meta: Optional[MapMeta] = None
        self.map_png: Optional[bytes] = None
        self.map_revision: int = 0
        self.map_pose = Pose()
        self.map_pose_valid: bool = False

    # -- writers (called from ROS callbacks) --

    def update_odom(self, x: float, y: float, yaw: float,
                    lin: float, ang: float):
        with self._lock:
            self.pose.x = x
            self.pose.y = y
            self.pose.yaw = yaw
            self.velocity.linear = lin
            self.velocity.angular = ang

    def update_voltage(self, volts: float):
        with self._lock:
            self.battery = volts

    def update_imu(self, yaw: float):
        with self._lock:
            self.imu_yaw = yaw

    def update_joints(self, names: list[str], positions: list[float]):
        with self._lock:
            self.joints = dict(zip(names, positions))

    def update_scan(self, angle_min: float, angle_max: float,
                    angle_increment: float, range_min: float,
                    range_max: float, ranges: list[float]):
        with self._lock:
            self.scan = ScanData(
                angle_min=angle_min,
                angle_max=angle_max,
                angle_increment=angle_increment,
                range_min=range_min,
                range_max=range_max,
                ranges=list(ranges),
            )

    def update_nav(self, status: str, **kwargs):
        with self._lock:
            self.nav.status = status
            for k, v in kwargs.items():
                if hasattr(self.nav, k):
                    setattr(self.nav, k, v)

    def update_path(self, points: list[dict]):
        with self._lock:
            self.path = points

    def update_map_pose(self, x: float, y: float, yaw: float):
        with self._lock:
            self.map_pose.x = x
            self.map_pose.y = y
            self.map_pose.yaw = yaw
            self.map_pose_valid = True

    def update_map(self, meta: MapMeta, png_bytes: bytes):
        with self._lock:
            self.map_meta = meta
            self.map_png = png_bytes
            self.map_revision += 1

    # -- readers (called from FastAPI / MQTT) --

    def snapshot_map_pose(self) -> dict:
        with self._lock:
            return {
                "x": round(self.map_pose.x, 4),
                "y": round(self.map_pose.y, 4),
                "yaw": round(self.map_pose.yaw, 4),
                "valid": self.map_pose_valid,
            }

    def snapshot_pose(self) -> dict:
        with self._lock:
            return {
                "x": round(self.pose.x, 4),
                "y": round(self.pose.y, 4),
                "yaw": round(self.pose.yaw, 4),
                "linear_vel": round(self.velocity.linear, 4),
                "angular_vel": round(self.velocity.angular, 4),
            }

    def snapshot_imu(self) -> dict:
        with self._lock:
            return {"yaw": round(self.imu_yaw, 4)}

    def snapshot_voltage(self) -> dict:
        with self._lock:
            return {"voltage": round(self.battery, 2)}

    def snapshot_joints(self) -> dict:
        with self._lock:
            return {
                "joints": {k: round(v, 4) for k, v in self.joints.items()}
            }

    def snapshot_scan(self, downsample: int = 1) -> dict:
        with self._lock:
            ranges = self.scan.ranges[::downsample] if downsample > 1 else self.scan.ranges
            return {
                "angle_min": self.scan.angle_min,
                "angle_max": self.scan.angle_max,
                "angle_increment": self.scan.angle_increment * downsample,
                "range_min": self.scan.range_min,
                "range_max": self.scan.range_max,
                "ranges": [round(r, 3) if math.isfinite(r) else None for r in ranges],
            }

    def snapshot_nav(self) -> dict:
        with self._lock:
            return {
                "status": self.nav.status,
                "goal_x": round(self.nav.goal_x, 4),
                "goal_y": round(self.nav.goal_y, 4),
                "goal_theta": round(self.nav.goal_theta, 4),
                "feedback_distance": round(self.nav.feedback_distance, 3),
            }

    def snapshot_path(self) -> dict:
        with self._lock:
            return {"poses": list(self.path)}

    def snapshot_map_revision(self) -> int:
        with self._lock:
            return self.map_revision

    def snapshot_map_png(self) -> tuple[Optional[bytes], Optional[MapMeta]]:
        with self._lock:
            return self.map_png, self.map_meta

    def snapshot_full(self) -> dict:
        with self._lock:
            return {
                "pose": {
                    "x": round(self.pose.x, 4),
                    "y": round(self.pose.y, 4),
                    "yaw": round(self.pose.yaw, 4),
                },
                "velocity": {
                    "linear": round(self.velocity.linear, 4),
                    "angular": round(self.velocity.angular, 4),
                },
                "battery": round(self.battery, 2),
                "imu_yaw": round(self.imu_yaw, 4),
                "joints": {k: round(v, 4) for k, v in self.joints.items()},
                "nav": {
                    "status": self.nav.status,
                    "goal_x": round(self.nav.goal_x, 4),
                    "goal_y": round(self.nav.goal_y, 4),
                },
                "map_revision": self.map_revision,
            }

    def snapshot_for_db(self) -> dict:
        with self._lock:
            return {
                "pose_x": self.pose.x,
                "pose_y": self.pose.y,
                "pose_yaw": self.pose.yaw,
                "battery": self.battery,
                "linear_vel": self.velocity.linear,
                "angular_vel": self.velocity.angular,
                "nav_status": self.nav.status,
            }
