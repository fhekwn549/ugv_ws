#!/usr/bin/env python3
"""
Fake Scan Node

맵 데이터와 TF를 사용하여 가상 LiDAR 스캔을 생성합니다.
OccupancyGrid 맵에서 레이캐스팅하여 /scan 토픽으로 LaserScan을 발행합니다.

실제 로봇과 동일한 TF 체인(map→odom→base_footprint→base_link→base_lidar_link)을
사용하여 로봇 위치를 조회합니다.

D500 LiDAR 스펙에 맞춘 360도 스캔을 시뮬레이션합니다.
MultiThreadedExecutor를 사용하여 TF 콜백이 타이머에 블로킹되지 않도록 합니다.
numpy 벡터화로 전체 360개 레이를 동시 처리합니다.
"""

import json
import math
import os

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener, TransformException


class FakeScanNode(Node):

    def __init__(self):
        super().__init__('fake_scan_node')

        # D500 LiDAR parameters
        self.declare_parameter('scan_rate', 10.0)
        self.declare_parameter('num_samples', 360)
        self.declare_parameter('range_min', 0.16)
        self.declare_parameter('range_max', 3.5)
        self.declare_parameter('noise_stddev', 0.01)
        self.declare_parameter('scan_frame', 'base_lidar_link')
        self.declare_parameter('occupancy_threshold', 50)
        self.declare_parameter('enable_angle_crop', True)
        self.declare_parameter('angle_crop_min', 30.0)   # degrees
        self.declare_parameter('angle_crop_max', 149.0)   # degrees

        self.num_samples = self.get_parameter('num_samples').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.noise_stddev = self.get_parameter('noise_stddev').value
        self.scan_frame = self.get_parameter('scan_frame').value
        self.occ_thresh = self.get_parameter('occupancy_threshold').value
        self.enable_angle_crop = self.get_parameter('enable_angle_crop').value
        self.angle_crop_min_rad = math.radians(
            self.get_parameter('angle_crop_min').value)
        self.angle_crop_max_rad = math.radians(
            self.get_parameter('angle_crop_max').value)

        # Map storage
        self.map_data = None       # 2D numpy array of occupancy values
        self.map_resolution = 0.0
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_width = 0
        self.map_height = 0

        # Pre-compute ray angles (match real LD19 driver: 0 → 2π, CCW)
        self.angle_min = 0.0
        self.angle_max = 2.0 * math.pi
        self.angle_increment = (self.angle_max - self.angle_min) / self.num_samples
        self.angles = np.linspace(
            self.angle_min, self.angle_max, self.num_samples, endpoint=False)

        # Pre-compute angle crop mask (robot arm blockage)
        if self.enable_angle_crop:
            self.crop_mask = ((self.angles >= self.angle_crop_min_rad) &
                              (self.angles <= self.angle_crop_max_rad))
            self.active_mask = ~self.crop_mask
        else:
            self.crop_mask = np.zeros(self.num_samples, dtype=bool)
            self.active_mask = np.ones(self.num_samples, dtype=bool)

        # TF — spin_thread=True: TF 콜백이 별도 스레드에서 처리됨
        # → 타이머 콜백이 TF 업데이트를 블로킹하지 않음
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(
            self.tf_buffer, self, spin_thread=True)

        # Debug counters
        self._tf_ok_count = 0
        self._tf_fail_count = 0
        self._last_log_x = None
        self._last_log_y = None

        # Subscribe to /map with TRANSIENT_LOCAL QoS
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.create_subscription(OccupancyGrid, 'map', self._map_cb, map_qos)

        # Publish /scan
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)

        # Timer
        rate = self.get_parameter('scan_rate').value
        self.create_timer(1.0 / rate, self._scan_timer)

        self.get_logger().info('Fake scan node started, waiting for map and TF...')

    def _map_cb(self, msg: OccupancyGrid):
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.map_data = np.array(msg.data, dtype=np.int8).reshape(
            (self.map_height, self.map_width))
        self.get_logger().info(
            f'Map received: {self.map_width}x{self.map_height}, '
            f'res={self.map_resolution}m', once=True)

    def _scan_timer(self):
        if self.map_data is None:
            return

        # Look up TF: map → scan_frame (same chain as real robot)
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', self.scan_frame, rclpy.time.Time())
        except TransformException as e:
            self._tf_fail_count += 1
            if self._tf_fail_count % 50 == 1:
                self.get_logger().warn(
                    f'TF lookup failed ({self._tf_fail_count}x): {e}')
            return

        robot_x = tf.transform.translation.x
        robot_y = tf.transform.translation.y
        q = tf.transform.rotation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        robot_yaw = math.atan2(siny_cosp, cosy_cosp)

        # Debug: log position changes and stamp info
        self._tf_ok_count += 1
        now = self.get_clock().now()
        tf_stamp = rclpy.time.Time.from_msg(tf.header.stamp)
        dt_ms = (now.nanoseconds - tf_stamp.nanoseconds) / 1e6
        if (self._last_log_x is None or
                abs(robot_x - self._last_log_x) > 0.05 or
                abs(robot_y - self._last_log_y) > 0.05):
            self.get_logger().info(
                f'TF pos: ({robot_x:.3f}, {robot_y:.3f}, '
                f'yaw={math.degrees(robot_yaw):.1f}°) '
                f'tf_stamp_sec={tf.header.stamp.sec} '
                f'now-tf={dt_ms:.0f}ms '
                f'[ok={self._tf_ok_count}]')
            self._last_log_x = robot_x
            self._last_log_y = robot_y

        # Vectorized raycasting — only active rays (skip cropped region)
        world_angles = robot_yaw + self.angles
        cos_a = np.cos(world_angles)
        sin_a = np.sin(world_angles)

        ranges = np.full(self.num_samples, self.range_max, dtype=np.float64)
        ranges[self.crop_mask] = float('nan')
        active = self.active_mask.copy()
        step = self.map_resolution * 0.5

        d = self.range_min
        while d < self.range_max and np.any(active):
            wx = robot_x + d * cos_a[active]
            wy = robot_y + d * sin_a[active]

            mx = ((wx - self.map_origin_x) / self.map_resolution).astype(np.int32)
            my = ((wy - self.map_origin_y) / self.map_resolution).astype(np.int32)

            # Out of bounds → hit
            oob = (mx < 0) | (mx >= self.map_width) | (my < 0) | (my >= self.map_height)

            # Occupancy check (only for in-bounds rays)
            in_bounds = ~oob
            occupied = np.zeros(len(mx), dtype=bool)
            if np.any(in_bounds):
                ib_idx = np.where(in_bounds)[0]
                occupied[ib_idx] = self.map_data[my[ib_idx], mx[ib_idx]] >= self.occ_thresh

            hit = oob | occupied

            if np.any(hit):
                # Map back to original indices
                active_idx = np.where(active)[0]
                hit_idx = active_idx[hit]
                ranges[hit_idx] = d
                active[hit_idx] = False

            d += step

        # Add gaussian noise to hits (not max-range readings)
        hits = ranges < self.range_max
        n_hits = np.sum(hits)
        if n_hits > 0:
            ranges[hits] += np.random.normal(0.0, self.noise_stddev, n_hits)
            np.clip(ranges, self.range_min, self.range_max, out=ranges)

        # Publish LaserScan
        scan = LaserScan()
        scan.header.stamp = now.to_msg()
        scan.header.frame_id = self.scan_frame
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / self.get_parameter('scan_rate').value
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        ranges_list = ranges.tolist()
        scan.ranges = ranges_list
        self.scan_pub.publish(scan)

        # Write to /dev/shm for bridge (bypasses DDS delivery issues on WSL2)
        try:
            shm_data = json.dumps({
                "angle_min": self.angle_min,
                "angle_max": self.angle_max,
                "angle_increment": self.angle_increment,
                "range_min": self.range_min,
                "range_max": self.range_max,
                "ranges": ranges_list,
            }, separators=(",", ":"))
            tmp = "/dev/shm/ugv_scan.json.tmp"
            with open(tmp, "w") as f:
                f.write(shm_data)
            os.rename(tmp, "/dev/shm/ugv_scan.json")
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = FakeScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
