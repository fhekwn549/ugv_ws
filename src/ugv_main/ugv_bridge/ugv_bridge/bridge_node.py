"""Main entry point: multi-robot fleet bridge.

Runs on a central server. Communicates with RPi robots via CycloneDDS,
exposes MQTT + REST API for web UI.
"""

import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

import uvicorn

from .shared_state import RobotState
from .db_writer import DbWriter
from .ros_interface import RosInterface
from .mqtt_bridge import MqttBridge
from .api_app import RobotHandle, create_app


class BridgeNode(Node):
    def __init__(self):
        super().__init__("bridge_node")

        # -- declare parameters --
        self.declare_parameter("robot_ids", "ugv01")
        self.declare_parameter("mqtt_host", "127.0.0.1")
        self.declare_parameter("mqtt_port", 1883)
        self.declare_parameter("mqtt_keepalive", 60)
        self.declare_parameter("api_host", "0.0.0.0")
        self.declare_parameter("api_port", 8081)
        self.declare_parameter("db_path", "~/ugv_bridge.db")
        self.declare_parameter("static_dir", "")
        self.declare_parameter("pose_rate", 10.0)
        self.declare_parameter("voltage_rate", 1.0)
        self.declare_parameter("joint_rate", 5.0)
        self.declare_parameter("scan_rate", 5.0)
        self.declare_parameter("scan_downsample", 4)
        self.declare_parameter("cmd_vel_timeout", 0.5)
        self.declare_parameter("sensor_snapshot_interval", 10.0)
        self.declare_parameter("db_retention_days", 30)

        # -- read parameters --
        robot_ids_str = self.get_parameter("robot_ids").value
        robot_ids = [r.strip() for r in robot_ids_str.split(",")]

        mqtt_host = self.get_parameter("mqtt_host").value
        mqtt_port = self.get_parameter("mqtt_port").value
        mqtt_keepalive = self.get_parameter("mqtt_keepalive").value
        api_host = self.get_parameter("api_host").value
        api_port = self.get_parameter("api_port").value
        db_path = self.get_parameter("db_path").value
        static_dir = self.get_parameter("static_dir").value
        pose_rate = self.get_parameter("pose_rate").value
        voltage_rate = self.get_parameter("voltage_rate").value
        joint_rate = self.get_parameter("joint_rate").value
        scan_rate = self.get_parameter("scan_rate").value
        scan_downsample = self.get_parameter("scan_downsample").value
        cmd_vel_timeout = self.get_parameter("cmd_vel_timeout").value
        snapshot_interval = self.get_parameter("sensor_snapshot_interval").value
        retention_days = self.get_parameter("db_retention_days").value

        # -- shared database (single for all robots) --
        self._db = DbWriter(db_path, retention_days, self.get_logger())
        self._db.start()

        # -- per-robot instances --
        self._robots: dict[str, RobotHandle] = {}
        self._mqtt_bridges: list[MqttBridge] = []

        for rid in robot_ids:
            # Read optional per-robot topic prefix
            prefix_param = f"{rid}_topic_prefix"
            self.declare_parameter(prefix_param, "")
            topic_prefix = self.get_parameter(prefix_param).value

            state = RobotState()
            ros_if = RosInterface(self, rid, topic_prefix, state, self._db)
            self._robots[rid] = RobotHandle(rid, state, ros_if)

            mqtt_br = MqttBridge(
                mqtt_host, mqtt_port, mqtt_keepalive,
                rid, state, ros_if,
                logger=self.get_logger(),
                pose_rate=pose_rate,
                voltage_rate=voltage_rate,
                joint_rate=joint_rate,
                scan_rate=scan_rate,
                scan_downsample=scan_downsample,
                cmd_vel_timeout=cmd_vel_timeout,
            )
            mqtt_br.start()
            self._mqtt_bridges.append(mqtt_br)

            self.get_logger().info(
                f"Robot '{rid}' registered (topic_prefix='{topic_prefix}')")

        # -- FastAPI --
        self._app = create_app(self._robots, self._db, static_dir)
        self._api_host = api_host
        self._api_port = api_port

        # -- sensor snapshot timer --
        if snapshot_interval > 0:
            self.create_timer(snapshot_interval, self._snapshot_cb)

        self.get_logger().info(
            f"Bridge started: {len(robot_ids)} robot(s), "
            f"API={api_host}:{api_port}, MQTT={mqtt_host}:{mqtt_port}")

    def _snapshot_cb(self):
        for rid, rh in self._robots.items():
            data = rh.state.snapshot_for_db()
            self._db.log_sensor_snapshot(rid, data)

    def run_api(self):
        """Run FastAPI/uvicorn (blocking). Call from main thread."""
        config = uvicorn.Config(
            self._app,
            host=self._api_host,
            port=self._api_port,
            log_level="warning",
        )
        server = uvicorn.Server(config)
        server.run()

    def destroy_node(self):
        for mb in self._mqtt_bridges:
            mb.stop()
        self._db.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()

    # Run ROS executor in a background thread
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    try:
        # FastAPI runs on the main thread
        node.run_api()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
