"""MQTT publish/subscribe bridge with robot_id namespace."""

import json
import threading
import time

import paho.mqtt.client as mqtt

from .shared_state import RobotState
from .ros_interface import RosInterface


class MqttBridge:
    """Bridges ROS 2 state to MQTT and MQTT commands to ROS 2."""

    def __init__(self, host: str, port: int, keepalive: int,
                 robot_id: str, state: RobotState, ros_if: RosInterface,
                 logger=None,
                 pose_rate: float = 10.0,
                 voltage_rate: float = 1.0,
                 joint_rate: float = 5.0,
                 scan_rate: float = 5.0,
                 scan_downsample: int = 4,
                 cmd_vel_timeout: float = 0.5):
        self._host = host
        self._port = port
        self._keepalive = keepalive
        self._robot_id = robot_id
        self._state = state
        self._ros_if = ros_if
        self._logger = logger

        self._pose_rate = pose_rate
        self._voltage_rate = voltage_rate
        self._joint_rate = joint_rate
        self._scan_rate = scan_rate
        self._scan_downsample = scan_downsample
        self._cmd_vel_timeout = cmd_vel_timeout

        self._client: mqtt.Client | None = None
        self._running = False
        self._pub_thread: threading.Thread | None = None

        # cmd_vel watchdog
        self._last_cmd_vel_time: float = 0.0
        self._cmd_vel_active = False

    def _topic(self, name: str) -> str:
        return f"{self._robot_id}/{name}"

    # -- lifecycle --

    def start(self):
        self._client = mqtt.Client(
            client_id=f"ugv_bridge_{self._robot_id}",
            protocol=mqtt.MQTTv311,
        )
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message
        self._client.on_disconnect = self._on_disconnect
        self._client.reconnect_delay_set(min_delay=1, max_delay=30)

        try:
            self._client.connect(self._host, self._port, self._keepalive)
        except Exception as exc:
            if self._logger:
                self._logger.error(f"MQTT connect failed: {exc}")
            return

        self._client.loop_start()
        self._running = True
        self._pub_thread = threading.Thread(target=self._publish_loop,
                                            daemon=True)
        self._pub_thread.start()

    def stop(self):
        self._running = False
        if self._pub_thread:
            self._pub_thread.join(timeout=3)
        if self._client:
            self._client.loop_stop()
            self._client.disconnect()

    # -- MQTT callbacks --

    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            if self._logger:
                self._logger.info(f"MQTT connected to {self._host}:{self._port}")
            client.subscribe(self._topic("cmd_vel"), qos=0)
        else:
            if self._logger:
                self._logger.error(f"MQTT connect rc={rc}")

    def _on_disconnect(self, client, userdata, rc):
        if self._logger:
            self._logger.warn(f"MQTT disconnected rc={rc}")

    def _on_message(self, client, userdata, msg: mqtt.MQTTMessage):
        topic_suffix = msg.topic.replace(f"{self._robot_id}/", "", 1)

        if topic_suffix == "cmd_vel":
            try:
                data = json.loads(msg.payload)
                linear = float(data.get("linear", 0.0))
                angular = float(data.get("angular", 0.0))
                self._ros_if.publish_cmd_vel(linear, angular)
                self._last_cmd_vel_time = time.monotonic()
                self._cmd_vel_active = True
            except Exception as exc:
                if self._logger:
                    self._logger.error(f"Bad cmd_vel payload: {exc}")

    # -- publish loop --

    def _publish_loop(self):
        intervals = {
            "pose": 1.0 / max(self._pose_rate, 0.1),
            "voltage": 1.0 / max(self._voltage_rate, 0.1),
            "joints": 1.0 / max(self._joint_rate, 0.1),
            "scan": 1.0 / max(self._scan_rate, 0.1),
        }
        last = {k: 0.0 for k in intervals}
        last_map_rev = -1
        last_nav_status = ""

        while self._running:
            now = time.monotonic()

            # Pose
            if now - last["pose"] >= intervals["pose"]:
                self._publish_json("pose", self._state.snapshot_pose(), qos=0)
                last["pose"] = now

            # Voltage
            if now - last["voltage"] >= intervals["voltage"]:
                self._publish_json("voltage", self._state.snapshot_voltage(), qos=0)
                last["voltage"] = now

            # Joint states
            if now - last["joints"] >= intervals["joints"]:
                self._publish_json("joint_states", self._state.snapshot_joints(), qos=0)
                last["joints"] = now

            # Scan
            if now - last["scan"] >= intervals["scan"]:
                self._publish_json("scan",
                                   self._state.snapshot_scan(self._scan_downsample),
                                   qos=0)
                last["scan"] = now

            # Nav status (event-driven)
            nav = self._state.snapshot_nav()
            if nav["status"] != last_nav_status:
                self._publish_json("nav_status", nav, qos=1)
                last_nav_status = nav["status"]

            # Path (event-driven via map revision as proxy)
            path = self._state.snapshot_path()
            if path["poses"]:
                self._publish_json("path", path, qos=0)

            # Map updated notification
            rev = self._state.snapshot_map_revision()
            if rev != last_map_rev and rev > 0:
                self._publish_json("map_updated", {"revision": rev}, qos=1)
                last_map_rev = rev

            # cmd_vel watchdog
            if self._cmd_vel_active:
                if now - self._last_cmd_vel_time > self._cmd_vel_timeout:
                    self._ros_if.publish_cmd_vel(0.0, 0.0)
                    self._cmd_vel_active = False

            time.sleep(0.02)  # 50Hz tick

    def _publish_json(self, suffix: str, data: dict, qos: int = 0):
        if not self._client or not self._client.is_connected():
            return
        try:
            payload = json.dumps(data, separators=(",", ":"))
            self._client.publish(self._topic(suffix), payload, qos=qos)
        except Exception:
            pass
