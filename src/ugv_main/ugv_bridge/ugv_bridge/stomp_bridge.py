"""STOMP publish/subscribe bridge with robot_id namespace.

Replaces MqttBridge (paho-mqtt) with WebSocket + STOMP protocol.
RabbitMQ Web STOMP plugin (:15674) 에 직접 연결한다.
"""

import json
import threading
import time

import stomper
import websocket

from .shared_state import RobotState
from .ros_interface import RosInterface


def _to_stomp(robot_id: str, name: str) -> str:
    """MQTT 스타일 토픽을 STOMP destination으로 변환한다.

    예: ("ugv01", "pose") → "/topic/ugv01.pose"
    """
    return f"/topic/{robot_id}.{name}"


class StompBridge:
    """Bridges ROS 2 state to STOMP and STOMP commands to ROS 2."""

    def __init__(self, host: str, port: int,
                 robot_id: str, state: RobotState, ros_if: RosInterface,
                 user: str = "guest", password: str = "guest",
                 logger=None,
                 pose_rate: float = 10.0,
                 voltage_rate: float = 1.0,
                 joint_rate: float = 5.0,
                 scan_rate: float = 5.0,
                 scan_downsample: int = 4,
                 cmd_vel_timeout: float = 0.5,
                 reconnect_min: float = 1.0,
                 reconnect_max: float = 30.0):
        self._host = host
        self._port = port
        self._robot_id = robot_id
        self._state = state
        self._ros_if = ros_if
        self._user = user
        self._password = password
        self._logger = logger

        self._pose_rate = pose_rate
        self._voltage_rate = voltage_rate
        self._joint_rate = joint_rate
        self._scan_rate = scan_rate
        self._scan_downsample = scan_downsample
        self._cmd_vel_timeout = cmd_vel_timeout
        self._reconnect_min = reconnect_min
        self._reconnect_max = reconnect_max

        self._ws: websocket.WebSocketApp | None = None
        self._connected = False
        self._running = False
        self._lock = threading.Lock()
        self._ws_thread: threading.Thread | None = None
        self._pub_thread: threading.Thread | None = None
        self._sub_counter = 0

        # cmd_vel watchdog
        self._last_cmd_vel_time: float = 0.0
        self._cmd_vel_active = False

        # STOMP destinations
        self._dest = {name: _to_stomp(robot_id, name) for name in [
            "pose", "voltage", "joint_states", "imu", "map_pose",
            "scan", "nav_status", "path", "map_updated",
            "cmd_vel", "arm", "gripper", "navigate", "cancel", "initial_pose",
        ]}

    @property
    def url(self) -> str:
        return f"ws://{self._host}:{self._port}/ws"

    # -- lifecycle --

    def start(self):
        self._running = True
        self._ws_thread = threading.Thread(target=self._ws_loop, daemon=True)
        self._ws_thread.start()

        self._pub_thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._pub_thread.start()

    def stop(self):
        self._running = False

        with self._lock:
            if self._ws and self._connected:
                try:
                    self._ws.send(stomper.disconnect())
                except Exception:
                    pass

        if self._ws:
            try:
                self._ws.close()
            except Exception:
                pass

        if self._pub_thread:
            self._pub_thread.join(timeout=3)
        if self._ws_thread:
            self._ws_thread.join(timeout=3)

    # -- WebSocket + STOMP --

    def _ws_loop(self):
        """재연결 루프. stop()이 호출되면 종료된다."""
        delay = self._reconnect_min
        while self._running:
            try:
                self._connect_once()
            except Exception as exc:
                if self._logger:
                    self._logger.error(f"STOMP connection error: {exc}")

            with self._lock:
                self._connected = False

            if not self._running:
                break

            if self._logger:
                self._logger.warn(f"STOMP disconnected, retry in {delay:.0f}s")
            time.sleep(delay)
            delay = min(delay * 2, self._reconnect_max)

    def _connect_once(self):
        self._ws = websocket.WebSocketApp(
            self.url,
            on_open=self._on_open,
            on_message=self._on_ws_message,
            on_error=self._on_error,
            on_close=self._on_close,
        )
        if self._logger:
            self._logger.info(f"STOMP connecting to {self.url}")
        self._ws.run_forever()

    def _on_open(self, ws):
        connect_frame = stomper.connect(self._user, self._password, host="/")
        ws.send(connect_frame)

    def _on_ws_message(self, ws, message):
        frame = stomper.unpack_frame(message)
        command = frame.get("cmd", "")

        if command == "CONNECTED":
            with self._lock:
                self._connected = True
            if self._logger:
                self._logger.info(
                    f"STOMP connected to {self._host}:{self._port}")
            self._subscribe_commands()

        elif command == "MESSAGE":
            self._handle_message(frame)

        elif command == "ERROR":
            if self._logger:
                self._logger.error(f"STOMP error: {frame.get('body', '')}")

    def _on_error(self, ws, error):
        if self._running and self._logger:
            self._logger.error(f"WebSocket error: {error}")

    def _on_close(self, ws, close_status, close_msg):
        with self._lock:
            self._connected = False

    def _subscribe_commands(self):
        """제어 명령 토픽을 구독한다."""
        for name in ["cmd_vel", "arm", "gripper", "navigate", "cancel",
                      "initial_pose"]:
            self._sub_counter += 1
            sub_id = f"sub-{self._sub_counter}"
            sub_frame = stomper.subscribe(self._dest[name], sub_id, ack="auto")
            try:
                self._ws.send(sub_frame)
                if self._logger:
                    self._logger.info(
                        f"Subscribed: {self._dest[name]} (id={sub_id})")
            except Exception as exc:
                if self._logger:
                    self._logger.error(f"Subscribe failed: {exc}")

    # -- inbound message handling --

    def _handle_message(self, frame: dict):
        destination = frame.get("headers", {}).get("destination", "")
        raw_body = frame.get("body", "")

        try:
            data = json.loads(raw_body)
        except Exception as exc:
            if self._logger:
                self._logger.error(
                    f"Bad STOMP payload on {destination}: {exc}")
            return

        if destination == self._dest["cmd_vel"]:
            try:
                linear = float(data.get("linear", 0.0))
                angular = float(data.get("angular", 0.0))
                self._ros_if.publish_cmd_vel(linear, angular)
                self._last_cmd_vel_time = time.monotonic()
                self._cmd_vel_active = True
            except Exception as exc:
                if self._logger:
                    self._logger.error(f"Bad cmd_vel payload: {exc}")

        elif destination == self._dest["arm"]:
            try:
                positions = [float(p) for p in data.get("positions", [])]
                self._ros_if.publish_arm(positions)
            except Exception as exc:
                if self._logger:
                    self._logger.error(f"Bad arm payload: {exc}")

        elif destination == self._dest["gripper"]:
            try:
                value = float(data.get("value", 0.0))
                self._ros_if.publish_gripper(value)
            except Exception as exc:
                if self._logger:
                    self._logger.error(f"Bad gripper payload: {exc}")

        elif destination == self._dest["navigate"]:
            try:
                x = float(data.get("x", 0.0))
                y = float(data.get("y", 0.0))
                theta = float(data.get("theta", 0.0))
                self._ros_if.send_nav_goal(x, y, theta)
            except Exception as exc:
                if self._logger:
                    self._logger.error(f"Bad navigate payload: {exc}")

        elif destination == self._dest["cancel"]:
            try:
                self._ros_if.cancel_nav()
            except Exception as exc:
                if self._logger:
                    self._logger.error(f"Bad cancel payload: {exc}")

        elif destination == self._dest["initial_pose"]:
            try:
                x = float(data.get("x", 0.0))
                y = float(data.get("y", 0.0))
                yaw = float(data.get("yaw", 0.0))
                self._ros_if.publish_initial_pose(x, y, yaw)
            except Exception as exc:
                if self._logger:
                    self._logger.error(f"Bad initial_pose payload: {exc}")

    # -- publish loop --

    def _publish_loop(self):
        intervals = {
            "pose": 1.0 / max(self._pose_rate, 0.1),
            "voltage": 1.0 / max(self._voltage_rate, 0.1),
            "joints": 1.0 / max(self._joint_rate, 0.1),
            "scan": 1.0 / max(self._scan_rate, 0.1),
            "imu": 1.0 / max(self._pose_rate, 0.1),
            "map_pose": 1.0 / max(self._pose_rate, 0.1),
        }
        last = {k: 0.0 for k in intervals}
        last_map_rev = -1
        last_nav_status = ""
        last_path_len = 0

        while self._running:
            now = time.monotonic()

            # Pose
            if now - last["pose"] >= intervals["pose"]:
                self._publish_json("pose", self._state.snapshot_pose())
                last["pose"] = now

            # IMU
            if now - last["imu"] >= intervals["imu"]:
                self._publish_json("imu", self._state.snapshot_imu())
                last["imu"] = now

            # Voltage
            if now - last["voltage"] >= intervals["voltage"]:
                self._publish_json("voltage", self._state.snapshot_voltage())
                last["voltage"] = now

            # Joint states
            if now - last["joints"] >= intervals["joints"]:
                self._publish_json("joint_states",
                                   self._state.snapshot_joints())
                last["joints"] = now

            # Scan
            if now - last["scan"] >= intervals["scan"]:
                self._publish_json("scan",
                                   self._state.snapshot_scan(
                                       self._scan_downsample))
                last["scan"] = now

            # Map pose (TF-based)
            if now - last["map_pose"] >= intervals["map_pose"]:
                self._publish_json("map_pose",
                                   self._state.snapshot_map_pose())
                last["map_pose"] = now

            # Nav status (event-driven + periodic during navigating)
            nav = self._state.snapshot_nav()
            nav_changed = nav["status"] != last_nav_status
            nav_periodic = (nav["status"] == "navigating" and
                            now - last.get("nav", 0) >= 1.0)
            if nav_changed or nav_periodic:
                self._publish_json("nav_status", nav)
                last_nav_status = nav["status"]
                last["nav"] = now

            # Path (publish only when changed)
            path = self._state.snapshot_path()
            path_len = len(path["poses"])
            if path_len != last_path_len:
                self._publish_json("path", path)
                last_path_len = path_len

            # Map updated notification
            rev = self._state.snapshot_map_revision()
            if rev != last_map_rev and rev > 0:
                self._publish_json("map_updated", {"revision": rev})
                last_map_rev = rev

            # cmd_vel watchdog
            if self._cmd_vel_active:
                if now - self._last_cmd_vel_time > self._cmd_vel_timeout:
                    self._ros_if.publish_cmd_vel(0.0, 0.0)
                    self._cmd_vel_active = False

            time.sleep(0.02)  # 50Hz tick

    def _publish_json(self, suffix: str, data: dict):
        with self._lock:
            if not self._ws or not self._connected:
                return

        try:
            payload = json.dumps(data, separators=(",", ":"),
                                 allow_nan=False)
        except ValueError:
            payload = json.dumps(data, separators=(",", ":"),
                                 default=str)
            payload = payload.replace("NaN", "null").replace("Infinity", "null")

        destination = self._dest[suffix]
        send_frame = stomper.send(destination, payload,
                                  content_type="application/json")
        try:
            with self._lock:
                if self._ws and self._connected:
                    self._ws.send(send_frame)
        except Exception:
            pass
