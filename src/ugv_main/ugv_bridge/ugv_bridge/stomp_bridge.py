"""STOMP publish/subscribe bridge with robot_id namespace.

python-stomp-client의 SessionImpl을 사용하여 RabbitMQ Web STOMP에 연결한다.
"""

import time
import threading

from lib import SessionImpl, SessionHandler, SessionSubscriber

from .shared_state import RobotState
from .ros_interface import RosInterface


def _to_stomp(robot_id: str, name: str) -> str:
    return f"/topic/{robot_id}.{name}"


class _BridgeHandler(SessionHandler):
    """SessionImpl 연결 이벤트를 StompBridge에 전달한다."""

    def __init__(self, bridge: "StompBridge"):
        self._bridge = bridge

    def on_connected(self):
        if self._bridge._logger:
            self._bridge._logger.info(
                f"STOMP connected to {self._bridge._session._ws_url}")

    def on_disconnected(self):
        if self._bridge._logger:
            self._bridge._logger.warn("STOMP disconnected")

    def on_error(self, error):
        if self._bridge._logger:
            self._bridge._logger.error(f"STOMP error: {error}")


class _CommandSubscriber(SessionSubscriber):
    """수신된 명령 메시지를 ROS 2로 라우팅한다."""

    def __init__(self, name: str, bridge: "StompBridge"):
        self._name = name
        self._bridge = bridge

    def on_message(self, body, headers):
        if not isinstance(body, dict):
            if self._bridge._logger:
                self._bridge._logger.error(
                    f"Bad STOMP payload on {self._name}: not a dict")
            return
        self._bridge._dispatch_command(self._name, body)


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

        # cmd_vel watchdog
        self._last_cmd_vel_time: float = 0.0
        self._cmd_vel_active = False

        # STOMP destinations
        self._dest = {name: _to_stomp(robot_id, name) for name in [
            "pose", "voltage", "joint_states", "imu", "map_pose",
            "scan", "nav_status", "path", "map_updated",
            "cmd_vel", "arm", "gripper", "navigate", "cancel", "initial_pose",
        ]}

        # SessionImpl
        ws_url = f"ws://{host}:{port}/ws"
        self._session = SessionImpl(
            ws_url, _BridgeHandler(self),
            user=user, password=password,
            reconnect_min=reconnect_min,
            reconnect_max=reconnect_max,
        )

        # 명령 토픽 구독 등록
        for name in ["cmd_vel", "arm", "gripper", "navigate", "cancel",
                      "initial_pose"]:
            self._session.subscribe(
                self._dest[name], _CommandSubscriber(name, self))

        self._running = False
        self._pub_thread: threading.Thread | None = None

    # -- lifecycle --

    def start(self):
        self._running = True
        self._session.connect()

        self._pub_thread = threading.Thread(
            target=self._publish_loop, daemon=True)
        self._pub_thread.start()

    def stop(self):
        self._running = False
        self._session.disconnect()

        if self._pub_thread:
            self._pub_thread.join(timeout=3)

    # -- command dispatch --

    def _dispatch_command(self, name: str, data: dict):
        try:
            if name == "cmd_vel":
                linear = float(data.get("linear", 0.0))
                angular = float(data.get("angular", 0.0))
                self._ros_if.publish_cmd_vel(linear, angular)
                self._last_cmd_vel_time = time.monotonic()
                self._cmd_vel_active = True

            elif name == "arm":
                positions = [float(p) for p in data.get("positions", [])]
                self._ros_if.publish_arm(positions)

            elif name == "gripper":
                value = float(data.get("value", 0.0))
                self._ros_if.publish_gripper(value)

            elif name == "navigate":
                x = float(data.get("x", 0.0))
                y = float(data.get("y", 0.0))
                theta = float(data.get("theta", 0.0))
                self._ros_if.send_nav_goal(x, y, theta)

            elif name == "cancel":
                self._ros_if.cancel_nav()

            elif name == "initial_pose":
                x = float(data.get("x", 0.0))
                y = float(data.get("y", 0.0))
                yaw = float(data.get("yaw", 0.0))
                self._ros_if.publish_initial_pose(x, y, yaw)

        except Exception as exc:
            if self._logger:
                self._logger.error(f"Bad {name} payload: {exc}")

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

            if now - last["pose"] >= intervals["pose"]:
                self._session.publish(
                    self._dest["pose"], self._state.snapshot_pose())
                last["pose"] = now

            if now - last["imu"] >= intervals["imu"]:
                self._session.publish(
                    self._dest["imu"], self._state.snapshot_imu())
                last["imu"] = now

            if now - last["voltage"] >= intervals["voltage"]:
                self._session.publish(
                    self._dest["voltage"], self._state.snapshot_voltage())
                last["voltage"] = now

            if now - last["joints"] >= intervals["joints"]:
                self._session.publish(
                    self._dest["joint_states"],
                    self._state.snapshot_joints())
                last["joints"] = now

            if now - last["scan"] >= intervals["scan"]:
                self._session.publish(
                    self._dest["scan"],
                    self._state.snapshot_scan(self._scan_downsample))
                last["scan"] = now

            if now - last["map_pose"] >= intervals["map_pose"]:
                self._session.publish(
                    self._dest["map_pose"],
                    self._state.snapshot_map_pose())
                last["map_pose"] = now

            # Nav status (event-driven + periodic during navigating)
            nav = self._state.snapshot_nav()
            nav_changed = nav["status"] != last_nav_status
            nav_periodic = (nav["status"] == "navigating" and
                            now - last.get("nav", 0) >= 1.0)
            if nav_changed or nav_periodic:
                self._session.publish(self._dest["nav_status"], nav)
                last_nav_status = nav["status"]
                last["nav"] = now

            # Path (publish only when changed)
            path = self._state.snapshot_path()
            path_len = len(path["poses"])
            if path_len != last_path_len:
                self._session.publish(self._dest["path"], path)
                last_path_len = path_len

            # Map updated notification
            rev = self._state.snapshot_map_revision()
            if rev != last_map_rev and rev > 0:
                self._session.publish(
                    self._dest["map_updated"], {"revision": rev})
                last_map_rev = rev

            # cmd_vel watchdog
            if self._cmd_vel_active:
                if now - self._last_cmd_vel_time > self._cmd_vel_timeout:
                    self._ros_if.publish_cmd_vel(0.0, 0.0)
                    self._cmd_vel_active = False

            time.sleep(0.02)  # 50Hz tick
