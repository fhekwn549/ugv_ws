# ugv_ws

> [waveshareteam/ugv_ws](https://github.com/waveshareteam/ugv_ws) 포크 - C++ 실시간 드라이버 + MQTT 웹 브릿지 통합

UGV Wave Rover + RoArm-M2 로봇팔을 구동하는 ROS 2 워크스페이스입니다.
RPi에서 하드웨어를 직접 제어하고, MQTT를 통해 웹 대시보드와 연동합니다.

## 시스템 아키텍처

```
┌──────────────────────────────────────────────────────────────┐
│  웹 대시보드 (ugv_dashboard)                                  │
│  Vue 3 + Three.js(3D URDF) + STOMP/WebSocket                │
│  맵/Nav2 목표 | LiDAR 2D | 팔/바퀴/그리퍼 제어 | 상태 패널   │
└──────────┬───────────────────────────────────────────────────┘
           │ STOMP/WS (:15674)
           │ 제어: cmd_vel, arm, gripper, navigate, cancel
           │ 상태: pose, joints, scan, voltage
┌──────────▼───────────────────────────────────────────────────┐
│  RPi: ugv_bridge (ROS 2 노드)                                │
│  paho-mqtt → ROS 2 토픽 변환 | FastAPI (맵/로그 REST)        │
│  RabbitMQ: MQTT:1883 + STOMP:61613 + Web STOMP:15674         │
└──────────┬───────────────────────────────────────────────────┘
           │ ROS 2 토픽 (CycloneDDS)
┌──────────▼───────────────────────────────────────────────────┐
│  RPi ROS 2 노드                                              │
│                                                               │
│  ┌─────────────────┐  ┌──────────────────┐  ┌─────────────┐ │
│  │ ugv_driver_node  │  │ roarm_driver_node│  │ ldlidar_node│ │
│  │ (C++)            │  │ (C++)            │  │ (C++)       │ │
│  │ cmd_vel→모터     │  │ joint_traj→팔    │  │ /scan       │ │
│  │ imu/voltage/odom │  │ gripper→T:106    │  │             │ │
│  └────────┬─────────┘  └────────┬─────────┘  └──────┬──────┘ │
│           │ serial              │ serial             │ serial │
│  ┌────────▼─────────┐  ┌───────▼──────────┐  ┌──────▼──────┐ │
│  │ /dev/ttyAMA0     │  │ /dev/ttyUSB0     │  │ /dev/ttyUSB1│ │
│  │ UGV 바퀴 ESP32   │  │ RoArm-M2 ESP32   │  │ LDLidar LD19│ │
│  └──────────────────┘  └──────────────────┘  └─────────────┘ │
│                                                               │
│  + robot_state_publisher | rf2o_laser_odometry                │
│  + Cartographer (SLAM) | Nav2 (자율주행)                      │
└───────────────────────────────────────────────────────────────┘
```

## 리포 구성

| 리포 | 역할 |
|------|------|
| **이 리포 (`ugv_ws`)** | 하드웨어 드라이버, 센서 처리, MQTT 브릿지, URDF, launch, Nav2 |
| [ugv_dashboard](https://github.com/fhekwn549/ugv_dashboard) | 웹 대시보드 (Vue 3 + Three.js + STOMP) |

## 시리얼 포트 매핑 (RPi)

| 포트 | 장치 | 드라이버 | 패키지 |
|------|------|---------|--------|
| `/dev/ttyAMA0` | UGV 바퀴 ESP32 | `ugv_driver_node` (C++) | `ugv_cpp_nodes` |
| `/dev/ttyUSB0` | RoArm-M2 ESP32 | `roarm_driver_node` (C++) | `ugv_cpp_nodes` |
| `/dev/ttyUSB1` | LDLidar LD19 | `ldlidar_ros2_node` (C++) | `ldlidar_ros2` |

## 주요 노드 상세

### ugv_driver_node (UGV 통합: 제어 + 센서)

- **Subscribe**: `cmd_vel` (Twist) → T:13 모터 속도
- **Publish**: `imu/data` (Imu), `voltage` (Float32), `odom/odom_raw` (Float32MultiArray)
- ESP32 초기화 순서: T:142(피드백 간격 50ms) → T:131(연속 피드백 ON) → T:143(에코 OFF)
- 전용 스레드에서 T:1001 피드백 버퍼 읽기 (20Hz)
- 노드 종료 시 자동 정지 명령

### roarm_driver_node (RoArm-M2 로봇팔)

- **Subscribe**: `/arm_controller/joint_trajectory` (JointTrajectory) → T:102
- **Subscribe**: `/roarm/gripper_cmd` (Float64) → T:106
- **Publish**: `/joint_states` (JointState) ← T:105 (5Hz)

### LiDAR 설정

- Angle Crop: 210~329도 제거 (로봇팔이 가리는 후방)
- 유효 시야: 약 240도

---

## RPi 초기 세팅

### 전제 조건

- RPi 4B + Ubuntu 22.04 Server (arm64)
- ROS 2 Humble 설치 완료
- 네트워크 연결 (IP: 192.168.0.71)

### 1. 클론 및 의존성

```bash
ssh pi@192.168.0.71

# 클론
cd ~ && git clone -b ros2-humble-develop https://github.com/fhekwn549/ugv_ws.git

# ROS 2 의존성
sudo apt install ros-humble-rmw-cyclonedds-cpp ros-humble-xacro \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher

# Python 의존성 (ugv_bridge용)
pip3 install fastapi uvicorn paho-mqtt

# RabbitMQ (MQTT + STOMP 브로커)
sudo apt install rabbitmq-server
sudo cp ~/ugv_ws/src/ugv_main/ugv_bridge/config/enabled_plugins /etc/rabbitmq/enabled_plugins
sudo cp ~/ugv_ws/src/ugv_main/ugv_bridge/config/rabbitmq.conf /etc/rabbitmq/rabbitmq.conf
sudo systemctl restart rabbitmq-server
```

### 2. 스왑 추가 (RAM 1GB인 경우)

rf2o C++ 빌드 시 OOM 방지:

```bash
sudo fallocate -l 2G /swapfile
sudo chmod 600 /swapfile && sudo mkswap /swapfile && sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

### 3. 빌드

```bash
cd ~/ugv_ws && source /opt/ros/humble/setup.bash
colcon build --packages-select ugv_cpp_nodes ugv_bridge ugv_roarm_description \
  ugv_description rf2o_laser_odometry ugv_interface ldlidar
source install/setup.bash
```

### 4. bashrc 설정

```bash
cat >> ~/.bashrc << 'EOF'
source /opt/ros/humble/setup.bash
source ~/ugv_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
EOF
```

### 5. USB 포트 권한

```bash
sudo usermod -aG dialout pi
# 재부팅 필요
```

---

## WSL 초기 세팅

### 전제 조건

- WSL2 + Ubuntu 22.04
- ROS 2 Humble 설치 완료

### 1. 클론 및 의존성

```bash
cd ~ && git clone -b ros2-humble-develop https://github.com/fhekwn549/ugv_ws.git

sudo apt install ros-humble-rmw-cyclonedds-cpp ros-humble-xacro \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui ros-humble-rviz2 ros-humble-tf2-ros
```

### 2. 빌드

```bash
cd ~/ugv_ws && source /opt/ros/humble/setup.bash
colcon build --packages-select ugv_cpp_nodes ugv_roarm_description ugv_description
source install/setup.bash
```

### 3. DDS 설정

`RMW_IMPLEMENTATION`은 **워크스페이스별 ament env-hook**으로 자동 전환됩니다:

```
source ~/ugv_ws/install/setup.bash   → rmw_cyclonedds_cpp (CycloneDDS)
source ~/gmoma_ws/install/setup.bash → rmw_fastrtps_cpp   (FastDDS)
```

설정 파일: `ugv_nav/env-hooks/rmw_implementation.sh`
`.bashrc`에 `RMW_IMPLEMENTATION`을 직접 설정할 필요 없습니다.

WSL2는 NAT 모드라 DDS 멀티캐스트가 안 되므로, CycloneDDS에 RPi 피어를 명시해야 합니다:

```bash
# bashrc에 추가
export CYCLONEDDS_URI=file://$HOME/ugv_ws/src/ugv_main/ugv_bridge/config/cyclonedds.xml
```

`cyclonedds.xml`에 RPi IP(192.168.0.71)가 피어로 설정되어 있습니다.

---

## 매일 실행

### RPi: 로봇 기동

> systemd 서비스는 현재 비활성화 상태입니다. SSH로 수동 실행합니다.

```bash
ssh pi@192.168.0.71
ugv-launch
```

> `ugv-launch`는 `~/.bashrc`에 등록된 alias입니다:
> ```
> ros2 launch ugv_roarm_description rasp_bringup.launch.py 2>&1 | grep -v -E "serdata|rcutils_reset_error|error state is being overwritten|..."
> ```
> CycloneDDS의 `deserialize failed` 경고 등을 필터링합니다.

### RPi: Nav2 자율주행 (별도 터미널)

```bash
ssh pi@192.168.0.71
ugv-nav
```

> `ugv-nav`는 `~/.bashrc`에 등록된 alias입니다:
> ```
> ros2 launch ugv_roarm_description nav_real.launch.py pbstream:=/home/pi/maps/lab_map.pbstream 2>&1 | grep -v -E "serdata|rcutils_reset_error|..."
> ```

> Nav2는 DDS 파티시펀트를 많이 생성합니다.
> `cyclonedds_local.xml`에 `MaxAutoParticipantIndex: 200`이 설정되어 있습니다.

### RPi: alias 등록 (최초 1회)

```bash
cat >> ~/.bashrc << 'ALIASES'
alias ugv-launch='ros2 launch ugv_roarm_description rasp_bringup.launch.py 2>&1 | grep -v -E "serdata|rcutils_reset_error|error state is being overwritten|with this new error message|error_handling.c|>>>|<<<"'
alias ugv-nav='ros2 launch ugv_roarm_description nav_real.launch.py pbstream:=/home/pi/maps/lab_map.pbstream 2>&1 | grep -v -E "serdata|rcutils_reset_error|error state is being overwritten|with this new error message|error_handling.c|>>>|<<<"'
ALIASES
source ~/.bashrc
```

### WSL: 대시보드

```bash
# 윈도우 터미널에서 실행
cd C:\Users\User\Desktop\com.ubisam.boilerplate.frontend
npm run dev.robot
# → http://localhost:3000 접속
```

### WSL: RViz 시각화 (선택)

```bash
source ~/ugv_ws/install/setup.bash
ros2 launch ugv_roarm_description remote_view.launch.py
```

> Nav2는 DDS 파티시펀트를 많이 생성합니다.
> `cyclonedds_local.xml`에 `MaxAutoParticipantIndex: 200`이 설정되어 있습니다.

---

## 배포 (코드 수정 후)

```bash
# [WSL] push
cd ~/ugv_ws
git add -A && git commit -m "설명" && git push origin ros2-humble-develop

# [RPi] pull & build
ssh pi@192.168.0.71
cd ~/ugv_ws && git pull origin ros2-humble-develop
colcon build --packages-select ugv_cpp_nodes ugv_bridge rf2o_laser_odometry ugv_roarm_description
source install/setup.bash
```

---

## 패키지 구조

```
ugv_ws/src/
├── ugv_main/
│   ├── ugv_cpp_nodes/          # C++ 실시간 시리얼 드라이버 (ugv_driver, roarm_driver)
│   ├── ugv_bridge/             # MQTT + REST API 웹 브릿지
│   ├── ugv_roarm_description/  # URDF, launch, RViz, Gazebo, Nav2 설정
│   ├── ugv_description/        # 기본 UGV URDF
│   ├── ugv_bringup/            # Python 드라이버 (롤백용, 현재 미사용)
│   ├── ugv_nav/                # Nav2 설정, 맵
│   ├── ugv_interface/          # 커스텀 메시지/서비스
│   └── ugv_fleet_sim/          # 멀티로봇 시뮬레이션 (개발 중)
├── ugv_else/
│   └── ldlidar/                # LDLidar ROS 2 드라이버
└── third_party/
    └── rf2o_laser_odometry/    # LiDAR 스캔 매칭 오도메트리
```

## C++ 드라이버 아키텍처 (ugv_cpp_nodes)

3계층 분리로 테스트 용이성과 재사용성을 확보합니다:

```
ROS2 노드 계층 (ugv_driver_node, roarm_driver_node)
  - 토픽 구독/발행, 파라미터, 타이머 (rclcpp 의존)
        │
프로토콜 드라이버 계층 (ugv_serial_driver, roarm_serial_driver)
  - ESP32 JSON 프로토콜, 에코 처리, JSON 파싱 (순수 C++)
        │
시리얼 포트 계층 (serial_driver)
  - POSIX termios, select(), 버퍼 읽기 (순수 C++)
```

### Python 롤백

기존 Python 드라이버(`ugv_bringup`)는 그대로 유지됩니다.
`rasp_bringup.launch.py`에서 패키지명만 교체하면 롤백 가능:

```python
# C++ (현재)
Node(package='ugv_cpp_nodes', executable='ugv_driver_node', name='ugv_driver')
Node(package='ugv_cpp_nodes', executable='roarm_driver_node', name='roarm_driver')

# Python (롤백) — 시리얼 포트 이중 접근 주의
Node(package='ugv_bringup', executable='ugv_bringup', name='ugv_bringup')
Node(package='ugv_bringup', executable='ugv_driver', name='ugv_driver')
Node(package='ugv_bringup', executable='roarm_driver', name='roarm_driver')
```

## 기술 스택 요약

| 항목 | 기술 |
|------|------|
| OS | Ubuntu 22.04 (RPi arm64 / WSL2 x86) |
| ROS 2 | Humble |
| DDS | CycloneDDS (`rmw_cyclonedds_cpp`) — ament env-hook으로 워크스페이스별 자동 전환 |
| 시리얼 | POSIX termios (C++) |
| 메시지 브로커 | RabbitMQ (MQTT + STOMP + WebSocket) |
| 웹 브릿지 | paho-mqtt + FastAPI |
| SLAM | Cartographer |
| 오도메트리 | rf2o (LiDAR 스캔 매칭) |
| 자율주행 | Nav2 |

---

> **Upstream 문서**: 원본 Waveshare ugv_ws 문서는 [waveshareteam/ugv_ws](https://github.com/waveshareteam/ugv_ws) 참조
