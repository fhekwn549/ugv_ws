# ugv_ws

> [waveshareteam/ugv_ws](https://github.com/waveshareteam/ugv_ws) 포크 — C++ 실시간 드라이버 + STOMP 웹 브릿지 통합

UGV Wave Rover + RoArm-M2 로봇팔을 구동하는 ROS 2 워크스페이스입니다.
RPi에서 하드웨어를 직접 제어하고, STOMP/WebSocket을 통해 웹 대시보드와 연동합니다.

## 시스템 아키텍처

```
┌─────────────────────────────────────────────────────────────────┐
│  웹 대시보드 (ugv-frontend, boilerplate 02_robot)                │
│  Vue 3 + Vuetify + Three.js(3D URDF) + stompjs                 │
│  Light/Dark 테마 | MoveIt2 스타일 ghost arm preview              │
└──────────┬────────────────────────────┬─────────────────────────┘
           │ STOMP/WS                   │ REST (맵 PNG)
           │ (stomp-web 프록시 경유)      │ (FastAPI 직접)
┌──────────▼────────────────────┐       │
│  Docker (윈도우)               │       │
│  stomp-web(:9030) ← OAuth2    │       │
│  oauth2-web(:9020)            │       │
│  RabbitMQ(:15674 Web STOMP)   │       │
│  Spring Boot(:8080)           │       │
└──────────┬────────────────────┘       │
           │ STOMP/WS (:15674)          │
┌──────────▼────────────────────────────▼─────────────────────────┐
│  RPi: ugv_bridge (ROS 2 Python 노드)                            │
│  python-stomp-client → ROS 2 토픽 변환 | FastAPI (:8081) REST   │
└──────────┬──────────────────────────────────────────────────────┘
           │ ROS 2 토픽 (CycloneDDS)
┌──────────▼──────────────────────────────────────────────────────┐
│  RPi ROS 2 노드                                                  │
│                                                                   │
│  ┌─────────────────┐  ┌──────────────────┐  ┌─────────────────┐ │
│  │ ugv_driver_node  │  │ roarm_driver_node│  │ ldlidar_node    │ │
│  │ (C++)            │  │ (C++)            │  │ (C++)           │ │
│  │ cmd_vel→모터     │  │ joint_traj→팔    │  │ /scan           │ │
│  │ imu/voltage/odom │  │ gripper→T:106    │  │                 │ │
│  └────────┬─────────┘  └────────┬─────────┘  └───────┬─────────┘ │
│           │ serial              │ serial              │ serial   │
│  ┌────────▼─────────┐  ┌───────▼──────────┐  ┌──────▼────────┐ │
│  │ /dev/ttyAMA0     │  │ /dev/ttyUSB0     │  │ /dev/ttyUSB1  │ │
│  │ UGV 바퀴 ESP32   │  │ RoArm-M2 ESP32   │  │ LDLidar LD19  │ │
│  └──────────────────┘  └──────────────────┘  └───────────────┘ │
│                                                                   │
│  + robot_state_publisher | rf2o_laser_odometry                    │
│  + Cartographer (SLAM) | Nav2 (자율주행)                          │
└───────────────────────────────────────────────────────────────────┘
```

### 통신 경로 요약

| 데이터 | 경로 | 인증 |
|--------|------|------|
| 센서/제어 (소량, 빈번) | 브라우저 → stomp-web(:9030, OAuth2) → RabbitMQ → ugv_bridge | OAuth2 (stomp-web이 검증) |
| 맵 이미지 (대용량, 드물게) | 브라우저 → FastAPI(:8081, RPi) 직접 | 개발: 없음 / 프로덕션: Spring Boot 경유 예정 |
| 맵 업데이트 알림 | STOMP `map_updated` → stomp-web 경유 | OAuth2 |

## 리포 구성

| 리포 | 역할 |
|------|------|
| **이 리포 (`ugv_ws`)** | 하드웨어 드라이버, 센서 처리, STOMP 브릿지, URDF, launch, Nav2 |
| [ugv-frontend](https://github.com/ubisamRAD/ugv-frontend) | 웹 대시보드 (boilerplate 02_robot 앱) |

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

### ugv_bridge (STOMP + REST 브릿지)

- **STOMP**: python-stomp-client(`SessionImpl`)로 RabbitMQ Web STOMP(:15674)에 연결
- **REST**: FastAPI(:8081)로 맵 PNG, 로그, 제어 명령 제공
- **구독 (하향)**: `ugv01.cmd_vel`, `ugv01.arm`, `ugv01.gripper`, `ugv01.navigate`, `ugv01.cancel`, `ugv01.initial_pose`
- **발행 (상향)**: `ugv01.pose`(5Hz), `ugv01.voltage`(0.2Hz), `ugv01.joint_states`(3Hz), `ugv01.scan`(2Hz, 4x 다운샘플), `ugv01.map_pose`(5Hz), `ugv01.nav_status`, `ugv01.path`
- **안전장치**: 0.5초 이상 cmd_vel 없으면 자동 정지 명령 발행
- **OAuth2**: `oauth2_jwks_uri` 파라미터로 JWT 검증 활성화 가능 (현재 개발 환경에서는 비활성화)

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

cd ~ && git clone -b ros2-humble-develop https://github.com/ubisamRAD/ugv_ws.git

# ROS 2 의존성
sudo apt install ros-humble-rmw-cyclonedds-cpp ros-humble-xacro \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher

# Python 의존성 (ugv_bridge용)
pip3 install fastapi uvicorn "PyJWT[crypto]"
pip3 install "python-stomp-client @ git+https://github.com/ubisamRAD/python-stomp-client.git@main"
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

alias ugv-launch='ros2 launch ugv_roarm_description rasp_bringup.launch.py 2>&1 | grep -v -E "serdata|rcutils_reset_error|error state is being overwritten|with this new error message|error_handling.c|>>>|<<<"'
alias ugv-nav='ros2 launch ugv_roarm_description nav_real.launch.py pbstream:=/home/pi/maps/lab_map.pbstream 2>&1 | grep -v -E "serdata|rcutils_reset_error|error state is being overwritten|with this new error message|error_handling.c|>>>|<<<"'
EOF
source ~/.bashrc
```

### 5. USB 포트 권한

```bash
sudo usermod -aG dialout pi
# 재부팅 필요
```

---

## 윈도우 Docker 세팅

### 전제 조건

- Docker Desktop 설치 완료
- `com.ubisam.boilerplate.frontend` 리포 클론 완료

### Docker 서비스 실행

```bash
cd C:\Users\User\Desktop\com.ubisam.boilerplate.frontend
docker compose -f Dockerfile-dev.yml up -d
```

실행되는 서비스:

| 컨테이너 | 포트 | 역할 |
|----------|------|------|
| oauth2-web | :9020 | OAuth2 인증 서버 |
| stomp-web | :9030 | STOMP 프록시 (OAuth2 검증 + RabbitMQ 중계) |
| stomp-mq | :15674, :15672, :61613 | RabbitMQ (Web STOMP + 관리 UI) |
| examples-backend | :8080 | Spring Boot 백엔드 |
| oauth2-db | :5432 | PostgreSQL |

---

## WSL 초기 세팅

### 전제 조건

- WSL2 + Ubuntu 22.04
- ROS 2 Humble 설치 완료

### 1. 클론 및 의존성

```bash
cd ~ && git clone -b ros2-humble-develop https://github.com/ubisamRAD/ugv_ws.git

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

WSL2는 NAT 모드라 DDS 멀티캐스트가 안 되므로, CycloneDDS에 RPi 피어를 명시해야 합니다:

```bash
# bashrc에 추가
export CYCLONEDDS_URI=file://$HOME/ugv_ws/src/ugv_main/ugv_bridge/config/cyclonedds.xml
```

`cyclonedds.xml`에 RPi IP(192.168.0.71)가 피어로 설정되어 있습니다.

---

## 매일 실행

### 1. Docker 서비스 확인 (윈도우)

```powershell
docker ps --format "table {{.Names}}\t{{.Ports}}"
```

### 2. RPi: 로봇 기동

```bash
ssh pi@192.168.0.71
ugv-launch
```

### 3. RPi: Nav2 자율주행 (별도 터미널)

```bash
ssh pi@192.168.0.71
ugv-nav
```

### 4. 대시보드 (윈도우 터미널)

```bash
cd C:\Users\User\Desktop\com.ubisam.boilerplate.frontend
npm run dev.robot
# → http://localhost:3000 접속
```

### 5. RViz 시각화 (WSL, 선택)

```bash
source ~/ugv_ws/install/setup.bash
ros2 launch ugv_roarm_description remote_view.launch.py
```

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
│   ├── ugv_bridge/             # STOMP + REST API 웹 브릿지 (python-stomp-client + FastAPI)
│   ├── ugv_roarm_description/  # URDF, launch, RViz, Gazebo, Nav2 설정
│   ├── ugv_description/        # 기본 UGV URDF
│   ├── ugv_bringup/            # Python 드라이버 (롤백용, 현재 미사용)
│   ├── ugv_nav/                # Nav2 설정, 맵, DDS env-hook
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
| 메시지 브로커 | RabbitMQ (STOMP + WebSocket) |
| STOMP 클라이언트 | python-stomp-client (SessionImpl) |
| REST API | FastAPI (맵 PNG, 로그) |
| 인증 | OAuth2 (stomp-web 프록시가 검증, FastAPI는 개발 환경에서 비활성화) |
| SLAM | Cartographer |
| 오도메트리 | rf2o (LiDAR 스캔 매칭) |
| 자율주행 | Nav2 |

---

> **Upstream 문서**: 원본 Waveshare ugv_ws 문서는 [waveshareteam/ugv_ws](https://github.com/waveshareteam/ugv_ws) 참조
