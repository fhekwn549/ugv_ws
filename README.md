# ugv_ws (Fork)

> Fork of [waveshareteam/ugv_ws](https://github.com/waveshareteam/ugv_ws) with RoArm-M2 integration + MQTT/REST API 웹 브릿지.

> **처음 세팅하시나요?** → [개발 환경 세팅 가이드 (DEV_SETUP_GUIDE.md)](DEV_SETUP_GUIDE.md)를 참고하세요.

## 이 리포의 역할

**로봇을 구동하는 드라이버 코드와 웹 브릿지**를 관리합니다. 시리얼 통신으로 하드웨어(모터, 센서, 로봇팔)를 직접 제어하고, MQTT + REST API를 통해 웹 대시보드와 연동하는 ROS 2 노드들이 포함되어 있습니다.

주로 **RPi에서 실행**되며, 로봇의 URDF 모델이나 시뮬레이션, launch 파일은 `src/ugv_main/ugv_roarm_description/` 패키지에서 관리합니다.

### 리포 구조

| 리포 | 역할 | 내용 |
|------|------|------|
| **이 리포 (`ugv_ws`)** | 하드웨어 구동 + 웹 브릿지 + 로봇 정의 | 시리얼 드라이버, 센서 처리, MQTT/REST 브릿지, URDF, launch, Nav2, Gazebo 시뮬레이션 |
| [ugv_dashboard](https://github.com/fhekwn549/ugv_dashboard) | 웹 대시보드 프론트엔드 | Vue 3 + STOMP/WebSocket, 맵/LiDAR 시각화, 원격 제어 |

RPi에서는 `ugv_ws` 하나만 클론하면 됩니다. `ugv_roarm_description`의 `rasp_bringup.launch.py`가 드라이버 노드들을 실행합니다. 웹 대시보드는 `ugv_bridge`의 FastAPI가 정적 파일을 서빙합니다.

### 시리얼 포트 매핑 (RPi)

| 포트 | 장치 | 드라이버 노드 | 언어 | 패키지 |
|------|------|--------------|------|--------|
| `/dev/ttyAMA0` | UGV 바퀴 ESP32 | `ugv_driver_node` | **C++** | `ugv_cpp_nodes` |
| `/dev/ttyUSB0` | RoArm-M2 ESP32 | `roarm_driver_node` | **C++** | `ugv_cpp_nodes` |
| `/dev/ttyUSB1` | LDLidar (STL-19P) | `ldlidar_ros2_node` | C++ | `ldlidar_ros2` |

> **C++ 드라이버 통합 (2026-03)**: 실시간 통신 성능 개선을 위해 `ugv_driver`, `roarm_driver`, `ugv_bringup`(센서 피드백)을 Python에서 C++로 전환했습니다.
> `ugv_driver_node`가 제어(cmd_vel)와 센서(IMU, 전압, 인코더)를 단일 노드에서 처리하여 시리얼 포트 이중 접근 문제를 해결합니다.
> 기존 Python 드라이버(`ugv_bringup` 패키지)는 그대로 유지되므로, 필요 시 launch 파일에서 패키지명만 바꾸면 롤백 가능합니다.
> 자세한 내용은 아래 [C++ 실시간 드라이버](#added-ugv_cpp_nodes-c-실시간-시리얼-드라이버) 섹션을 참고하세요.

### LiDAR 설정

- **Angle Crop**: 210°~329° 영역 제거 (로봇팔이 가리는 후방 영역)
- 유효 시야: 약 240° (자율주행에 충분)
- 설정 위치: `rasp_bringup.launch.py`의 `angle_crop_min/max` 파라미터

### 시스템 아키텍처

```
┌─────────────────────────────────────────────────────────────┐
│  웹 브라우저 (ugv_dashboard)                                │
│  Vue 3 + @stomp/stompjs + Canvas                            │
│  맵 시각화 / LiDAR 2D / 원격 제어 / Nav2 목표 전송          │
└──────────┬──────────────────────┬───────────────────────────┘
           │ STOMP/WS (15674)     │ REST (8081, 맵/로그만)
           │ 제어: cmd_vel, arm,  │
           │ gripper, navigate    │
┌──────────▼──────────────────────▼───────────────────────────┐
│  RPi: ugv_bridge + RabbitMQ                                 │
│  paho-mqtt(MQTT 구독→ROS 2) + FastAPI(맵/로그 REST)        │
│  - MQTT 구독: cmd_vel, arm, gripper, navigate, cancel       │
│  - MQTT 발행: pose, map_pose, joints, scan, voltage         │
│  - REST: 맵 PNG, 로그 조회 (읽기 전용)                      │
│  - RabbitMQ: MQTT:1883 + STOMP:61613 + Web STOMP:15674     │
└──────────┬──────────────────────────────────────────────────┘
           │ FastDDS (ROS 2 토픽)
┌──────────▼──────────────────────────────────────────────────┐
│  RPi ROS 2 노드                                            │
│  ugv_driver(C++, 제어+센서) + roarm_driver(C++) + ldlidar   │
│  + rf2o_odom + Cartographer (SLAM/localization) + Nav2      │
└─────────────────────────────────────────────────────────────┘
```

---

## Quick Start: 처음부터 실행까지

> **전제 조건**: RPi에 Ubuntu 22.04 Server (arm64) + ROS 2 Humble 설치 완료, WSL2에 Ubuntu 22.04 + ROS 2 Humble 설치 완료

**실행 순서와 상세 가이드는 [ugv_roarm_description README](src/ugv_main/ugv_roarm_description/README.md)를 참조하세요.**

### RPi 초기 세팅 요약

```bash
ssh pi@192.168.0.71

# 클론
cd ~ && git clone -b ros2-humble-develop https://github.com/fhekwn549/ugv_ws.git

# 의존성
pip3 install pyserial fastapi uvicorn paho-mqtt
sudo apt install ros-humble-rmw-fastrtps-cpp ros-humble-xacro \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher \
  rabbitmq-server
# RabbitMQ 플러그인 활성화
sudo cp ~/ugv_ws/src/ugv_main/ugv_bridge/config/enabled_plugins /etc/rabbitmq/enabled_plugins
sudo cp ~/ugv_ws/src/ugv_main/ugv_bridge/config/rabbitmq.conf /etc/rabbitmq/rabbitmq.conf
sudo systemctl restart rabbitmq-server

# 스왑 추가 (RPi RAM 1GB인 경우, rf2o C++ 빌드 OOM 방지)
sudo fallocate -l 2G /swapfile
sudo chmod 600 /swapfile && sudo mkswap /swapfile && sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab

# 빌드
cd ~/ugv_ws && source /opt/ros/humble/setup.bash
colcon build --packages-select ugv_cpp_nodes ugv_bridge ugv_roarm_description \
  ugv_description rf2o_laser_odometry ugv_interface ldlidar
source install/setup.bash
```

### WSL 초기 세팅 요약

```bash
# 클론
cd ~ && git clone -b ros2-humble-develop https://github.com/fhekwn549/ugv_ws.git

# 의존성
sudo apt install ros-humble-rmw-fastrtps-cpp ros-humble-xacro \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui ros-humble-rviz2 ros-humble-tf2-ros

# 빌드
cd ~/ugv_ws && source /opt/ros/humble/setup.bash
colcon build --packages-select ugv_cpp_nodes ugv_roarm_description ugv_description
source install/setup.bash
```

### 매번 실행

#### 방법 A: 웹 대시보드로 제어 (권장)

RPi 전원을 켜면 `ugv_bringup.service`(systemd)가 자동으로 `rasp_bringup.launch.py`를 실행합니다.
별도의 SSH 접속 없이, WSL에서 대시보드만 실행하면 됩니다.

```bash
# [WSL] 웹 대시보드
cd ~/ugv_dashboard && npm run dev
# → 브라우저에서 http://localhost:5173 접속
# → .env의 VITE_ROBOT_HOST=192.168.0.71 확인
```

RPi bringup 서비스 관리 (SSH 접속 시):
```bash
# 상태 확인
sudo systemctl status ugv_bringup.service

# 재시작 (코드 수정 후 colcon build 이후)
sudo systemctl restart ugv_bringup.service

# 실시간 로그 확인
journalctl -u ugv_bringup -f

# 자율주행 (SLAM 맵 필요 시, 별도 터미널)
source ~/ugv_ws/install/setup.bash
ros2 launch ugv_roarm_description nav_real.launch.py pbstream:=/home/pi/maps/lab_map.pbstream
```

#### 방법 B: RViz로 시각화 (FastDDS 설정 필요)

```bash
# RPi는 자동 실행 (ugv_bringup.service)

# [터미널 1: WSL] RViz (CycloneDDS로 자동 수신)
source ~/ugv_ws/install/setup.bash
ros2 launch ugv_roarm_description remote_view.launch.py

# [터미널 2: WSL] 키보드 텔레옵
source ~/ugv_ws/install/setup.bash
ros2 run ugv_roarm_description teleop_all.py --ros-args -p mode:=rviz -p model:=rasp_rover
```

> **참고**: WSL2는 기본적으로 NAT 모드라 DDS 멀티캐스트가 RPi까지 안 갑니다. 방법 B는 FastDDS unicast 설정이 필요합니다.

---

## Changes from upstream

### Added: `ugv_bridge` (MQTT + REST API 웹 브릿지)

ROS 2 토픽을 MQTT/REST API로 변환하여 웹 대시보드와 연동하는 브릿지 노드.

| 모듈 | 역할 |
|------|------|
| `bridge_node.py` | 메인 ROS 2 노드 |
| `ros_interface.py` | ROS 2 토픽 구독/발행 + TF2 리스너 |
| `mqtt_bridge.py` | MQTT 발행 (센서) + MQTT 구독 (제어: cmd_vel, arm, gripper, navigate, cancel) |
| `api_app.py` | FastAPI REST 엔드포인트 (맵 PNG, 로그 조회 — 읽기 전용) |
| `db_writer.py` | SQLite 로깅 (명령, 네비게이션, 이벤트) |
| `map_converter.py` | OccupancyGrid → PNG 변환 (순수 stdlib, Pillow 불필요) |

- **메시지 브로커**: RabbitMQ (MQTT:1883, STOMP:61613, Web STOMP:15674, Management:15672)
  - Bridge(paho-mqtt) → MQTT:1883, Dashboard(@stomp/stompjs) → Web STOMP:15674
- **통신 방식**: 모든 로봇 제어(cmd_vel, arm, gripper, navigate, cancel)는 MQTT로 통일. REST API는 맵/로그 조회 전용 (읽기 전용)
- **REST API**: FastAPI on port 8081 (맵 PNG, 로그 조회)
- **DB**: SQLite WAL 모드 (`~/ugv_bridge.db`)
- **멀티 로봇**: `robot_id` 파라미터로 MQTT 네임스페이스 분리 (default: `ugv01`)
- **Factory API 불필요**: 로봇 제어는 MQTT로 직접 통신하므로 Factory API 프록시 없이 동작

### Added: `ugv_cpp_nodes` (C++ 실시간 시리얼 드라이버)

실시간 통신 성능 개선을 위해 `ugv_driver`, `roarm_driver`, `ugv_bringup`(센서 피드백)을 Python에서 C++로 전환한 패키지입니다.
`ugv_driver_node`가 제어와 센서 피드백을 단일 노드에서 처리하여, 시리얼 포트 이중 접근으로 인한 데이터 깨짐 문제를 해결합니다.

#### 왜 C++로 전환했는가?

이 두 노드는 **시리얼 포트를 통해 ESP32 모터 컨트롤러와 직접 통신**하는 하드웨어 드라이버입니다.
`/cmd_vel`이 들어오면 즉시 바퀴가 반응해야 하고, 로봇팔 피드백도 주기적으로 읽어야 합니다.
Python의 GIL(Global Interpreter Lock)과 가비지 컬렉션이 이 실시간성을 방해할 수 있어 C++로 전환했습니다.

| 항목 | Python (기존) | C++ (현재) | 개선 |
|------|--------------|-----------|------|
| 시리얼 I/O | pyserial + GIL 경합 | POSIX termios 직접 제어 | GIL 없는 진정한 멀티스레딩 |
| JSON 직렬화 | `json.dumps()` 매 콜백 | `snprintf` 사전 포맷팅 | 힙 할당 최소화 |
| 스레드 안전 | `threading.Lock` + GIL | `std::mutex` | 예측 가능한 타이밍 |
| GC 영향 | 있음 (간헐적 지연) | 없음 | 일정한 반응 시간 |

반면, `ugv_bridge`(MQTT/REST), `fake_odom_node`(시뮬레이션), `teleop_all`(키보드 입력) 등은
네트워크 I/O 바운드이거나 사람 입력 속도에 맞추면 되므로 Python으로 충분합니다.

#### 아키텍처 (3계층 분리)

```
┌─────────────────────────────────────────────────────────┐
│ ROS2 노드 계층 (roarm_driver_node, ugv_driver_node)     │
│  - 토픽 구독/발행, 파라미터, 타이머                      │
│  - rclcpp에 의존                                        │
└──────────────────────┬──────────────────────────────────┘
                       │ 사용
┌──────────────────────▼──────────────────────────────────┐
│ 프로토콜 드라이버 계층 (roarm_serial_driver, ugv_serial_driver) │
│  - ESP32 JSON 프로토콜 (T:102, T:105, T:106, T:13 등)  │
│  - 에코 처리, JSON 파싱, 뮤텍스                         │
│  - ROS2 의존성 없음 (순수 C++ 정적 라이브러리)          │
└──────────────────────┬──────────────────────────────────┘
                       │ 사용
┌──────────────────────▼──────────────────────────────────┐
│ 시리얼 포트 계층 (serial_driver)                         │
│  - POSIX termios, select(), 타임아웃                     │
│  - ROS2 의존성 없음 (순수 C++ 정적 라이브러리)          │
└─────────────────────────────────────────────────────────┘
```

#### 패키지 파일 구조

```
src/ugv_main/ugv_cpp_nodes/
├── CMakeLists.txt                              # 빌드 설정 (정적 라이브러리 + 실행 파일)
├── package.xml                                 # ROS2 패키지 매니페스트
├── include/ugv_cpp_nodes/
│   ├── serial_driver.hpp                       # POSIX 시리얼 포트 드라이버
│   ├── roarm_serial_driver.hpp                 # RoArm-M2 ESP32 프로토콜
│   └── ugv_serial_driver.hpp                   # UGV 바퀴 ESP32 프로토콜
└── src/
    ├── serial_driver.cpp                       # termios 시리얼 통신 구현
    ├── roarm_serial_driver.cpp                 # T:102/105/106/210 명령/응답
    ├── ugv_serial_driver.cpp                   # T:13/132/134 명령 + T:1001 피드백 읽기
    ├── roarm_driver_node.cpp                   # ROS2 노드: /joint_states 발행
    └── ugv_driver_node.cpp                     # ROS2 노드: 제어(/cmd_vel) + 센서(/imu, /voltage)
```

#### roarm_driver_node (RoArm-M2 로봇팔)

- **Subscribe**: `/arm_controller/joint_trajectory` (JointTrajectory) → T:102 관절 이동
- **Subscribe**: `/roarm/gripper_cmd` (Float64) → T:106 그리퍼 제어
- **Publish**: `/joint_states` (JointState) ← T:105 주기적 피드백 (5Hz)
- **Parameters**: `serial_port` (`/dev/ttyUSB0`), `baud_rate` (115200), `feedback_rate` (5.0)
- 팔 관절 이동 시 그리퍼 토크 유지 (T:102에 항상 `hand` 값 포함)
- 바퀴 관절 (인코더 없음)은 0.0으로 발행 → TF 트리 완성

#### ugv_driver_node (UGV 통합: 제어 + 센서 피드백)

- **Subscribe**: `cmd_vel` (Twist) → T:13 모터 속도
- **Subscribe**: `ugv/joint_states` (JointState) → T:134 팬틸트 (rad→deg 변환)
- **Subscribe**: `ugv/led_ctrl` (Float32MultiArray) → T:132 LED
- **Publish**: `imu/data` (Imu) ← T:1001 피드백 (r/p/y → 쿼터니언, 20Hz)
- **Publish**: `odom/odom_raw` (Float32MultiArray) ← T:1001 인코더 (L/R)
- **Publish**: `voltage` (Float32) ← T:1001 배터리 전압 + 저전압 경고 로그
- **Parameters**: `serial_port` (자동 감지), `baud_rate` (115200), `angular_scale` (2.5)
- 시작 시 T:131로 ESP32 연속 피드백 활성화, 전용 스레드에서 읽기
- 노드 종료 시 자동으로 바퀴 정지 명령 전송

#### Python → C++ 롤백 방법

launch 파일에서 `ugv_driver_node`를 `ugv_bringup` + `ugv_driver`로 되돌리면 됩니다:

```python
# C++ (현재) — 제어 + 센서를 단일 노드가 처리
Node(package='ugv_cpp_nodes', executable='ugv_driver_node', name='ugv_driver')
Node(package='ugv_cpp_nodes', executable='roarm_driver_node', name='roarm_driver')

# Python (롤백) — 센서와 제어가 별도 노드 (시리얼 포트 이중 접근 주의)
Node(package='ugv_bringup', executable='ugv_bringup', name='ugv_bringup')  # 센서 피드백
Node(package='ugv_bringup', executable='ugv_driver', name='ugv_driver')    # 제어 명령
Node(package='ugv_bringup', executable='roarm_driver', name='roarm_driver')
```

### (Legacy) `roarm_driver` Python 버전 (in `ugv_bringup`)

> C++ 버전(`ugv_cpp_nodes`)으로 대체되었습니다. Python 코드는 `ugv_bringup` 패키지에 그대로 남아 있어 롤백 가능합니다.

RoArm-M2 로봇팔을 시리얼(`/dev/ttyUSB0`)로 제어하는 ROS 2 드라이버 노드 (Python 버전).

- **Subscribe**: `/arm_controller/joint_trajectory` (JointTrajectory) → T:102 시리얼 명령 (팔 3관절만)
- **Subscribe**: `/roarm/gripper_cmd` (Float64) → T:106 그리퍼 명령 (독립 제어)
- **Publish**: `/joint_states` (JointState) ← T:105 주기적 조회 (5Hz)
- **Parameters**: `serial_port` (default: `/dev/ttyUSB0`), `baud_rate` (115200), `feedback_rate` (5.0)
- 팔 관절 이동 시 그리퍼 토크 유지 (T:102에 항상 `hand` 값 포함)
- 시리얼 에러 발생 시 노드 크래시 방지 (try/catch)

### Removed: `rosbridge_relay` (in `ugv_bringup`)

CycloneDDS 직접 통신으로 전환하여 rosbridge WebSocket 브릿지는 더 이상 사용하지 않습니다.

### Odometry: `base_node` → `rf2o_laser_odometry`

Wave Rover에 인코더가 없어 cmd_vel dead reckoning 방식의 오도메트리가 부정확했습니다.
rf2o_laser_odometry (LiDAR 스캔 매칭 기반)로 교체하여 오도메트리 품질을 개선했습니다.

### SLAM: `slam_toolbox` → `Cartographer`

Cartographer SLAM으로 전환하여 매핑 품질을 개선했습니다. Cartographer의 pure localization 모드를 활용한 Nav2 자율주행을 지원합니다.

## 배포 (코드 수정 후)

```bash
# WSL: push
cd ~/ugv_ws
git add -A && git commit -m "설명" && git push origin ros2-humble-develop

# RPi: pull & build (SSH)
cd ~/ugv_ws && git pull origin ros2-humble-develop
colcon build --packages-select ugv_cpp_nodes ugv_bridge rf2o_laser_odometry ugv_roarm_description
source install/setup.bash
```

---

# Original README (ugv_ws Workspace Description)

1.Environment

- pc software：VMware Workstation 17Pro、mobarxterm
- ugv Version：UGV ROVER、UGV BEAST

2.Architecture

- project：https://github.com/DUDULRX/ugv_ws/tree/ros2-humble
    
    ```jsx
    git clone -b ros2-humble-develop https://github.com/DUDULRX/ugv_ws.git
    ```
    
    - First compilation on the virtual machine (compiling one by one on the pi or jetson)
        
        ```jsx
        cd /home/ws/ugv_ws
        . build_first.sh
        ```
        
        build_first.sh content
        
        ```jsx
        cd /home/ws/ugv_ws
        colcon build --packages-select apriltag apriltag_msgs apriltag_ros cartographer costmap_converter_msgs costmap_converter emcl2 explore_lite openslam_gmapping slam_gmapping ldlidar rf2o_laser_odometry robot_pose_publisher teb_msgs teb_local_planner vizanti vizanti_cpp vizanti_demos vizanti_msgs vizanti_server ugv_base_node ugv_interface
        colcon build --packages-select ugv_bringup ugv_chat_ai ugv_description ugv_gazebo ugv_nav ugv_slam ugv_tools ugv_vision ugv_web_app --symlink-install 
        echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
        echo "eval "$(register-python-argcomplete ros2)"" >> ~/.bashrc
        echo "eval "$(register-python-argcomplete colcon)"" >> ~/.bashrc
        echo "source /home/ws/ugv_ws/install/setup.bash" >> ~/.bashrc
        source ~/.bashrc 
        ```
        
    - Daily compilation of virtual machines (one by one on the car)
        
        ```jsx
        cd /home/ws/ugv_ws
        . build_common.sh
        ```
        
        build_common.sh content
        
        ```jsx
        cd /home/ws/ugv_ws
        colcon build --packages-select apriltag apriltag_msgs apriltag_ros cartographer costmap_converter_msgs costmap_converter emcl2 explore_lite openslam_gmapping slam_gmapping ldlidar rf2o_laser_odometry robot_pose_publisher teb_msgs teb_local_planner vizanti vizanti_cpp vizanti_demos vizanti_msgs vizanti_server ugv_base_node ugv_interface
        colcon build --packages-select ugv_bringup ugv_chat_ai ugv_description ugv_gazebo ugv_nav ugv_slam ugv_tools ugv_vision ugv_web_app --symlink-install 
        source install/setup.bash 
        ```
        
    - Compile apriltag
        
        ```jsx
        cd /home/ws/ugv_ws
        . build_apriltag.sh
        ```
        
        build_apriltag.sh content
        
        ```jsx
        cd /home/ws/ugv_ws/src/ugv_else/apriltag_ros/apriltag
        cmake -B build -DCMAKE_BUILD_TYPE=Release
        cmake --build build --target install
        cd /home/ws/ugv_ws
        ```
        
- Ubuntu software：
    
    **Install according to wiki install ros2 humble**
    
    ```jsx
    apt-get update 
    apt-get upgrade 
    
    apt install python3-pip
    apt-get install alsa-utils
    apt install python3-colcon-argcomplete
    
    apt install ros-humble-cartographer-*
    apt install ros-humble-desktop-*
    apt install ros-humble-joint-state-publisher-*
    apt install ros-humble-nav2-*
    apt install ros-humble-rosbridge-*
    apt install ros-humble-rqt-*
    apt install ros-humble-rtabmap-*
    apt install ros-humble-usb-cam
    apt install ros-humble-depthai-*
    
    #Simulation virtual machine installation
    apt install gazebo
    apt install ros-humble-gazebo-*
    ```
    
- Python3 Library：
    
    domestic
    
    ```jsx
    cd ~/ugv_ws
    python3 -m pip install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple
    ```
    
    foreign
    
    ```jsx
    cd ~/ugv_ws
    python3 -m pip install -r requirements.txt
    ```
    
    requirements.txt content
    
    ```jsx
    pyserial
    flask
    mediapipe
    requests
    ```
    
- Feature pack ugv_ws 
    
    > ugv_main Main functions
    > 
    > 
    > > ugv_base_node Two-wheel differential kinematics
    > > 
    > 
    > > ugv_bringup drive, control
    > > 
    > 
    > > ugv_chat_ai web ai interaction
    > > 
    > 
    > > ugv_description Model
    > > 
    > 
    > > ugv_gazebo simulation
    > > 
    > 
    > > ugv_interface Information interface
    > > 
    > 
    > > ugv_nav navigation
    > > 
    > 
    > > ugv_slam Mapping
    > > 
    > 
    > > ugv_tools tool
    > > 
    > 
    > > ugv_vision visual interaction
    > > 
    > 
    > > ugv_web_app web
    > > 
    
    > ugv_else ( ugv_main dependence)
    > 
    > 
    > > apriltag_ros
    > > 
    > 
    > > cartographer
    > > 
    > 
    > > costmap_converter
    > > 
    > 
    > > emcl_ros2
    > > 
    > 
    > > explore_lite
    > > 
    > 
    > > gmapping
    > > 
    > 
    > > ldlidar
    > > 
    > 
    > > rf2o_laser_odometry
    > > 
    > 
    > > robot_pose_publisher
    > > 
    > 
    > > teb_local_planner
    > > 
    > 
    > > vizanti
    > > 

3.Use (ros packages on the car are all executed in docker)

use_rviz optional true, false (default)

car model optional rasp_rover, ugv_rover, ugv_beast

lidar model optional ld06, ld19 (default), stl27l

- Start the car and turn off the auto-start script.
    
    ```jsx
    sudo killall -9 python
    ```
    

Enter docker and start ssh to remotely access docker and the visual interface

- Car settings docker
    - Set up docker remote login
        
        Execute on the host and enter the directory
        
        ```jsx
        cd /home/ws/ugv_ws
        sudo chmod +x ros2_humble.sh remotessh.sh
        ./ros2_humble.sh
        ```
        
        1进入docker
        
        ![image.png](images/Enter%20docker.png)
        
    - Exit docker
        
        Execute within docker
        
        ```jsx
        exit
        ```
        
- Remote to docker
    
    ![image.png](images/Connect%20to%20docker%20remotely.png)
    
    ![image.png](images/Docker%20username.png)
    
    ```jsx
    #username
    root
    #Password needs to be set in advance
    ws
    ```
    
    Enter workspace
    
    ```jsx
    cd /home/ws/ugv_ws
    ```
    
- View model joints
    - rasp_rover
        
        ```jsx
        export UGV_MODEL=rasp_rover
        ```
        
        start up
        
        ```jsx
         ros2 launch ugv_description display.launch.py use_rviz:=true
        ```
        
        ![image.png](images/Rasp_rover.png)
        
    - ugv_rover
        
        ```jsx
        export UGV_MODEL=ugv_rover
        ```
        
        start up
        
        ```jsx
         ros2 launch ugv_description display.launch.py use_rviz:=true
        ```
        
        ![image.png](images/Ugv_rover.png)
        
    - ugv_beast
        
        ```jsx
        export UGV_MODEL=ugv_beast
        ```
        
        start up
        
        ```jsx
         ros2 launch ugv_description display.launch.py use_rviz:=true
        ```
        
        ![image.png](images/Ugv_beast.png)
        
    - Drive the car (can control the pan/tilt and LED lights)
        
        ```jsx
         ros2 run ugv_bringup ugv_driver
        ```
        
        Drag the slider related to the joint angle publisher to control the gimbal
        
        [![](https://res.cloudinary.com/marcomontalbano/image/upload/v1727491041/video_to_markdown/images/youtube--jA9LJTBRQqY-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://youtu.be/jA9LJTBRQqY "")
        
        Control the light data 0-255 data[0] control the light IO4 near the oak camera data[1] control the light IO5 near the usb camera
        
        ```jsx
        ros2 topic pub /ugv/led_ctrl std_msgs/msg/Float32MultiArray "{data: [0, 0]}" -1
        ```
        
- Chassis driver (executed within docker)
    
    If you switch to another radar, modify
    
    ```jsx
    export LDLIDAR_MODEL=
    ```
    - Use radar as imu sensor data (more stable)
        
        ```jsx
        ros2 launch ugv_bringup bringup_lidar.launch.py use_rviz:=true
        ```
        
    
    Rotate the car in place to check the posture
    
    [![](https://res.cloudinary.com/marcomontalbano/image/upload/v1727491431/video_to_markdown/images/youtube--5neLr1Q2ddM-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://youtu.be/5neLr1Q2ddM "")
    
- Joystick, keyboard control
    
    Start the car
    
    ```jsx
    ros2 launch ugv_bringup bringup_lidar.launch.py use_rviz:=true
    ```
    
    - Joystick control (the joystick USB interface needs to be connected to the car or virtual machine)
        
        ```jsx
        ros2 launch ugv_tools teleop_twist_joy.launch.py
        ```
        
    - keyboard control
        
        ```jsx
        ros2 run ugv_tools keyboard_ctrl
        ```
        
        ![image.png](images/Keyboard%20controls.png)
        
- Visual interaction
    
    Start the car
    
    ```jsx
    ros2 launch ugv_bringup bringup_lidar.launch.py use_rviz:=true
    ```
    
    - Start related interfaces
        
        control car
        
        ```jsx
        ros2 run ugv_tools behavior_ctrl
        ```
        
        Turn on the camera, easy
        
        ```jsx
        ros2 run usb_cam usb_cam_node_exe
        ```
        
        Turn on the camera and remove distortion
        
        ```jsx
        ros2 launch ugv_vision camera.launch.py
        ```
        
    - Monocular
            
        - Apriltag control
            
            apriltag only sets tag36h11, which can be modified by yourself
            
            - Apriltag control
                
                1 2 3 4 Right, left, front and rear, other stops
                
                ```jsx
                ros2 run ugv_vision apriltag_ctrl
                ```
                
            - Apriltag Simple tracking
                
                Select the left and right according to the x coordinate of the center point of the ar tag. After centering, select the front and rear according to the y coordinate. If the y is upward, the front is forward, and if the y is downward, the rear is
                
                ```jsx
                ros2 run ugv_vision apriltag_track_0
                ```
                
            - Apriltag Target tracking (AR code needs to specify size 0.08)
                
                pose recognition
                
                Here, the previous command to turn on the camera is turned off and replaced with the following
                
                ```jsx
                ros2 launch ugv_vision apriltag_track.launch.py
                ```
                
                ![image.png](images/APIELTAG%20object%20tracking.jpg)
                
                - Simply drive to the target point (rotate, go straight)
                    
                    Turn on tracking
                    
                    ```jsx
                    ros2 run ugv_vision apriltag_track_1
                    ```
                    
                    command line
                    
                    ```jsx
                    ros2 topic pub /apriltag/track std_msgs/msg/Int8 -1
                    ```
                    
                - Combine nav2 to drive to the target point (you need to close the previous startup file and change to open nav)
                    
                    Turn on navigation
                    
                    ```jsx
                    ros2 launch ugv_nav nav.launch.py use_rviz:=true
                    ```
                    
                    Turn on tracking
                    
                    ```jsx
                    ros2 run ugv_vision apriltag_track_2
                    ```
                    
- Mapping
    - 2D (LiDAR)
        - Gmapping
            
            ```jsx
             ros2 launch ugv_slam gmapping.launch.py use_rviz:=true
            ```
            
            [![](https://res.cloudinary.com/marcomontalbano/image/upload/v1727493329/video_to_markdown/images/youtube--cBiuYmxGWks-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://youtu.be/cBiuYmxGWks "")
            
            control car
            
            ```jsx
            ros2 run ugv_tools keyboard_ctrl
            ```
            
            save map
            
            ```jsx
            ./save_2d_gmapping_map.sh
            ```
            
            ![image.png](images/Save_2d_gmapping_map.sh.png)
            
            save_2d_gmapping_map.sh内容
            
            ```jsx
            cd /home/ws/ugv_ws/src/ugv_main/ugv_nav/maps
            ros2 run nav2_map_server map_saver_cli -f ./map
            ```
            
        - Cartographer
            
            ```jsx
            ros2 launch ugv_slam cartographer.launch.py use_rviz:=true
            ```
            
            [![](https://res.cloudinary.com/marcomontalbano/image/upload/v1727491911/video_to_markdown/images/youtube--dHyNeuJ0k3U-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://youtu.be/dHyNeuJ0k3U "")
            
            control car
            
            ```jsx
            ros2 run ugv_tools keyboard_ctrl
            ```
            
            save map
            
            ```jsx
            ./save_2d_cartographer_map.sh
            ```
            
            ![image.png](images/Save_2d_cartographer_map.sh.png)
            
            save_2d_cartographer_map.sh内容
            
            ```jsx
            cd /home/ws/ugv_ws/src/ugv_main/ugv_nav/maps
            ros2 run nav2_map_server map_saver_cli -f ./map && ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/home/ws/ugv_ws/src/ugv_main/ugv_nav/maps/map.pbstream'}"
            ```
            
    - 3D (lidar + depth camera)
        - Rtabmap
            - Rtabmap_viz Visualization
                
                ```jsx
                ros2 launch ugv_slam rtabmap_rgbd.launch.py use_rviz:=false
                ```
                
                [![](https://res.cloudinary.com/marcomontalbano/image/upload/v1727492108/video_to_markdown/images/youtube--J3_QCGVF7Jc-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://youtu.be/J3_QCGVF7Jc "")
                
                control car
                
                ```jsx
                ros2 run ugv_tools keyboard_ctrl
                ```
                
            - Rviz Visualization
                
                ```jsx
                ros2 launch ugv_slam rtabmap_rgbd.launch.py use_rviz:=true
                ```
                
                [![](https://res.cloudinary.com/marcomontalbano/image/upload/v1727492190/video_to_markdown/images/youtube--dxey_90tdFI-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://youtu.be/dxey_90tdFI "")
                
                control car
                
                ```jsx
                ros2 run ugv_tools keyboard_ctrl
                ```
                
            
            After the mapping is completed, directly press ctrl+c to exit the mapping node, and the system will automatically save the map. Map default save path ~/.ros/rtabmap.db 
            
- Navigation
    - 2D
        - Local localization
            
            use_localization amcl（default），emcl，cartographer
            
            - amcl
                
                Start first, you need to manually specify the approximate initial position
                
                ```jsx
                ros2 launch ugv_nav nav.launch.py use_localization:=amcl use_rviz:=true
                ```
                
                Then by controlling the car, simply move and rotate to assist in initial positioning.
                
                ```jsx
                ros2 run ugv_tools keyboard_ctrl
                ```
                
            - emcl
                
                After startup, you need to manually specify the approximate initial position
                
                ```jsx
                ros2 launch ugv_nav nav.launch.py use_localization:=emcl use_rviz:=true
                ```
                
            - cartographer
                
                Note that you need to use Cartographer to build the map before you can proceed.
                
                ```jsx
                ros2 launch ugv_nav nav.launch.py use_localization:=cartographer use_rviz:=true
                ```
                
                ![image.png](images/Cartographer%20pure_localization.png)
                
                After startup, if the accurate position has not been located, you can control the car and simply move it to assist in the initial positioning.
                
                ```jsx
                ros2 run ugv_tools keyboard_ctrl
                ```
                
        - Local navigation
            
            use_localplan dwa，teb（default）
            
            - dwa
                
                ```jsx
                 ros2 launch ugv_nav nav.launch.py use_localplan:=dwa use_rviz:=true
                ```
                
            - teb
                
                ```jsx
                 ros2 launch ugv_nav nav.launch.py use_localplan:=teb use_rviz:=true
                ```
                
    - 3D
        - Rtabmap
            - Local navigation
                
                Turn on positioning
                
                ```jsx
                ros2 launch ugv_nav rtabmap_localization_launch.py
                ```
                
                Turn on nav (you can wait slowly until the 3D data is loaded before navigating, it will take a while)
                
                use_localplan dwa，teb（default）
                
                - dwa
                    
                    ```jsx
                     ros2 launch ugv_nav nav_rtabmap.launch.py use_localplan:=dwa use_rviz:=true
                    ```
                    
                - teb
                    
                    ```jsx
                     ros2 launch ugv_nav nav_rtabmap.launch.py use_localplan:=teb use_rviz:=true
                    ```
                    
- Mapping and navigation are enabled at the same time (two-dimensional)
    
    ```jsx
    ros2 launch ugv_nav slam_nav.launch.py use_rviz:=true
    ```
    
    - Rviz manually publishes navigation points for exploration (you can also use the keyboard, handle, and web side for remote exploration)
        
        ![image.png](images/Rviz%20manually%20publishes%20navigation%20points%20for%20exploration.png)
        
    - Automatic exploration (to be in a closed rule area)
        
        ```jsx
         ros2 launch explore_lite explore.launch.py 
        ```

    - Save map
            
        ```jsx
        ./save_2d_gmapping_map.sh
        ```
        
- Web ai interaction
    - Start the car
        
        ```jsx
        ros2 launch ugv_bringup bringup_lidar.launch.py use_rviz:=true
        ```
        
    - Start related interfaces
        
        ```jsx
        ros2 run ugv_tools behavior_ctrl
        ```
        
    - Web ai Interaction (requires relevant ai interface, currently ollama local deployment)
        
        ```jsx
        ros2 run ugv_chat_ai app
        ```
        
- Web side control
    
    Drive the car first, refer to the above chassis drive, map construction and navigation, and start mapping and navigation at the same time.
    
    - ugv web
        
        ```jsx
        ros2 launch ugv_web_app bringup.launch.py host:=ip
        ```
        
- Command interaction
    
    ```jsx
    ros2 run ugv_tools behavior_ctrl
    ```
    
    - Basic control (you need to put the car down and run, and judge whether the goal has been completed based on the odometer)
        
        ```jsx
        ros2 launch ugv_bringup bringup_lidar.launch.py use_rviz:=true
        ```
        
        Forward data unit meters
        
        ```jsx
        ros2 action send_goal /behavior ugv_interface/action/Behavior "{command: '[{\"T\": 1, \"type\": \"drive_on_heading\", \"data\": 0.1}]'}"
        ```
        
        Back data unit meters
        
        ```jsx
        ros2 action send_goal /behavior ugv_interface/action/Behavior "{command: '[{\"T\": 1, \"type\": \"back_up\", \"data\": 0.1}]'}"
        ```
        
        Rotation data unit degree ,positive number left rotation, negative number right rotation
        
        ```jsx
        ros2 action send_goal /behavior ugv_interface/action/Behavior "{command: '[{\"T\": 1, \"type\": \"spin\", \"data\": -1}]'}"
        ```
        
        Stop
        
        ```jsx
        ros2 action send_goal /behavior ugv_interface/action/Behavior "{command: '[{\"T\": 1, \"type\": \"stop\", \"data\": 0}]'}"
        ```
        
    
    Navigation needs to be enabled below
    
    ```jsx
    ros2 launch ugv_nav nav.launch.py use_rviz:=true
    ```
    
    - Get current point position
        
        ```elm
        ros2 topic echo /robot_pose --once
        ```
        
    - Save as navigation point
        
        data Navigation point name, optional a-g
        
        ```jsx
        ros2 action send_goal /behavior ugv_interface/action/Behavior "{command: '[{\"T\": 1, \"type\": \"save_map_point\", \"data\": \"a\"}]'}"
        ```
        
    - Move to navigation point
        
        data Navigation point name, optional a-g
        
        ```jsx
        ros2 action send_goal /behavior ugv_interface/action/Behavior "{command: '[{\"T\": 1, \"type\": \"pub_nav_point\", \"data\": \"a\"}]'}"
        ```
        
    
    The saved points will also be stored in the file.
    
    ![image.png](images/The%20saved%20points%20will%20also%20be%20stored%20in%20the%20file.png)
    
- Gazebo comprehensive simulation (executed on virtual machine)
    
    ```jsx
    cd ~/.gazebo/
    git clone https://github.com/osrf/gazebo_models.git models
    cp -r /home/ws/ugv_ws/src/ugv_main/ugv_gazebo/models/world models
    cp /home/ws/ugv_ws/ugv_description.zip models 
    cd ~/.gazebo/models/
    unzip ugv_description.zip
    rm -rf ugv_description.zip
    ```
    
    - View model
        - rasp_rover
            
            ```jsx
            export UGV_MODEL=rasp_rover
            ```
            
            start up
            
            ```jsx
             ros2 launch ugv_gazebo display.launch.py 
            ```
            
        - ugv_rover
            
            ```jsx
            export UGV_MODEL=ugv_rover
            ```
            
            start up
            
            ```jsx
             ros2 launch ugv_gazebo display.launch.py 
            ```
            
        - ugv_beast
            
            ```jsx
            export UGV_MODEL=ugv_beast
            ```
            
            start up
            
            ```jsx
             ros2 launch ugv_gazebo display.launch.py 
            ```
            
    - Load model
        - Empty
            
            ```elm
            ros2 launch ugv_gazebo bringup_test.launch.py
            ```
            
            ![image.png](images/Gazebo%20empty%20world.png)
            
        - House
            
            ```jsx
            ros2 launch ugv_gazebo bringup.launch.py
            ```
            
            ![image.png](images/Gazebo%20house%20world.png)
            
    
    The following takes ugv_rover as an example
    
    Specify model
    
    ```jsx
    export UGV_MODEL=ugv_rover
    ```
    
    start up
    
    ```jsx
    ros2 launch ugv_gazebo bringup.launch.py
    ```
    
    - Joystick, keyboard control
        - Joystick control (the joystick USB interface needs to be connected to the virtual machine)
            
            ```jsx
            ros2 launch ugv_tools teleop_twist_joy.launch.py
            ```
            
        - Keyboard control
            
            ```jsx
            ros2 run ugv_tools keyboard_ctrl
            ```
            
            ![image.png](images/Keyboard%20controls.png)
            
    - Mapping
        - 2D (LiDAR)
            
            ![image.png](images/Gazebo%202D%20mapping.png)
            
            - Gmapping
                
                ```elm
                ros2 launch ugv_gazebo gmapping.launch.py
                ```
                
                control car
                
                ```jsx
                ros2 run ugv_tools keyboard_ctrl
                ```
                
                save map
                
                ```jsx
                ./save_2d_gmapping_map_gazebo.sh
                ```
                
                save_2d_gmapping_map_gazebo.sh content
                
                ```jsx
                cd /home/ws/ugv_ws/src/ugv_main/ugv_gazebo/maps
                ros2 run nav2_map_server map_saver_cli -f ./map
                ```
                
            - Cartographer
                
                ```elm
                ros2 launch ugv_gazebo cartographer.launch.py
                ```
                
                control car
                
                ```jsx
                ros2 run ugv_tools keyboard_ctrl
                ```
                
                save map
                
                ```jsx
                ./save_2d_cartographer_map_gazebo.sh
                ```
                
                save_2d_cartographer_map_gazebo.sh content
                
                ```jsx
                cd /home/ws/ugv_ws/src/ugv_main/ugv_gazebo/maps
                ros2 run nav2_map_server map_saver_cli -f ./map && ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/home/ws/ugv_ws/src/ugv_main/ugv_gazebo/maps/map.pbstream'}"
                ```
                
        - 3D (lidar + depth camera)
            - Rtabmap
                - Rtabmap_viz Visualization
                    
                    ```jsx
                    ros2 launch ugv_gazebo rtabmap_rgbd.launch.py
                    ```
                    
                    ![image.png](images/Gazebo%20rtabmap_viz%20visualization.png)
                    
                    control car
                    
                    ```jsx
                    ros2 run ugv_tools keyboard_ctrl
                    ```
                    
                - Rviz Visualization
                    
                    ```jsx
                    ros2 launch ugv_gazebo rtabmap_rgbd.launch.py use_rviz:=true
                    ```
                    
                    control car
                    
                    ```jsx
                    ros2 run ugv_tools keyboard_ctrl
                    ```
                    
                
                After the mapping is completed, directly press ctrl+c to exit the mapping node, and the system will automatically save the map. Map default save path ~/.ros/rtabmap.db 
                
    - Navigation
        - 2D
            - Local positioning
                
                use_localization amcl（default），emcl，cartographer
                
                - amcl
                    
                    Start first, you need to manually specify the approximate initial position
                    
                    ```jsx
                    ros2 launch ugv_gazebo nav.launch.py use_localization:=amcl 
                    ```
                    
                    Then by controlling the car, simply move and rotate to assist in initial positioning.
                    
                    ```jsx
                    ros2 run ugv_tools keyboard_ctrl
                    ```
                    
                - emcl
                    
                    After startup, you need to manually specify the approximate initial position
                    
                    ```jsx
                    ros2 launch ugv_gazebo nav.launch.py use_localization:=emcl 
                    ```
                    
                - cartographer
                    
                    Note that you need to use Cartographer to build the map before you can proceed.
                    
                    ```jsx
                    ros2 launch ugv_gazebo nav.launch.py use_localization:=cartographer 
                    ```
                    
                    After startup, if the accurate position has not been located, you can control the car and simply move it to assist in the initial positioning.
                    
                    ```jsx
                    ros2 run ugv_tools keyboard_ctrl
                    ```
                    
            - Local navigation
                
                use_localplan dwa，teb（默认）
                
                - dwa
                    
                    ```jsx
                     ros2 launch ugv_gazebo nav.launch.py use_localplan:=dwa 
                    ```
                    
                - teb
                    
                    ```jsx
                     ros2 launch ugv_gazebo nav.launch.py use_localplan:=teb 
                    ```
                    
        - 3D
            - Rtabmap
                - Local navigation
                    
                    Turn on positioning
                    
                    ```jsx
                    ros2 launch ugv_gazebo rtabmap_localization_launch.py
                    ```
                    
                    Turn on nav (you can wait slowly until the 3D data is loaded before navigating, it will take a while)
                    
                    ![image.png](images/Gazebo%20rtabmap%203D%20navigation.png)
                    
                    use_localplan dwa，teb（默认）
                    
                    - dwa
                        
                        ```jsx
                         ros2 launch ugv_gazebo nav_rtabmap.launch.py use_localplan:=dwa 
                        ```
                        
                    - teb
                        
                        ```jsx
                         ros2 launch ugv_ngazebo nav_rtabmap.launch.py use_localplan:=teb
                        ```
                        
    - Mapping and navigation are enabled at the same time (two-dimensional)
        
        ```jsx
        ros2 launch ugv_gazebo slam_nav.launch.py
        ```
        
        - Automatic exploration (to be in a closed rule area)
            
            ```jsx
             ros2 launch explore_lite explore.launch.py 
            ```
            
    - Web ai interaction
        - Start related interfaces
            
            ```jsx
            ros2 run ugv_tools behavior_ctrl
            ```
            
        - web ai Interaction (requires relevant ai interface, currently ollama local deployment)
            
            ```jsx
            ros2 run ugv_chat_ai app
            ```
            
    - Web control
        - ugv web
            
            ```jsx
            ros2 launch ugv_web_app bringup.launch.py host:=ip
            ```
            
    - Command interaction
        
        ```jsx
        ros2 run ugv_tools behavior_ctrl
        ```
        
        - Basic control (you need to put the car down and run, and judge whether the goal has been completed based on the odometer)
            
            Forward data unit meters
            
            ```jsx
            ros2 action send_goal /behavior ugv_interface/action/Behavior "{command: '[{\"T\": 1, \"type\": \"drive_on_heading\", \"data\": 0.5}]'}”
            ```
            
            Back data unit meters
            
            ```jsx
            ros2 action send_goal /behavior ugv_interface/action/Behavior "{command: '[{\"T\": 1, \"type\": \"back_up\", \"data\": 0.5}]'}”
            ```
            
            Rotation data unit degree positive number rotate right, negative number rotate left
            
            ```jsx
            ros2 action send_goal /behavior ugv_interface/action/Behavior "{command: '[{\"T\": 1, \"type\": \"spin\", \"data\": -1}]'}”
            ```
            
            Stop
            
            ```jsx
            ros2 action send_goal /behavior ugv_interface/action/Behavior "{command: '[{\"T\": 1, \"type\": \"spin\", \"data\": 0}]'}”
            ```
            
        
        Navigation needs to be enabled below
        
        ```jsx
        ros2 launch ugv_gazebo nav.launch.py use_rviz:=true
        ```
        
        - Get current point position
            
            ```elm
            ros2 topic echo /robot_pose --once
            ```
            
        - Save as navigation point
            
            data navigation point name, optional a-g
            
            ```jsx
            ros2 action send_goal /behavior ugv_interface/action/Behavior "{command: '[{\"T\": 1, \"type\": \"save_map_point\", \"data\": \"a\"}]'}"
            ```
            
        - Move to navigation point
            
            data navigation point name, optional a-g
            
            ```jsx
            ros2 action send_goal /behavior ugv_interface/action/Behavior "{command: '[{\"T\": 1, \"type\": \"pub_nav_point\", \"data\": \"a\"}]'}"
            ```
            
        
        The saved points will also be stored in the file.
        
        ![image.png](images/The%20saved%20points%20will%20also%20be%20stored%20in%20the%20file.png)