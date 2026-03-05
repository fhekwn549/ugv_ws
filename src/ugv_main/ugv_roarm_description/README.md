# ugv_roarm_description

ROS 2 Humble package for the **UGV Rover + RoArm-M2** mobile manipulator.

Waveshare UGV Rover (4WD skid-steer) 위에 RoArm-M2 (4-DOF + gripper) 로봇팔을 통합한 URDF 모델, 시뮬레이션, 키보드 텔레옵 패키지입니다.

## 리포 구조

| 리포 | 역할 | 내용 |
|------|------|------|
| **이 리포 (`ugv_roarm_description`)** | 로봇 정의 + 실행 구성 | URDF, launch, Gazebo 시뮬레이션, 텔레옵, Nav2 |
| [fhekwn549/ugv_ws](https://github.com/fhekwn549/ugv_ws) | 하드웨어 구동 + 웹 브릿지 | 시리얼 드라이버, 센서 처리, MQTT/REST 브릿지 (`ugv_bridge`) |
| [fhekwn549/ugv_dashboard](https://github.com/fhekwn549/ugv_dashboard) | 웹 대시보드 프론트엔드 | Vue 3 + Vuetify + STOMP, 맵/LiDAR 시각화, 원격 제어, 공장 관리 |

RPi에서는 `ugv_roarm_description` + `ugv_ws` 두 리포가 필요합니다. 이 리포의 `rasp_bringup.launch.py`가 `ugv_ws`의 드라이버 노드들과 `ugv_bridge`를 실행합니다. 웹 대시보드(`ugv_dashboard`)는 별도로 빌드하여 `ugv_bridge`의 FastAPI가 정적 파일을 서빙합니다.

---

## 통신 아키텍처 (CycloneDDS)

WSL2와 RPi는 **CycloneDDS**를 통해 직접 DDS 통신합니다. rosbridge/WebSocket 브릿지 없이 네이티브 ROS 2 토픽이 양방향으로 전달됩니다.

```
RPi (rasp_bringup.launch.py)                WSL (slam_real / nav_real / remote_view)
├── robot_state_publisher ──DDS──→          ├── Cartographer / Nav2
├── ugv_driver                               ├── RViz2
├── rf2o_laser_odometry (odom + TF) ←DDS──  └── teleop_all.py
├── roarm_driver                  │
├── ldlidar_ros2                  │
├── ugv_bridge (MQTT + REST)      │         웹 브라우저 (ugv_dashboard)
│   ├── FastAPI (:8081) ←─────────────────  ├── Vue 3 + MQTT.js
│   └── MQTT (:1884/ws) ─────────────────→  └── 맵/LiDAR 시각화, 원격 제어
└── static TF (base_lidar→laser)  └── /cmd_vel, /arm_controller/...
```

### CycloneDDS 설정

RPi와 WSL 양쪽에 동일한 설정 파일이 필요합니다.

**`~/cyclonedds.xml`:**
```xml
<CycloneDDS>
  <Domain>
    <General>
      <AllowMulticast>spdp</AllowMulticast>
      <Interfaces>
        <NetworkInterface name="eth0" />  <!-- RPi: eth0, WSL: eth1 등 환경에 맞게 -->
      </Interfaces>
    </General>
    <Discovery>
      <Peers>
        <Peer address="192.168.0.71" />   <!-- RPi IP -->
        <Peer address="localhost" />
      </Peers>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>120</MaxAutoParticipantIndex>
    </Discovery>
  </Domain>
</CycloneDDS>
```

> Nav2 스택은 많은 노드를 생성하므로 `MaxAutoParticipantIndex`를 120 이상으로 설정해야 합니다. 기본값(10)에서는 "Failed to find a free participant index" 오류가 발생합니다.

**환경변수 (`.bashrc`):**
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
```

---

## Quick Start: 실제 로봇 제어 (WSL + RPi)

> 로봇 하드웨어가 조립되어 있고, RPi에 Ubuntu 22.04 + ROS 2 Humble이 설치된 상태를 전제합니다.

### 하드웨어 시리얼 포트

| 포트 | 장치 | 드라이버 |
|------|------|---------|
| `/dev/ttyAMA0` | UGV 바퀴 ESP32 (General Driver for Robots) | `ugv_bringup` + `ugv_driver` |
| `/dev/ttyUSB0` | RoArm-M2 ESP32 | `roarm_driver` |
| `/dev/ttyUSB1` | LDLidar STL-19P | `ldlidar_ros2` |

### ESP32 시리얼 프로토콜 (바퀴 ESP32)

- 포트: `/dev/ttyAMA0`, 115200 baud
- 피드백은 기본 비활성 → `ugv_bringup`이 시작 시 `{"T":131,"cmd":1}` 전송하여 활성화
- 활성화 후 ESP32가 ~85Hz로 T:1001 피드백 전송:
  ```json
  {"T":1001,"L":0,"R":0,"r":-0.19,"p":1.45,"y":-165.79,"temp":75.0,"v":11.53}
  ```
  - `L/R`: 좌/우 인코더 값
  - `r/p/y`: roll/pitch/yaw (degrees) — IMU 센서 (QMI8658 + AK09918)
  - `v`: 배터리 전압 (V)

### 전원 구성

- UPS 모듈: 3S 18650 (9~12.6V)
- Rover 보드: UPS BAT 포트에서 배터리 전압 직통
- RoArm-M2 보드: UPS BAT 포트에서 Y케이블 분기 (12V급 필요, 5V 포트 사용 불가)
- ST3215 서보 동작 전압: 6~12.6V

### Step 1: RPi 세팅 (SSH 터미널)

```bash
# SSH 접속
ssh pi@192.168.0.71

# 1. 워크스페이스 클론
cd ~
git clone -b ros2-humble-develop https://github.com/fhekwn549/ugv_ws.git
cd ~/ugv_ws/src/ugv_main
git clone https://github.com/fhekwn549/ugv_roarm_description.git

# 2. Python 의존성 설치
pip3 install pyserial fastapi uvicorn paho-mqtt

# 3. apt 패키지 설치
sudo apt install ros-humble-rmw-cyclonedds-cpp ros-humble-xacro \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher \
  mosquitto

# 4. CycloneDDS 설정 (위의 "CycloneDDS 설정" 섹션 참고)
# ~/cyclonedds.xml 생성 + .bashrc에 환경변수 추가

# 5. 스왑 추가 (RPi RAM 1GB인 경우, C++ 빌드 OOM 방지)
sudo fallocate -l 2G /swapfile
sudo chmod 600 /swapfile && sudo mkswap /swapfile && sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab

# 6. 빌드 (필요한 패키지만)
cd ~/ugv_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ugv_bringup ugv_bridge ugv_roarm_description \
  ugv_description rf2o_laser_odometry ugv_interface ldlidar
source install/setup.bash

# 7. bashrc에 자동 source 추가 (최초 1회)
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ugv_ws/install/setup.bash" >> ~/.bashrc
```

### Step 2: WSL 세팅

```bash
# 1. 워크스페이스 클론
cd ~
git clone -b ros2-humble-develop https://github.com/fhekwn549/ugv_ws.git
cd ~/ugv_ws/src/ugv_main
git clone https://github.com/fhekwn549/ugv_roarm_description.git

# 2. apt 패키지 설치
sudo apt install ros-humble-rmw-cyclonedds-cpp ros-humble-xacro \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui ros-humble-rviz2 ros-humble-tf2-ros

# 3. CycloneDDS 설정 (위의 "CycloneDDS 설정" 섹션 참고)
# ~/cyclonedds.xml 생성 + .bashrc에 환경변수 추가

# 4. 빌드
cd ~/ugv_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ugv_bringup ugv_roarm_description ugv_description
source install/setup.bash

# 5. bashrc에 자동 source 추가 (최초 1회)
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ugv_ws/install/setup.bash" >> ~/.bashrc
```

### Step 3: 실행 (매번)

**총 3개의 터미널**이 필요합니다.

#### 터미널 1 — RPi: 하드웨어 드라이버 (SSH)

```bash
ssh pi@192.168.0.71
source ~/ugv_ws/install/setup.bash

# 모든 하드웨어 드라이버 실행
ros2 launch ugv_roarm_description rasp_bringup.launch.py
```

#### 터미널 2 — WSL: RViz

```bash
source ~/ugv_ws/install/setup.bash

# DDS를 통해 RPi 토픽 자동 수신 — host 파라미터 불필요
ros2 launch ugv_roarm_description remote_view.launch.py
```

> RViz가 열리고 RPi의 센서 데이터(LiDAR, IMU, 관절 상태)가 표시됩니다.

#### 터미널 3 — WSL: 키보드 텔레옵

```bash
source ~/ugv_ws/install/setup.bash

ros2 run ugv_roarm_description teleop_all.py --ros-args -p mode:=rviz
```

> 이 터미널에 포커스를 두고 키보드를 누르면 로봇이 움직입니다.
> `mode:=rviz`는 RViz TF를 직접 publish하면서 동시에 실제 로봇도 제어합니다.
> 기본 model은 `rasp_rover`입니다. UGV Rover 사용 시 `-p model:=ugv_rover` 추가.

### 실행 순서 요약

```
[RPi SSH]  rasp_bringup.launch.py        ← 먼저 실행 (하드웨어 준비)
    ↓ CycloneDDS (자동 discovery)
[WSL 1]    remote_view.launch.py          ← RViz 시각화
[WSL 2]    teleop_all.py mode:=rviz       ← 마지막 실행 (키보드 조작)
```

### 종료 순서

```
[WSL 2]    Ctrl+C (텔레옵 종료)
[WSL 1]    Ctrl+C (RViz 종료)
[RPi SSH]  Ctrl+C (하드웨어 드라이버 종료)
```

---

## Quick Start: RViz 단독 시각화 (로봇 없이)

실제 로봇 없이 URDF 모델과 키보드 텔레옵을 테스트합니다.

### GUI 슬라이더로 관절 확인

```bash
source ~/ugv_ws/install/setup.bash
ros2 launch ugv_roarm_description display.launch.py
```

### 키보드 텔레옵 (터미널 2개)

**터미널 1** — RViz:
```bash
source ~/ugv_ws/install/setup.bash
ros2 launch ugv_roarm_description display.launch.py use_gui:=false use_jsp:=false
```

**터미널 2** — 텔레옵:
```bash
source ~/ugv_ws/install/setup.bash
ros2 run ugv_roarm_description teleop_all.py --ros-args -p mode:=rviz
```

---

## Quick Start: 실제 로봇 SLAM (WSL + RPi)

> LiDAR로 주변 환경을 스캔하며 2D 맵을 생성합니다. Cartographer + rf2o_laser_odometry 사용.

### 사전 조건

```bash
# WSL에 Cartographer 설치
sudo apt install ros-humble-cartographer ros-humble-cartographer-ros
```

**총 3개의 터미널**이 필요합니다.

#### 터미널 1 — RPi: 하드웨어 드라이버 (SSH)

```bash
ssh pi@192.168.0.71
source ~/ugv_ws/install/setup.bash

ros2 launch ugv_roarm_description rasp_bringup.launch.py
```

#### 터미널 2 — WSL: SLAM + RViz

```bash
source ~/ugv_ws/install/setup.bash

ros2 launch ugv_roarm_description slam_real.launch.py
```

> RViz가 Top-Down 뷰로 열리며 LiDAR 스캔과 점진적으로 생성되는 맵이 표시됩니다.
> rf2o_laser_odometry가 LiDAR 스캔 매칭으로 odom을 생성하고, Cartographer가 이를 사용하여 SLAM을 수행합니다.

#### 터미널 3 — WSL: 키보드 텔레옵

```bash
source ~/ugv_ws/install/setup.bash

ros2 run ugv_roarm_description teleop_all.py
```

> 로봇을 천천히 움직이며 맵을 완성합니다. 속도를 낮게 유지하세요 (`e` 키).

#### 맵 저장

SLAM 실행 중에 **새 터미널에서 수동으로 저장**해야 합니다:

```bash
source ~/ugv_ws/install/setup.bash

# 1. Cartographer trajectory 마무리
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"

# 2. pbstream 저장 (Cartographer 내부 상태)
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '$HOME/maps/my_map.pbstream'}"

# 3. occupancy grid 맵 저장 (Nav2에서 사용)
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

저장되는 파일:
- `my_map.pbstream` — Cartographer 내부 상태 (localization 재사용 가능)
- `my_map.pgm` / `my_map.yaml` — Nav2에서 사용할 occupancy grid 맵

> **주의:** `Ctrl+C` 자동 저장은 레이스 컨디션으로 실패할 수 있습니다 (cartographer_node가 서비스 콜보다 먼저 종료). 반드시 수동 저장 후 `Ctrl+C`를 누르세요.

#### 맵 편집 (유리벽 등 불필요 영역 제거)

```bash
# PGM → PNG 변환 (편집 프로그램 호환성)
convert ~/maps/my_map.pgm ~/maps/my_map.png

# Windows에서 편집 (WSL2)
cp ~/maps/my_map.png /mnt/c/Users/$USER/Desktop/
# → Windows 그림판에서 편집: 불필요 영역을 RGB(205,205,205)로 칠하기
# → 저장 후 다시 복사

# PNG → PGM 변환
cp /mnt/c/Users/$USER/Desktop/my_map.png ~/maps/my_map.png
convert ~/maps/my_map.png ~/maps/my_map.pgm
```

> PGM 색상 의미: 흰색(254)=자유공간, 검정(0)=장애물, 회색(205)=미지 영역

---

## Quick Start: 자율주행 Navigation (WSL + RPi)

> SLAM으로 생성한 맵 파일이 필요합니다. 먼저 위의 SLAM 섹션으로 맵을 만드세요.

### 사전 조건

```bash
# WSL에 Nav2 패키지 설치
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

**총 3개의 터미널**이 필요합니다.

#### 터미널 1 — RPi: 하드웨어 드라이버 (SSH)

```bash
ssh pi@192.168.0.71
source ~/ugv_ws/install/setup.bash

ros2 launch ugv_roarm_description rasp_bringup.launch.py
```

#### 터미널 2 — WSL: Nav2 + RViz

```bash
source ~/ugv_ws/install/setup.bash

# map 인자에 SLAM으로 저장한 맵 YAML 경로를 지정
ros2 launch ugv_roarm_description nav_real.launch.py map:=~/maps/map_20250101_120000.yaml
```

> Nav2 lifecycle 노드들이 순서대로 활성화됩니다 (map_server → amcl → planner → controller → ...).
> 모든 노드가 `active` 상태가 되면 자율주행 준비 완료입니다.

#### 웹 대시보드 (RViz 대안)

RViz 없이 웹 브라우저에서 자율주행을 제어할 수도 있습니다. `rasp_bringup.launch.py`가 `ugv_bridge`를 자동 실행하며, 웹 대시보드에서 맵 위 로봇 위치 확인, 클릭으로 Nav2 목표 전송, 경로 표시가 가능합니다.

```bash
# 웹 대시보드 접속
# 브라우저에서 http://192.168.0.71:8081 (RPi에서 서빙)
# 또는 개발 모드: cd ~/ugv_dashboard && npm run dev → http://localhost:5173
```

#### 터미널 3 — WSL: 키보드 텔레옵 (선택사항)

```bash
source ~/ugv_ws/install/setup.bash

ros2 run ugv_roarm_description teleop_all.py
```

> 자율주행 중 수동 개입이 필요할 때 사용합니다. Nav2가 cmd_vel을 제어하므로 동시 사용 시 충돌에 주의하세요.

### RViz에서 자율주행 사용법

1. **초기 위치 설정**: RViz 상단 도구에서 `2D Pose Estimate` 클릭 → 맵 위에서 로봇의 실제 위치에서 실제 방향으로 드래그 (클릭=위치, 드래그 방향=로봇 heading)
2. **AMCL 수렴 확인**: 파란 파티클 클라우드가 로봇 주변에 모이고 LiDAR 스캔(빨강)이 맵 벽과 겹칠 때까지 대기 (텔레옵으로 약간 회전하면 수렴 가속)
3. **목표점 설정**: `Nav2 Goal` 클릭 → 목표 위치에서 원하는 도착 방향으로 드래그
4. 로봇이 전역 경로(초록선)를 따라 자율주행합니다

> **velocity_smoother 비활성화:** 이 로봇의 모터 데드밴드가 높아서 velocity_smoother의 점진적 가속이 실제 움직임을 만들지 못합니다. nav_real.launch.py에서 제거되었으며, controller_server가 `/cmd_vel`에 직접 publish합니다.

### 실행 순서 요약

```
[RPi SSH]  rasp_bringup.launch.py                  ← 먼저 실행 (하드웨어 준비)
    ↓ CycloneDDS (자동 discovery)
[WSL 1]    nav_real.launch.py map:=~/maps/map.yaml  ← Nav2 + RViz
[WSL 2]    teleop_all.py                            ← 선택사항 (수동 개입용)
```

---

## Quick Start: Gazebo 시뮬레이션

**터미널 1** — Gazebo:
```bash
source ~/ugv_ws/install/setup.bash
ros2 launch ugv_roarm_description gazebo.launch.py
```

> WSL2에서는 Gazebo 플러그인 초기화에 약 2분 소요.

**터미널 2** — 텔레옵:
```bash
source ~/ugv_ws/install/setup.bash
ros2 run ugv_roarm_description teleop_all.py
```

### Gazebo 디지털 트윈 + Nav2 자율주행 시뮬레이션

실제 SLAM으로 생성한 맵을 Gazebo 월드로 변환하여 자율주행을 테스트합니다.

```bash
# 1. 맵 → Gazebo 월드 변환 (최초 1회)
cd ~/ugv_ws/src/ugv_main/ugv_roarm_description
python3 scripts/map_to_gazebo_world.py --wall-height 0.8 --downsample 2

# 2. Gazebo + Nav2 실행
source ~/ugv_ws/install/setup.bash
ros2 launch ugv_roarm_description gazebo_nav.launch.py gui:=false use_rviz:=true

# 3. (별도 터미널) ugv_bridge 실행하면 웹 대시보드에서도 제어 가능
ros2 run ugv_bridge ugv_bridge_node
```

RViz에서 `Nav2 Goal`로 목표를 설정하거나, 웹 대시보드에서 Shift+클릭으로 목표를 전송할 수 있습니다.

> **WSL2 주의사항:** Gazebo가 느리므로 `controller_frequency: 5.0`, `bt_loop_duration: 100ms` 등 낮은 주파수로 설정되어 있습니다. `gui:=false`로 gzclient를 끄면 성능이 향상됩니다.

> **물리 튜닝:** `max_step_size: 0.003`, `contact_max_correcting_vel: 2.0`, `iters: 80`으로 설정. 바퀴 마찰 `mu1/mu2: 1.0` 적용 (팔 동작 시 미끄러짐 방지).

## Quick Start: 경량 Nav2 시뮬레이션 (Gazebo 불필요)

Gazebo 없이 **fake odom + fake scan**으로 Nav2 알고리즘만 테스트하는 경량 시뮬레이션입니다.
WSL2에서 Gazebo 초기화 지연(~2분)과 물리엔진 불일치 문제를 우회합니다.

```bash
source ~/ugv_ws/install/setup.bash

# 기본 실행 (RViz + Nav2)
ros2 launch ugv_roarm_description nav_sim.launch.py

# 초기 위치 지정
ros2 launch ugv_roarm_description nav_sim.launch.py spawn_x:=-0.81 spawn_y:=-2.70

# 웹 대시보드 포함
ros2 launch ugv_roarm_description nav_sim.launch.py use_bridge:=true
```

**구성:**
- `fake_odom_node` — `/cmd_vel` → 2D 운동학 적분 → `/odom` + TF (map→odom→base_footprint)
- `fake_scan_node` — 맵 레이캐스팅 → `/scan` + `/dev/shm` (bridge용)
  - LiDAR angle crop (30°~149°): 실제 LD19 LiDAR에서 로봇팔에 가려지는 영역을 시뮬레이션
- Nav2 stack — planner, controller, behavior, bt_navigator
- `use_sim_time: false` — 벽시계 사용
- `yaw_goal_tolerance: 0.15 rad` — 목적지 도착 시 지정 방향으로 회전

> **WSL2 LiDAR 전달 이슈:** CycloneDDS가 WSL2에서 RELIABLE LaserScan을 간헐적으로 전달하지 못하는 문제가 있어, fake_scan_node가 `/dev/shm/ugv_scan.json`에 scan 데이터를 직접 쓰고 bridge가 이를 읽습니다. 실제 로봇에서는 DDS 구독으로 자동 fallback됩니다.

> **LiDAR Angle Crop:** 실제 로봇은 LiDAR(LD19) 후방에 로봇팔(RoArm-M2)이 위치하여 30°~149° 범위의 readings이 차단됩니다. fake_scan_node와 rasp_bringup의 LD19 드라이버 모두 이 범위를 NaN으로 처리합니다.

---

## Keyboard Controls

주행과 팔 제어가 **동시에** 가능합니다.

| Key | Function |
|-----|----------|
| **Drive** | |
| `W` / `S` | Forward / Backward |
| `A` / `D` | Turn Left / Right |
| `Q` / `E` | Increase / Decrease speed |
| `Space` | Emergency Stop |
| **Arm** | |
| `M` | Joint ↔ TCP 모드 전환 |
| `1` / `2` / `3` | 관절 1~3 선택 (Joint) / X/Y/Z 축 선택 (TCP) |
| `I` / `K` | 관절 또는 TCP +/- |
| `U` / `J` | Step size +/- |
| **Gripper** | |
| `O` | Gripper open (+) |
| `P` | Gripper close (-) |

### TCP 제어 모드

`M` 키로 전환. 로봇팔 끝단의 직교 좌표(X, Y, Z)를 직접 제어합니다.
내부적으로 `roarm_m2` IK/FK 솔버를 사용하며, 도달 불가능한 위치로의 이동은 자동 무시됩니다.

### 시작 시 자세 동기화

시작 시 `/joint_states`에서 현재 로봇팔 관절값을 읽어와 초기 명령 상태를 동기화합니다.
기존 자세를 유지한 채 바로 제어를 시작할 수 있습니다 (5초 타임아웃, 수신 실패 시 0으로 시작).

### 실시간 피드백

상태줄 하단에 `/joint_states` 피드백 값(FB)이 실시간으로 표시됩니다.

---

## 아키텍처

### 통신 흐름 (WSL ↔ RPi)

```
[WSL]                                          [RPi]
teleop_all.py                                  rasp_bringup.launch.py
  ├─ /cmd_vel ──────┐                            ├─ ugv_driver (/dev/ttyAMA0)
  ├─ /arm_controller │     CycloneDDS             ├─ roarm_driver (/dev/ttyUSB0)
  │   /joint_trajectory┤  ←── DDS direct ──→       ├─ rf2o_laser_odometry (odom)
  ├─ /roarm/gripper_cmd┘                           ├─ ugv_bringup (IMU, voltage)
  │                                                 ├─ ldlidar_ros2 (/dev/ttyUSB1)
  │  ← /joint_states, /scan, /odom, /imu ←         └─ static TF (lidar→laser)
  │
  └─ RViz (TF 시각화)
```

### 그리퍼 값 변환

| 단계 | 값 | 범위 |
|------|-----|------|
| `grip_level` (teleop 내부) | 양수 | 0 (닫힘) ~ 2.094 (열림) |
| URDF joint value | 음수 (`-grip_level`) | -2.094 ~ 0 |
| RViz 시각 회전 | 양수 (axis -1이 부호 반전) | 0° ~ +120° |
| ESP32 명령 | `(max - grip) × 1.5` | π (닫힘) ~ 0 (열림) |

---

## Package Structure

```
ugv_roarm_description/
├── urdf/
│   ├── ugv_roarm.xacro            # UGV Rover + RoArm URDF
│   ├── ugv_roarm.gazebo.xacro     # UGV Rover Gazebo 플러그인
│   ├── rasp_roarm.xacro           # Rasp Rover + RoArm URDF (RPi용)
│   └── rasp_roarm.gazebo.xacro    # Rasp Rover Gazebo 플러그인
├── launch/
│   ├── display.launch.py          # RViz 단독 시각화
│   ├── gazebo.launch.py           # Gazebo 시뮬레이션
│   ├── gazebo_nav.launch.py       # Gazebo + Nav2 자율주행 시뮬레이션
│   ├── nav_sim.launch.py          # 경량 Nav2 시뮬레이션 (Gazebo 불필요)
│   ├── slam.launch.py             # SLAM (Gazebo 시뮬레이션)
│   ├── slam_real.launch.py        # SLAM (실제 로봇)
│   ├── nav_real.launch.py         # Nav2 자율주행 (실제 로봇)
│   ├── rasp_bringup.launch.py     # RPi 하드웨어 드라이버 + ugv_bridge
│   └── remote_view.launch.py      # WSL RViz 뷰어
├── scripts/
│   ├── teleop_all.py              # 통합 키보드 텔레옵 노드
│   ├── fake_odom_node.py          # 가상 오도메트리 노드 (nav_sim용)
│   ├── fake_scan_node.py          # 가상 LiDAR 스캔 노드 (nav_sim용, angle crop 30°~149°)
│   ├── sim_pose_bridge.py         # RViz 2D Pose Estimate → Gazebo 텔레포트
│   └── map_to_gazebo_world.py     # SLAM 맵 → Gazebo 월드 변환 스크립트
├── config/
│   ├── ugv_arm_controllers.yaml       # ros2_control 컨트롤러 설정
│   ├── ugv_arm_ros2_control.xacro     # ros2_control 하드웨어 인터페이스
│   ├── cartographer_mapping_2d.lua    # Cartographer SLAM 설정
│   ├── slam_toolbox.yaml              # SLAM 파라미터 (레거시, 롤백용)
│   ├── nav2_params.yaml               # Nav2 자율주행 파라미터 (실제 로봇)
│   ├── nav2_params_sim.yaml           # Nav2 자율주행 파라미터 (Gazebo 시뮬레이션)
│   └── nav2_params_fake.yaml          # Nav2 자율주행 파라미터 (경량 시뮬레이션, yaw_goal_tolerance=0.15)
├── rviz/
│   ├── view_ugv_roarm.rviz        # 기본 RViz 설정
│   ├── remote_view.rviz           # 원격 제어용 RViz 설정
│   ├── slam_view.rviz             # SLAM용 RViz 설정 (Top-Down)
│   ├── nav_view.rviz              # Nav2 자율주행용 RViz 설정
│   └── nav_sim_view.rviz          # Nav2 시뮬레이션용 RViz 설정
├── meshes/
│   ├── rasp_rover/                # Rasp Rover 메시 (STL)
│   └── roarm_m2/                  # RoArm-M2 메시 (STL)
└── worlds/
    ├── ugv_roarm.world            # Gazebo 월드 (빈 환경)
    ├── digital_twin.world         # Gazebo 디지털 트윈 월드 (SLAM 맵 기반)
    └── map_walls/                 # 맵 벽 모델 (자동 생성)
```

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | 주행 속도 명령 |
| `/arm_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | 팔 관절 목표 |
| `/roarm/gripper_cmd` | `std_msgs/Float64` | 그리퍼 제어 (ESP32) |
| `/gripper_controller/gripper_cmd` | `control_msgs/GripperCommand` | 그리퍼 제어 (Gazebo) |
| `/joint_states` | `sensor_msgs/JointState` | 관절 상태 피드백 |
| `/scan` | `sensor_msgs/LaserScan` | 2D LiDAR |
| `/odom` | `nav_msgs/Odometry` | 오도메트리 (rf2o_laser_odometry) |
| `/imu/data` | `sensor_msgs/Imu` | IMU 방위 (쿼터니언) |
| `/voltage` | `std_msgs/Float32` | 배터리 전압 (V) |

## 배포 (코드 수정 후)

```bash
# WSL에서
cd ~/ugv_ws/src/ugv_main/ugv_roarm_description
git add -A && git commit -m "설명" && git push origin main

cd ~/ugv_ws
git add -A && git commit -m "설명" && git push origin ros2-humble-develop

# RPi에서 (SSH)
cd ~/ugv_ws
git pull origin ros2-humble-develop
cd src/ugv_main/ugv_roarm_description && git pull origin main
cd ~/ugv_ws
colcon build --packages-select ugv_bringup ugv_bridge rf2o_laser_odometry ugv_roarm_description
source install/setup.bash
```

## 파라미터

### ugv_driver

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `angular_scale` | `2.5` | 회전 속도 배율. 스키드 스티어는 지면 마찰 때문에 높은 토크가 필요합니다 |

> `ugv_driver`는 `cmd_vel`의 `linear.x`를 부호 반전하여 ESP32에 전달합니다 (ESP32 모터 방향이 URDF 규약과 반대).

### rf2o_laser_odometry

LiDAR 스캔 매칭 기반 오도메트리. Wave Rover는 인코더가 없으므로 cmd_vel dead reckoning 대신 rf2o를 사용합니다.

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `laser_scan_topic` | `/scan` | LiDAR 스캔 토픽 |
| `odom_topic` | `/odom` | 발행할 오도메트리 토픽 |
| `publish_tf` | `true` | odom → base_footprint TF 발행 여부 |
| `base_frame_id` | `base_footprint` | 로봇 베이스 프레임 |
| `odom_frame_id` | `odom` | 오도메트리 프레임 |
| `laser_frame_id` | `base_laser` | LiDAR 프레임 |
| `init_pose_from_topic` | `''` (빈 문자열) | 초기 위치 토픽. 비워야 즉시 시작. 기본값(`/base_pose_ground_truth`)은 존재하지 않아 영원히 대기 |
| `freq` | `20.0` | 오도메트리 계산 주기 (Hz) |

> RPi에서 빌드 시 RAM 부족으로 OOM이 발생할 수 있습니다. 2GB 스왑 추가 후 `--parallel-workers 1`로 빌드하세요.

### roarm_driver

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `serial_port` | `/dev/ttyUSB0` | RoArm-M2 ESP32 시리얼 포트 |
| `baud_rate` | `115200` | 시리얼 통신 속도 |
| `feedback_rate` | `5.0` | T:105 관절 피드백 주기 (Hz) |

> 드라이버 시작 시 토크 ON 후 홈 포즈 (base=0, shoulder=-1.6, elbow=3.2, hand=3)로 이동합니다. 팔을 접은 자세로 그리퍼가 닫힌 상태입니다.

## Troubleshooting

| 증상 | 원인 | 해결 |
|------|------|------|
| RViz에서 로봇이 안 보임 | `ugv_description` 미빌드 | `colcon build --packages-select ugv_description` 후 source |
| WSL에서 RPi 토픽 안 보임 | CycloneDDS 미설정 | `~/cyclonedds.xml` 확인 + `RMW_IMPLEMENTATION`, `CYCLONEDDS_URI` 환경변수 확인 |
| `ros2 topic list`에 RPi 토픽 없음 | 네트워크/방화벽 문제 | `ping 192.168.0.71` 확인, WSL 방화벽 규칙 확인 |
| 로봇이 RViz에서 안 움직임 | teleop 미실행 또는 Fixed Frame 설정 | teleop 실행 + Fixed Frame을 `odom`으로 설정 |
| SLAM에서 odom 위치가 안 변함 | rf2o가 /scan 수신 못함 | `ros2 topic echo /scan --once`로 LiDAR 데이터 확인, `laser_frame_id` 파라미터 확인 |
| RPi에서 rf2o 빌드 중 멈춤 | RAM 부족 (C++ Eigen 컴파일) | 2GB 스왑 추가 후 `colcon build --parallel-workers 1` |
| 로봇팔 토크 걸리지만 안 움직임 | 전압 부족 (5V) | UPS BAT 포트에서 12V 공급. 5V 포트 사용 불가 |
| 팔 관절 움직이면 그리퍼 토크 풀림 | `roarm_driver` 미업데이트 | RPi에서 `ugv_bringup` 리빌드 |
| roarm_driver SerialException | 시리얼 포트 다중 접근 | 수동 테스트 스크립트 종료 후 launch 재시작 |
| 바닥에서 제자리 회전 안 됨 | 스키드 스티어 마찰 | `angular_scale` 파라미터 증가 (기본 2.5) |
| 직진 시 한쪽으로 치우침 | 좌/우 바퀴 저항 차이 | 모터 교체 또는 하드웨어 점검 |
| WSL2에서 RViz 크래시 | GPU 호환성 | `LIBGL_ALWAYS_SOFTWARE=1` 환경변수 설정 |
| 그리퍼 방향 반대 | roarm_driver 버전 불일치 | RPi에서 git pull + 리빌드 |
| rf2o "Waiting for laser_scans..." 멈춤 | `init_pose_from_topic` 기본값이 존재하지 않는 토픽 | launch에서 `'init_pose_from_topic': ''` 설정 (빈 문자열) |
| Nav2 "Failed to find a free participant index" | CycloneDDS 참가자 한도 초과 | `cyclonedds.xml`에 `MaxAutoParticipantIndex: 120` 추가 |
| RViz에서 맵이 안 보임 (Nav2) | Map 토픽 QoS 불일치 | RViz Map display의 Durability Policy를 `Transient Local`로 설정 |
| Nav2 Goal 후 로봇이 안 움직임 | 모터 데드밴드보다 낮은 속도 명령 | `nav2_params.yaml`에서 `min_vel_x`, `min_speed_xy` 증가 (0.05 이상) |
| Nav2 회전 후 AMCL 위치 틀어짐 | rf2o odom drift + AMCL 업데이트 느림 | `update_min_a: 0.05`, `alpha1/2/4: 0.5`로 설정, 파티클 수 증가 |
| Nav2 목적지 도착 후 방향 안 맞춤 | `yaw_goal_tolerance`가 6.28 (360°) | `nav2_params_fake.yaml`에서 `yaw_goal_tolerance: 0.15` (약 8.6°)로 설정 |
| WSL2 대시보드 LiDAR 안 보임 | CycloneDDS 크로스 프로세스 전달 실패 | `use_bridge:=true`로 실행, fake_scan_node가 `/dev/shm`에 직접 쓰고 bridge가 폴링 |
| 이전 세션 좀비 프로세스로 로봇 진동 | 이전 fake_odom_node/bridge_node 미종료 | `ps aux \| grep fake_odom`으로 확인 후 kill, 재시작 |
