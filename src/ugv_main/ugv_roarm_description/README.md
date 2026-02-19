# ugv_roarm_description

ROS 2 Humble package for the **UGV Rover + RoArm-M2** mobile manipulator.

Waveshare UGV Rover (4WD skid-steer) 위에 RoArm-M2 (4-DOF + gripper) 로봇팔을 통합한 URDF 모델, 시뮬레이션, 키보드 텔레옵 패키지입니다.

## Features

- **통합 URDF/Xacro** - UGV Rover 베이스 + RoArm-M2 매니퓰레이터 + 센서(2D LiDAR, Depth Camera, IMU)
- **Gazebo Classic 시뮬레이션** - ros2_control 기반 관절 제어, skid-steer 주행
- **통합 키보드 텔레옵** - 주행 + 팔 + 그리퍼 동시 제어, Joint/TCP 모드 지원
- **RViz 시각화** - 직접 TF 퍼블리싱을 통한 로봇 제어 (Gazebo 없이)
- **SLAM** - slam_toolbox를 이용한 실시간 맵 생성

## Prerequisites

### 1. ROS 2 환경

ROS 2 Humble이 설치되어 있어야 합니다. 아직 설치하지 않았다면 [공식 설치 가이드](https://docs.ros.org/en/humble/Installation.html)를 참조하세요.

```bash
# 매 터미널마다 ROS 2 환경을 불러와야 합니다 (또는 .bashrc에 추가)
source /opt/ros/humble/setup.bash
```

### 2. apt 패키지 설치

```bash
sudo apt install ros-humble-gazebo-ros ros-humble-gazebo-ros2-control \
  ros-humble-joint-state-broadcaster ros-humble-joint-trajectory-controller \
  ros-humble-gripper-controllers ros-humble-xacro ros-humble-slam-toolbox \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher-gui
```

### 3. 워크스페이스 및 소스 패키지 설정

이 패키지는 **단독으로 사용할 수 없습니다.** URDF에서 UGV Rover의 차체/바퀴 메시 파일을 `package://ugv_description/meshes/ugv_rover/...` 경로로 참조하기 때문에, Waveshare의 **`ugv_description`** 패키지가 반드시 같은 워크스페이스에 있어야 합니다.

`ugv_description`은 Waveshare의 공식 레포지토리 [waveshareteam/ugv_ws](https://github.com/waveshareteam/ugv_ws)에 포함되어 있습니다.

아래 순서대로 워크스페이스를 구성합니다:

```bash
# 1. 워크스페이스 디렉토리 생성
mkdir -p ~/ugv_ws/src
cd ~/ugv_ws/src

# 2. Waveshare 공식 레포 클론 (ugv_description 포함)
#    ros2-humble-develop 브랜치를 ugv_main 디렉토리로 클론합니다
git clone -b ros2-humble-develop https://github.com/waveshareteam/ugv_ws.git ugv_main

# 3. 이 패키지를 ugv_main 안에 클론
cd ugv_main
git clone https://github.com/fhekwn549/ugv_roarm_description.git

# 4. (Gazebo 시뮬레이션 사용 시) gazebo_ros2_control 소스 빌드 필요
cd ~/ugv_ws/src
git clone -b humble https://github.com/ros-controls/gazebo_ros2_control.git
```

#### 최종 디렉토리 구조

설정이 완료되면 아래와 같은 구조가 됩니다:

```
~/ugv_ws/                              ← colcon 워크스페이스 루트
├── src/
│   ├── ugv_main/                      ← waveshareteam/ugv_ws 클론 (ros2-humble-develop)
│   │   ├── ugv_description/           ← ★ 필수: UGV Rover 메시 파일 제공
│   │   │   ├── meshes/
│   │   │   │   └── ugv_rover/         ← base_link.stl, wheel STL 등
│   │   │   ├── urdf/
│   │   │   └── package.xml
│   │   ├── ugv_roarm_description/     ← ★ 이 패키지 (별도 git 레포)
│   │   │   ├── meshes/roarm_m2/       ← RoArm-M2 메시 파일
│   │   │   ├── urdf/
│   │   │   ├── launch/
│   │   │   ├── scripts/teleop_all.py
│   │   │   └── package.xml
│   │   ├── ugv_gazebo/                ← (참고) Waveshare Gazebo 패키지
│   │   └── ...                        ← 기타 Waveshare 패키지
│   └── gazebo_ros2_control/           ← (선택) Gazebo 시뮬레이션용
├── build/                             ← colcon build 결과 (자동 생성)
├── install/                           ← colcon install 결과 (자동 생성)
└── log/                               ← 빌드 로그 (자동 생성)
```

> **핵심**: `colcon build`는 `src/` 하위를 재귀적으로 탐색하여 `package.xml`이 있는 디렉토리를 자동으로 패키지로 인식합니다. `ugv_description`과 `ugv_roarm_description`이 모두 `src/` 아래에 있으면 경로 깊이와 관계없이 빌드됩니다.

#### 설정 확인

```bash
# ugv_description의 메시 파일이 존재하는지 확인
ls ~/ugv_ws/src/ugv_main/ugv_description/meshes/ugv_rover/base_link.stl
# 파일이 존재하면 정상
```

## Build

```bash
cd ~/ugv_ws
source /opt/ros/humble/setup.bash

# 처음 빌드 시 ugv_description도 함께 빌드해야 합니다
colcon build --packages-up-to ugv_roarm_description --symlink-install
source install/setup.bash
```

> **`--packages-up-to`**는 `ugv_roarm_description`과 그 의존 패키지(`ugv_description` 등)를 자동으로 함께 빌드합니다.
> 이후 이 패키지만 수정했을 때는 `--packages-select ugv_roarm_description`으로 빠르게 빌드할 수 있습니다.

빌드가 성공했는지 확인:

```bash
source ~/ugv_ws/install/setup.bash
ros2 pkg prefix ugv_roarm_description
# 정상이면 /home/<user>/ugv_ws/install/ugv_roarm_description 출력

ros2 pkg prefix ugv_description
# 정상이면 /home/<user>/ugv_ws/install/ugv_description 출력
```

## Usage

### 1. RViz 시각화 (GUI 슬라이더)

가장 간단한 방법. 슬라이더로 관절을 드래그하여 로봇 모델을 확인합니다.

```bash
source ~/ugv_ws/install/setup.bash
ros2 launch ugv_roarm_description display.launch.py
```

RViz 창이 열리고 로봇 모델이 표시됩니다. 왼쪽 `joint_state_publisher_gui` 패널에서 슬라이더를 움직이면 관절이 회전합니다.

### 2. RViz 키보드 텔레옵

**터미널 2개**가 필요합니다.

**터미널 1** — RViz 실행 (GUI 슬라이더 비활성화):

```bash
source ~/ugv_ws/install/setup.bash
ros2 launch ugv_roarm_description display.launch.py use_gui:=false use_jsp:=false
```

**터미널 2** — 텔레옵 실행:

```bash
source ~/ugv_ws/install/setup.bash
ros2 run ugv_roarm_description teleop_all.py --ros-args -p mode:=rviz
```

터미널 2에 키 안내 배너가 출력되면, 해당 터미널에 포커스를 두고 키보드를 누릅니다.

#### Fixed Frame 설정 (중요)

텔레옵 RViz 모드는 `odom → base_footprint → ...` TF 트리를 직접 퍼블리시합니다.
RViz 좌측 **Displays** 패널 → **Global Options** → **Fixed Frame**을 용도에 맞게 설정하세요:

| Fixed Frame | 동작 | 용도 |
|---|---|---|
| `odom` | 로봇이 월드 공간에서 이동 (W/S/A/D로 주행 시 로봇이 움직임) | 주행 테스트 |
| `base_footprint` | 로봇이 항상 화면 중앙에 고정 (배경이 움직임) | 팔 조작에 집중할 때 |

> **Tip**: 기본값은 `base_footprint`입니다. 주행을 확인하려면 RViz에서 Fixed Frame을 `odom`으로 변경하세요.

### 3. Gazebo 시뮬레이션

**터미널 2개**가 필요합니다.

**터미널 1** — Gazebo 실행:

```bash
source ~/ugv_ws/install/setup.bash
ros2 launch ugv_roarm_description gazebo.launch.py
```

> WSL2에서는 Gazebo 플러그인 초기화에 약 2분이 소요됩니다. 로그가 멈춘 것처럼 보여도 기다려 주세요.

**터미널 2** — 텔레옵 실행:

```bash
source ~/ugv_ws/install/setup.bash
ros2 run ugv_roarm_description teleop_all.py
```

### 4. SLAM 맵핑

**터미널 2개**가 필요합니다.

**터미널 1** — Gazebo + slam_toolbox + RViz 통합 실행:

```bash
source ~/ugv_ws/install/setup.bash
ros2 launch ugv_roarm_description slam.launch.py
```

**터미널 2** — 텔레옵으로 주행하며 맵 생성:

```bash
source ~/ugv_ws/install/setup.bash
ros2 run ugv_roarm_description teleop_all.py
```

## Keyboard Controls

주행과 팔 제어가 **동시에** 가능합니다. `M` 키로 팔 제어 모드를 전환합니다.

| Key | Function |
|-----|----------|
| **Drive** | |
| `W` / `S` | Forward / Backward |
| `A` / `D` | Turn Left / Right |
| `Q` / `E` | Increase / Decrease speed |
| `Space` | Emergency Stop |
| **Arm** | |
| `M` | Joint ↔ TCP 모드 전환 |
| `1` / `2` / `3` | Joint 모드: 관절 1~3 선택 / TCP 모드: X/Y/Z 축 선택 |
| `I` / `K` | Joint 모드: 관절 +/- / TCP 모드: 선택 축 +/- (mm) |
| `U` / `J` | Step size +/- |
| **Gripper** | |
| `O` / `P` | Gripper open(+) / close(-) |

### TCP 제어 모드

TCP(Tool Center Point) 모드에서는 로봇팔 끝단의 직교 좌표(X, Y, Z)를 직접 제어합니다.
내부적으로 `roarm_moveit_cmd/solver.hpp`의 `roarm_m2` IK/FK 솔버를 Python으로 포팅하여 사용합니다.

- **FK**: 현재 관절 각도 → TCP 위치(mm) 계산
- **IK**: 목표 TCP 위치 → 관절 각도 역변환
- 도달 불가능한 위치로의 이동 명령은 자동으로 무시됩니다

## Package Structure

```
ugv_roarm_description/
├── urdf/
│   ├── ugv_roarm.xacro            # Display용 URDF
│   └── ugv_roarm.gazebo.xacro     # Gazebo 플러그인 포함 URDF
├── launch/
│   ├── display.launch.py          # RViz 시각화
│   ├── gazebo.launch.py           # Gazebo 시뮬레이션
│   └── slam.launch.py             # SLAM 맵핑
├── config/
│   ├── ugv_arm_controllers.yaml   # ros2_control 컨트롤러 설정
│   ├── ugv_arm_ros2_control.xacro # ros2_control 하드웨어 인터페이스
│   └── slam_toolbox.yaml          # SLAM 파라미터
├── scripts/
│   └── teleop_all.py              # 통합 키보드 텔레옵 노드
├── rviz/
│   ├── view_ugv_roarm.rviz        # 기본 RViz 설정
│   └── slam_view.rviz             # SLAM용 RViz 설정
├── meshes/roarm_m2/               # RoArm-M2 메시 파일
└── worlds/
    └── ugv_roarm.world            # 기본 Gazebo 월드
```

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | 주행 속도 명령 |
| `/arm_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | 팔 관절 목표 |
| `/gripper_controller/gripper_cmd` | `control_msgs/GripperCommand` | 그리퍼 제어 |
| `/joint_states` | `sensor_msgs/JointState` | 관절 상태 |
| `/scan` | `sensor_msgs/LaserScan` | 2D LiDAR |
| `/odom` | `nav_msgs/Odometry` | 오도메트리 |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM 맵 |

## Troubleshooting

| 증상 | 원인 | 해결 |
|------|------|------|
| RViz에서 로봇이 안 보임 | `ugv_description` 미빌드 | `colcon build`로 ugv_description 포함 빌드 후 `source install/setup.bash` |
| `Fixed Frame [base_footprint] does not exist` | 텔레옵 미실행 | 터미널 2에서 `teleop_all.py --ros-args -p mode:=rviz` 실행 |
| W/S 키로 주행해도 로봇이 안 움직임 | Fixed Frame이 `base_footprint` | RViz에서 Fixed Frame을 `odom`으로 변경 |
| Gazebo에서 로봇이 바닥을 뚫고 떨어짐 | 메시 로드 실패 | `GAZEBO_MODEL_PATH` 설정 확인 (launch 파일에서 자동 설정됨) |
| WSL2에서 Gazebo GUI 크래시 | D3D12 호환성 문제 | `gui:=false` 옵션으로 headless 실행 |

## Notes

- **WSL2**: Gazebo GUI는 소프트웨어 렌더링(`LIBGL_ALWAYS_SOFTWARE=1`) 사용. 플러그인 초기화에 ~2분 소요.
- **RViz 모드**: `robot_state_publisher`를 통하지 않고 직접 TF를 퍼블리시하여 비동기 TF로 인한 진동 방지.
