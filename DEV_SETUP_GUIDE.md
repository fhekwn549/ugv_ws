# 개발 환경 세팅 가이드

UGV RoArm 프로젝트의 개발 환경을 처음부터 세팅하는 가이드입니다.

---

## 1. WSL2 + Ubuntu 22.04 설치

### WSL2 설치 (Windows)

```powershell
# PowerShell (관리자 권한)
wsl --install -d Ubuntu-22.04
```

설치 후 Ubuntu 터미널을 열어 사용자명/비밀번호를 설정합니다.

### WSL2 메모리 제한 (권장)

`%USERPROFILE%\.wslconfig` 파일을 생성/수정:

```ini
[wsl2]
memory=8GB
swap=4GB
processors=4
```

변경 후 PowerShell에서 `wsl --shutdown` 실행.

---

## 2. ROS 2 Humble 설치

```bash
# 로캘 설정
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# ROS 2 GPG 키 및 리포 추가
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# ROS 2 Humble 설치
sudo apt update
sudo apt install -y ros-humble-desktop

# 개발 도구 설치
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pip \
  ros-humble-rmw-cyclonedds-cpp

# rosdep 초기화
sudo rosdep init
rosdep update
```

---

## 3. 워크스페이스 빌드

### ugv_ws 클론 및 빌드

```bash
mkdir -p ~/ugv_ws/src
cd ~/ugv_ws
git clone https://github.com/fhekwn549/ugv_ws.git .

# 의존성 설치
rosdep install --from-paths src --ignore-src -r -y

# 빌드
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 빌드 문제 해결

**메모리 부족 시:**
```bash
colcon build --symlink-install --parallel-workers 2
```

**특정 패키지만 빌드:**
```bash
colcon build --symlink-install --packages-select ugv_bridge ugv_nav
```

---

## 4. CycloneDDS 설정 (RPi ↔ WSL2 통신)

### DDS 미들웨어 설정

```bash
# ~/.bashrc에 추가
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0  # RPi와 동일하게 설정
```

### CycloneDDS 설정 파일

RPi와 WSL2 간 통신을 위한 CycloneDDS 설정:

```bash
export CYCLONEDDS_URI=file://$HOME/ugv_ws/src/ugv_main/ugv_bringup/config/cyclonedds.xml
```

### 네트워크 확인

```bash
# WSL2 IP 확인
ip addr show eth0 | grep inet

# RPi에 ping 테스트
ping <RPi-IP>

# ROS 2 토픽 수신 테스트
ros2 topic list
```

### WSL2 네트워크 주의사항

- WSL2의 IP는 재부팅마다 변경됨
- RPi와 동일 서브넷에 있어야 DDS 통신 가능
- 방화벽에서 UDP 7400~7500 포트 허용 필요
- Windows 방화벽 규칙도 확인 필요

---

## 5. RPi SSH 접속 및 배포

### SSH 키 설정

```bash
# SSH 키 생성 (아직 없는 경우)
ssh-keygen -t ed25519

# RPi에 공개키 복사
ssh-copy-id ubuntu@<RPi-IP>
```

### SSH 접속

```bash
ssh ubuntu@<RPi-IP>
```

### 코드 배포 (rsync)

```bash
# 소스 코드만 동기화 (빌드 결과 제외)
rsync -avz --exclude='build/' --exclude='install/' --exclude='log/' \
  ~/ugv_ws/src/ ubuntu@<RPi-IP>:~/ugv_ws/src/
```

### RPi에서 빌드 및 실행

```bash
ssh pi@192.168.0.71
cd ~/ugv_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ugv_bringup ugv_bridge ugv_roarm_description \
  ugv_description rf2o_laser_odometry ugv_interface ldlidar
source install/setup.bash
ros2 launch ugv_roarm_description rasp_bringup.launch.py
```

### RPi RabbitMQ 설정

ugv_bridge가 MQTT로, 대시보드가 STOMP WebSocket으로 통신하기 위해 RabbitMQ가 필요합니다:

```bash
# 설치
sudo apt install -y rabbitmq-server

# 플러그인 활성화
sudo rabbitmq-plugins enable rabbitmq_mqtt rabbitmq_web_stomp rabbitmq_management

# 외부 접속 허용 (guest 계정)
echo "loopback_users = none" | sudo tee /etc/rabbitmq/rabbitmq.conf
sudo systemctl restart rabbitmq-server

# Mosquitto가 실행 중이면 중지 (포트 1883 충돌 방지)
sudo systemctl stop mosquitto
sudo systemctl disable mosquitto
```

RabbitMQ 포트:
| 포트 | 프로토콜 | 용도 |
|------|----------|------|
| 1883 | MQTT | ugv_bridge 연결 |
| 15674 | Web STOMP | 대시보드 WebSocket 연결 |
| 15672 | HTTP | RabbitMQ 관리 UI |

---

## 6. ugv_dashboard 개발 환경

### Node.js 설치

```bash
# nvm 설치
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.7/install.sh | bash
source ~/.bashrc

# Node.js 18 설치
nvm install 18
nvm use 18
```

### 대시보드 클론 및 실행

```bash
git clone https://github.com/fhekwn549/ugv_dashboard.git ~/ugv_dashboard
cd ~/ugv_dashboard
npm install

# 환경 변수 설정
echo "VITE_ROBOT_HOST=<RPi-IP>" > .env

# 개발 서버 실행
npm run dev
```

브라우저에서 `http://localhost:5173` 접속.

---

## 7. 유용한 alias 및 스크립트

`~/.bashrc`에 추가하면 편리한 설정들:

```bash
# ROS 2 환경 자동 소스
source /opt/ros/humble/setup.bash
[ -f ~/ugv_ws/install/setup.bash ] && source ~/ugv_ws/install/setup.bash

# DDS 설정
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0

# alias
alias cb='cd ~/ugv_ws && colcon build --symlink-install'
alias cbs='cd ~/ugv_ws && colcon build --symlink-install --packages-select'
alias src='source ~/ugv_ws/install/setup.bash'
alias rpi='ssh ubuntu@<RPi-IP>'
alias deploy='rsync -avz --exclude="build/" --exclude="install/" --exclude="log/" ~/ugv_ws/src/ ubuntu@<RPi-IP>:~/ugv_ws/src/'

# 대시보드
alias dash='cd ~/ugv_dashboard && npm run dev'
```

### 자주 사용하는 ROS 2 명령어

```bash
# 토픽 목록
ros2 topic list

# 토픽 모니터링
ros2 topic echo /scan

# 노드 목록
ros2 node list

# TF 트리 확인
ros2 run tf2_tools view_frames

# RViz
rviz2
```

---

## 문제 해결

### Gazebo GUI (WSL2 + Intel GPU)

`~/.bashrc`에 아래 환경변수가 설정되어 있으면 Gazebo GUI가 Intel GPU 가속으로 정상 작동합니다:

```bash
export MESA_D3D12_DEFAULT_ADAPTER_NAME=Intel
export MESA_LOADER_DRIVER_OVERRIDE=d3d12
export LIBGL_ALWAYS_SOFTWARE=0
export MESA_GL_VERSION_OVERRIDE=4.5
```

GPU 설정이 안 된 환경에서는 GUI 없이 실행:

```bash
ros2 launch ugv_roarm_description gazebo.launch.py gui:=false
```

### DDS 통신이 안 됨

1. `ROS_DOMAIN_ID`가 양쪽에서 동일한지 확인
2. `ping <RPi-IP>`로 네트워크 연결 확인
3. CycloneDDS 설정 파일의 네트워크 인터페이스 확인
4. Windows 방화벽에서 UDP 포트 허용 확인

### colcon build 중 메모리 부족

```bash
colcon build --symlink-install --parallel-workers 1
```

또는 WSL2 메모리 할당을 늘려주세요 (`.wslconfig`).
