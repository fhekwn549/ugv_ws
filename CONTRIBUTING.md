# 기여 가이드 (ugv_ws)

UGV RoArm 프로젝트의 메인 워크스페이스 리포지토리에 기여하기 위한 가이드입니다.

---

## 개발 환경 요구사항

| 항목 | 버전 |
|------|------|
| OS | Ubuntu 22.04 (WSL2 또는 네이티브) |
| ROS 2 | Humble Hawksbill |
| Python | 3.10+ |
| DDS | CycloneDDS |
| 빌드 도구 | colcon |

> 상세 설치 가이드는 [DEV_SETUP_GUIDE.md](./DEV_SETUP_GUIDE.md)를 참고하세요.

---

## 리포지토리 클론 및 빌드

### 클론

```bash
mkdir -p ~/ugv_ws/src
cd ~/ugv_ws
git clone https://github.com/fhekwn549/ugv_ws.git .
```

### 의존성 설치

```bash
cd ~/ugv_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 빌드

```bash
cd ~/ugv_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

**특정 패키지만 빌드:**

```bash
colcon build --symlink-install --packages-select ugv_bridge ugv_nav
```

**병렬 빌드 제한 (메모리 부족 시):**

```bash
colcon build --symlink-install --parallel-workers 2
```

---

## 주요 패키지 구조

```
src/ugv_main/
├── ugv_base_node/       # 하드웨어 통신 노드
├── ugv_bridge/          # FastAPI 브릿지 서버
├── ugv_bringup/         # 전체 시스템 런치
├── ugv_description/     # 기본 UGV URDF
├── ugv_gazebo/          # Gazebo 시뮬레이션
├── ugv_interface/       # 커스텀 메시지/서비스
├── ugv_nav/             # 내비게이션
├── ugv_roarm_description/ # RoArm URDF (서브모듈)
├── ugv_slam/            # SLAM
├── ugv_tools/           # 유틸리티
├── ugv_vision/          # 카메라/비전
└── ugv_web_app/         # 웹 인터페이스
```

---

## ugv_bridge (FastAPI) 개발 가이드

`ugv_bridge`는 ROS 2 노드와 웹 클라이언트 간 통신을 담당하는 FastAPI 서버입니다.

### 실행

```bash
ros2 launch ugv_bridge ugv_bridge_launch.py
```

### 개발 시 주의사항

- FastAPI 엔드포인트 추가 시 ROS 2 토픽/서비스와의 매핑을 명확히 문서화
- 실시간 메시징은 RabbitMQ를 통해 처리 (Bridge: MQTT:1883, Dashboard: STOMP/WS:15674)
- 새 API 엔드포인트 추가 시 `ugv_dashboard`의 해당 호출부도 함께 업데이트

### RabbitMQ 설정

브로커가 Mosquitto에서 RabbitMQ로 전환되었습니다. Bridge(paho-mqtt)는 MQTT 포트 1883을 그대로 사용하고, Dashboard는 STOMP over WebSocket(15674)을 사용합니다.

```bash
# RabbitMQ 설치 및 설정 배포
sudo apt install rabbitmq-server
sudo cp config/rabbitmq.conf /etc/rabbitmq/rabbitmq.conf
sudo cp config/enabled_plugins /etc/rabbitmq/enabled_plugins
sudo systemctl restart rabbitmq-server

# 상태 확인
sudo rabbitmqctl status
# Management UI: http://localhost:15672 (guest/guest)
```

| 포트 | 프로토콜 | 용도 |
|------|----------|------|
| 1883 | MQTT | Bridge (paho-mqtt) |
| 61613 | STOMP | MES/ERP (향후) |
| 15674 | Web STOMP (WS) | Dashboard (@stomp/stompjs) |
| 15672 | HTTP | Management UI |

---

## CycloneDDS 설정

RPi와 WSL2 간 통신을 위해 CycloneDDS를 사용합니다.

### 설정 파일 위치

```bash
export CYCLONEDDS_URI=file://$HOME/ugv_ws/src/ugv_main/ugv_bringup/config/cyclonedds.xml
```

### 주의사항

- WSL2와 RPi가 동일 네트워크 서브넷에 있어야 함
- `ROS_DOMAIN_ID`를 양쪽에서 동일하게 설정
- 방화벽이 UDP 7400~7500 포트를 허용하는지 확인
- WSL2의 IP가 재부팅 시 변경될 수 있으므로 CycloneDDS 설정의 `NetworkInterface`를 확인

---

## RPi 배포 방법

### SSH 접속

```bash
ssh ubuntu@<RPi-IP>
```

### 코드 동기화

```bash
# 워크스페이스 전체 동기화 (빌드 결과 제외)
rsync -avz --exclude='build/' --exclude='install/' --exclude='log/' \
  ~/ugv_ws/src/ ubuntu@<RPi-IP>:~/ugv_ws/src/
```

### RPi에서 빌드 및 실행

```bash
ssh ubuntu@<RPi-IP>
cd ~/ugv_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch ugv_bringup bringup_launch.py
```

---

## 브랜치 전략

[BRANCHING_STRATEGY.md](./BRANCHING_STRATEGY.md)를 참고하세요.

---

## PR 작성 가이드

1. **브랜치 생성**: 기본 브랜치(`ros2-humble-develop`)에서 `<이름>/<설명>` 브랜치 생성
2. **작업 수행**: 변경사항 커밋 (Conventional Commits 형식, 영어)
3. **빌드 확인**: `colcon build`가 성공하는지 확인
4. **PR 생성**: GitHub에서 기본 브랜치로 PR 생성
5. **PR 본문**: 변경 사항, 테스트 방법, 관련 이슈를 기재

---

## 코드 스타일

### Python

- [PEP 8](https://peps.python.org/pep-0008/) 준수
- 타입 힌트 사용 권장
- ROS 2 노드는 프로젝트의 Clean Architecture 패턴을 따름
  - Domain 계층에 ROS 2 의존성 금지
  - Infrastructure 계층에서 ROS 2 어댑터 구현

### C++

- ROS 2 C++ 스타일 가이드 준수
- 헤더 파일은 `include/<package_name>/` 하위에 배치
- `ament_lint_auto` 사용

### Launch 파일

- Python launch 파일 사용 (`*.launch.py`)
- 파라미터는 YAML 파일로 분리 (`config/` 디렉토리)

---

## 테스트

```bash
# 전체 테스트
cd ~/ugv_ws
colcon test
colcon test-result --verbose

# 특정 패키지 테스트
colcon test --packages-select ugv_bridge
```

---

## 문의

프로젝트 관련 문의는 GitHub Issues를 통해 등록해 주세요.
