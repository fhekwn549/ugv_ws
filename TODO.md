# UGV RoArm 프로젝트 TODO

## 완료된 작업

### 기본 인프라
- [x] WSL ↔ RPi rosbridge 통신 구축
- [x] rosbridge → CycloneDDS 직접 통신으로 전환
- [x] CycloneDDS MaxAutoParticipantIndex 설정 (Nav2 다수 노드 지원)

### 원격 제어 / 시각화
- [x] 원격 키보드 텔레옵 (주행 + 팔 + 그리퍼 동시 제어)
- [x] RViz 원격 시각화 (LiDAR, 로봇 모델, 오도메트리)
- [x] 그리퍼 방향/범위 RViz ↔ 실제 로봇 일치
- [x] 팔 움직일 때 그리퍼 토크 유지 (T:102 hand 필드)
- [x] teleop_all.py: 시작 시 현재 자세 동기화 + 실시간 피드백 표시

### 센서 / 오도메트리
- [x] base_node (cmd_vel dead reckoning) → rf2o_laser_odometry 전환
- [x] RPi rf2o_laser_odometry 빌드 완료 (스왑 2GB 추가)

### SLAM
- [x] slam_toolbox → Cartographer 전환 (launch + config)
- [x] Cartographer SLAM 실전 테스트 — 맵 생성 성공 (lab_map)
- [x] 맵 편집 (유리벽 너머 불필요 영역 제거)

### Nav2 자율주행
- [x] Nav2 설치 및 자율주행 기본 테스트
- [x] Nav2 파라미터 튜닝 (모터 데드밴드 대응, AMCL 최적화)
- [x] velocity_smoother 제거 (모터 데드밴드가 높아 점진적 가속 불가)
- [x] nav_view.rviz QoS 수정 (맵 표시 Transient Local)

### 웹 대시보드 + MQTT 브릿지
- [x] ugv_bridge 구현 (FastAPI + paho-mqtt + Nav2 Simple Commander + SQLite)
- [x] REST API 엔드포인트 구현 (navigate, cancel, initial_pose, map, arm, gripper, logs)
- [x] MQTT 실시간 발행 (pose, map_pose, voltage, joint_states, scan, path, nav_status)
- [x] 웹 프론트엔드 구현 (Vue 3 + MQTT.js + Canvas 맵/LiDAR 렌더링)
- [x] LiDAR 2D 시각화 (Canvas 렌더링, 줌 지원)
- [x] 웹 네비게이션 시각화 — 목표 마커, Shift+드래그 방향 지정, 경로 오버레이, 상태 패널
- [x] nav_status 주기적 발행 (navigating 중 1Hz) + path 스팸 수정

### 메시지 브로커 마이그레이션
- [x] Mosquitto → RabbitMQ 전환 (MQTT + STOMP + Web STOMP 동시 지원)
- [x] RabbitMQ 설정 파일 추가 (rabbitmq.conf, enabled_plugins)
- [x] Dashboard: MQTT.js → @stomp/stompjs 전환 (useStomp.js + shim)
- [x] STOMP 토픽 자동 변환 (ugv01/pose → /topic/ugv01.pose)

### Gazebo 시뮬레이션 (디지털 트윈)
- [x] SLAM 맵 → Gazebo 월드 변환 스크립트 (map_to_gazebo_world.py)
- [x] Gazebo + Nav2 통합 런치 (gazebo_nav.launch.py)
- [x] 시뮬레이션 물리 튜닝 (바퀴 마찰, contact velocity, step size)
- [x] Nav2 시뮬레이션 파라미터 튜닝 (RPP controller, costmap inflation)
- [x] sim_pose_bridge.py (RViz 2D Pose Estimate → Gazebo 텔레포트)

### 문서화
- [x] 발표자료 PPT 작성 (16슬라이드, 2개 합본)
- [x] README 전면 업데이트 (ugv_roarm_description, ugv_dashboard, ugv_ws)

### 경량 Nav2 시뮬레이션 (nav_sim)
- [x] Gazebo 제거한 경량 시뮬레이션 환경 구축 (fake_odom + fake_scan + Nav2)
- [x] fake_odom_node.py (cmd_vel → 2D 운동학 적분 → odom + TF)
- [x] fake_scan_node.py (맵 기반 360도 레이캐스팅 → /scan + /dev/shm)
- [x] nav_sim.launch.py (robot_state_publisher + Nav2 + RViz 통합 런치)
- [x] nav2_params_fake.yaml (경량 시뮬레이션 전용 Nav2 파라미터)
- [x] ugv_roarm.xacro use_gazebo 조건부 추가 (Gazebo 플러그인 선택적 포함)

### WSL2 CycloneDDS LiDAR 바이패스
- [x] /dev/shm 파일 기반 IPC로 DDS 우회 (fake_scan_node → ugv_bridge)
- [x] ugv_bridge ros_interface.py: /dev/shm 폴링 + DDS fallback (실제 로봇 대응)
- [x] 웹 대시보드 LiDAR 실시간 업데이트 확인

### 팀 업무 분담 환경 구축
- [x] 리포별 업무 영역 정의 및 담당자 분배 기준 작성 (TEAM_ROLES.md)
- [x] 각 리포에 CONTRIBUTING.md 작성 (빌드 방법, 브랜치 전략, PR 규칙)
- [x] GitHub CODEOWNERS 설정 (ugv_ws, ugv_roarm_description, ugv_dashboard)
- [x] PR/Issue 템플릿 작성 (.github/PULL_REQUEST_TEMPLATE.md, ISSUE_TEMPLATE/)
- [ ] GitHub branch protection 규칙 설정 (main/develop 브랜치 보호)
- [ ] 팀원별 개발 환경 세팅 가이드 (WSL + RPi + CycloneDDS)

#### 업무 분담 기준안

| 영역 | 리포 | 주요 작업 |
|------|------|----------|
| **하드웨어/드라이버** | ugv_ws | 모터 드라이버, 센서 인터페이스, 시리얼 통신 |
| **네비게이션/SLAM** | ugv_ws (ugv_roarm_description) | Nav2 튜닝, 맵 관리, Cartographer 설정 |
| **로봇 모델/시뮬레이션** | ugv_ws (ugv_roarm_description) | URDF, nav_sim/Gazebo, ros2_control |
| **웹 프론트엔드** | ugv_dashboard | Vue 3 UI, Canvas 시각화, STOMP 구독 |
| **웹 백엔드/브릿지** | ugv_ws (ugv_bridge) | FastAPI, MQTT 발행, Nav2 연동, DB |
| **미션/태스크 관리** | 신규 리포 | Spring Boot 백엔드, 웨이포인트 미션 |

---

## 3/4 (화) 이후: 자율주행 정밀화

### 자율주행 추가 튜닝

- [ ] RPi에 ugv_bridge 최신 버전 배포 및 실 로봇 네비게이션 테스트
- [ ] 실 로봇에서 Nav2 경로 추종 정밀도 검증 (RPP 파라미터 실환경 확인)
- [ ] 웹 대시보드에서 Cancel Navigation 동작 안정성 개선
- [ ] Cartographer localization 모드에서 pbstream 경로 파라미터화 테스트

---

## 향후 계획

### 단기 (1~2주)
- [ ] 웨이포인트 기반 순회 미션 구현
- [ ] Spring Boot 백엔드 + MQTT (미션/태스크 관리)
- [ ] 모바일 매니퓰레이션 기본 시나리오 (주행 → 팔 제어 → 주행)

### 중기 (3~4주)
- [ ] 다중 로봇 지원 (MQTT namespace 기반 로봇 간 위치 공유)
- [ ] 로봇 간 교착 방지 로직 (shared costmap 또는 fleet manager)
- [ ] 카메라 영상 스트리밍 + 객체 인식 연동

---

## 환경 정보

| 항목 | 값 |
|------|-----|
| RPi IP | `pi@192.168.0.71` |
| 통신 | CycloneDDS (rosbridge 제거됨) |
| `/dev/ttyAMA0` | UGV 바퀴 ESP32 |
| `/dev/ttyUSB0` | RoArm-M2 ESP32 |
| `/dev/ttyUSB1` | LDLidar STL-19P |

## 리포 구조

| 리포 | 역할 | branch |
|------|------|--------|
| [ugv_ws](https://github.com/fhekwn549/ugv_ws) | 하드웨어 드라이버 + ugv_bridge + ugv_roarm_description (URDF, Nav2, 시뮬레이션) | `ros2-humble-develop` |
| [ugv_dashboard](https://github.com/fhekwn549/ugv_dashboard) | 웹 대시보드 (Vue 3 + STOMP/WS) | `main` |
