# 팀 업무 분담

UGV RoArm 프로젝트의 업무 영역 정의와 담당자 배정 문서입니다.

---

## 업무 영역 개요

### ugv_ws (메인 워크스페이스)

| 영역 | 패키지 | 주요 업무 | 담당자 |
|------|--------|----------|--------|
| 하드웨어 통신 | `ugv_base_node` | 모터/센서 드라이버, 시리얼 통신 | TBD |
| 브릿지 서버 | `ugv_bridge` | FastAPI 서버, MQTT 연동, REST API | TBD |
| 시스템 런치 | `ugv_bringup` | 전체 시스템 launch, CycloneDDS 설정 | TBD |
| 로봇 모델 (기본) | `ugv_description` | 기본 UGV URDF/Xacro | TBD |
| 시뮬레이션 | `ugv_gazebo` | Gazebo 월드, 스폰 설정 | TBD |
| 인터페이스 | `ugv_interface` | 커스텀 msg/srv/action 정의 | TBD |
| 내비게이션 | `ugv_nav` | Nav2 설정, 경로 계획 | TBD |
| SLAM | `ugv_slam` | SLAM Toolbox / Cartographer 설정 | TBD |
| 유틸리티 | `ugv_tools` | 유틸리티 스크립트, 도구 | TBD |
| 비전 | `ugv_vision` | 카메라 노드, 영상 처리 | TBD |
| 웹앱 | `ugv_web_app` | 내장 웹 인터페이스 | TBD |

### ugv_roarm_description

| 영역 | 파일/디렉토리 | 주요 업무 | 담당자 |
|------|-------------|----------|--------|
| 로봇 모델 | `urdf/` | RoArm URDF/Xacro 모델링 | TBD |
| 시뮬레이션 | `launch/gazebo*.py` | Gazebo 시뮬레이션 launch | TBD |
| 내비게이션 | `config/nav2_*.yaml`, `launch/nav*.py` | Nav2 파라미터 튜닝 | TBD |
| SLAM | `config/slam*.yaml`, `launch/slam*.py` | SLAM 파라미터 튜닝 | TBD |
| 로봇팔 제어 | `config/ugv_arm_*` | ros2_control, 컨트롤러 설정 | TBD |

### ugv_dashboard

| 영역 | 파일/디렉토리 | 주요 업무 | 담당자 |
|------|-------------|----------|--------|
| 주행 제어 UI | `DriveControl.vue`, `useRobotControl.js` | 조이스틱, 속도 제어 | TBD |
| 로봇팔 UI | `ArmControl.vue` | 로봇팔 조작 인터페이스 | TBD |
| 지도/내비게이션 | `MapView.vue`, `useMap.js`, `useNavigation.js` | 지도 시각화, 목표점 설정 | TBD |
| LiDAR 시각화 | `LidarView.vue`, `useLidar.js` | LiDAR 포인트 렌더링 | TBD |
| 상태 모니터링 | `StatusPanel.vue`, `useRobotState.js` | 배터리, 연결 상태 | TBD |
| MQTT 통신 | `useMqtt.js` | MQTT 연결, 토픽 관리 | TBD |
| 로그 시스템 | `LogPanel.vue`, `useLogs.js` | 로그 수집, 이력 조회 | TBD |

---

## 영역 간 의존성

```
ugv_dashboard ──(MQTT/REST)──> ugv_bridge ──(ROS 2)──> 각 ROS 2 노드
                                                          │
ugv_roarm_description ──(URDF/TF)──> ugv_gazebo           │
                      ──(Nav2 params)──> ugv_nav ──────────┘
                      ──(SLAM params)──> ugv_slam
```

### 주요 의존성 관계

| 변경 영역 | 영향받는 영역 | 소통 방법 |
|-----------|-------------|----------|
| `ugv_interface` (msg/srv 변경) | `ugv_bridge`, `ugv_nav`, 기타 모든 노드 | 변경 전 팀 전체 공유 |
| `ugv_bridge` API 변경 | `ugv_dashboard` | 양쪽 PR 상호 참조 |
| URDF 변경 | `ugv_gazebo`, Nav2, SLAM | 시뮬레이션 재검증 필요 |
| Nav2 파라미터 변경 | 실제 로봇 주행 | 시뮬레이션에서 먼저 테스트 |
| MQTT 토픽 추가/변경 | `ugv_bridge` + `ugv_dashboard` | 양쪽 동시 PR |

---

## 소통 규칙

### 코드 변경 시

1. **단일 영역 변경**: 해당 영역 담당자가 독립적으로 진행, PR 리뷰 요청
2. **크로스 영역 변경**: 관련 담당자와 사전 협의 후 진행
3. **인터페이스 변경** (msg/srv/action, API): 팀 전체 공유 후 합의

### GitHub 활용

- **Issue**: 작업 단위로 이슈 생성, 담당자 배정
- **PR**: 이슈 번호 연결, 관련 담당자를 리뷰어로 지정
- **Discussion**: 설계 결정, 아키텍처 논의
- **Project Board**: 전체 진행 상황 추적 (선택)

### 코드 리뷰

- 자신의 영역: 변경 내용의 정확성, 테스트 검증
- 다른 영역 리뷰 시: 인터페이스 호환성, 부작용 확인
- 리뷰 요청 후 **24시간 내** 리뷰 진행 권장

---

## 담당자 배정 방법

1. 팀 구성이 확정되면 위 테이블의 `TBD`를 GitHub 사용자명으로 교체
2. 한 사람이 여러 영역을 담당할 수 있음
3. 주 담당자와 부 담당자를 지정하면 백업 가능
4. 분기별로 담당 영역 재검토
