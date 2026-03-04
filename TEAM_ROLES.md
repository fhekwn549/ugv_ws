# 팀 업무 분담

UGV RoArm 프로젝트의 업무 영역 정의와 담당자 배정 문서입니다.

---

## 팀 구성

### Team A: 비전 & UI

| 담당자 | GitHub ID | 역할 |
|--------|-----------|------|
| TBD | `@TBD` | TBD |
| TBD | `@TBD` | TBD |

### Team B: HW 제어 & 자율주행

| 담당자 | GitHub ID | 역할 |
|--------|-----------|------|
| TBD | `@TBD` | TBD |
| TBD | `@TBD` | TBD |

---

## 팀별 담당 영역

### ugv_ws (메인 워크스페이스)

| 패키지 | 주요 업무 | 담당 팀 |
|--------|----------|---------|
| `ugv_vision` | 카메라 노드, 영상 처리 | **A** (비전 & UI) |
| `ugv_bridge` | FastAPI 서버, MQTT 연동, REST API | **A + B** (공동) |
| `ugv_web_app` | 내장 웹 인터페이스 | **A** (비전 & UI) |
| `ugv_base_node` | 모터/센서 드라이버, 시리얼 통신 | **B** (HW & 자율주행) |
| `ugv_nav` | Nav2 설정, 경로 계획 | **B** (HW & 자율주행) |
| `ugv_slam` | SLAM Toolbox / Cartographer 설정 | **B** (HW & 자율주행) |
| `ugv_bringup` | 전체 시스템 launch, CycloneDDS 설정 | **B** (HW & 자율주행) |
| `ugv_description` | 기본 UGV URDF/Xacro | **B** (HW & 자율주행) |
| `ugv_gazebo` | Gazebo 월드, 스폰 설정 | **B** (HW & 자율주행) |
| `ugv_roarm_description` | RoArm URDF, Nav2/SLAM, nav_sim/Gazebo 시뮬레이션, ros2_control | **B** (HW & 자율주행) |
| `ugv_interface` | 커스텀 msg/srv/action 정의 | **공동** (변경 시 양팀 합의) |
| `ugv_tools` | 유틸리티 스크립트, 도구 | **공동** |

### ugv_dashboard (별도 리포)

| 영역 | 담당 팀 |
|------|---------|
| Vue 컴포넌트 / UI | **A** (비전 & UI) |
| MQTT / API 통신 | **A** (비전 & UI) |
| 지도/내비게이션 시각화 | **A** (비전 & UI) |

---

## 리포별 CODEOWNERS 요약

> 각 리포의 `.github/CODEOWNERS` 파일로 PR 리뷰어가 자동 배정됩니다.

| 리포 | 주 담당 팀 | 비고 |
|------|-----------|------|
| `ugv_ws` | 경로별로 A 또는 B | CODEOWNERS에 패키지별 팀 지정 (ugv_roarm_description 포함) |
| `ugv_dashboard` | **A** | 전체 A팀 관할 |

---

## 팀 간 접점 (크로스 영역)

```
Team A (비전 & UI)              Team B (HW & 자율주행)
─────────────────              ──────────────────────
ugv_vision                     ugv_base_node
ugv_web_app                    ugv_nav / ugv_slam
ugv_dashboard                  ugv_bringup
        │                      ugv_roarm_description
        │                      ugv_description
        │                              │
        └──── ugv_bridge ──────────────┘
              (공동 관할)

              ugv_interface
              (변경 시 양팀 합의)
```

### 크로스 영역 변경 규칙

| 변경 영역 | 필요 조치 |
|-----------|----------|
| `ugv_bridge` | 변경하는 팀이 PR 생성, **상대 팀에서 반드시 리뷰** |
| `ugv_interface` (msg/srv) | 양팀 합의 후 변경, **양팀 각 1명 리뷰 필수** |
| MQTT/STOMP 토픽 추가/변경 | `ugv_bridge` + `ugv_dashboard` 동시 PR, 상호 링크 |
| API 엔드포인트 추가/변경 | `ugv_bridge` + `ugv_dashboard` 동시 PR, 상호 링크 |

---

## 소통 규칙

### 팀 내부

- 같은 팀 내에서는 자유롭게 PR 생성 및 리뷰
- 최소 1명 리뷰 후 머지

### 팀 간

- 크로스 영역 변경 시 상대 팀에 사전 공유 (Issue 또는 Discussion)
- 인터페이스(msg/srv/action) 변경은 **반드시 양팀 합의** 후 진행
- 리뷰 요청 후 **24시간 내** 리뷰 진행 권장

### GitHub 활용

- **Issue**: 작업 단위로 이슈 생성, 담당 팀/담당자 배정
- **PR**: 이슈 번호 연결, CODEOWNERS에 의해 자동 리뷰어 배정
- **Discussion**: 설계 결정, 아키텍처 논의 (특히 크로스 영역)

---

## 담당자 배정 방법

1. 팀 구성이 확정되면 위 테이블의 `TBD`를 GitHub 사용자명으로 교체
2. `.github/CODEOWNERS` 파일의 팀명도 함께 업데이트
3. 한 사람이 여러 패키지를 담당할 수 있음
4. 분기별로 담당 영역 재검토
