# 브랜치 전략 및 컨벤션

UGV RoArm 프로젝트의 모든 리포지토리에 공통 적용되는 브랜치 전략과 컨벤션을 정의합니다.

---

## 리포지토리별 기본 브랜치

| 리포지토리 | 기본 브랜치 | 설명 |
|------------|-------------|------|
| `ugv_ws` | `ros2-humble-develop` | ROS 2 워크스페이스 |
| `ugv_roarm_description` | `main` | URDF/Xacro, Launch, Nav2/SLAM 설정 |
| `ugv_dashboard` | `main` | Vue 3 웹 대시보드 |

---

## 브랜치 모델

기여자(팀원)별 브랜치를 생성하고, 기본 브랜치로 PR을 보내는 단순한 모델을 사용합니다.

```
main (또는 ros2-humble-develop)  ← 보호된 기본 브랜치
 ├── jeonghun/lidar-filter       ← 기여자별 작업 브랜치
 ├── minsoo/nav2-tuning
 └── jiyeon/dashboard-chart
```

### 규칙

- 기본 브랜치에 직접 push 금지 (Branch Ruleset으로 보호)
- 모든 변경은 **작업 브랜치 → 기본 브랜치 PR**을 통해 머지
- PR 머지 후 작업 브랜치는 삭제

### 브랜치 네이밍 컨벤션

```
<이름>/<간단한-설명>
```

**예시:**
- `jeonghun/add-lidar-filter`
- `minsoo/fix-nav2-param`
- `jiyeon/update-map-view`

**규칙:**
- 소문자만 사용
- 단어 구분은 하이픈(`-`) 사용
- `<이름>`은 GitHub 사용자명 또는 이름 (팀 내 합의)
- 설명은 영어로 간결하게 (3~5 단어)

---

## 커밋 메시지 컨벤션

[Conventional Commits](https://www.conventionalcommits.org/) 스타일을 따릅니다. **커밋 메시지는 영어로 작성합니다.**

### 형식

```
<type>(<scope>): <subject>

<body>       (선택)

<footer>     (선택)
```

### Type

| Type | 설명 | 예시 |
|------|------|------|
| `feat` | 새로운 기능 추가 | `feat(nav): add obstacle avoidance` |
| `fix` | 버그 수정 | `fix(bridge): resolve MQTT reconnect issue` |
| `docs` | 문서 변경 | `docs: update CONTRIBUTING.md` |
| `refactor` | 기능 변경 없는 코드 리팩토링 | `refactor(slam): simplify map update logic` |
| `test` | 테스트 추가/수정 | `test(nav): add waypoint following test` |
| `chore` | 빌드, 설정 등 기타 변경 | `chore: update colcon build flags` |
| `style` | 코드 포맷팅 (동작 변경 없음) | `style: fix indentation in launch files` |
| `perf` | 성능 개선 | `perf(vision): optimize image processing pipeline` |

### Scope (선택)

리포지토리 내 패키지명 또는 주요 모듈명을 사용합니다.

- `ugv_ws`: `bridge`, `nav`, `slam`, `gazebo`, `bringup`, `description`, `vision` 등
- `ugv_roarm_description`: `urdf`, `launch`, `nav2`, `slam`, `gazebo`
- `ugv_dashboard`: `stomp`, `components`, `router`, `config`

### 예시

```bash
feat(bridge): add battery status endpoint

fix(nav2): correct DWB planner velocity limits

docs: add development setup guide
```

### 주의사항

- Subject는 50자 이내, 명령형 현재시제로 작성 (예: `add`, `fix`, `update`)
- Subject 끝에 마침표(`.`) 사용하지 않음
- Body는 72자에서 줄바꿈
- Breaking change가 있으면 footer에 `BREAKING CHANGE:` 추가

---

## Pull Request 규칙

### PR 작성 가이드

1. PR 제목은 커밋 메시지 컨벤션과 동일한 형식 사용
2. PR 본문에 변경 사항을 명확히 설명
3. 관련 이슈 번호를 `Closes #이슈번호`로 연결

### 리뷰 요구사항

| 머지 대상 | 최소 리뷰어 수 | 비고 |
|-----------|---------------|------|
| 작업 브랜치 → 기본 브랜치 | 1명 | 같은 팀 또는 상대 팀 |
| 크로스 영역 (`ugv_bridge` 등) | 1명 (양팀 각) | CODEOWNERS 자동 배정 |

### 머지 전 체크리스트

- [ ] 코드 리뷰 승인 완료
- [ ] 빌드 성공 확인 (`colcon build` 또는 `npm run build`)
- [ ] 관련 테스트 통과
- [ ] 충돌(conflict) 해결 완료
- [ ] 커밋 메시지 컨벤션 준수

---

## Branch Ruleset 설정 가이드

> GitHub의 **Branch Ruleset** (Settings → Rules → Rulesets)을 사용합니다.
> Classic Branch Protection Rule은 레거시이므로 사용하지 않습니다.

### 설정 경로

`Settings` → `Rules` → `Rulesets` → `New ruleset` → `New branch ruleset`

### 권장 설정

| 항목 | 설정값 |
|------|--------|
| Ruleset name | `protect-main` (또는 `protect-ros2-humble-develop`) |
| Enforcement status | **Active** |
| Target branches | `Default branch` 선택 |
| **Restrict creations** | OFF (팀원이 브랜치 생성 가능하도록) |
| **Restrict deletions** | ON (기본 브랜치 삭제 방지) |
| **Block force pushes** | ON |
| **Require a pull request before merging** | ON |
| → Required approvals | `1` |
| → Dismiss stale reviews on push | ON |
| → Require review from Code Owners | ON (CODEOWNERS 활용 시) |
| **Require status checks to pass** | ON (CI 추가 후) |

### Bypass 설정

Ruleset 상단의 **Bypass list**에서 리포지토리 관리자(admin)를 추가하면, 긴급 시 직접 push가 가능합니다. 일반적으로는 비워두는 것을 권장합니다.

### 설정 절차

1. GitHub 리포지토리 → `Settings` → `Rules` → `Rulesets`
2. `New ruleset` → `New branch ruleset` 클릭
3. Ruleset name 입력
4. Target branches → `Add target` → `Include default branch`
5. 위 표의 Rules 항목들을 체크
6. `Create` 클릭

---

## 워크플로우 요약

```
1. 이슈 생성 (GitHub Issues)

2. 기본 브랜치에서 작업 브랜치 생성
   $ git checkout main                          # (ugv_ws는 ros2-humble-develop)
   $ git pull origin main
   $ git checkout -b jeonghun/add-lidar-filter

3. 작업 후 커밋
   $ git add <files>
   $ git commit -m "feat(nav): add lidar noise filter"

4. 원격에 push
   $ git push -u origin jeonghun/add-lidar-filter

5. GitHub에서 PR 생성 (작업 브랜치 → main)
6. 코드 리뷰 (최소 1명) 후 머지
7. 머지 완료 후 작업 브랜치 삭제
```
