# 브랜치 전략 및 컨벤션

UGV RoArm 프로젝트의 모든 리포지토리에 공통 적용되는 브랜치 전략과 컨벤션을 정의합니다.

---

## 리포지토리별 기본 브랜치

| 리포지토리 | 기본 브랜치 | 설명 |
|------------|-------------|------|
| `ugv_ws` | `ros2-humble-develop` | ROS 2 워크스페이스 (upstream fork) |
| `ugv_roarm_description` | `main` | URDF/Xacro, Launch, Nav2/SLAM 설정 |
| `ugv_dashboard` | `main` | Vue 3 웹 대시보드 |

---

## 브랜치 모델

```
main (또는 ros2-humble-develop)
 └── develop
      ├── feature/*
      ├── bugfix/*
      └── hotfix/*
```

### 브랜치 유형

| 브랜치 | 분기 원본 | 머지 대상 | 용도 |
|--------|-----------|-----------|------|
| `main` / `ros2-humble-develop` | - | - | 안정 릴리스 브랜치 |
| `develop` | `main` | `main` | 개발 통합 브랜치 |
| `feature/*` | `develop` | `develop` | 새 기능 개발 |
| `bugfix/*` | `develop` | `develop` | 버그 수정 |
| `hotfix/*` | `main` | `main` + `develop` | 긴급 수정 |

### 브랜치 네이밍 컨벤션

```
feature/<이슈번호>-<간단한-설명>
bugfix/<이슈번호>-<간단한-설명>
hotfix/<이슈번호>-<간단한-설명>
```

**예시:**
- `feature/12-add-lidar-filter`
- `bugfix/35-fix-nav2-param-loading`
- `hotfix/42-fix-mqtt-connection-crash`

**규칙:**
- 소문자만 사용
- 단어 구분은 하이픈(`-`) 사용
- 이슈 번호를 반드시 포함
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
- `ugv_dashboard`: `mqtt`, `components`, `router`, `config`

### 예시

```bash
feat(bridge): add battery status endpoint

fix(nav2): correct DWB planner velocity limits

docs: add development setup guide

refactor(slam): extract map processor into separate module

test(gazebo): add spawn entity integration test

chore(ci): update GitHub Actions workflow
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
| `feature/*` → `develop` | 1명 | 일반 기능/버그 |
| `develop` → `main` | 2명 | 릴리스 통합 |
| `hotfix/*` → `main` | 1명 | 긴급 수정 |

### 머지 전 체크리스트

- [ ] 코드 리뷰 승인 완료
- [ ] 빌드 성공 확인 (`colcon build` 또는 `npm run build`)
- [ ] 관련 테스트 통과
- [ ] 충돌(conflict) 해결 완료
- [ ] 커밋 메시지 컨벤션 준수

---

## Branch Protection 설정 가이드

> 아래는 GitHub 웹 UI에서 설정하는 방법입니다. 리포지토리 관리자가 설정해 주세요.

### 설정 경로

`Settings` → `Branches` → `Add branch protection rule`

### 권장 설정 (main / ros2-humble-develop)

| 항목 | 설정값 |
|------|--------|
| Branch name pattern | `main` 또는 `ros2-humble-develop` |
| Require a pull request before merging | **ON** |
| Required approving reviews | `1` (develop→main은 `2`) |
| Dismiss stale pull request approvals | **ON** |
| Require status checks to pass | **ON** (CI가 추가된 후) |
| Require branches to be up to date | **ON** |
| Include administrators | **ON** |
| Allow force pushes | **OFF** |
| Allow deletions | **OFF** |

### 설정 절차

1. GitHub 리포지토리 → `Settings` → `Branches`
2. `Add rule` 클릭
3. `Branch name pattern`에 보호할 브랜치명 입력
4. 위 표의 항목들을 체크
5. `Create` 클릭

### develop 브랜치

`develop` 브랜치에도 동일한 보호 규칙을 적용하되, 리뷰어 수를 `1`명으로 설정합니다.

---

## 워크플로우 요약

```
1. 이슈 생성 (GitHub Issues)
2. develop에서 feature 브랜치 생성
   $ git checkout develop
   $ git pull origin develop
   $ git checkout -b feature/12-add-lidar-filter

3. 작업 후 커밋
   $ git add <files>
   $ git commit -m "feat(nav): add lidar noise filter"

4. 원격에 push
   $ git push origin feature/12-add-lidar-filter

5. GitHub에서 PR 생성 (feature → develop)
6. 코드 리뷰 후 머지
7. 릴리스 시 develop → main PR 생성 및 머지
```
