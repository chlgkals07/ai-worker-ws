# ai_worker_manipulation — 작업 내용 정리

브랜치: `moveit-fix`

---

## 1. 전체 구조 개요

```
wrist 카메라 → PCD 저장 → GPD grasp 검출 (오프라인) → 로봇 실행
```

| 단계 | 담당 모듈 |
|------|-----------|
| 캡처 자세 이동 | `move_wrist_capture_pose` |
| PCD 저장 | perception 팀 노드 (별도 문서 참고) |
| GPD 오프라인 검증 + 필터링 | `test_gpd_wrist150.py` |
| Pick & Place 실행 | `demo_gpd_pick_place` |

---

## 2. 아키텍처 변경사항 (jiwoo/develop → moveit-fix)

### MoveItClient 업데이트 (`robot_interface/moveit_client.py`)

- **`_moveit_lift`** 추가: lift 그룹용 MoveIt2 인스턴스 (`lift_joint`, group=`lift`)
- **`move_lift(position, velocity, acceleration, timeout)`** 추가
  - lift_joint를 MoveIt move_group 액션으로 제어
  - 0.0 = 최상단, 음수 = 내리기 (단위: m)
- **`move_to_home(arm, velocity, acceleration)`** 추가
  - TODO: 실제 home joint configuration 정의 후 교체 (현재 all-zeros)
- `_wait_for_servers()` 에 lift 서버 대기 포함

### GPD 소스 패치 (`gpd/`)

| 파일 | 변경 내용 |
|------|-----------|
| `cfg/eigen_params.cfg` | 경로 수정 (`../cfg/` → `cfg/`), `num_samples` 30→200, `num_selected` 5→100, `remove_outliers` 0→1 |
| `src/gpd/grasp_detector.cpp` | 출력 형식 변경 — `clusters[i]->print()` 추가 (position, approach, binormal, axis 출력, 파싱에 필수) |

---

## 3. 빌드

### 사전 조건

```bash
cd ~/ros2_ws && colcon build --packages-select ai_worker_manipulation
source install/setup.bash
```

### GPD C++ 바이너리 빌드

`grasp_detector.cpp` 패치 후 반드시 클린 빌드 필요:

```bash
cd ~/ros2_ws/src/ai_worker/gpd
rm -rf build && mkdir build && cd build
cmake .. && make -j$(nproc)
ls detect_grasps   # 바이너리 확인
```

---

## 4. 테스트 절차 (오늘 기준 파이프라인)

### Step 1 — 캡처 자세 이동

```bash
ros2 run ai_worker_manipulation move_wrist_capture_pose
```

옵션:

| 옵션 | 기본값 | 설명 |
|------|--------|------|
| `--lift` | `0.0` | lift_joint 목표값 (m), 음수=내리기 |
| `--x/y/z` | `0.35/-0.20/1.20` | end-effector 위치 (base_link 기준) |
| `--settle` | `2.0` | 캡처 전 대기 시간 (s) |
| `--skip-home` | False | home 이동 생략 |
| `--keep-gripper` | False | 그리퍼 열기 생략 |

---

### Step 2 — PCD 저장

perception 팀 절차 실행 → PCD 파일이 아래 구조로 저장됨:

```
<BASE_DIR>/
  ├── mask_cloud/   # frame_000.pcd ~ (base_link 기준, 단위: m)
  ├── rgb/          # 대응 RGB 이미지
  └── pose/target_pose.csv  # ground truth (있으면 시각화에 노란 구 표시)
```

> perception 팀 상세 절차: Notion 문서 참고

---

### Step 3 — GPD 오프라인 실행 + 시각화

`test_gpd_wrist150.py` 상단 `BASE_DIR`을 Step 2에서 저장된 경로로 수정:

```python
# test_gpd_wrist150.py 28번째 줄
BASE_DIR = "/root/ros2_ws/src/ai_worker/wrist_outputs_20/<새_디렉토리명>"
```

실행:

```bash
# Open3D 시각화 (DISPLAY 필요)
DISPLAY=:0 python3 ~/ros2_ws/src/ai_worker/ai_worker_manipulation/ai_worker_manipulation/tests/test_gpd_wrist150.py --frame 0

# sequential 모드 (N키로 다음 프레임)
DISPLAY=:0 python3 ... test_gpd_wrist150.py --mode sequential

# GPD 없이 포인트 클라우드만 확인
DISPLAY=:0 python3 ... test_gpd_wrist150.py --frame 0 --no-gpd
```

시각화에서 확인할 것:
- 좌표축 (RGB = XYZ): GPD grasp pose
- 노란 구: ground truth target position
- 터미널 출력에서 position, orientation 값 복사

필터 조건 (자동 적용):
1. approach 벡터와 카메라 방향 cosine similarity ≥ 0.3
2. approach X ≥ 0 (로봇 전방 진입 방향)
3. grasp position이 ground truth 중심에서 7cm 이내
4. score 내림차순 상위 8개

---

### Step 4 — Pick & Place 실행

터미널 출력에서 나온 grasp 값을 `demo_gpd_pick_place.py` 상단에 입력:

```python
# demo_gpd_pick_place.py 상단
GRASP_POSITION    = [x, y, z]          # test_gpd_wrist150 출력값
GRASP_ORIENTATION = [qx, qy, qz, qw]   # test_gpd_wrist150 출력값

PLACE_POSITION    = [x, y, z]          # 실제 place 위치 (직접 설정)
PLACE_ORIENTATION = [0.0, 0.0, 0.0, 1.0]
```

실행:

```bash
ros2 run ai_worker_manipulation demo_gpd_pick_place

# 캡처 자세에서 바로 시작 (home 이동 생략)
ros2 run ai_worker_manipulation demo_gpd_pick_place -- --skip-home
```

Enter를 누를 때마다 다음 단계로 진행:

```
1. [Enter] open gripper + home
2. [Enter] lift → 0.0 m  +  arm → pre-grasp (grasp_z + 10cm)
3. [Enter] lift descend → -0.10 m  (grasp 높이 도달)
4. [Enter] close gripper
5. [Enter] lift ascend → 0.0 m
6. [Enter] arm → place position
7. [Enter] open gripper (release)
8. [Enter] home
```

---

## 5. 테스트 스크립트 목록

| 스크립트 | 실행 방법 | 역할 |
|----------|-----------|------|
| `move_wrist_capture_pose` | `ros2 run` | 캡처 자세 이동 |
| `demo_gpd_grasp` | `ros2 run` | 하드코딩 grasp로 lift 접근 단독 테스트 |
| `test_gpd_wrist150` | `python3` 직접 실행 (ROS 불필요) | 오프라인 GPD + Open3D 시각화 |
| `demo_gpd_pick_place` | `ros2 run` | GPD 결과 하드코딩 후 pick & place 실행 |
| `gpd_wrist` | `ros2 run` | 라이브 GPD 노드 (추후 연동용) |

---

## 6. 향후 작업

- [ ] `move_to_home()` 실제 home joint configuration 정의
- [ ] `PLACE_POSITION` 실측 후 고정값 등록
- [ ] `demo_gpd_pick_place` → `gpd_wrist` 노드와 연동해 라이브 pick & place 완성
- [ ] `pick_and_place.py` 리팩토링 (현재 broken — `check_reachable`, `cartesian_move` 등 수정 필요)
- [ ] grasp assessment 연동 (`GraspSkill` 활용)
