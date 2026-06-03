# ai_worker_manipulation — 작업 내용 정리

브랜치: `jiwoo/develop`

---

## 1. 전체 구조 개요

```
wrist 카메라 → PCD 캡처 → GPD grasp 검출 → 로봇 실행
```

| 단계 | 담당 모듈 |
|------|-----------|
| 캡처 자세 이동 | `move_wrist_capture_pose` |
| grasp 실행 (테스트용) | `demo_gpd_grasp` |
|pcd data gpd 테스트| `test_gpd_wrist150.py` |
| 전체 pick 시퀀스 | `skill_primitives/pick_skill.py` |

---

### 테스트 스크립트
| 파일 | 역할 |
|------|------|
| `tests/move_wrist_capture_pose.py` | 손목 PCD 캡처 자세로 이동 |
| `tests/demo_gpd_grasp.py` | 하드코딩 grasp pose 실행 테스트 |

---

## 3. 실행 방법

### 사전 조건
```bash
cd ~/ros2_ws && colcon build --packages-select ai_worker_manipulation
source install/setup.bash
```

---

### 3-1. 손목 PCD 캡처 자세로 이동

```bash
# 기본 자세 (lift=0, ee=(0.35, -0.20, 1.20))
ros2 run ai_worker_manipulation move_wrist_capture_pose

# 위치 직접 지정
ros2 run ai_worker_manipulation move_wrist_capture_pose -- --x 0.45 --y -0.2 --z 1.2

# 리프트 내리고 정착 시간 3초
ros2 run ai_worker_manipulation move_wrist_capture_pose -- --lift -0.15 --settle 3.0

# home 이동 생략 (이미 근처에 있을 때)
ros2 run ai_worker_manipulation move_wrist_capture_pose -- --skip-home
```

| 옵션 | 기본값 | 설명 |
|------|--------|------|
| `--lift` | `0.0` | lift_joint 목표값 (m), 음수=내리기 |
| `--x/y/z` | `0.35/-0.20/1.20` | 엔드이펙터 위치 (base_link 기준) |
| `--settle` | `2.0` | 캡처 전 대기 시간 (s) |
| `--skip-home` | False | home 이동 생략 |
| `--keep-gripper` | False | 그리퍼 열기 생략 |

---

### 3-2. test_gpd_wrist150.py — 오프라인 GPD 검증 + Open3D 시각화

저장된 PCD 데이터셋(`wrist_outputs_150`)으로 GPD 결과를 오프라인에서 확인하는 스크립트입니다.
ROS 없이 단독 실행 가능합니다.

**다른 데이터로 테스트하려면** `test_gpd_wrist150.py` 28번째 줄 `BASE_DIR`을 변경:
```python
# 기존
BASE_DIR = "/root/ros2_ws/src/ai_worker/wrist_outputs_20/wrist_outputs_150_20260527_100328"

# 새 데이터 경로로 교체
BASE_DIR = "/path/to/your/new_dataset"
```

데이터 디렉토리 구조는 아래를 따라야 합니다:
```
<BASE_DIR>/
  ├── mask_cloud/   # frame_000.pcd ~ (base_link 기준, 단위: m)
  ├── rgb/          # 대응 RGB 이미지 (없어도 동작)
  └── pose/target_pose.csv  # ground truth (없으면 노란 구 미표시)
```

**실행 방법:**

Open3D 시각화를 위해 `DISPLAY=:0` 필요합니다.

```bash
SCRIPT=/root/ros2_ws/src/ai_worker/ai_worker_manipulation/ai_worker_manipulation/tests/test_gpd_wrist150.py

# 단일 프레임 (frame 0)
DISPLAY=:0 python3 $SCRIPT --frame 0

# 특정 프레임 지정
DISPLAY=:0 python3 $SCRIPT --frame 42

# sequential 모드 (N키로 다음 프레임)
DISPLAY=:0 python3 $SCRIPT --mode sequential

# GPD 없이 포인트 클라우드만 시각화
DISPLAY=:0 python3 $SCRIPT --frame 0 --no-gpd
```

| 옵션 | 기본값 | 설명 |
|------|--------|------|
| `--frame` | `0` | 표시할 프레임 인덱스 (0~149) |
| `--mode` | `single` | `single` 또는 `sequential` |
| `--no-gpd` | False | GPD 실행 없이 PCD만 시각화 |

**시각화 내용:**
- 포인트 클라우드 (물체 마스크 영역)
- GPD grasp pose (RGB 좌표축으로 표시)
- 노란 구: ground truth target position (`target_pose.csv`)

**필터 조건 (자동 적용):**
1. approach 벡터와 카메라 방향 cosine similarity ≥ 0.3
2. approach X ≥ 0 (로봇 전방 진입 방향)
3. grasp position이 ground truth 중심에서 7cm 이내
4. score 내림차순 상위 8개만 표시

-> 해당 threshold 값들은 변경가능


**시각화 조작:**
- 마우스 드래그: 회전 / 스크롤: 줌 / `Q`: 종료
- sequential 모드: `N` 또는 `n` → 다음 프레임



### 3-4. Demo GPD Grasp 실행 (단독 테스트)

하드코딩된 grasp pose로 파지 동작을 테스트합니다.

```bash
# home에서 출발
ros2 run ai_worker_manipulation demo_gpd_grasp

# 캡처 자세에서 바로 grasp (home 이동 생략)
ros2 run ai_worker_manipulation demo_gpd_grasp -- --skip-home
```

**실행 순서:**
1. 그리퍼 열기
2. (옵션) home으로 이동
3. lift를 `LIFT_POSITION`으로 내리기
4. **pre-grasp 이동**: grasp 목표 위 10cm(`APPROACH_HEIGHT`)로 arm 이동
5. **lift 추가 하강**: lift를 10cm 더 내려 목표 z에 도달 (충돌 방지)
6. 그리퍼 닫기 → 1.5초 대기
7. lift 복귀 → home 복귀

**주요 상수** (`demo_gpd_grasp.py` 상단에서 수정):
```python
GRASP_POSITION    = [0.3014, -0.2542, 0.8930]   # 목표 grasp 위치 (base_link, m)
GRASP_ORIENTATION = [0.3846, -0.0636, 0.0507, 0.9195]  # 쿼터니언 [x,y,z,w]
LIFT_POSITION     = 0.0   # lift 초기 내림 (m)
APPROACH_HEIGHT   = 0.10    # pre-grasp 오프셋 - 물체 위 +z 방향 값 조절 가능 (m)
```
----> gpd가 반환하는 값들로 교체 가능


## 5. GPD 빌드 확인

GPD C++ 바이너리가 필요합니다.

```bash
cd ~/ros2_ws/src/ai_worker/gpd
mkdir -p build && cd build
cmake .. && make -j$(nproc)
ls detect_grasps   # 바이너리 확인
```

변경된 GPD 파일:
- `gpd/CMakeLists.txt`
- `gpd/cfg/eigen_params.cfg` (camera_position 등 파라미터)
- `gpd/src/gpd/grasp_detector.cpp`

---

## 6. 사용 흐름

```bash
# 터미널 1: 캡처 자세로 이동 (PCD 촬영 준비)
ros2 run ai_worker_manipulation move_wrist_capture_pose

# 터미널 3: 캡처 자세에서 바로 grasp 실행
ros2 run ai_worker_manipulation demo_gpd_grasp -- --skip-home
```
