# Pick and Place End-to-End Design
**Date:** 2026-06-05
**Branch:** moveit-fix
**Status:** Proof of concept — values and sequences subject to change

---

## Context

This is the full end-to-end pick and place pipeline for the 2026 Humanoid Challenge.
Robot: FFW SG2 (ROBOTIS). Stack: ROS2 Jazzy + MoveIt2 + pymoveit2.

**This is not the final version.** Calibration offsets and tuned values will differ
between simulation and real hardware. All tunable values must live in config yaml.

---

## Architecture — 3 Layers

```
Mission Layer
  └── calls PickAndPlace ROS Action Server (Goal: object_class + place_pose)

nodes/pick_and_place_server.py          ← thin ROS Action Server (~30 lines)
  └── skill_primitives/pick_and_place_orchestrator.py   ← all logic

skill_primitives/
  pick_skill.py        ← upgraded: cartesian + lift fallback
  place_skill.py       ← upgraded: cartesian + lift fallback
  grasp_skill.py       ← unchanged
  grasp_assessment.py  ← unchanged

robot_interface/
  moveit_client.py     ← unchanged
  gripper_controller.py
  tf_transformer.py

gpd_grasp_publisher.py ← GPD node: PCD → /gpd/grasp_poses
```

**Key rule:** `nodes/` → `skill_primitives/` → `robot_interface/`. No reverse deps.

---

## ROS Action Interface

**Action type:** `ai_worker_manipulation/action/PickAndPlace`

```
# Goal
string object_class          # e.g. "bottle" — used for grasp assessment LUT
geometry_msgs/PoseStamped place_pose   # where to place; sent by mission team
---
# Result
bool success
string failure_reason        # "grasp_failed" | "planning_failed" | "no_gpd_candidates" | "timeout"
---
# Feedback
string phase                 # "moving_to_capture_pose" | "waiting_for_gpd" | "picking" |
                             # "assessing_grasp" | "placing" | "returning_to_capture_pose"
```

**Note:** `place_pose` is obtained by the mission team from a prior perception action
(head camera finds bin position). Our server does not own that step.

---

## Perception Boundary

- Perception team's node publishes PCD externally.
- Mission team triggers perception separately, then calls our Action Server.
- `gpd_grasp_publisher.py` subscribes to `/perception/wrist/target_pcd/<class_name>`,
  runs GPD, publishes to `/gpd/grasp_poses` (PoseArray).
- Our Action Server waits on `/gpd/grasp_poses` after receiving a Goal.

---

## Arm Selection

1. **Y-threshold:** if `grasp_pose.position.y >= 0` → left arm, else → right arm.
2. **Reachability confirmation:** run IK check on selected arm via `moveit_client.check_reachable()`.
3. If selected arm not reachable, try the other arm.
4. If neither reachable → fail with `"no_reachable_arm"`.

Y-threshold value goes in config yaml.

---

## Pick Sequence (PickSkill)

Two modes tried in order. All offsets come from config yaml.

### Mode 1 — Cartesian (preferred)
1. Open gripper
2. Move to `pre_grasp` pose (OMPL) — grasp_pose stepped back along approach vector
3. Cartesian move to `grasp_pose`
4. Grasp + assess (`GraspSkill`)
5. On success: cartesian retract to `pre_grasp`
6. On failure: open gripper, retract to `pre_grasp` → return FAILURE (caller tries Mode 2)

### Mode 2 — Lift fallback (if cartesian fails)
1. Open gripper
2. Move lift to `LIFT_HOME` (0.0 m)
3. Move arm to `pre_grasp` pose at `grasp_z + APPROACH_HEIGHT` (OMPL)
4. Lower lift by `APPROACH_HEIGHT` → arm reaches grasp z
5. Grasp + assess (`GraspSkill`)
6. On success: raise lift to `LIFT_HOME`
7. On failure: open gripper, raise lift → return FAILURE

### Tunable values (config yaml)
```yaml
pick:
  pre_grasp_offset: 0.15      # metres along approach vector (Mode 1)
  approach_height: 0.10       # metres z offset above grasp (Mode 2)
  lift_home: 0.0              # lift_joint home position (m)
  planning_retries: 3         # same-pose retries before jitter
  planning_jitter_retries: 3  # jitter retries after same-pose fails
  planning_jitter_std: 0.01   # std dev of position jitter (m)
```

---

## Place Sequence (PlaceSkill)

Same two-mode structure as pick.

### Mode 1 — Cartesian
1. Move to `pre_place` (OMPL)
2. Cartesian move to `place_pose`
3. Open gripper
4. Cartesian retract to `pre_place`

### Mode 2 — Lift fallback
1. Move lift to `LIFT_HOME`
2. Move arm to `pre_place` at `place_z + APPROACH_HEIGHT` (OMPL)
3. Lower lift by `APPROACH_HEIGHT`
4. Open gripper
5. Raise lift to `LIFT_HOME`

---

## Planning Failure Retry Logic (shared by Pick and Place)

Applied to every OMPL move call:
1. Retry same pose up to `planning_retries` times — OMPL randomness may succeed.
2. If still failing, retry up to `planning_jitter_retries` times with small random
   perturbation (gaussian, std=`planning_jitter_std`) on goal position.
3. If all retries exhausted → return `MoveResult.FAILED`.

This logic lives in `pick_skill.py` / `place_skill.py` as a shared helper, or
optionally promoted to `moveit_client.py` if it becomes widely used.

---

## Orchestrator — PickAndPlaceOrchestrator

`skill_primitives/pick_and_place_orchestrator.py`

Pure Python class — no ROS node, no Action Server. Testable standalone.

```python
class PickAndPlaceOrchestrator:
    def run(self, object_class: str, place_pose: Pose) -> OrchestratorResult
```

**Full sequence:**
1. Move to capture pose (joint config from yaml)
2. Wait for `/gpd/grasp_poses` (timeout from yaml)
3. Arm selection (Y-threshold + reachability)
4. `PickSkill.pick()` — Mode 1, fallback to Mode 2
5. If pick fails: open gripper → return to capture pose → retry once (back to step 2)
6. If pick succeeds: `PlaceSkill.place()` — Mode 1, fallback to Mode 2
7. Return to home/capture pose
8. Return `OrchestratorResult`

**OrchestratorResult:**
```python
class OrchestratorResult(Enum):
    SUCCESS = 'success'
    GRASP_FAILED = 'grasp_failed'
    PLANNING_FAILED = 'planning_failed'
    NO_GPD_CANDIDATES = 'no_gpd_candidates'
    NO_REACHABLE_ARM = 'no_reachable_arm'
    TIMEOUT = 'timeout'
```

---

## Action Server Node

`nodes/pick_and_place_server.py` — thin wrapper only:
- Accepts Goal → calls `orchestrator.run()`
- Publishes Feedback from orchestrator callbacks
- Returns Result

No logic here. If the orchestrator needs to publish feedback, it accepts a callback
from the server node.

---

## Standalone Test

`tests/test_pick_and_place_orchestrator.py`

```python
# Hardcoded pose — no perception team, no mission team needed
orchestrator = PickAndPlaceOrchestrator(moveit_client, gripper, ...)
result = orchestrator.run(
    object_class="bottle",
    place_pose=hardcoded_place_pose,
)
```

Validates full pick → assess → place loop before Action Server integration.

---

## Per-File Detailed Plan

---

### 1. `robot_interface/moveit_client.py` — Add `check_reachable()`

**Why:** Arm selection needs IK validation before committing to a pick. This method
does not exist yet — `pick_and_place.py` (old demo) calls it but it's broken.

**What to add:**
```python
def check_reachable(self, pose: Pose, arm: Arm = Arm.RIGHT) -> bool
```
- Use pymoveit2's `compute_ik()` or attempt `set_pose_goal` + plan only (no execute)
- Return `True` if a valid IK solution exists, `False` otherwise
- Must not move the robot — query only
- Log result at debug level

**Nothing else changes in this file.**

---

### 2. `skill_primitives/pick_skill.py` — Upgrade

**Current state:** Mode 1 (cartesian) only. No retry logic.

**Changes:**

Add module-level helper (not a class method — shared logic):
```python
def _move_with_retry(
    move_fn: Callable[[], MoveResult],
    log,
    label: str,
    same_retries: int,
    jitter_retries: int,
    jitter_std: float,
    pose: Pose | None = None,   # needed for jitter — None means no jitter
) -> MoveResult
```
- First tries `move_fn()` up to `same_retries` times (same pose, OMPL randomness)
- Then perturbs `pose.position` with gaussian noise (std=`jitter_std`) and retries
  up to `jitter_retries` times
- Returns first `MoveResult.SUCCEEDED`, or last failure result

Update `PickSkill.pick()` signature:
```python
def pick(
    self,
    grasp_pose: Pose,
    arm: Arm = Arm.RIGHT,
    object_name: str = 'ETC',
    pre_grasp_offset: float = _PRE_GRASP_OFFSET,
    approach_height: float = _APPROACH_HEIGHT,
    lift_home: float = _LIFT_HOME,
    planning_retries: int = _PLANNING_RETRIES,
    jitter_retries: int = _JITTER_RETRIES,
    jitter_std: float = _JITTER_STD,
) -> PickResult
```

**Mode 1 (cartesian) — wrap moves with `_move_with_retry`:**
1. `gripper.open(side)` + `wait_until_executed()`
2. `_move_with_retry(lambda: moveit.move_to_pose(pre), ...)` — move to pre_grasp
3. `moveit.move_cartesian(grasp_pose)` — no retry for cartesian (straight-line or nothing)
4. If cartesian fails → `moveit.move_to_pose(pre)` retract → fall through to Mode 2
5. `grasp_skill.grasp(side, object_name)`
6. On success: `moveit.move_cartesian(pre)` retract; if retract fails, `move_to_pose(pre)`
7. On failure: gripper already opened by GraspSkill → `moveit.move_to_pose(pre)` → FAILURE

**Mode 2 (lift fallback) — triggered only if Mode 1 cartesian step fails:**
1. `gripper.open(side)`
2. `moveit.move_lift(lift_home)`
3. Build `pre_grasp_lift` = grasp_pose with `z += approach_height`, same orientation
4. `_move_with_retry(lambda: moveit.move_to_pose(pre_grasp_lift), ...)`
5. `moveit.move_lift(lift_home - approach_height)` — descend
6. `grasp_skill.grasp(side, object_name)`
7. On success: `moveit.move_lift(lift_home)` — ascend → SUCCESS
8. On failure: `gripper.open()` → `moveit.move_lift(lift_home)` → FAILURE

**Module-level defaults (all overridable via yaml):**
```python
_PRE_GRASP_OFFSET  = 0.15
_APPROACH_HEIGHT   = 0.10
_LIFT_HOME         = 0.0
_PLANNING_RETRIES  = 3
_JITTER_RETRIES    = 3
_JITTER_STD        = 0.01
```

---

### 3. `skill_primitives/place_skill.py` — Upgrade

**Current state:** Mode 1 (cartesian) only. No retry logic.

**Changes:** Mirror pick_skill structure exactly.

Update `PlaceSkill.place()` with same extra parameters as `PickSkill.pick()`.

**Mode 1 (cartesian):**
1. `_move_with_retry(lambda: moveit.move_to_pose(pre_place), ...)`
2. `moveit.move_cartesian(place_pose)`
3. If cartesian fails → retract → fall through to Mode 2
4. `gripper.open(side)` + `wait_until_executed()`
5. `moveit.move_cartesian(pre_place)` retract; fallback to `move_to_pose(pre_place)`

**Mode 2 (lift fallback):**
1. `moveit.move_lift(lift_home)`
2. Build `pre_place_lift` = place_pose with `z += approach_height`
3. `_move_with_retry(lambda: moveit.move_to_pose(pre_place_lift), ...)`
4. `moveit.move_lift(lift_home - approach_height)` — descend
5. `gripper.open(side)` + `wait_until_executed()`
6. `moveit.move_lift(lift_home)` — ascend
7. Return SUCCESS

Note: `_move_with_retry` is defined in `pick_skill.py` and imported by `place_skill.py`.

---

### 4. `skill_primitives/pick_and_place_orchestrator.py` — New

**Purpose:** Full end-to-end logic. Pure Python, no ROS node. Accepts a feedback
callback so the Action Server can publish phases without the orchestrator knowing
about ROS actions.

**Constructor:**
```python
def __init__(
    self,
    node: Node,
    moveit: MoveItClient,
    gripper: GripperInterface,
    grasp_skill: GraspSkill,
    pick_skill: PickSkill,
    place_skill: PlaceSkill,
    config: dict,                        # loaded from pick_and_place.yaml
    feedback_cb: Callable[[str], None] | None = None,
)
```

**Main method:**
```python
def run(self, object_class: str, place_pose: Pose) -> OrchestratorResult
```

**Internal sequence (with feedback phases):**
```
phase: "moving_to_capture_pose"
  → moveit.move_to_joints(capture_pose_joints, arm=both or dominant)

phase: "waiting_for_gpd"
  → subscribe to /gpd/grasp_poses, wait up to gpd_timeout seconds
  → if no poses received → return NO_GPD_CANDIDATES

phase: "selecting_arm"
  → select_arm(grasp_poses, config) → (arm, best_pose)
  → if no reachable arm → return NO_REACHABLE_ARM

phase: "picking"
  → pick_skill.pick(best_pose, arm, object_class, ...)

phase: "assessing_grasp"  ← GraspSkill handles internally, but orchestrator logs

  if pick fails:
    phase: "returning_to_capture_pose"
    → move to capture pose, retry once (back to waiting_for_gpd)
    → if retry also fails → return GRASP_FAILED

phase: "placing"
  → place_skill.place(place_pose, arm, ...)
  → if place fails → return PLANNING_FAILED

phase: "returning_home"
  → moveit.move_to_home(arm)

→ return SUCCESS
```

**Arm selection helper (internal):**
```python
def _select_arm(self, poses: list[Pose]) -> tuple[Arm, Pose] | None:
    # 1. For each pose: Y >= y_threshold → LEFT, else RIGHT
    # 2. check_reachable on selected arm
    # 3. If not reachable, try other arm
    # 4. Return (arm, pose) for first reachable candidate
    # 5. Return None if nothing reachable
```

**Config keys used from yaml:**
- `capture_pose_joints` (list[float], 7 values)
- `gpd_topic` (default `/gpd/grasp_poses`)
- `gpd_timeout` (seconds)
- `y_threshold` (metres, arm selection boundary)
- `max_retries` (default 1)
- pick/place offset values (passed through to skills)

---

### 5. `nodes/pick_and_place_server.py` — New

Thin ROS 2 Action Server. No logic.

```python
class PickAndPlaceServer(Node):
    def __init__(self):
        # load config yaml
        # construct MoveItClient, GripperInterface, GraspAssessment,
        #   GraspSkill, PickSkill, PlaceSkill, PickAndPlaceOrchestrator
        # create ActionServer for PickAndPlace

    def _execute_cb(self, goal_handle):
        # call orchestrator.run(object_class, place_pose)
        # publish feedback via orchestrator's feedback_cb
        # set result
```

Entry point registered in `setup.py`:
```
pick_and_place_server = ai_worker_manipulation.nodes.pick_and_place_server:main
```

---

### 6. `action/PickAndPlace.action` — New

```
string object_class
geometry_msgs/PoseStamped place_pose
---
bool success
string failure_reason
---
string phase
```

Requires update to `package.xml` (add `rosidl_default_generators` dep) and
`CMakeLists.txt` (register action file).

---

### 7. `config/pick_and_place.yaml` — New

```yaml
pick_and_place:
  # Arm selection
  y_threshold: 0.0           # metres; >= threshold → left arm

  # GPD
  gpd_topic: /gpd/grasp_poses
  gpd_timeout: 30.0          # seconds to wait for GPD output

  # Capture pose (joint config for PCD acquisition, 7 DOF)
  capture_pose_joints: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   # TODO: set real values

  # Retry
  max_retries: 1

  # Pick / Place motion
  pre_grasp_offset: 0.15     # metres (cartesian mode approach vector offset)
  approach_height: 0.10      # metres (lift mode z offset above grasp/place)
  lift_home: 0.0             # lift_joint home position (m)
  planning_retries: 3
  jitter_retries: 3
  jitter_std: 0.01           # metres, gaussian std for pose jitter
```

---

### 8. `tests/test_pick_and_place_orchestrator.py` — New

Standalone test with hardcoded poses. No Action Server, no perception needed.

```python
GRASP_POSE  = ...   # from demo_gpd_pick_place.py GRASP_POSITION/ORIENTATION
PLACE_POSE  = ...   # fixed place location

orchestrator = PickAndPlaceOrchestrator(moveit, gripper, ..., config=load_yaml(...))
result = orchestrator.run(object_class='ETC', place_pose=PLACE_POSE)
```

---

## Files to Create / Modify

| File | Action |
|---|---|
| `robot_interface/moveit_client.py` | Add `check_reachable()` method |
| `skill_primitives/pick_skill.py` | Add `_move_with_retry()` + Mode 2 lift fallback |
| `skill_primitives/place_skill.py` | Add Mode 2 lift fallback, import `_move_with_retry` |
| `skill_primitives/pick_and_place_orchestrator.py` | New |
| `nodes/__init__.py` | New (empty) |
| `nodes/pick_and_place_server.py` | New |
| `action/PickAndPlace.action` | New |
| `config/pick_and_place.yaml` | New |
| `tests/test_pick_and_place_orchestrator.py` | New |
| `package.xml` | Add rosidl deps for action |
| `setup.py` | Add nodes entry point |
| `CMakeLists.txt` | Register action file |

---

## Open Items / Known Limitations

- `capture_pose_joints` in yaml is all-zeros placeholder — needs real joint config
  from robot testing before orchestrator can reliably move to capture pose.
- `move_to_home()` in MoveItClient also uses all-zeros — separate issue, tracked in WORK_SUMMARY.
- Sim vs real calibration: `approach_height`, `pre_grasp_offset`, `y_threshold` all
  need tuning on real hardware. Yaml makes this easy to change without code edits.
- `check_reachable()` implementation depends on pymoveit2 IK API — needs verification
  that it works correctly for both left and right arm move groups.
- GPD topic name (`/gpd/grasp_poses`) must match what `gpd_grasp_publisher.py` publishes
  (currently it publishes to `/gpd/grasps` and `/gpd/best_grasp`) — reconcile before integration.