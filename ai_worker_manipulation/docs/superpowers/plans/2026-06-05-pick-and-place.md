# Pick and Place End-to-End Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a fully testable end-to-end pick-and-place pipeline — GPD poses in, object placed, wrapped as a ROS 2 Action Server the mission team can call.

**Architecture:** `PickAndPlaceOrchestrator` (pure Python, testable standalone) owns all logic — arm selection, GPD waiting, pick/place with cartesian+lift fallback, retry on failure. A thin `nodes/pick_and_place_server.py` wraps it as a ROS 2 Action Server. Skills (`PickSkill`, `PlaceSkill`) are upgraded with planning-retry helpers and lift fallback.

**Tech Stack:** ROS 2 Jazzy, pymoveit2, MoveIt2, Python 3.10+, ament_python + ament_cmake (msgs package)

---

## File Map

| File | Status | Responsibility |
|---|---|---|
| `robot_interface/moveit_client.py` | Modify | Add `check_reachable()` using pymoveit2 `compute_ik_async` |
| `skill_primitives/pick_skill.py` | Modify | Add `_move_with_retry()` helper + Mode 2 lift fallback |
| `skill_primitives/place_skill.py` | Modify | Add Mode 2 lift fallback, import `_move_with_retry` |
| `skill_primitives/pick_and_place_orchestrator.py` | Create | Full end-to-end logic, pure Python |
| `nodes/__init__.py` | Create | Empty, marks nodes/ as package |
| `nodes/pick_and_place_server.py` | Create | Thin ROS 2 Action Server wrapper |
| `config/pick_and_place.yaml` | Create | All tunable values |
| `tests/test_pick_and_place_orchestrator.py` | Create | Standalone hardcoded-pose integration test |
| `setup.py` | Modify | Add entry points for server + new test scripts |
| `../ai_worker_manipulation_msgs/` | Create | Separate ament_cmake package for PickAndPlace.action |

---

## Task 1: Action Interface Package

ROS 2 action interfaces must live in an `ament_cmake` package — they cannot be defined inside an `ament_python` package. We create a minimal sibling package.

**Files:**
- Create: `../ai_worker_manipulation_msgs/package.xml`
- Create: `../ai_worker_manipulation_msgs/CMakeLists.txt`
- Create: `../ai_worker_manipulation_msgs/action/PickAndPlace.action`

- [ ] **Step 1: Create the package directory structure**

```bash
cd ~/ai_worker_dev/ai_worker_ws_main/ai-worker-ws
mkdir -p ai_worker_manipulation_msgs/action
```

- [ ] **Step 2: Write the action file**

Create `ai_worker_manipulation_msgs/action/PickAndPlace.action`:

```
# Goal
string object_class
geometry_msgs/PoseStamped place_pose
---
# Result
bool success
string failure_reason
---
# Feedback
string phase
```

- [ ] **Step 3: Write package.xml**

Create `ai_worker_manipulation_msgs/package.xml`:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>ai_worker_manipulation_msgs</name>
  <version>0.0.1</version>
  <description>Action interfaces for ai_worker_manipulation</description>
  <maintainer email="chlgkals0730@gmail.com">hamin</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <depend>geometry_msgs</depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 4: Write CMakeLists.txt**

Create `ai_worker_manipulation_msgs/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(ai_worker_manipulation_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PickAndPlace.action"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
```

- [ ] **Step 5: Build and verify**

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select ai_worker_manipulation_msgs
source install/setup.bash
```

Expected: build succeeds, no errors.

Verify the action type exists:
```bash
ros2 action list --help   # just confirms ros2 action tooling works
ros2 interface show ai_worker_manipulation_msgs/action/PickAndPlace
```

Expected output:
```
string object_class
geometry_msgs/PoseStamped place_pose
---
bool success
string failure_reason
---
string phase
```

- [ ] **Step 6: Add dependency to ai_worker_manipulation/package.xml**

Add inside the `<package>` block of `ai_worker_manipulation/package.xml`:
```xml
  <depend>ai_worker_manipulation_msgs</depend>
```

- [ ] **Step 7: Commit**

```bash
git add ai_worker_manipulation_msgs/ ai_worker_manipulation/package.xml
git commit -m "[PickAndPlace.action] add action interface msgs package"
```

---

## Task 2: Config YAML

All tunable values in one place. Skills and orchestrator read from this — nothing hardcoded.

**Files:**
- Create: `ai_worker_manipulation/config/pick_and_place.yaml`
- Modify: `setup.py` (register yaml in data_files)

- [ ] **Step 1: Create config directory and yaml**

```bash
mkdir -p ~/ros2_ws/src/ai_worker/ai_worker_manipulation/ai_worker_manipulation/config
```

Create `ai_worker_manipulation/config/pick_and_place.yaml`:

```yaml
pick_and_place:
  # Arm selection
  y_threshold: 0.0           # metres; pose.y >= threshold -> left arm, else right

  # GPD
  gpd_topic: /gpd/grasp_poses
  gpd_timeout: 30.0          # seconds to wait for GPD output after goal received

  # Capture pose — joint config [j1..j7] for moving to PCD acquisition position
  # TODO: replace with real joint values after robot testing
  capture_pose_position: [0.35, -0.20, 1.20]   # from move_wrist_capture_pose.py
  capture_pose_orientation: [0.086, -0.173, 0.015, 0.981]  # [x, y, z, w]

  # Retry
  max_retries: 1             # retry pick once on grasp failure before aborting

  # Motion parameters
  pre_grasp_offset: 0.15     # metres — step back along approach vector (cartesian mode)
  approach_height: 0.10      # metres — z offset above grasp/place pose (lift mode)
  lift_home: 0.0             # lift_joint home position in metres (0.0 = top)
  planning_retries: 3        # same-pose OMPL retries before trying jitter
  jitter_retries: 3          # jitter retries after same-pose retries exhausted
  jitter_std: 0.01           # gaussian std dev for position jitter in metres
```

- [ ] **Step 2: Register yaml in setup.py**

In `ai_worker_manipulation/setup.py`, add to `data_files`:

```python
('share/' + package_name + '/config',
    ['ai_worker_manipulation/config/pick_and_place.yaml']),
```

Full `data_files` after change:
```python
data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/data',
        ['ai_worker_manipulation/data/object_lut.json']),
    ('share/' + package_name + '/config',
        ['ai_worker_manipulation/config/pick_and_place.yaml']),
],
```

- [ ] **Step 3: Commit**

```bash
git add ai_worker_manipulation/config/pick_and_place.yaml ai_worker_manipulation/setup.py
git commit -m "[pick_and_place.yaml] add config with all tunable values"
```

---

## Task 3: `check_reachable()` in MoveItClient

`compute_ik()` in pymoveit2 calls `rclpy.spin_once()` internally — this conflicts with `MoveItClient`'s executor thread. We must use `compute_ik_async()` and poll the future ourselves.

**Files:**
- Modify: `ai_worker_manipulation/robot_interface/moveit_client.py`

- [ ] **Step 1: Write a failing test**

Add to `ai_worker_manipulation/tests/test_moveit_client.py` (or create standalone):

```python
# In test_moveit_client.py, add this test at the end:
def test_check_reachable_returns_bool(client: MoveItClient):
    """check_reachable must return bool without raising, for both arms."""
    from geometry_msgs.msg import Pose
    pose = Pose()
    pose.position.x = 0.35
    pose.position.y = -0.20
    pose.position.z = 1.20
    pose.orientation.w = 1.0
    result = client.check_reachable(pose, arm=Arm.RIGHT)
    assert isinstance(result, bool), f"Expected bool, got {type(result)}"
```

- [ ] **Step 2: Run test to verify it fails**

```bash
# Inside container
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run ai_worker_manipulation test_moveit_client
```

Expected: `AttributeError: 'MoveItClient' object has no attribute 'check_reachable'`

- [ ] **Step 3: Implement `check_reachable()`**

In `moveit_client.py`, add after the `get_pose()` method (before `destroy()`):

```python
def check_reachable(self, pose: Pose, arm: Arm = Arm.RIGHT) -> bool:
    """Return True if a valid IK solution exists for pose on the given arm.

    Uses compute_ik_async + manual future polling to avoid rclpy.spin_once()
    conflict with the executor thread.
    """
    self._guard()
    moveit2 = self._moveit(arm)

    position = (pose.position.x, pose.position.y, pose.position.z)
    quat     = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )

    future = moveit2.compute_ik_async(position, quat)
    if future is None:
        self._log.warn(f'[check_reachable] [{arm.value}] compute_ik_async returned None')
        return False

    deadline = time.time() + 5.0
    while not future.done():
        if time.time() > deadline:
            self._log.warn(f'[check_reachable] [{arm.value}] IK timeout')
            return False
        time.sleep(0.02)

    result = moveit2.get_compute_ik_result(future)
    reachable = result is not None
    self._log.info(f'[check_reachable] [{arm.value}] reachable={reachable}')
    return reachable
```

- [ ] **Step 4: Run test to verify it passes**

```bash
ros2 run ai_worker_manipulation test_moveit_client
```

Expected: test passes, logs show `[check_reachable] [right] reachable=True` or `False`.

- [ ] **Step 5: Commit**

```bash
git add ai_worker_manipulation/robot_interface/moveit_client.py
git commit -m "[moveit_client] add check_reachable() via compute_ik_async"
```

---

## Task 4: Upgrade `pick_skill.py`

Add `_move_with_retry()` helper and Mode 2 (lift fallback).

**Files:**
- Modify: `ai_worker_manipulation/skill_primitives/pick_skill.py`

- [ ] **Step 1: Add module-level defaults and imports**

Replace the top of `pick_skill.py` (after existing imports) with:

```python
import random
from typing import Callable

from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Pose

from ai_worker_manipulation.robot_interface.moveit_client import (
    MoveItClient,
    Arm,
    MoveResult,
)
from ai_worker_manipulation.robot_interface.gripper_controller import GripperInterface
from ai_worker_manipulation.skill_primitives.grasp_skill import GraspSkill, GraspResult


_PRE_GRASP_OFFSET  = 0.15
_APPROACH_HEIGHT   = 0.10
_LIFT_HOME         = 0.0
_PLANNING_RETRIES  = 3
_JITTER_RETRIES    = 3
_JITTER_STD        = 0.01
```

- [ ] **Step 2: Add `_move_with_retry()` module-level helper**

Add this function after the imports, before `pre_grasp_of()`:

```python
def _move_with_retry(
    move_fn: Callable[[], MoveResult],
    log,
    label: str,
    pose: Pose,
    same_retries: int = _PLANNING_RETRIES,
    jitter_retries: int = _JITTER_RETRIES,
    jitter_std: float = _JITTER_STD,
) -> MoveResult:
    """Retry a move callable with same pose, then with gaussian position jitter."""
    for attempt in range(same_retries):
        result = move_fn()
        if result == MoveResult.SUCCEEDED:
            return result
        log.warn(f'[{label}] same-pose attempt {attempt + 1}/{same_retries} failed: {result.value}')

    for attempt in range(jitter_retries):
        jittered = Pose()
        jittered.position.x    = pose.position.x + random.gauss(0, jitter_std)
        jittered.position.y    = pose.position.y + random.gauss(0, jitter_std)
        jittered.position.z    = pose.position.z + random.gauss(0, jitter_std)
        jittered.orientation   = pose.orientation
        result = move_fn.__self__._moveit.move_to_pose(jittered, arm=move_fn.__self__._arm_hint) \
            if hasattr(move_fn, '__self__') else move_fn()
        log.warn(f'[{label}] jitter attempt {attempt + 1}/{jitter_retries}: {result.value}')
        if result == MoveResult.SUCCEEDED:
            return result

    return result
```

Wait — the closure approach above is fragile. Use a simpler design: callers pass both the move callable AND a jitter-aware callable:

```python
def _move_with_retry(
    move_fn: Callable[[Pose], MoveResult],
    pose: Pose,
    log,
    label: str,
    same_retries: int = _PLANNING_RETRIES,
    jitter_retries: int = _JITTER_RETRIES,
    jitter_std: float = _JITTER_STD,
) -> MoveResult:
    """
    Retry move_fn(pose) with:
      1. same pose, up to same_retries times
      2. gaussian-jittered pose, up to jitter_retries times

    move_fn receives a Pose and returns MoveResult.
    """
    result = MoveResult.FAILED

    for attempt in range(same_retries):
        result = move_fn(pose)
        if result == MoveResult.SUCCEEDED:
            return result
        log.warn(
            f'[{label}] same-pose attempt {attempt + 1}/{same_retries} → {result.value}'
        )

    for attempt in range(jitter_retries):
        jittered = Pose()
        jittered.position.x  = pose.position.x + random.gauss(0.0, jitter_std)
        jittered.position.y  = pose.position.y + random.gauss(0.0, jitter_std)
        jittered.position.z  = pose.position.z + random.gauss(0.0, jitter_std)
        jittered.orientation = pose.orientation
        result = move_fn(jittered)
        log.warn(
            f'[{label}] jitter attempt {attempt + 1}/{jitter_retries} → {result.value}'
        )
        if result == MoveResult.SUCCEEDED:
            return result

    return result
```

- [ ] **Step 3: Rewrite `PickSkill.pick()` with Mode 1 + Mode 2**

Replace the full `pick()` method:

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
) -> PickResult:
    side = arm.value
    pre  = pre_grasp_of(grasp_pose, offset=pre_grasp_offset)

    self._log.info(f'[PickSkill] [{side}] starting pick — object={object_name!r}')

    # ── Mode 1: cartesian approach ────────────────────────────────────
    self._log.info(f'[PickSkill] [{side}] Mode 1 (cartesian)')
    self._gripper.open(side)
    self._gripper.wait_until_executed()

    result = _move_with_retry(
        lambda p: self._moveit.move_to_pose(p, arm=arm),
        pre, self._log, f'PickSkill/{side}/pre_grasp',
        same_retries=planning_retries, jitter_retries=jitter_retries, jitter_std=jitter_std,
    )
    if result != MoveResult.SUCCEEDED:
        self._log.error(f'[PickSkill] [{side}] pre-grasp failed — trying Mode 2')
        return self._pick_lift(grasp_pose, arm, object_name, approach_height, lift_home,
                               planning_retries, jitter_retries, jitter_std)

    result = self._moveit.move_cartesian(grasp_pose, arm=arm)
    if result != MoveResult.SUCCEEDED:
        self._log.warn(f'[PickSkill] [{side}] cartesian approach failed → Mode 2')
        self._moveit.move_to_pose(pre, arm=arm)
        return self._pick_lift(grasp_pose, arm, object_name, approach_height, lift_home,
                               planning_retries, jitter_retries, jitter_std)

    grasp_result = self._grasp.grasp(side, object_name=object_name)
    if grasp_result != GraspResult.SUCCESS:
        self._log.error(f'[PickSkill] [{side}] grasp FAILED — retracting')
        self._moveit.move_to_pose(pre, arm=arm)
        return PickResult.FAILURE

    retract = self._moveit.move_cartesian(pre, arm=arm)
    if retract != MoveResult.SUCCEEDED:
        self._moveit.move_to_pose(pre, arm=arm)

    self._log.info(f'[PickSkill] [{side}] pick SUCCEEDED (cartesian)')
    return PickResult.SUCCESS

def _pick_lift(
    self,
    grasp_pose: Pose,
    arm: Arm,
    object_name: str,
    approach_height: float,
    lift_home: float,
    planning_retries: int,
    jitter_retries: int,
    jitter_std: float,
) -> PickResult:
    """Mode 2: lift-based approach fallback."""
    side = arm.value
    self._log.info(f'[PickSkill] [{side}] Mode 2 (lift)')

    pre_lift = Pose()
    pre_lift.position.x    = grasp_pose.position.x
    pre_lift.position.y    = grasp_pose.position.y
    pre_lift.position.z    = grasp_pose.position.z + approach_height
    pre_lift.orientation   = grasp_pose.orientation

    self._gripper.open(side)
    self._gripper.wait_until_executed()
    self._moveit.move_lift(lift_home)

    result = _move_with_retry(
        lambda p: self._moveit.move_to_pose(p, arm=arm),
        pre_lift, self._log, f'PickSkill/{side}/pre_lift',
        same_retries=planning_retries, jitter_retries=jitter_retries, jitter_std=jitter_std,
    )
    if result != MoveResult.SUCCEEDED:
        self._log.error(f'[PickSkill] [{side}] Mode 2 pre-lift move failed')
        return PickResult.FAILURE

    self._moveit.move_lift(lift_home - approach_height)

    grasp_result = self._grasp.grasp(side, object_name=object_name)
    if grasp_result != GraspResult.SUCCESS:
        self._log.error(f'[PickSkill] [{side}] Mode 2 grasp FAILED — ascending')
        self._moveit.move_lift(lift_home)
        return PickResult.FAILURE

    self._moveit.move_lift(lift_home)
    self._log.info(f'[PickSkill] [{side}] pick SUCCEEDED (lift)')
    return PickResult.SUCCESS
```

- [ ] **Step 4: Build and smoke-check**

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select ai_worker_manipulation
source install/setup.bash
python3 -c "from ai_worker_manipulation.skill_primitives.pick_skill import PickSkill, _move_with_retry; print('OK')"
```

Expected: `OK`

- [ ] **Step 5: Commit**

```bash
git add ai_worker_manipulation/skill_primitives/pick_skill.py
git commit -m "[pick_skill] add _move_with_retry helper and Mode 2 lift fallback"
```

---

## Task 5: Upgrade `place_skill.py`

Mirror pick_skill structure. Import `_move_with_retry` and `pre_grasp_of` from pick_skill.

**Files:**
- Modify: `ai_worker_manipulation/skill_primitives/place_skill.py`

- [ ] **Step 1: Rewrite place_skill.py**

Full replacement of `place_skill.py`:

```python
from enum import Enum

from geometry_msgs.msg import Pose

from ai_worker_manipulation.robot_interface.moveit_client import (
    MoveItClient,
    Arm,
    MoveResult,
)
from ai_worker_manipulation.robot_interface.gripper_controller import GripperInterface
from ai_worker_manipulation.skill_primitives.pick_skill import (
    _move_with_retry,
    pre_grasp_of,
    _APPROACH_HEIGHT,
    _LIFT_HOME,
    _PLANNING_RETRIES,
    _JITTER_RETRIES,
    _JITTER_STD,
)

_PRE_PLACE_OFFSET = 0.15


class PlaceResult(Enum):
    SUCCESS = 'success'
    FAILURE = 'failure'


class PlaceSkill:
    """
    Single place sequence.
    Mode 1 (cartesian): pre_place → cartesian → open → cartesian retract
    Mode 2 (lift):      lift_home → pre_place_z+offset → lower lift → open → raise lift
    """

    def __init__(self, node, moveit: MoveItClient, gripper: GripperInterface):
        self._node    = node
        self._log     = node.get_logger()
        self._moveit  = moveit
        self._gripper = gripper

    def place(
        self,
        place_pose: Pose,
        arm: Arm = Arm.RIGHT,
        pre_place_offset: float = _PRE_PLACE_OFFSET,
        approach_height: float = _APPROACH_HEIGHT,
        lift_home: float = _LIFT_HOME,
        planning_retries: int = _PLANNING_RETRIES,
        jitter_retries: int = _JITTER_RETRIES,
        jitter_std: float = _JITTER_STD,
    ) -> PlaceResult:
        side = arm.value
        pre  = pre_grasp_of(place_pose, offset=pre_place_offset)

        self._log.info(f'[PlaceSkill] [{side}] starting place')

        # ── Mode 1: cartesian ─────────────────────────────────────────
        result = _move_with_retry(
            lambda p: self._moveit.move_to_pose(p, arm=arm),
            pre, self._log, f'PlaceSkill/{side}/pre_place',
            same_retries=planning_retries, jitter_retries=jitter_retries, jitter_std=jitter_std,
        )
        if result != MoveResult.SUCCEEDED:
            self._log.warn(f'[PlaceSkill] [{side}] pre-place failed → Mode 2')
            return self._place_lift(place_pose, arm, approach_height, lift_home,
                                    planning_retries, jitter_retries, jitter_std)

        result = self._moveit.move_cartesian(place_pose, arm=arm)
        if result != MoveResult.SUCCEEDED:
            self._log.warn(f'[PlaceSkill] [{side}] cartesian approach failed → Mode 2')
            self._moveit.move_to_pose(pre, arm=arm)
            return self._place_lift(place_pose, arm, approach_height, lift_home,
                                    planning_retries, jitter_retries, jitter_std)

        self._gripper.open(side)
        self._gripper.wait_until_executed()

        retract = self._moveit.move_cartesian(pre, arm=arm)
        if retract != MoveResult.SUCCEEDED:
            self._moveit.move_to_pose(pre, arm=arm)

        self._log.info(f'[PlaceSkill] [{side}] place SUCCEEDED (cartesian)')
        return PlaceResult.SUCCESS

    def _place_lift(
        self,
        place_pose: Pose,
        arm: Arm,
        approach_height: float,
        lift_home: float,
        planning_retries: int,
        jitter_retries: int,
        jitter_std: float,
    ) -> PlaceResult:
        """Mode 2: lift-based approach."""
        side = arm.value
        self._log.info(f'[PlaceSkill] [{side}] Mode 2 (lift)')

        pre_lift = Pose()
        pre_lift.position.x  = place_pose.position.x
        pre_lift.position.y  = place_pose.position.y
        pre_lift.position.z  = place_pose.position.z + approach_height
        pre_lift.orientation = place_pose.orientation

        self._moveit.move_lift(lift_home)

        result = _move_with_retry(
            lambda p: self._moveit.move_to_pose(p, arm=arm),
            pre_lift, self._log, f'PlaceSkill/{side}/pre_lift',
            same_retries=planning_retries, jitter_retries=jitter_retries, jitter_std=jitter_std,
        )
        if result != MoveResult.SUCCEEDED:
            self._log.error(f'[PlaceSkill] [{side}] Mode 2 pre-lift move failed')
            return PlaceResult.FAILURE

        self._moveit.move_lift(lift_home - approach_height)

        self._gripper.open(side)
        self._gripper.wait_until_executed()

        self._moveit.move_lift(lift_home)
        self._log.info(f'[PlaceSkill] [{side}] place SUCCEEDED (lift)')
        return PlaceResult.SUCCESS
```

- [ ] **Step 2: Build and smoke-check**

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select ai_worker_manipulation
source install/setup.bash
python3 -c "from ai_worker_manipulation.skill_primitives.place_skill import PlaceSkill; print('OK')"
```

Expected: `OK`

- [ ] **Step 3: Commit**

```bash
git add ai_worker_manipulation/skill_primitives/place_skill.py
git commit -m "[place_skill] add Mode 2 lift fallback, use _move_with_retry from pick_skill"
```

---

## Task 6: `PickAndPlaceOrchestrator`

Pure Python. No ROS node, no Action Server. Accepts an optional `feedback_cb` so the Action Server can publish phases.

**Files:**
- Create: `ai_worker_manipulation/skill_primitives/pick_and_place_orchestrator.py`

- [ ] **Step 1: Create the file**

```python
"""
pick_and_place_orchestrator.py
-------------------------------
End-to-end pick and place logic. Pure Python — no ROS Action Server here.
Testable with hardcoded poses by calling run() directly.

Sequence:
  1. Move to capture pose
  2. Wait for GPD grasp poses on gpd_topic
  3. Arm selection: Y-threshold + IK reachability check
  4. PickSkill.pick() — Mode 1 (cartesian), fallback to Mode 2 (lift)
  5. On grasp failure: return to capture pose, retry once
  6. PlaceSkill.place() — Mode 1 (cartesian), fallback to Mode 2 (lift)
  7. Return to home
"""

import time
from enum import Enum
from typing import Callable, Optional

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node

from ai_worker_manipulation.robot_interface.moveit_client import MoveItClient, Arm, MoveResult
from ai_worker_manipulation.robot_interface.gripper_controller import GripperInterface
from ai_worker_manipulation.skill_primitives.grasp_skill import GraspSkill
from ai_worker_manipulation.skill_primitives.pick_skill import PickSkill, PickResult
from ai_worker_manipulation.skill_primitives.place_skill import PlaceSkill, PlaceResult


class OrchestratorResult(Enum):
    SUCCESS           = 'success'
    GRASP_FAILED      = 'grasp_failed'
    PLANNING_FAILED   = 'planning_failed'
    NO_GPD_CANDIDATES = 'no_gpd_candidates'
    NO_REACHABLE_ARM  = 'no_reachable_arm'
    TIMEOUT           = 'timeout'


class PickAndPlaceOrchestrator:
    """
    Orchestrates the full pick-and-place pipeline.

    Parameters
    ----------
    node         : Active rclpy Node (used for subscriptions and logging).
    moveit       : Initialised MoveItClient.
    gripper      : Initialised GripperInterface.
    pick_skill   : Initialised PickSkill.
    place_skill  : Initialised PlaceSkill.
    config       : Dict loaded from pick_and_place.yaml (pick_and_place key).
    feedback_cb  : Optional callable(phase: str) — called at each phase transition.
                   Use this to publish ROS Action feedback from the server node.
    """

    def __init__(
        self,
        node: Node,
        moveit: MoveItClient,
        gripper: GripperInterface,
        pick_skill: PickSkill,
        place_skill: PlaceSkill,
        config: dict,
        feedback_cb: Optional[Callable[[str], None]] = None,
    ):
        self._node        = node
        self._log         = node.get_logger()
        self._moveit      = moveit
        self._gripper     = gripper
        self._pick        = pick_skill
        self._place       = place_skill
        self._cfg         = config
        self._feedback_cb = feedback_cb or (lambda phase: None)

    # ── Public API ────────────────────────────────────────────────────

    def run(self, object_class: str, place_pose: Pose) -> OrchestratorResult:
        """
        Execute a full pick-and-place cycle.

        Parameters
        ----------
        object_class : Key in object_lut.json for grasp assessment thresholds.
        place_pose   : Target pose for placing the object.

        Returns
        -------
        OrchestratorResult
        """
        max_retries = self._cfg.get('max_retries', 1)

        for attempt in range(max_retries + 1):
            if attempt > 0:
                self._log.info(f'[Orchestrator] retry attempt {attempt}/{max_retries}')

            # ── 1. Move to capture pose ───────────────────────────────
            self._feedback_cb('moving_to_capture_pose')
            if not self._move_to_capture_pose():
                return OrchestratorResult.PLANNING_FAILED

            # ── 2. Wait for GPD poses ─────────────────────────────────
            self._feedback_cb('waiting_for_gpd')
            grasp_poses = self._wait_for_gpd()
            if not grasp_poses:
                return OrchestratorResult.NO_GPD_CANDIDATES

            # ── 3. Arm selection ──────────────────────────────────────
            selection = self._select_arm(grasp_poses)
            if selection is None:
                return OrchestratorResult.NO_REACHABLE_ARM
            arm, grasp_pose = selection

            # ── 4. Pick ───────────────────────────────────────────────
            self._feedback_cb('picking')
            pick_result = self._pick.pick(
                grasp_pose=grasp_pose,
                arm=arm,
                object_name=object_class,
                approach_height=self._cfg.get('approach_height', 0.10),
                lift_home=self._cfg.get('lift_home', 0.0),
                planning_retries=self._cfg.get('planning_retries', 3),
                jitter_retries=self._cfg.get('jitter_retries', 3),
                jitter_std=self._cfg.get('jitter_std', 0.01),
            )

            if pick_result != PickResult.SUCCESS:
                self._log.warn(f'[Orchestrator] pick failed on attempt {attempt + 1}')
                self._feedback_cb('returning_to_capture_pose')
                self._gripper.open(arm.value)
                continue  # retry

            # ── 5. Place ──────────────────────────────────────────────
            self._feedback_cb('placing')
            place_result = self._place.place(
                place_pose=place_pose,
                arm=arm,
                approach_height=self._cfg.get('approach_height', 0.10),
                lift_home=self._cfg.get('lift_home', 0.0),
                planning_retries=self._cfg.get('planning_retries', 3),
                jitter_retries=self._cfg.get('jitter_retries', 3),
                jitter_std=self._cfg.get('jitter_std', 0.01),
            )

            if place_result != PlaceResult.SUCCESS:
                return OrchestratorResult.PLANNING_FAILED

            # ── 6. Return home ────────────────────────────────────────
            self._feedback_cb('returning_home')
            self._moveit.move_to_home(arm=arm)

            self._log.info('[Orchestrator] pick and place SUCCEEDED')
            return OrchestratorResult.SUCCESS

        self._log.error(f'[Orchestrator] all {max_retries + 1} attempts failed')
        return OrchestratorResult.GRASP_FAILED

    # ── Internal helpers ──────────────────────────────────────────────

    def _move_to_capture_pose(self) -> bool:
        pos = self._cfg.get('capture_pose_position', [0.35, -0.20, 1.20])
        ori = self._cfg.get('capture_pose_orientation', [0.086, -0.173, 0.015, 0.981])
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = pos
        pose.orientation.x, pose.orientation.y = ori[0], ori[1]
        pose.orientation.z, pose.orientation.w = ori[2], ori[3]
        result = self._moveit.move_to_pose(pose, arm=Arm.RIGHT)
        if result != MoveResult.SUCCEEDED:
            self._log.error(f'[Orchestrator] move to capture pose failed: {result.value}')
            return False
        return True

    def _wait_for_gpd(self) -> list[Pose]:
        """Subscribe to gpd_topic and return poses from first message received."""
        topic   = self._cfg.get('gpd_topic', '/gpd/grasp_poses')
        timeout = self._cfg.get('gpd_timeout', 30.0)

        from geometry_msgs.msg import PoseArray
        poses: list[Pose] = []

        def _cb(msg: PoseArray):
            if not poses:
                poses.extend(msg.poses)
                self._log.info(f'[Orchestrator] received {len(poses)} GPD candidates')

        sub      = self._node.create_subscription(PoseArray, topic, _cb, 10)
        deadline = time.time() + timeout

        while not poses and time.time() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.1)

        self._node.destroy_subscription(sub)

        if not poses:
            self._log.error(f'[Orchestrator] no GPD poses within {timeout}s')

        return poses

    def _select_arm(self, poses: list[Pose]) -> Optional[tuple[Arm, Pose]]:
        """Y-threshold arm selection with IK reachability confirmation."""
        y_threshold = self._cfg.get('y_threshold', 0.0)

        for pose in poses:
            primary   = Arm.LEFT if pose.position.y >= y_threshold else Arm.RIGHT
            secondary = Arm.RIGHT if primary == Arm.LEFT else Arm.LEFT

            for arm in (primary, secondary):
                if self._moveit.check_reachable(pose, arm=arm):
                    self._log.info(
                        f'[Orchestrator] selected {arm.value} arm '
                        f'(pose y={pose.position.y:.3f})'
                    )
                    return arm, pose

        self._log.error('[Orchestrator] no reachable arm for any GPD candidate')
        return None
```

- [ ] **Step 2: Build and smoke-check**

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select ai_worker_manipulation
source install/setup.bash
python3 -c "from ai_worker_manipulation.skill_primitives.pick_and_place_orchestrator import PickAndPlaceOrchestrator, OrchestratorResult; print('OK')"
```

Expected: `OK`

- [ ] **Step 3: Commit**

```bash
git add ai_worker_manipulation/skill_primitives/pick_and_place_orchestrator.py
git commit -m "[pick_and_place_orchestrator] add end-to-end orchestrator with arm selection and retry"
```

---

## Task 7: Standalone Hardcoded Test

Test the orchestrator directly with hardcoded poses — no Action Server, no perception team needed.

**Files:**
- Create: `ai_worker_manipulation/tests/test_pick_and_place_orchestrator.py`
- Modify: `setup.py` (add entry point)

- [ ] **Step 1: Create the test script**

```python
#!/usr/bin/env python3
"""
Standalone orchestrator test with hardcoded GPD output.
No Action Server or perception team needed.

Usage:
  ros2 run ai_worker_manipulation test_pick_and_place_orchestrator

Update GRASP_POSITION / GRASP_ORIENTATION from demo_gpd_pick_place.py output.
"""

import rclpy
from geometry_msgs.msg import Pose

from ai_worker_manipulation.robot_interface.moveit_client import MoveItClient, Arm
from ai_worker_manipulation.robot_interface.gripper_controller import GripperInterface
from ai_worker_manipulation.skill_primitives.grasp_assessment import GraspAssessment
from ai_worker_manipulation.skill_primitives.grasp_skill import GraspSkill
from ai_worker_manipulation.skill_primitives.pick_skill import PickSkill
from ai_worker_manipulation.skill_primitives.place_skill import PlaceSkill
from ai_worker_manipulation.skill_primitives.pick_and_place_orchestrator import (
    PickAndPlaceOrchestrator,
    OrchestratorResult,
)

# ---------------------------------------------------------------------------
# Update these from test_gpd_wrist_live.py visualization output
# ---------------------------------------------------------------------------
GRASP_POSITION    = [0.3014, -0.2542, 0.8930]
GRASP_ORIENTATION = [0.3846, -0.0636, 0.0507, 0.9195]  # [x, y, z, w]

PLACE_POSITION    = [0.30, 0.20, 1.00]
PLACE_ORIENTATION = [0.0, 0.0, 0.0, 1.0]

CONFIG = {
    'y_threshold':        0.0,
    'gpd_topic':          '/gpd/grasp_poses',
    'gpd_timeout':        5.0,   # short — we inject the pose manually below
    'capture_pose_position':    [0.35, -0.20, 1.20],
    'capture_pose_orientation': [0.086, -0.173, 0.015, 0.981],
    'max_retries':        1,
    'approach_height':    0.10,
    'lift_home':          0.0,
    'planning_retries':   3,
    'jitter_retries':     3,
    'jitter_std':         0.01,
}
# ---------------------------------------------------------------------------


def make_pose(pos, ori) -> Pose:
    p = Pose()
    p.position.x = float(pos[0])
    p.position.y = float(pos[1])
    p.position.z = float(pos[2])
    p.orientation.x = float(ori[0])
    p.orientation.y = float(ori[1])
    p.orientation.z = float(ori[2])
    p.orientation.w = float(ori[3])
    return p


def main():
    rclpy.init()
    node    = rclpy.create_node('test_pick_and_place_orchestrator')
    log     = node.get_logger()

    moveit     = MoveItClient(node)
    gripper    = GripperInterface(node=node)
    assessment = GraspAssessment(node)
    grasp      = GraspSkill(node, gripper, assessment)
    pick       = PickSkill(node, moveit, gripper, grasp)
    place      = PlaceSkill(node, moveit, gripper)

    orchestrator = PickAndPlaceOrchestrator(
        node=node,
        moveit=moveit,
        gripper=gripper,
        pick_skill=pick,
        place_skill=place,
        config=CONFIG,
        feedback_cb=lambda phase: log.info(f'[TEST] phase → {phase}'),
    )

    # Inject hardcoded grasp pose directly — bypasses GPD wait
    grasp_pose  = make_pose(GRASP_POSITION, GRASP_ORIENTATION)
    place_pose  = make_pose(PLACE_POSITION, PLACE_ORIENTATION)

    # Directly test pick + place without GPD wait
    log.info('Testing PickSkill directly...')
    from ai_worker_manipulation.robot_interface.moveit_client import Arm
    pick_result = pick.pick(grasp_pose, arm=Arm.RIGHT, object_name='ETC')
    log.info(f'Pick result: {pick_result.value}')

    if pick_result.value == 'success':
        log.info('Testing PlaceSkill directly...')
        place_result = place.place(place_pose, arm=Arm.RIGHT)
        log.info(f'Place result: {place_result.value}')

    moveit.destroy()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 2: Add entry point to setup.py**

Add to `console_scripts` in `setup.py`:

```python
'test_pick_and_place_orchestrator = ai_worker_manipulation.tests.test_pick_and_place_orchestrator:main',
'gpd_grasp_publisher              = ai_worker_manipulation.gpd_grasp_publisher:main',
'test_gpd_wrist_live              = ai_worker_manipulation.tests.test_gpd_wrist_live:main',
```

- [ ] **Step 3: Build**

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select ai_worker_manipulation
source install/setup.bash
```

- [ ] **Step 4: Run the test (with sim running)**

Terminal 1:
```bash
ros2 launch ffw_bringup ffw_bg2_follower_ai_gazebo.launch.py
```
Terminal 2:
```bash
ros2 launch ffw_moveit_config moveit.launch.py use_sim:=true
```
Terminal 3:
```bash
ros2 run ai_worker_manipulation test_pick_and_place_orchestrator
```

Expected: logs show `phase → picking`, `phase → placing`, pick + place results logged.

- [ ] **Step 5: Commit**

```bash
git add ai_worker_manipulation/tests/test_pick_and_place_orchestrator.py ai_worker_manipulation/setup.py
git commit -m "[test_pick_and_place_orchestrator] add standalone hardcoded test"
```

---

## Task 8: Action Server Node

Thin wrapper. All logic stays in the orchestrator.

**Files:**
- Create: `ai_worker_manipulation/nodes/__init__.py`
- Create: `ai_worker_manipulation/nodes/pick_and_place_server.py`
- Modify: `setup.py` (add entry point)

- [ ] **Step 1: Create nodes/__init__.py**

```bash
touch ai_worker_manipulation/ai_worker_manipulation/nodes/__init__.py
```

- [ ] **Step 2: Create pick_and_place_server.py**

```python
#!/usr/bin/env python3
"""
pick_and_place_server.py
-------------------------
Thin ROS 2 Action Server wrapping PickAndPlaceOrchestrator.
All logic lives in the orchestrator — this node only handles ROS interfaces.
"""

import yaml

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

import ament_index_python.packages as ament

from ai_worker_manipulation_msgs.action import PickAndPlace

from ai_worker_manipulation.robot_interface.moveit_client import MoveItClient
from ai_worker_manipulation.robot_interface.gripper_controller import GripperInterface
from ai_worker_manipulation.skill_primitives.grasp_assessment import GraspAssessment
from ai_worker_manipulation.skill_primitives.grasp_skill import GraspSkill
from ai_worker_manipulation.skill_primitives.pick_skill import PickSkill
from ai_worker_manipulation.skill_primitives.place_skill import PlaceSkill
from ai_worker_manipulation.skill_primitives.pick_and_place_orchestrator import (
    PickAndPlaceOrchestrator,
    OrchestratorResult,
)


def _load_config() -> dict:
    pkg_dir = ament.get_package_share_directory('ai_worker_manipulation')
    path    = f'{pkg_dir}/config/pick_and_place.yaml'
    with open(path) as f:
        return yaml.safe_load(f)['pick_and_place']


class PickAndPlaceServer(Node):

    def __init__(self):
        super().__init__('pick_and_place_server')

        config     = _load_config()
        moveit     = MoveItClient(self)
        gripper    = GripperInterface(node=self)
        assessment = GraspAssessment(self)
        grasp      = GraspSkill(self, gripper, assessment)
        pick       = PickSkill(self, moveit, gripper, grasp)
        place      = PlaceSkill(self, moveit, gripper)

        self._orchestrator = PickAndPlaceOrchestrator(
            node=self,
            moveit=moveit,
            gripper=gripper,
            pick_skill=pick,
            place_skill=place,
            config=config,
        )

        self._action_server = ActionServer(
            self,
            PickAndPlace,
            'pick_and_place',
            self._execute_cb,
        )
        self.get_logger().info('[PickAndPlaceServer] ready')

    def _execute_cb(self, goal_handle):
        goal = goal_handle.request

        def _feedback(phase: str):
            fb       = PickAndPlace.Feedback()
            fb.phase = phase
            goal_handle.publish_feedback(fb)

        self._orchestrator._feedback_cb = _feedback

        result_enum = self._orchestrator.run(
            object_class=goal.object_class,
            place_pose=goal.place_pose.pose,
        )

        result         = PickAndPlace.Result()
        result.success = result_enum == OrchestratorResult.SUCCESS
        result.failure_reason = '' if result.success else result_enum.value

        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return result


def main():
    rclpy.init()
    node = PickAndPlaceServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 3: Add entry point to setup.py**

Add to `console_scripts`:
```python
'pick_and_place_server = ai_worker_manipulation.nodes.pick_and_place_server:main',
```

- [ ] **Step 4: Add msgs dependency to package.xml**

Already done in Task 1 Step 6. Verify it's there:
```bash
grep ai_worker_manipulation_msgs ai_worker_manipulation/package.xml
```

Expected: `<depend>ai_worker_manipulation_msgs</depend>`

- [ ] **Step 5: Build both packages**

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select ai_worker_manipulation_msgs ai_worker_manipulation
source install/setup.bash
```

Expected: both packages build with no errors.

- [ ] **Step 6: Smoke-check server starts**

With sim running (Terminals 1 + 2 from Task 7 Step 4):
```bash
ros2 run ai_worker_manipulation pick_and_place_server
```

Expected: `[PickAndPlaceServer] ready` in logs. No crashes.

- [ ] **Step 7: Verify action is available**

```bash
ros2 action list
```

Expected: `/pick_and_place` appears in the list.

- [ ] **Step 8: Commit**

```bash
git add ai_worker_manipulation/ai_worker_manipulation/nodes/ ai_worker_manipulation/setup.py
git commit -m "[pick_and_place_server] add thin Action Server node"
```

---

## Task 9: GPD Topic Reconciliation

`gpd_grasp_publisher.py` publishes to `/gpd/grasps` and `/gpd/best_grasp`.
The orchestrator expects `/gpd/grasp_poses`. One of them must change.

**Decision:** Update `gpd_grasp_publisher.py` to also publish to `/gpd/grasp_poses`
(a `PoseArray` of all filtered grasps, matching what the orchestrator subscribes to).
Keep `/gpd/best_grasp` and `/gpd/grasps` for backwards compatibility with test scripts.

**Files:**
- Modify: `ai_worker_manipulation/gpd_grasp_publisher.py`

- [ ] **Step 1: Add `/gpd/grasp_poses` publisher to GpdGraspPublisher**

In `gpd_grasp_publisher.py`, in `__init__`, add:
```python
self._pub_poses = self.create_publisher(PoseArray, '/gpd/grasp_poses', 10)
```

In `_process()`, after building the filtered poses list, add a publish call:
```python
poses_msg        = PoseArray()
poses_msg.header.frame_id = 'base_link'
poses_msg.poses  = [grasp_to_pose(g) for g in filtered_grasps]
self._pub_poses.publish(poses_msg)
```

Find the exact location by searching for where `self._pub_all.publish(...)` is called
and mirror it for `self._pub_poses`.

- [ ] **Step 2: Build and verify**

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select ai_worker_manipulation
source install/setup.bash
```

- [ ] **Step 3: Commit**

```bash
git add ai_worker_manipulation/ai_worker_manipulation/gpd_grasp_publisher.py
git commit -m "[gpd_grasp_publisher] add /gpd/grasp_poses publisher for orchestrator"
```

---

## Open Items (not in this plan)

- Capture pose uses EE pose from `move_wrist_capture_pose.py` defaults (x=0.35, y=-0.20, z=1.20). Update `capture_pose_position` / `capture_pose_orientation` in yaml if the real robot capture position differs.
- `move_to_home()` in `moveit_client.py` uses all-zeros — separate fix needed once real home joint config is defined.
- Real-hardware calibration: `approach_height`, `pre_grasp_offset`, `y_threshold` all need tuning on real robot. Change values in `config/pick_and_place.yaml` only — no code changes required.