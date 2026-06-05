# Pick and Place Pipeline — Code Guide & Test Procedures

**Branch:** `moveit-fix`
**Last updated:** 2026-06-05

---

## What Was Built

An end-to-end pick-and-place pipeline for the FFW SG2 robot:

- GPD-based grasp detection (wrist camera PCD → grasp poses)
- Automatic arm selection (Y-threshold + IK reachability)
- Pick with cartesian approach → lift fallback
- Grasp assessment (did we actually grab it?)
- Place with cartesian approach → lift fallback
- Planning failure retry (same pose → jittered pose)
- ROS 2 Action Server for the mission team

---

## Architecture

```
Mission Team
    │
    │  ros2 action send_goal /pick_and_place
    │  Goal: { object_class, place_pose }
    ▼
nodes/pick_and_place_server.py       ← thin ROS 2 Action Server
    │
    ▼
skill_primitives/pick_and_place_orchestrator.py   ← all logic
    │
    ├── skill_primitives/pick_skill.py     ← Mode 1 (cartesian) + Mode 2 (lift)
    │       └── skill_primitives/grasp_skill.py
    │               └── skill_primitives/grasp_assessment.py
    ├── skill_primitives/place_skill.py    ← Mode 1 (cartesian) + Mode 2 (lift)
    │
    └── robot_interface/moveit_client.py   ← arm motion (move_to_pose, move_cartesian, move_lift, check_reachable)
            robot_interface/gripper_controller.py

gpd_grasp_publisher.py   ← PCD → GPD → /gpd/grasp_poses
```

**One-way dependency:** nodes → skill_primitives → robot_interface. Never reversed.

---

## File-by-File Explanation

### `config/pick_and_place.yaml`

**Read this first.** All tunable values live here — nothing is hardcoded in the logic.

```yaml
pick_and_place:
  y_threshold: 0.0              # arm selection: pose.y >= 0 → left, else right
  gpd_topic: /gpd/grasp_poses   # topic the orchestrator waits on
  gpd_timeout: 30.0             # seconds to wait for GPD output
  capture_pose_position: [0.35, -0.20, 1.20]       # EE position for PCD capture
  capture_pose_orientation: [0.086, -0.173, 0.015, 0.981]
  max_retries: 1                # retry pick once on grasp failure
  pre_grasp_offset: 0.15        # metres back along approach vector (cartesian mode)
  approach_height: 0.10         # metres above grasp/place (lift mode)
  lift_home: 0.0                # lift_joint fully raised position
  planning_retries: 3           # same-pose OMPL retries before jitter
  jitter_retries: 3             # jittered-pose retries
  jitter_std: 0.01              # gaussian std dev for jitter (metres)
```

---

### `robot_interface/moveit_client.py` — New: `check_reachable()`

Checks if a pose has a valid IK solution for a given arm **without moving the robot**.

Uses `compute_ik_async()` from pymoveit2 (not `compute_ik()` — that calls `rclpy.spin_once()` internally which conflicts with the background executor thread). Polls the future manually, with a 5-second timeout (`_IK_TIMEOUT`).

```python
reachable = moveit.check_reachable(grasp_pose, arm=Arm.RIGHT)  # True / False
```

Used by the orchestrator's arm selection step.

---

### `skill_primitives/pick_skill.py` — Upgraded

#### `_move_with_retry(move_fn, pose, log, label, ...)`

Module-level helper. Retries a move callable:
1. Same pose, up to `planning_retries` times (OMPL randomness may succeed)
2. Gaussian-jittered pose, up to `jitter_retries` times

```python
result = _move_with_retry(
    lambda p, _arm=arm: moveit.move_to_pose(p, arm=_arm),
    target_pose, log, 'label',
)
```

Note the `_arm=arm` default capture — prevents Python late-binding closure bugs.

#### `PickSkill.pick()` — Two modes

**Mode 1 (cartesian, preferred):**
```
open gripper
→ _move_with_retry to pre_grasp (OMPL)
→ move_cartesian to grasp_pose (Pilz LIN)
→ GraspSkill.grasp() + assess
→ on success: cartesian retract to pre_grasp
→ on failure: OMPL retract to pre_grasp → FAILURE
```

**Mode 2 (lift fallback, triggered if Mode 1 pre-grasp or cartesian fails):**
```
open gripper
→ move_lift to lift_home (0.0 m = fully raised)
→ _move_with_retry to pre_lift (grasp_pose.z + approach_height, OMPL)
→ move_lift down by approach_height  (descend onto object)
→ GraspSkill.grasp() + assess
→ on success: move_lift back to lift_home → SUCCESS
→ on failure: open gripper + move_lift to lift_home → FAILURE
```

---

### `skill_primitives/place_skill.py` — Upgraded

Mirrors pick_skill exactly. Mode 1 (cartesian) → Mode 2 (lift). Imports `_move_with_retry` and constants from `pick_skill`.

Key difference: no grasp assessment — just open gripper at the place pose. `wait_motion()` is called after `open()` to ensure the gripper physically opens before retracting.

---

### `skill_primitives/pick_and_place_orchestrator.py` — New

The brain. Pure Python — no ROS Action Server. Testable with hardcoded poses.

#### `run(object_class, place_pose, feedback_cb=None)`

Full sequence:

```
1. move to capture pose (EE pose from config, Arm.RIGHT always — wrist camera)
2. subscribe to /gpd/grasp_poses, wait up to gpd_timeout seconds
   ↳ uses threading.Event (NOT rclpy.spin_once — would conflict with executor thread)
3. arm selection:
   ↳ for each pose (up to 5): Y-threshold → primary arm, check_reachable()
   ↳ if primary not reachable, try secondary arm
   ↳ if nothing reachable → NO_REACHABLE_ARM
4. PickSkill.pick(grasp_pose, arm, object_class, ...)
   ↳ on failure: open gripper, wait, retry from step 1 (up to max_retries)
   ↳ on exhaustion → GRASP_FAILED
5. PlaceSkill.place(place_pose, arm, ...)
   ↳ on failure: open gripper, move_to_home → PLANNING_FAILED
6. move_to_home
→ SUCCESS
```

#### Why `threading.Event` in `_wait_for_gpd()`

`MoveItClient` runs a `MultiThreadedExecutor` in a background thread. That executor drives ALL ROS callbacks, including the `/gpd/grasp_poses` subscription. Calling `rclpy.spin_once()` from the main thread while an executor is running causes conflicts. Instead:

```python
received = threading.Event()
sub = node.create_subscription(PoseArray, topic, lambda msg: received.set(), 10)
received.wait(timeout=30.0)   # background executor will fire the callback
node.destroy_subscription(sub)
```

---

### `nodes/pick_and_place_server.py` — New

~100 lines. Thin wrapper only.

- Loads `pick_and_place.yaml` at startup
- Constructs all components once (`MoveItClient`, `GripperInterface`, etc.)
- `ActionServer` on `'pick_and_place'`
- `goal_callback` rejects new goals while busy (threading.Lock)
- `_execute_cb` passes a per-goal feedback callback to `orchestrator.run()`
- Calls `moveit.destroy()` on node shutdown

---

### `gpd_grasp_publisher.py`

Subscribes to `/perception/wrist/target_pcd/<class_name>` (PointCloud2), runs the GPD binary as a subprocess, parses stdout, filters candidates, publishes:

- `/gpd/best_grasp` (PoseStamped) — highest-score pose
- `/gpd/grasps` (PoseArray) — all filtered poses
- `/gpd/grasp_poses` (PoseArray) — **same as /gpd/grasps**, added for orchestrator compatibility

---

### `ai_worker_manipulation_msgs/action/PickAndPlace.action`

```
# Goal — sent by mission team
string object_class
geometry_msgs/PoseStamped place_pose
---
# Result
bool success
string failure_reason    # "success" | "grasp_failed" | "planning_failed" |
                         # "no_gpd_candidates" | "no_reachable_arm" | "timeout"
---
# Feedback
string phase             # "moving_to_capture_pose" | "waiting_for_gpd" | "picking" |
                         # "assessing_grasp" | "placing" | "returning_home" |
                         # "returning_to_capture_pose"
```

---

## Test Procedures

### Prerequisites

```bash
# 1. Start the container
cd ~/ai_worker_dev/ai_worker_ws/src/ai_worker/docker
./container.sh start
./container.sh enter

# 2. Install pick_ik (every session)
apt install ros-jazzy-pick-ik

# 3. Source
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# 4. Build
cd ~/ros2_ws
colcon build --symlink-install --packages-select ai_worker_manipulation_msgs ai_worker_manipulation
source install/setup.bash
```

---

### Test A — Standalone Pick + Place (no perception / no mission team)

The simplest end-to-end test. Hardcoded grasp pose — update values from `test_gpd_wrist_live.py` output.

**Terminal 1 — Gazebo:**
```bash
ros2 launch ffw_bringup ffw_bg2_follower_ai_gazebo.launch.py
```

**Terminal 2 — MoveIt:**
```bash
ros2 launch ffw_moveit_config moveit.launch.py use_sim:=true
```

**Terminal 3 — Test:**
```bash
ros2 run ai_worker_manipulation test_pick_and_place_orchestrator
```

Edit `GRASP_POSITION` / `GRASP_ORIENTATION` at the top of `tests/test_pick_and_place_orchestrator.py` with real values from the wrist camera before running.

Expected output:
```
[INFO] --- Testing PickSkill ---
[INFO] [PickSkill] [right] starting pick — object='ETC'
[INFO] [PickSkill] [right] Mode 1 (cartesian)
...
[INFO] Pick result: success
[INFO] --- Testing PlaceSkill ---
...
[INFO] Place result: success
```

---

### Test B — GPD Wrist Visualization (offline, no robot needed)

Verify GPD is detecting good grasp poses from a saved PCD.

```bash
# Inside container, with a wrist_outputs folder present
python3 ~/ros2_ws/src/ai_worker/ai_worker_manipulation/ai_worker_manipulation/tests/test_gpd_wrist_live.py

# Or specify a folder directly:
python3 test_gpd_wrist_live.py --data /root/ros2_ws/src/ai_worker/wrist_outputs_XXXXXX
```

Copy the printed `GRASP_POSITION` / `GRASP_ORIENTATION` values into `test_pick_and_place_orchestrator.py` before Test A.

---

### Test C — Full Action Server (mission team interface)

**Terminals 1 + 2** — same as Test A (Gazebo + MoveIt).

**Terminal 3 — GPD publisher:**
```bash
ros2 run ai_worker_manipulation gpd_grasp_publisher
```

**Terminal 4 — Action Server:**
```bash
ros2 run ai_worker_manipulation pick_and_place_server
```

**Terminal 5 — Send a goal:**
```bash
ros2 action send_goal /pick_and_place ai_worker_manipulation_msgs/action/PickAndPlace \
  "{object_class: 'ETC', place_pose: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.30, y: 0.20, z: 1.00}, orientation: {w: 1.0}}}}"
```

Watch feedback phases in Terminal 4:
```
moving_to_capture_pose → waiting_for_gpd → picking → placing → returning_home
```

---

## Tuning Guide

All in `config/pick_and_place.yaml`:

| Parameter | When to change |
|---|---|
| `approach_height` | Object is too tall / arm overshoots in lift mode |
| `pre_grasp_offset` | Cartesian approach starts too far / too close |
| `y_threshold` | Left/right arm selection boundary needs shifting |
| `planning_retries` | Too many failed plans — increase; too slow — decrease |
| `jitter_std` | Jitter not helping — increase slightly (max ~0.02) |
| `capture_pose_position` | Wrist camera capture position changed |
| `lift_home` | Lift joint "home" is not fully raised |

---

## Known Limitations (PoC)

- `move_to_home()` in `moveit_client.py` uses all-zeros joint config — needs real home pose
- Sim vs real calibration: `approach_height` and `pre_grasp_offset` will likely need tuning
- GPD topic: perception team publishes PCD → `gpd_grasp_publisher` runs GPD → `/gpd/grasp_poses`. Mission team triggers perception externally before calling the Action Server
- `capture_pose_joints` → currently uses EE pose from `move_wrist_capture_pose.py` defaults; update if capture position changes
