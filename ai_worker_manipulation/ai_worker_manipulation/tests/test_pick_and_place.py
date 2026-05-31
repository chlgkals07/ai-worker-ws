"""
Pick and Place test — tests PickSkill and PlaceSkill on the real robot.

GPD is not yet integrated. Fill in GRASP_POSE and PLACE_POSE manually before
running. Use Section 0 (current state) to read the FK pose of your arm and
calibrate the values below.

How to get real values:
  1. Run this test first — Section 0 prints current FK poses and joint positions
  2. Jog the arm to the grasp position in RViz, read FK pose, fill in GRASP_POSE
  3. Jog the arm to the place position, read FK pose, fill in PLACE_POSE
  4. Re-run to test the full sequence

Sections:
  0. Current state   — prints FK poses and joint positions (no motion)
  1. Init            — all components created and ready
  2. Pick only       — open → pre-grasp → approach → grasp → lift
  3. Place only      — pre-place → approach → release → retract
  4. Full sequence   — pick → place

Usage:
    ros2 run ai_worker_manipulation test_pick_and_place
"""

import sys

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Pose

from ai_worker_manipulation.robot_interface.moveit_client import (
    MoveItClient,
    Arm,
    MoveResult,
)
from ai_worker_manipulation.robot_interface.gripper_controller import GripperInterface
from ai_worker_manipulation.robot_interface.gripper_command import GripperCommand
from ai_worker_manipulation.skill_primitives.grasp_assessment import GraspAssessment
from ai_worker_manipulation.skill_primitives.grasp_skill import GraspSkill
from ai_worker_manipulation.skill_primitives.pick_skill import PickSkill, PickResult
from ai_worker_manipulation.skill_primitives.place_skill import PlaceSkill, PlaceResult


# ==============================================================================
# FILL IN BEFORE RUNNING
# Run Section 0 first to read current FK poses, then jog to target positions
# in RViz and fill in the values below.
# Format: (x, y, z, qx, qy, qz, qw)
# ==============================================================================

# GPD Grasp 0 (score=-1492.390) — kept for reference but LIKELY IN CAMERA FRAME.
# Roll=-160.5°, x=0.507 both cause IK failure. Needs camera→base_link transform first.
# GPD_GRASP_0 = (0.5073, -0.1971, 0.9444,  0.7780, 0.5259, -0.3392, 0.0556)

# Safe sim values — top-down approach, within tested workspace
# pre_grasp = (0.200, -0.300, 0.650+0.15) = (0.200, -0.300, 0.800) — 15cm above
GRASP_POSE = (0.200, -0.300, 0.650,  0.0, 0.707, 0.0, 0.707)

# Place pose — top-down approach (quat rotates x-axis downward → pre_place is 15cm ABOVE)
# pre_place = (0.250, -0.150, 0.675 + 0.15) = (0.250, -0.150, 0.825)
PLACE_POSE = (0.250, -0.150, 0.675,  0.0, 0.707, 0.0, 0.707)

# Arm to use
ARM = Arm.RIGHT

# Object name — must match a key in object_lut.json
OBJECT_NAME = 'ETC'

# Pre-grasp / pre-place offset in metres (step back along approach vector)
PRE_OFFSET = 0.15

# Joint config to return to after full sequence — copy HOME_R from test_arm_motion.py
HOME_JOINTS = [0.0, -0.5, 0.0, 0.5, 0.0, 0.0, 0.0]

# ==============================================================================


def make_pose(x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0) -> Pose:
    p = Pose()
    p.position.x, p.position.y, p.position.z       = x, y, z
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
    return p


class PickAndPlaceTest:

    def __init__(
        self,
        node: Node,
        moveit: MoveItClient,
        gripper: GripperInterface,
        pick_skill: PickSkill,
        place_skill: PlaceSkill,
    ):
        self._node        = node
        self._log         = node.get_logger()
        self._moveit      = moveit
        self._gripper     = gripper
        self._pick        = pick_skill
        self._place       = place_skill
        self._passed      = 0
        self._failed      = 0

    # ------------------------------------------------------------------
    # Helpers — same pattern as test_arm_motion.py
    # ------------------------------------------------------------------

    def _ok(self, name: str, detail: str = '') -> None:
        self._log.info(f'[PASS] {name}' + (f'  |  {detail}' if detail else ''))
        self._passed += 1

    def _fail(self, name: str, detail: str = '') -> None:
        self._log.error(f'[FAIL] {name}' + (f'  |  {detail}' if detail else ''))
        self._failed += 1

    def _check(self, name: str, ok: bool, detail: str = '') -> bool:
        (self._ok if ok else self._fail)(name, detail)
        return ok

    def _section(self, title: str) -> None:
        self._log.info('')
        self._log.info('─' * 60)
        self._log.info(f'  {title}')
        self._log.info('─' * 60)

    def _home(self) -> bool:
        """Return arm to home joint config between sections."""
        self._log.info(f'[home] returning to home joints...')
        result = self._moveit.move_to_joints(HOME_JOINTS, arm=ARM, velocity=0.1)
        ok = result == MoveResult.SUCCEEDED
        if not ok:
            self._log.error(f'[home] failed — {result.value}')
        return ok

    # ------------------------------------------------------------------
    # Sections
    # ------------------------------------------------------------------

    def section_current_state(self) -> None:
        """Print FK pose and joints — use to calibrate GRASP_POSE / PLACE_POSE."""
        self._section('0. Current state (read-only, no motion)')

        joints = self._moveit.get_joints(arm=ARM)
        if joints:
            jstr = ', '.join(f'{j:.3f}' for j in joints)
            self._log.info(f'[{ARM.value}] joints : [{jstr}]')
        else:
            self._log.warn(f'[{ARM.value}] joints : unavailable')

        pose = self._moveit.get_pose(arm=ARM)
        if pose:
            p, o = pose.position, pose.orientation
            self._log.info(
                f'[{ARM.value}] FK pose : '
                f'pos=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) '
                f'quat=({o.x:.3f}, {o.y:.3f}, {o.z:.3f}, {o.w:.3f})'
            )
            self._log.info(
                f'[{ARM.value}] Copy → GRASP_POSE = '
                f'({p.x:.3f}, {p.y:.3f}, {p.z:.3f}, '
                f'{o.x:.3f}, {o.y:.3f}, {o.z:.3f}, {o.w:.3f})'
            )
        else:
            self._log.warn(f'[{ARM.value}] FK pose : unavailable')

    def section_pick(self) -> bool:
        self._section('2. Pick only')

        if not self._home():
            self._fail('pick', 'could not reach home before starting')
            return False

        self._log.info(f'GRASP_POSE = {GRASP_POSE}')

        # if GRASP_POSE == (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0):
        #     self._fail('pick', 'GRASP_POSE not filled in — update the values at top of file')
        #     return False

        grasp_pose = make_pose(*GRASP_POSE)
        result     = self._pick.pick(
            grasp_pose,
            arm=ARM,
            object_name=OBJECT_NAME,
            pre_grasp_offset=PRE_OFFSET,
        )

        ok = result == PickResult.SUCCESS
        self._check('pick sequence', ok, f'result={result.value}')

        if ok:
            self._log.info('[pick] object in gripper — opening to release for next test')
            self._gripper.open(ARM.value)
            self._gripper.wait_motion()

        self._home()
        return ok

    def section_place(self) -> bool:
        self._section('3. Place only')

        if not self._home():
            self._fail('place', 'could not reach home before starting')
            return False

        self._log.info(f'PLACE_POSE = {PLACE_POSE}')

        # if PLACE_POSE == (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0):
        #     self._fail('place', 'PLACE_POSE not filled in — update the values at top of file')
        #     return False

        place_pose = make_pose(*PLACE_POSE)
        result     = self._place.place(
            place_pose,
            arm=ARM,
            pre_place_offset=PRE_OFFSET,
        )

        ok = result == PlaceResult.SUCCESS
        self._check('place sequence', ok, f'result={result.value}')

        self._home()
        return ok

    def section_full_sequence(self) -> None:
        self._section('4. Full sequence — pick → place')

        if not self._home():
            self._fail('full sequence', 'could not reach home before starting')
            return

        # if (GRASP_POSE == (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0) or
        #         PLACE_POSE == (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)):
        #     self._fail('full sequence', 'GRASP_POSE or PLACE_POSE not filled in')
        #     return

        grasp_pose = make_pose(*GRASP_POSE)
        place_pose = make_pose(*PLACE_POSE)

        # Pick
        self._log.info('[full] picking...')
        pick_result = self._pick.pick(
            grasp_pose,
            arm=ARM,
            object_name=OBJECT_NAME,
            pre_grasp_offset=PRE_OFFSET,
        )

        if not self._check('pick phase', pick_result == PickResult.SUCCESS,
                           f'result={pick_result.value}'):
            self._home()
            return

        # Place
        self._log.info('[full] placing...')
        place_result = self._place.place(
            place_pose,
            arm=ARM,
            pre_place_offset=PRE_OFFSET,
        )

        self._check('place phase', place_result == PlaceResult.SUCCESS,
                    f'result={place_result.value}')

        self._home()

    # ------------------------------------------------------------------
    # Run
    # ------------------------------------------------------------------

    def run(self) -> int:
        self._log.info('=' * 60)
        self._log.info('Pick and Place Test')
        self._log.info(f'ARM={ARM.value}  OBJECT={OBJECT_NAME}  OFFSET={PRE_OFFSET}m')
        self._log.info('=' * 60)

        self.section_current_state()
        self.section_pick()
        self.section_place()
        self.section_full_sequence()

        self._log.info('')
        self._log.info('--- Final: returning to home ---')
        self._home()

        return self._summary()

    def _summary(self) -> int:
        total = self._passed + self._failed
        self._log.info('')
        self._log.info('=' * 60)
        self._log.info(f'Results: {self._passed}/{total} passed    {self._failed} failed')
        self._log.info('=' * 60)
        return 0 if self._failed == 0 else 1


# ------------------------------------------------------------------


def main():
    rclpy.init()
    node = Node('test_pick_and_place')
    cb   = ReentrantCallbackGroup()

    # MoveItClient creates and spins its own executor — all other
    # subscriptions on this node are driven by the same executor.
    moveit     = MoveItClient(node)
    command    = GripperCommand(node, callback_group=cb)
    gripper    = GripperInterface(node)
    assessment = GraspAssessment(node, callback_group=cb)
    grasp_sk   = GraspSkill(node, gripper, assessment)
    pick_sk    = PickSkill(node, moveit, gripper, grasp_sk)
    place_sk   = PlaceSkill(node, moveit, gripper)

    test = PickAndPlaceTest(node, moveit, gripper, pick_sk, place_sk)

    try:
        code = test.run()
    except Exception as e:
        node.get_logger().error(f'Test aborted: {e}')
        code = 1
    finally:
        moveit.destroy()
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(code)


if __name__ == '__main__':
    main()
