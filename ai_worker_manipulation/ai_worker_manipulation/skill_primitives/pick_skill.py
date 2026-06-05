# skill_primitives/pick_skill.py
#
# Executes a single pick sequence given a pre-filtered grasp pose.
# Does NOT own retry logic or perception re-query — those belong in the action server.
#
# Contract:
#   SUCCESS → arm at pre_grasp pose, object in gripper
#   FAILURE → arm at pre_grasp pose, gripper open, bin as undisturbed as possible

import random
from enum import Enum
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


def pre_grasp_of(pose: Pose, offset: float = _PRE_GRASP_OFFSET) -> Pose:
    """Return a pose stepped back `offset` metres along the end-effector approach vector.

    The approach vector is the x-axis of the end-effector frame derived from the
    pose quaternion. Used for both pre-grasp and pre-place offsets.
    """
    q = pose.orientation
    rotation_matrix = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    approach_vector = rotation_matrix[:, 0]  # x-axis of EE frame

    pre = Pose()
    pre.position.x    = pose.position.x - offset * approach_vector[0]
    pre.position.y    = pose.position.y - offset * approach_vector[1]
    pre.position.z    = pose.position.z - offset * approach_vector[2]
    pre.orientation   = pose.orientation
    return pre


def _move_with_retry(
    move_fn: Callable[[Pose], MoveResult],
    pose: Pose,
    log,
    label: str,
    same_retries: int = _PLANNING_RETRIES,
    jitter_retries: int = _JITTER_RETRIES,
    jitter_std: float = _JITTER_STD,
) -> MoveResult:
    """Retry move_fn(pose) with same pose, then with gaussian-jittered pose."""
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


class PickResult(Enum):
    SUCCESS = 'success'
    FAILURE = 'failure'
    TIMEOUT = 'timeout'


class PickSkill:
    """
    Single pick sequence: open → pre-grasp → approach → grasp+assess → lift.

    Grasp quality is assessed immediately after closing — before any lift.
    If the grasp is unstable the gripper re-opens and the arm retracts cleanly,
    leaving the bin as undisturbed as possible for a retry.

    The caller (action server) is responsible for:
      - Providing a pre-filtered single grasp pose
      - Deciding whether and how to retry (including re-querying perception)
      - Moving to an inspect pose between retries if needed
    """

    def __init__(
        self,
        node,
        moveit: MoveItClient,
        gripper: GripperInterface,
        grasp_skill: GraspSkill,
    ):
        self._node       = node
        self._log        = node.get_logger()
        self._moveit     = moveit
        self._gripper    = gripper
        self._grasp      = grasp_skill

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
            lambda p, _arm=arm: self._moveit.move_to_pose(p, arm=_arm),
            pre, self._log, f'PickSkill/{side}/pre_grasp',
            same_retries=planning_retries,
            jitter_retries=jitter_retries,
            jitter_std=jitter_std,
        )
        if result != MoveResult.SUCCEEDED:
            self._log.error(f'[PickSkill] [{side}] pre-grasp failed — trying Mode 2')
            return self._pick_lift(
                grasp_pose, arm, object_name, approach_height, lift_home,
                planning_retries, jitter_retries, jitter_std,
            )

        result = self._moveit.move_cartesian(grasp_pose, arm=arm)
        if result != MoveResult.SUCCEEDED:
            self._log.warn(f'[PickSkill] [{side}] cartesian approach failed → Mode 2')
            self._moveit.move_to_pose(pre, arm=arm)
            return self._pick_lift(
                grasp_pose, arm, object_name, approach_height, lift_home,
                planning_retries, jitter_retries, jitter_std,
            )

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

        # Hover directly above the grasp point — no approach-vector offset.
        # The lift joint provides the Z descent, so only Z differs from grasp_pose.
        pre_lift = Pose()
        pre_lift.position.x  = grasp_pose.position.x
        pre_lift.position.y  = grasp_pose.position.y
        pre_lift.position.z  = grasp_pose.position.z + approach_height
        pre_lift.orientation = grasp_pose.orientation

        self._gripper.open(side)
        self._gripper.wait_until_executed()
        self._moveit.move_lift(lift_home)

        result = _move_with_retry(
            lambda p, _arm=arm: self._moveit.move_to_pose(p, arm=_arm),
            pre_lift, self._log, f'PickSkill/{side}/pre_lift',
            same_retries=planning_retries,
            jitter_retries=jitter_retries,
            jitter_std=jitter_std,
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
