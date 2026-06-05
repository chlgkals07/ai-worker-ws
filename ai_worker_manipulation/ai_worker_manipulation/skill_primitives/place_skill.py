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
    Mode 1 (cartesian): pre_place → cartesian → open gripper → cartesian retract
    Mode 2 (lift):      lift_home → pre_place_z+offset → lower lift → open gripper → raise lift
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

        # ── Mode 1: cartesian approach ────────────────────────────────────
        self._log.info(f'[PlaceSkill] [{side}] Mode 1 (cartesian)')
        result = _move_with_retry(
            lambda p, _arm=arm: self._moveit.move_to_pose(p, arm=_arm),
            pre, self._log, f'PlaceSkill/{side}/pre_place',
            same_retries=planning_retries,
            jitter_retries=jitter_retries,
            jitter_std=jitter_std,
        )
        if result != MoveResult.SUCCEEDED:
            self._log.warn(f'[PlaceSkill] [{side}] pre-place failed → Mode 2')
            return self._place_lift(
                place_pose, arm, approach_height, lift_home,
                planning_retries, jitter_retries, jitter_std,
            )

        result = self._moveit.move_cartesian(place_pose, arm=arm)
        if result != MoveResult.SUCCEEDED:
            self._log.warn(f'[PlaceSkill] [{side}] cartesian approach failed → Mode 2')
            self._moveit.move_to_pose(pre, arm=arm)
            return self._place_lift(
                place_pose, arm, approach_height, lift_home,
                planning_retries, jitter_retries, jitter_std,
            )

        self._gripper.open(side)
        self._gripper.wait_until_executed()
        self._gripper.wait_motion()

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

        # Hover directly above the place point — lift joint provides Z descent.
        pre_lift = Pose()
        pre_lift.position.x  = place_pose.position.x
        pre_lift.position.y  = place_pose.position.y
        pre_lift.position.z  = place_pose.position.z + approach_height
        pre_lift.orientation = place_pose.orientation

        self._moveit.move_lift(lift_home)

        result = _move_with_retry(
            lambda p, _arm=arm: self._moveit.move_to_pose(p, arm=_arm),
            pre_lift, self._log, f'PlaceSkill/{side}/pre_lift',
            same_retries=planning_retries,
            jitter_retries=jitter_retries,
            jitter_std=jitter_std,
        )
        if result != MoveResult.SUCCEEDED:
            self._log.error(f'[PlaceSkill] [{side}] Mode 2 pre-lift move failed')
            return PlaceResult.FAILURE

        self._moveit.move_lift(lift_home - approach_height)

        self._gripper.open(side)
        self._gripper.wait_until_executed()
        self._gripper.wait_motion()

        self._moveit.move_lift(lift_home)
        self._log.info(f'[PlaceSkill] [{side}] place SUCCEEDED (lift)')
        return PlaceResult.SUCCESS
