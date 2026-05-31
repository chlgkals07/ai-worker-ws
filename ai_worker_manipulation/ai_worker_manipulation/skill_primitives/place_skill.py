# skill_primitives/place_skill.py
#
# Executes a single place sequence given a pre-determined place pose.
# Does NOT own retry logic — that belongs in the action server.
#
# Contract:
#   SUCCESS → arm at pre_place pose, gripper open, object released
#   FAILURE → arm at pre_place pose (best effort), gripper open

from enum import Enum

from geometry_msgs.msg import Pose

from ai_worker_manipulation.robot_interface.moveit_client import (
    MoveItClient,
    Arm,
    MoveResult,
)
from ai_worker_manipulation.robot_interface.gripper_controller import GripperInterface
from ai_worker_manipulation.skill_primitives.pick_skill import pre_grasp_of


_PRE_PLACE_OFFSET = 0.15  # metres — same offset logic as pre-grasp


class PlaceResult(Enum):
    SUCCESS = 'success'
    FAILURE = 'failure'
    TIMEOUT = 'timeout'


class PlaceSkill:
    """
    Single place sequence: pre-place → cartesian approach → release → cartesian retract.

    Cartesian is used for both approach and retract — straight-line motion avoids
    disturbing already-placed objects in the box.

    The caller (action server) is responsible for:
      - Providing the place pose (box pose or fallback pose)
      - Deciding whether to retry on failure
    """

    def __init__(
        self,
        node,
        moveit: MoveItClient,
        gripper: GripperInterface,
    ):
        self._node    = node
        self._log     = node.get_logger()
        self._moveit  = moveit
        self._gripper = gripper

    def place(
        self,
        place_pose: Pose,
        arm: Arm = Arm.RIGHT,
        pre_place_offset: float = _PRE_PLACE_OFFSET,
    ) -> PlaceResult:
        """
        Execute a place sequence for the given place pose.

        Parameters
        ----------
        place_pose       : Target place pose.
        arm              : Which arm to use.
        pre_place_offset : Distance in metres to step back along approach vector.

        Returns
        -------
        PlaceResult : SUCCESS or FAILURE.
                      On both outcomes the arm is at pre_place and gripper is open.
        """
        side = arm.value
        pre  = pre_grasp_of(place_pose, offset=pre_place_offset)

        self._log.info(f'[PlaceSkill] [{side}] starting place')

        # ── 1. Move to pre-place pose (free space) ────────────────────
        self._log.info(f'[PlaceSkill] [{side}] moving to pre-place...')
        result = self._moveit.move_to_pose(pre, arm=arm)
        if result != MoveResult.SUCCEEDED:
            self._log.error(
                f'[PlaceSkill] [{side}] pre-place move failed — {result.value}'
            )
            return PlaceResult.FAILURE

        # ── 2. Cartesian approach to place pose ───────────────────────
        # Straight-line approach avoids disturbing objects already in the box.
        self._log.info(f'[PlaceSkill] [{side}] cartesian approach to place pose...')
        result = self._moveit.move_cartesian(place_pose, arm=arm)
        if result != MoveResult.SUCCEEDED:
            self._log.error(
                f'[PlaceSkill] [{side}] cartesian approach failed — {result.value}'
            )
            self._moveit.move_to_pose(pre, arm=arm)
            return PlaceResult.FAILURE

        # ── 3. Open gripper — release object ─────────────────────────
        self._log.info(f'[PlaceSkill] [{side}] releasing object...')
        self._gripper.open(side)
        self._gripper.wait_until_executed()
        self._gripper.wait_motion()

        # ── 4. Cartesian retract to pre-place ─────────────────────────
        # Straight-line retract avoids knocking the just-placed object.
        self._log.info(f'[PlaceSkill] [{side}] retracting...')
        result = self._moveit.move_cartesian(pre, arm=arm)
        if result != MoveResult.SUCCEEDED:
            self._log.error(
                f'[PlaceSkill] [{side}] cartesian retract failed — {result.value}'
            )
            self._moveit.move_to_pose(pre, arm=arm)
            return PlaceResult.FAILURE

        self._log.info(f'[PlaceSkill] [{side}] place SUCCEEDED')
        return PlaceResult.SUCCESS
