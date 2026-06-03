#
# Executes a single pick sequence given a pre-filtered grasp pose.
# Does NOT own retry logic or perception re-query — those belong in the action server.
#
# Contract:
#   SUCCESS → arm at pre_grasp pose, object in gripper
#   FAILURE → arm at pre_grasp pose, gripper open, bin as undisturbed as possible

from enum import Enum

from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Pose

from ai_worker_manipulation.robot_interface.moveit_client import (
    MoveItClient,
    Arm,
    MoveResult,
)
from ai_worker_manipulation.robot_interface.gripper_controller import GripperInterface
from ai_worker_manipulation.skill_primitives.grasp_skill import GraspSkill, GraspResult


_PRE_GRASP_OFFSET = 0.15  # metres — step back along approach vector before entering bin


class PickResult(Enum):
    SUCCESS = 'success'
    FAILURE = 'failure'
    TIMEOUT = 'timeout'


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
    ) -> PickResult:
        """
        Execute a pick sequence for the given grasp pose.

        Parameters
        ----------
        grasp_pose       : Target grasp pose (pre-filtered by caller).
        arm              : Which arm to use.
        object_name      : LUT key for grasp assessment thresholds.
        pre_grasp_offset : Distance in metres to step back along approach vector.

        Returns
        -------
        PickResult : SUCCESS or FAILURE.
                     On both outcomes the arm is at pre_grasp and gripper state
                     matches the outcome (closed on SUCCESS, open on FAILURE).
        """
        side    = arm.value
        pre     = pre_grasp_of(grasp_pose, offset=pre_grasp_offset)

        self._log.info(f'[PickSkill] [{side}] starting pick — object={object_name!r}')

        # ── 1. Open gripper before entering the bin ───────────────────
        self._gripper.open(side)
        self._gripper.wait_until_executed()

        # ── 2. Move to pre-grasp pose (free space) ────────────────────
        self._log.info(f'[PickSkill] [{side}] moving to pre-grasp...')
        result = self._moveit.move_to_pose(pre, arm=arm)
        if result != MoveResult.SUCCEEDED:
            self._log.error(
                f'[PickSkill] [{side}] pre-grasp move failed — {result.value}'
            )
            return PickResult.FAILURE

        # ── 3. Cartesian approach into the bin ────────────────────────
        # Straight-line approach minimises disturbance to stacked objects.
        self._log.info(f'[PickSkill] [{side}] cartesian approach to grasp pose...')
        result = self._moveit.move_cartesian(grasp_pose, arm=arm)
        if result != MoveResult.SUCCEEDED:
            self._log.error(
                f'[PickSkill] [{side}] cartesian approach failed — {result.value}'
            )
            self._moveit.move_to_pose(pre, arm=arm)
            return PickResult.FAILURE

        # ── 4. Grasp + assess AT grasp pose (before any lift) ─────────
        # If the grasp is unstable, GraspSkill re-opens automatically and we
        # retract cleanly — the object drops back into the bin undisturbed.
        self._log.info(f'[PickSkill] [{side}] grasping...')
        grasp_result = self._grasp.grasp(side, object_name=object_name)

        if grasp_result != GraspResult.SUCCESS:
            self._log.error(
                f'[PickSkill] [{side}] grasp FAILED — {grasp_result.value} — retracting'
            )
            # Gripper already re-opened by GraspSkill — just retract
            self._moveit.move_to_pose(pre, arm=arm)
            return PickResult.FAILURE

        # ── 5. Lift to pre-grasp (OMPL — reliable with object in hand) ─
        self._log.info(f'[PickSkill] [{side}] grasp stable — lifting...')
        result = self._moveit.move_to_pose(pre, arm=arm)
        if result != MoveResult.SUCCEEDED:
            self._log.error(
                f'[PickSkill] [{side}] lift failed — {result.value}'
            )
            # Object in gripper but arm stuck — report failure, let action server decide
            return PickResult.FAILURE

        self._log.info(f'[PickSkill] [{side}] pick SUCCEEDED')
        return PickResult.SUCCESS
