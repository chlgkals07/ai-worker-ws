"""
pick_and_place_orchestrator.py
-------------------------------
End-to-end pick and place logic. Pure Python — no ROS Action Server here.
Testable with hardcoded poses by calling run() directly.

Sequence:
  1. Move to capture pose (EE pose from config)
  2. Wait for GPD grasp poses on gpd_topic
  3. Arm selection: Y-threshold + IK reachability check
  4. PickSkill.pick() — Mode 1 (cartesian), fallback to Mode 2 (lift)
  5. On grasp failure: return to capture pose, retry once
  6. PlaceSkill.place() — Mode 1 (cartesian), fallback to Mode 2 (lift)
  7. Return to home
"""

import threading
from enum import Enum
from typing import Callable, Optional

from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node

from ai_worker_manipulation.robot_interface.moveit_client import MoveItClient, Arm, MoveResult
from ai_worker_manipulation.robot_interface.gripper_controller import GripperInterface
from ai_worker_manipulation.skill_primitives.pick_skill import PickSkill, PickResult
from ai_worker_manipulation.skill_primitives.place_skill import PlaceSkill, PlaceResult


class OrchestratorResult(Enum):
    SUCCESS           = 'success'
    GRASP_FAILED      = 'grasp_failed'
    PLANNING_FAILED   = 'planning_failed'
    NO_GPD_CANDIDATES = 'no_gpd_candidates'
    NO_REACHABLE_ARM  = 'no_reachable_arm'
    TIMEOUT           = 'timeout'


class OrchestratorPickResult:
    """pick() 반환값 — 성공 시 arm과 result를 함께 반환."""
    def __init__(self, result: OrchestratorResult, arm: 'Optional[Arm]' = None):
        self.result = result
        self.arm    = arm

    @property
    def success(self) -> bool:
        return self.result == OrchestratorResult.SUCCESS


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
    feedback_cb  : Optional callable(phase: str). Called at each phase transition.
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

    def run(
        self,
        object_class: str,
        place_pose: Pose,
        feedback_cb: Optional[Callable[[str], None]] = None,
    ) -> OrchestratorResult:
        """
        Execute a full pick-and-place cycle.

        Parameters
        ----------
        object_class : Key in object_lut.json for grasp assessment thresholds.
        place_pose   : Target pose for placing the object.
        feedback_cb  : Optional per-call feedback callback, overrides the instance default.
        """
        _fb = feedback_cb or self._feedback_cb
        max_retries = self._cfg.get('max_retries', 1)

        for attempt in range(max_retries + 1):
            if attempt > 0:
                self._log.info(f'[Orchestrator] retry attempt {attempt}/{max_retries}')

            # 1. Move to capture pose
            _fb('moving_to_capture_pose')
            if not self._move_to_capture_pose():
                return OrchestratorResult.PLANNING_FAILED

            # 2. Wait for GPD poses
            _fb('waiting_for_gpd')
            grasp_poses = self._wait_for_gpd()
            if not grasp_poses:
                return OrchestratorResult.NO_GPD_CANDIDATES

            # 3. Arm selection
            selection = self._select_arm(grasp_poses)
            if selection is None:
                return OrchestratorResult.NO_REACHABLE_ARM
            arm, grasp_pose = selection

            # 4. Pick
            _fb('picking')
            pick_result = self._pick.pick(
                grasp_pose=grasp_pose,
                arm=arm,
                object_name=object_class,
                pre_grasp_offset=self._cfg.get('pre_grasp_offset', 0.15),
                approach_height=self._cfg.get('approach_height', 0.10),
                lift_home=self._cfg.get('lift_home', 0.0),
                planning_retries=self._cfg.get('planning_retries', 3),
                jitter_retries=self._cfg.get('jitter_retries', 3),
                jitter_std=self._cfg.get('jitter_std', 0.01),
            )

            if pick_result != PickResult.SUCCESS:
                self._log.warn(f'[Orchestrator] pick failed on attempt {attempt + 1}')
                _fb('returning_to_capture_pose')
                self._gripper.open(arm.value)
                self._gripper.wait_until_executed()
                continue

            # 5. Place
            _fb('placing')
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
                self._log.error('[Orchestrator] place failed — opening gripper and returning home')
                self._gripper.open(arm.value)
                self._gripper.wait_until_executed()
                self._moveit.move_to_home(arm=arm)
                return OrchestratorResult.PLANNING_FAILED

            # 6. Return home
            _fb('returning_home')
            self._moveit.move_to_home(arm=arm)

            self._log.info('[Orchestrator] pick and place SUCCEEDED')
            return OrchestratorResult.SUCCESS

        self._log.error(f'[Orchestrator] all {max_retries + 1} attempts failed')
        return OrchestratorResult.GRASP_FAILED

    def pick(
        self,
        object_class: str = '',
        feedback_cb: Optional[Callable[[str], None]] = None,
    ) -> OrchestratorPickResult:
        """Pick 단계만 실행. 완료 후 inspection pose로 이동해 대기."""
        _fb = feedback_cb or self._feedback_cb
        max_retries = self._cfg.get('max_retries', 1)

        for attempt in range(max_retries + 1):
            if attempt > 0:
                self._log.info(f'[Orchestrator] pick retry attempt {attempt}/{max_retries}')

            _fb('moving_to_capture_pose')
            if not self._move_to_capture_pose():
                return OrchestratorPickResult(OrchestratorResult.PLANNING_FAILED)

            _fb('waiting_for_gpd')
            grasp_poses = self._wait_for_gpd()
            if not grasp_poses:
                return OrchestratorPickResult(OrchestratorResult.NO_GPD_CANDIDATES)

            selection = self._select_arm(grasp_poses)
            if selection is None:
                return OrchestratorPickResult(OrchestratorResult.NO_REACHABLE_ARM)
            arm, grasp_pose = selection

            _fb('picking')
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
                _fb('returning_to_capture_pose')
                self._gripper.open(arm.value)
                self._gripper.wait_until_executed()
                continue

            _fb('moving_to_inspection_pose')
            self._move_to_inspection_pose(arm)

            self._log.info(f'[Orchestrator] pick SUCCEEDED with {arm.value} arm')
            return OrchestratorPickResult(OrchestratorResult.SUCCESS, arm=arm)

        self._log.error(f'[Orchestrator] all {max_retries + 1} pick attempts failed')
        return OrchestratorPickResult(OrchestratorResult.GRASP_FAILED)

    def place(
        self,
        place_pose: Pose,
        arm: Arm,
        feedback_cb: Optional[Callable[[str], None]] = None,
    ) -> OrchestratorResult:
        """Place 단계만 실행. 완료 후 home 복귀."""
        _fb = feedback_cb or self._feedback_cb

        _fb('placing')
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
            self._log.error('[Orchestrator] place failed — opening gripper and returning home')
            self._gripper.open(arm.value)
            self._gripper.wait_until_executed()
            self._moveit.move_to_home(arm=arm)
            return OrchestratorResult.PLANNING_FAILED

        _fb('returning_home')
        self._moveit.move_to_home(arm=arm)
        self._log.info('[Orchestrator] place SUCCEEDED')
        return OrchestratorResult.SUCCESS

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
        """Subscribe to gpd_topic and return poses from first message.

        Uses a threading.Event driven by the subscription callback.
        The MoveItClient background executor drives all ROS callbacks —
        we must NOT call rclpy.spin_once() here.
        """
        topic   = self._cfg.get('gpd_topic', '/gpd/grasp_poses')
        timeout = self._cfg.get('gpd_timeout', 30.0)

        poses: list[Pose] = []
        received = threading.Event()

        def _cb(msg: PoseArray):
            if not received.is_set():
                poses.extend(msg.poses)
                received.set()
                self._log.info(f'[Orchestrator] received {len(poses)} GPD candidates')

        sub = self._node.create_subscription(PoseArray, topic, _cb, 10)
        received.wait(timeout=timeout)
        self._node.destroy_subscription(sub)

        if not poses:
            self._log.error(f'[Orchestrator] no GPD poses within {timeout}s')

        return poses

    def _select_arm(self, poses: list[Pose]) -> Optional[tuple[Arm, Pose]]:
        """Y-threshold arm selection with IK reachability confirmation.

        Evaluates at most 5 candidates to bound IK check time (5s each worst case).
        """
        y_threshold   = self._cfg.get('y_threshold', 0.0)
        max_candidates = 5

        for pose in poses[:max_candidates]:
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

    def _move_to_inspection_pose(self, arm: Arm) -> bool:
        pos = self._cfg.get('inspection_pose_position', [0.35, -0.20, 1.20])
        ori = self._cfg.get('inspection_pose_orientation', [0.086, -0.173, 0.015, 0.981])
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = pos
        pose.orientation.x, pose.orientation.y = ori[0], ori[1]
        pose.orientation.z, pose.orientation.w = ori[2], ori[3]
        result = self._moveit.move_to_pose(pose, arm=arm)
        if result != MoveResult.SUCCEEDED:
            self._log.warn(f'[Orchestrator] move to inspection pose failed: {result.value}')
            return False
        return True
