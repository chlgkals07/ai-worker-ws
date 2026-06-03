import time
import threading
from enum import Enum

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from pymoveit2 import MoveIt2
from pymoveit2.moveit2 import MoveIt2State
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import Pose


class MoveResult(Enum):
    SUCCEEDED = 'succeeded'
    FAILED    = 'failed'
    INVALID   = 'invalid'   # goal rejected before execution (IK, planning, bad pose)
    TIMEOUT   = 'timeout'   # executor never got a result back in time


class Arm(Enum):
    RIGHT = 'right'
    LEFT  = 'left'


_ARM_R_JOINTS = [
    'arm_r_joint1', 'arm_r_joint2', 'arm_r_joint3',
    'arm_r_joint4', 'arm_r_joint5', 'arm_r_joint6', 'arm_r_joint7',
]
_ARM_L_JOINTS = [
    'arm_l_joint1', 'arm_l_joint2', 'arm_l_joint3',
    'arm_l_joint4', 'arm_l_joint5', 'arm_l_joint6', 'arm_l_joint7',
]
_ARM_JOINTS = {Arm.RIGHT: _ARM_R_JOINTS, Arm.LEFT: _ARM_L_JOINTS}

# MoveIt error codes that indicate planning/IK failure, not execution failure.
_PLANNING_ERROR_CODES = {
    MoveItErrorCodes.FAILURE,
    MoveItErrorCodes.NO_IK_SOLUTION,
    MoveItErrorCodes.PLANNING_FAILED,
    MoveItErrorCodes.INVALID_MOTION_PLAN,
    MoveItErrorCodes.GOAL_IN_COLLISION,
    MoveItErrorCodes.GOAL_STATE_INVALID,
    MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED,
    MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS,
    MoveItErrorCodes.START_STATE_INVALID,
    MoveItErrorCodes.START_STATE_IN_COLLISION,
}

_JOINT_STATES_TIMEOUT      = 10.0
_DEFAULT_PLANNING_TIME     = 5.0
_DEFAULT_PLANNING_ATTEMPTS = 3


class MoveItClient:

    def __init__(self, node: Node):
        self._node      = node
        self._log       = node.get_logger()
        self._destroyed = False

        self._cb_group = ReentrantCallbackGroup()

        self._moveit_r = MoveIt2(
            node=self._node,
            joint_names=_ARM_R_JOINTS,
            base_link_name='base_link',
            end_effector_name='end_effector_r_link',
            group_name='arm_r',
            callback_group=self._cb_group,
            use_move_group_action=True,
        )
        self._moveit_l = MoveIt2(
            node=self._node,
            joint_names=_ARM_L_JOINTS,
            base_link_name='base_link',
            end_effector_name='end_effector_l_link',
            group_name='arm_l',
            callback_group=self._cb_group,
            use_move_group_action=True,
        )

        self._lock_r = threading.Lock()
        self._lock_l = threading.Lock()

        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._node)
        self._executor_thread = threading.Thread(
            target=self._executor.spin,
            daemon=True,
        )
        self._executor_thread.start()

        self._wait_for_servers()
        self._wait_for_joint_states()

    # ------------------------------------------------------------------
    # Startup helpers
    # ------------------------------------------------------------------

    def _wait_for_servers(self) -> None:
        for label, moveit in [('arm_r', self._moveit_r), ('arm_l', self._moveit_l)]:
            self._log.info(f'Waiting for move_group action server [{label}]...')
            while not moveit._MoveIt2__move_action_client.wait_for_server(timeout_sec=1.0):
                self._log.warn(f'[{label}] move_group not available, retrying...')

        deadline = time.time() + 10.0
        for label, moveit in [('arm_r', self._moveit_r), ('arm_l', self._moveit_l)]:
            client = moveit._MoveIt2__move_action_client
            while not client.server_is_ready():
                if time.time() > deadline:
                    raise RuntimeError(
                        f'[{label}] server_is_ready() never stabilised after wait_for_server() succeeded'
                    )
                self._log.warn(f'[{label}] server_is_ready() False — waiting for DDS to stabilise...')
                time.sleep(0.1)

        self._log.info('move_group action servers ready.')

    def _wait_for_joint_states(self) -> None:
        self._log.info('Waiting for joint states...')
        start = time.time()
        while self._moveit_r.joint_state is None or self._moveit_l.joint_state is None:
            if time.time() - start > _JOINT_STATES_TIMEOUT:
                raise RuntimeError(
                    f'Joint states not received within {_JOINT_STATES_TIMEOUT}s. '
                    'Check that /joint_states is publishing and QoS is compatible.'
                )
            time.sleep(0.05)
        self._log.info('Joint states ready.')

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _moveit(self, arm: Arm) -> MoveIt2:
        return self._moveit_r if arm == Arm.RIGHT else self._moveit_l

    def _lock(self, arm: Arm) -> threading.Lock:
        return self._lock_r if arm == Arm.RIGHT else self._lock_l

    def _guard(self) -> None:
        if self._destroyed:
            raise RuntimeError('MoveItClient has been destroyed — create a new instance.')

    def _log_pose(self, label: str, arm: Arm, pose: Pose,
                  vel: float, acc: float, tol_pos: float, tol_ori: float) -> None:
        p = pose.position
        o = pose.orientation
        self._log.info(
            f'[{label}] [{arm.value}] '
            f'pos=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) '
            f'quat=({o.x:.3f}, {o.y:.3f}, {o.z:.3f}, {o.w:.3f}) '
            f'| vel={vel} acc={acc} tol_pos={tol_pos} tol_ori={tol_ori}'
        )

    def _configure(self, moveit2: MoveIt2, vel: float, acc: float,
                   pipeline: str = 'ompl', planner: str = 'RRTConnect') -> None:
        moveit2.motion_suceeded       = False
        moveit2.pipeline_id           = pipeline
        moveit2.planner_id            = planner
        moveit2.max_velocity          = vel
        moveit2.max_acceleration      = acc
        moveit2.allowed_planning_time = _DEFAULT_PLANNING_TIME
        moveit2.num_planning_attempts = _DEFAULT_PLANNING_ATTEMPTS

    def _wait(self, moveit2: MoveIt2, label: str, arm: Arm, timeout: float) -> MoveResult:
        """Poll query_state() until done or timeout. Executor thread drives all callbacks."""
        start = time.time()

        _ACCEPT_TIMEOUT = 2.0
        while moveit2.query_state() == MoveIt2State.IDLE:
            if time.time() - start > _ACCEPT_TIMEOUT:
                self._log.error(
                    f'[{label}] [{arm.value}] goal never left IDLE — '
                    'action server may have dropped the request'
                )
                return MoveResult.INVALID
            time.sleep(0.01)

        while moveit2.query_state() != MoveIt2State.IDLE:
            elapsed = time.time() - start
            if elapsed > timeout:
                self._log.error(
                    f'[{label}] [{arm.value}] TIMEOUT after {elapsed:.1f}s '
                    f'| state={moveit2.query_state().name}'
                )
                return MoveResult.TIMEOUT
            time.sleep(0.05)

        elapsed = time.time() - start

        if moveit2.motion_suceeded:
            self._log.info(f'[{label}] [{arm.value}] SUCCEEDED in {elapsed:.2f}s')
            return MoveResult.SUCCEEDED

        error     = moveit2.get_last_execution_error_code()
        error_val = error.val if error is not None else MoveItErrorCodes.UNDEFINED

        if error_val in _PLANNING_ERROR_CODES:
            self._log.error(
                f'[{label}] [{arm.value}] INVALID in {elapsed:.2f}s '
                f'| error_code={error_val} — planning/IK failure'
            )
            return MoveResult.INVALID

        self._log.error(
            f'[{label}] [{arm.value}] FAILED in {elapsed:.2f}s '
            f'| error_code={error_val} — execution error'
        )
        return MoveResult.FAILED

    # ------------------------------------------------------------------
    # Move functions
    # ------------------------------------------------------------------

    def move_to_pose(
        self,
        pose: Pose,
        arm: Arm = Arm.RIGHT,
        velocity: float = 0.1,
        acceleration: float = 0.1,
        timeout: float = 30.0,
        pipeline: str = 'ompl',
        planner: str = 'RRTConnect',
    ) -> MoveResult:
        """Free-space motion to a Cartesian pose. Retries with LBKPIECE on IK failure."""
        self._guard()
        tol_pos, tol_ori = 0.001, 0.01
        self._log_pose('move_to_pose', arm, pose, velocity, acceleration, tol_pos, tol_ori)

        with self._lock(arm):
            moveit2 = self._moveit(arm)
            self._configure(moveit2, velocity, acceleration, pipeline, planner)
            moveit2.move_to_pose(
                pose=pose,
                tolerance_position=tol_pos,
                tolerance_orientation=tol_ori,
            )
            result = self._wait(moveit2, 'move_to_pose', arm, timeout)

        if result == MoveResult.INVALID and planner == 'RRTConnect':
            self._log.warn(
                f'[move_to_pose] [{arm.value}] RRTConnect IK failed — retrying with LBKPIECE'
            )
            return self.move_to_pose(
                pose, arm=arm, velocity=velocity, acceleration=acceleration,
                timeout=timeout, pipeline=pipeline, planner='LBKPIECE',
            )

        return result

    def move_to_joints(
        self,
        joint_positions: list[float],
        arm: Arm = Arm.RIGHT,
        velocity: float = 0.1,
        acceleration: float = 0.1,
        timeout: float = 30.0,
        pipeline: str = 'ompl',
        planner: str = 'RRTConnect',
    ) -> MoveResult:
        """Move to a joint configuration. joint_positions must be 7 floats in radians."""
        self._guard()
        if len(joint_positions) != 7:
            self._log.error(
                f'[move_to_joints] [{arm.value}] expected 7 joint positions, '
                f'got {len(joint_positions)}'
            )
            return MoveResult.INVALID

        joints_str = ', '.join(f'{j:.3f}' for j in joint_positions)
        self._log.info(
            f'[move_to_joints] [{arm.value}] '
            f'joints=[{joints_str}] | vel={velocity} acc={acceleration}'
        )

        with self._lock(arm):
            moveit2 = self._moveit(arm)
            self._configure(moveit2, velocity, acceleration, pipeline, planner)
            moveit2.move_to_configuration(joint_positions)
            return self._wait(moveit2, 'move_to_joints', arm, timeout)

    def move_cartesian(
        self,
        pose: Pose,
        arm: Arm = Arm.RIGHT,
        velocity: float = 0.1,
        acceleration: float = 0.1,
        timeout: float = 15.0,
    ) -> MoveResult:
        """Straight-line Cartesian motion using the Pilz LIN planner."""
        self._guard()
        tol_pos, tol_ori = 0.001, 0.005
        self._log_pose('move_cartesian', arm, pose, velocity, acceleration, tol_pos, tol_ori)

        with self._lock(arm):
            moveit2 = self._moveit(arm)
            self._configure(moveit2, velocity, acceleration,
                            pipeline='pilz_industrial_motion_planner', planner='LIN')
            try:
                moveit2.move_to_pose(
                    pose=pose,
                    tolerance_position=tol_pos,
                    tolerance_orientation=tol_ori,
                )
                return self._wait(moveit2, 'move_cartesian', arm, timeout)
            finally:
                moveit2.pipeline_id = 'ompl'
                moveit2.planner_id  = 'RRTConnect'

    def move_to_home(
        self,
        arm: Arm = Arm.RIGHT,
        velocity: float = 0.1,
        acceleration: float = 0.1,
    ) -> MoveResult:
        """Move to all-zero joint configuration."""
        return self.move_to_joints(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            arm=arm,
            velocity=velocity,
            acceleration=acceleration,
        )

    # ------------------------------------------------------------------
    # Query methods
    # ------------------------------------------------------------------

    def get_joints(self, arm: Arm = Arm.RIGHT) -> list[float] | None:
        """Return current joint positions [joint1..joint7] in radians, or None if unavailable."""
        self._guard()
        moveit2     = self._moveit(arm)
        joint_names = _ARM_JOINTS[arm]

        if moveit2.joint_state is None:
            self._log.warn(f'[get_joints] [{arm.value}] joint state not yet available')
            return None

        js          = moveit2.joint_state
        name_to_pos = dict(zip(js.name, js.position))

        if not all(n in name_to_pos for n in joint_names):
            self._log.error(
                f'[get_joints] [{arm.value}] some joints missing from /joint_states. '
                f'Expected: {joint_names} | Received: {list(js.name)}'
            )
            return None

        positions  = [name_to_pos[n] for n in joint_names]
        joints_str = ', '.join(f'{j:.3f}' for j in positions)
        self._log.debug(f'[get_joints] [{arm.value}] [{joints_str}]')
        return positions

    def get_pose(self, arm: Arm = Arm.RIGHT) -> Pose | None:
        """Return current end-effector pose via FK, or None on failure."""
        self._guard()
        moveit2 = self._moveit(arm)

        future = moveit2.compute_fk_async()
        if future is None:
            self._log.error(
                f'[get_pose] [{arm.value}] FK request failed — is move_group running?'
            )
            return None

        timeout = 5.0
        start   = time.time()
        while not future.done():
            if time.time() - start > timeout:
                self._log.error(
                    f'[get_pose] [{arm.value}] FK response timeout after {timeout}s'
                )
                return None
            time.sleep(0.05)

        result = moveit2.get_compute_fk_result(future)
        if result is None:
            self._log.error(f'[get_pose] [{arm.value}] FK returned no result')
            return None

        p = result.pose.position
        o = result.pose.orientation
        self._log.info(
            f'[get_pose] [{arm.value}] '
            f'pos=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) '
            f'quat=({o.x:.3f}, {o.y:.3f}, {o.z:.3f}, {o.w:.3f})'
        )
        return result.pose

    def add_collision_box(
        self,
        id: str,
        size,
        position,
        quat_xyzw=(0.0, 0.0, 0.0, 1.0),
        arm: Arm = Arm.RIGHT,
    ) -> None:
        """Add a collision box to the MoveIt planning scene."""
        self._moveit(arm).add_collision_box(
            id=id, size=size, position=position, quat_xyzw=quat_xyzw,
        )

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def node(self) -> Node:
        return self._node

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def destroy(self) -> None:
        """Shut down the executor and join the background thread."""
        self._destroyed = True
        self._executor.shutdown()
        self._executor_thread.join(timeout=5.0)
