import time
import threading
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from pymoveit2 import MoveIt2
from geometry_msgs.msg import Pose


class MoveResult(Enum):
    SUCCEEDED = 'succeeded'
    FAILED    = 'failed'
    INVALID   = 'invalid'   # goal rejected before planning (bad pose, IK failed)
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


class MoveItClient:

    def __init__(self, node: Node):
        self._node = node
        self._log  = node.get_logger()

        # ReentrantCallbackGroup lets action feedback + result callbacks
        # fire concurrently inside the MultiThreadedExecutor.
        self._cb_group = ReentrantCallbackGroup()

        # One MoveIt2 object per arm — each wraps the MoveGroup action client
        # for that planning group.
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

        # Start the executor in a background thread so all ROS2 callbacks
        # (action results, joint state updates, etc.) are handled automatically.
        # The main thread never needs to call spin_once.
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._node)
        self._executor_thread = threading.Thread(
            target=self._executor.spin,
            daemon=True,    # thread dies when the main program exits
        )
        self._executor_thread.start()

        # Block here until the infrastructure is ready.
        # Both waits rely on the executor thread being live.
        self._wait_for_servers()
        self._wait_for_joint_states()

    # ------------------------------------------------------------------
    # Startup helpers
    # ------------------------------------------------------------------

    def _wait_for_servers(self) -> None:
        """Block until both arm MoveGroup action servers are reachable."""
        for label, moveit in [('arm_r', self._moveit_r), ('arm_l', self._moveit_l)]:
            self._log.info(f'Waiting for move_group action server [{label}]...')
            while not moveit._MoveIt2__move_action_client.wait_for_server(timeout_sec=1.0):
                self._log.warn(f'[{label}] move_group not available, retrying...')
        self._log.info('move_group action servers ready.')

    def _wait_for_joint_states(self) -> None:
        """Block until joint state messages have been received for both arms."""
        self._log.info('Waiting for joint states...')
        while self._moveit_r.joint_state is None or self._moveit_l.joint_state is None:
            time.sleep(0.05)    # executor thread delivers the messages; main thread just waits
        self._log.info('Joint states ready.')

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _moveit(self, arm: Arm) -> MoveIt2:
        return self._moveit_r if arm == Arm.RIGHT else self._moveit_l

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def destroy(self) -> None:
        """Shut down the executor and join the background thread.
        The caller is responsible for destroying the node and calling rclpy.shutdown()."""
        self._executor.shutdown()
        self._executor_thread.join(timeout=5.0)
