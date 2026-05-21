# Controls the gripper joints via ROS2 JointTrajectory publishers.
# GripperDriver integration deferred — uses topic-based control for now.

import time
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from ai_worker_manipulation.skill_primitives.grasp_assesment import GraspAssessment


MAX_GRASP_RETRY = 3


class GripperController:
    OPEN   = 0.0
    CLOSED = 1.0

    def __init__(self, node: Node = None):
        if node is None:
            rclpy.init()
            self._node = Node('gripper_controller')
            self._own_node = True
        else:
            self._node = node
            self._own_node = False

        self._left_pub = self._node.create_publisher(
            JointTrajectory,
            '/leader/joint_trajectory_command_broadcaster_left/joint_trajectory',
            10,
        )
        self._right_pub = self._node.create_publisher(
            JointTrajectory,
            '/leader/joint_trajectory_command_broadcaster_right/joint_trajectory',
            10,
        )
        self._sub = self._node.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10,
        )

        self._left_joints = [
            'arm_l_joint1', 'arm_l_joint2', 'arm_l_joint3', 'arm_l_joint4',
            'arm_l_joint5', 'arm_l_joint6', 'arm_l_joint7', 'gripper_l_joint1',
        ]
        self._right_joints = [
            'arm_r_joint1', 'arm_r_joint2', 'arm_r_joint3', 'arm_r_joint4',
            'arm_r_joint5', 'arm_r_joint6', 'arm_r_joint7', 'gripper_r_joint1',
        ]

        self._current_positions = {}
        self._assessment = GraspAssessment(self._node)
        self._moving_thread: Optional[threading.Thread] = None

        self._node.get_logger().info('Waiting for /joint_states...')
        self._wait_for_joint_states()
        self._node.get_logger().info('GripperController ready!')

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------
    def _joint_state_callback(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self._current_positions[name] = pos

    def _wait_for_joint_states(self):
        timeout = 5.0
        start = time.time()
        while time.time() - start < timeout:
            if (
                'gripper_l_joint1' in self._current_positions
                and 'gripper_r_joint1' in self._current_positions
            ):
                return
            time.sleep(0.1)

    def _send(self, side: str, gripper_position: float):
        gripper_position = max(self.OPEN, min(self.CLOSED, gripper_position))

        if side == 'left':
            joint_names = self._left_joints
            publisher   = self._left_pub
        elif side == 'right':
            joint_names = self._right_joints
            publisher   = self._right_pub
        else:
            self._node.get_logger().error("side must be 'left' or 'right'")
            return

        positions = []
        for joint in joint_names:
            if 'gripper' in joint:
                positions.append(float(gripper_position))
            else:
                positions.append(float(self._current_positions.get(joint, 0.0)))

        msg = JointTrajectory()
        msg.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 1
        point.time_from_start.nanosec = 0
        msg.points.append(point)

        for _ in range(10):
            publisher.publish(msg)
            time.sleep(0.05)

        self._node.get_logger().info(f'gripper [{side}] → {gripper_position:.2f}')

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def control(self, side: str, position: float):
        if side == 'both':
            self._send('left', position)
            self._send('right', position)
        else:
            self._send(side, position)

    def open(self, side: str = 'both'):
        self._assessment.stop()
        if self._moving_thread and self._moving_thread.is_alive():
            self._moving_thread.join(timeout=1.0)
        self.control(side, self.OPEN)
        time.sleep(1.0)

    def close(self, side: str = 'both'):
        self.control(side, self.CLOSED)
        time.sleep(1.0)

    # Uppercase aliases for compatibility with gripper_controller calls
    def Open(self, side: str):
        self.open(side)

    def Close(self, side: str):
        self.close(side)

    def Grasp(self, side: str, object_name: str) -> bool:
        for attempt in range(1, MAX_GRASP_RETRY + 1):
            self._node.get_logger().info(
                f'[GripperController] Grasp({side}, {object_name}) — 시도 {attempt}/{MAX_GRASP_RETRY}'
            )
            self.Close(side)
            grasped = self._assessment.assess_on_close(side, object_name)

            if grasped:
                self._node.get_logger().info(f'[GripperController] 파지 성공! ({side}/{object_name})')
                return True

            self._node.get_logger().warn(f'[GripperController] 파지 실패 — 재시도.')
            self.Open(side)

        self._node.get_logger().error(
            f'[GripperController] Grasp({side}, {object_name}) — {MAX_GRASP_RETRY}회 모두 실패!'
        )
        return False

    def shutdown(self):
        self._node.destroy_node()
        if self._own_node:
            rclpy.shutdown()
