"""
gripper_controller.py
---------------------


RH-P12-RN Control Table 참고:
  - Goal Position (596) : 0(열림) ~ 1150(닫힘)
  - Present Position(611): 현재 위치
  - Present Current (621): 현재 전류 (~4.02 mA/단위)
  - Operating Mode  (11) : 5 = Current-based Position Control (기본값)

Public API:
    gc.open('left')              # 왼쪽 그리퍼 열기  (position → 0)
    gc.open('right')             # 오른쪽 그리퍼 열기
    gc.close('left')             # 왼쪽 그리퍼 닫기  (position → 740)
    gc.close('right')            # 오른쪽 그리퍼 닫기
    gc.control('both', 0.5)      # 양쪽 그리퍼를 position=0.5(정규화) 로 이동

    gc.grip('right', 'bottle')   # 오른쪽으로 'bottle' 파지 시도 → 성공/실패 출력
    gc.grip('left',  'cup')      # 왼쪽으로 'cup' 파지 시도

Joint position 정규화:
    0.0 = 완전히 열림  (raw 0)
    1.0 = 완전히 닫힘  (raw 1150)

close() 의 목표값:
    raw 740 → 정규화 740/1150 ≈ 0.6435
    (매뉴얼 Goal Position 그림에서 740이 실질적 닫힘 위치)
"""

import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# ── RH-P12-RN 상수 ────────────────────────────────────────────────────────────
_RAW_OPEN   = 0       # 완전히 열림
_RAW_CLOSE  = 740     # 매뉴얼 기준 실질적 닫힘 위치
_RAW_MAX    = 1150    # 물리적 최대 닫힘

# 정규화 변환 (0.0 ~ 1.0)
_NORM_OPEN  = _RAW_OPEN  / _RAW_MAX   # 0.0
_NORM_CLOSE = _RAW_CLOSE / _RAW_MAX   # ≈ 0.6435


class GripperController:
    """
    RH-P12-RN 그리퍼 컨트롤러.

    Parameters
    ----------
    node : rclpy.node.Node, optional
        외부 노드 주입. None 이면 내부에서 생성.
    """

    OPEN   = _NORM_OPEN    # 0.0
    CLOSED = _NORM_CLOSE   # ≈ 0.6435  (raw 740)
    MAX    = 1.0           # raw 1150 (물리적 최대)

    def __init__(self, node: Node = None):
        if node is None:
            rclpy.init()
            self._node     = Node('gripper_controller')
            self._own_node = True
        else:
            self._node     = node
            self._own_node = False

        # ── Publisher ─────────────────────────────────────────────────────
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

        # ── Subscriber ────────────────────────────────────────────────────
        self._sub = self._node.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10,
        )

        # ── Joint 이름 정의 ───────────────────────────────────────────────
        self._left_joints = [
            'arm_l_joint1', 'arm_l_joint2', 'arm_l_joint3', 'arm_l_joint4',
            'arm_l_joint5', 'arm_l_joint6', 'arm_l_joint7', 'gripper_l_joint1',
        ]
        self._right_joints = [
            'arm_r_joint1', 'arm_r_joint2', 'arm_r_joint3', 'arm_r_joint4',
            'arm_r_joint5', 'arm_r_joint6', 'arm_r_joint7', 'gripper_r_joint1',
        ]

        self._current_positions: dict[str, float] = {}

        # ── 초기화 대기 ───────────────────────────────────────────────────
        self._node.get_logger().info('Waiting for /joint_states...')
        self._wait_for_joint_states()
        self._node.get_logger().info('GripperController ready!')

        # ── GraspAssessment (지연 import → 순환 참조 방지) ─────────────────
        # grip() 호출 시 내부적으로 사용
        self._ga = None

    # ── 내부 메서드 ───────────────────────────────────────────────────────────
    def _joint_state_callback(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self._current_positions[name] = pos

    def _wait_for_joint_states(self, timeout: float = 5.0):
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self._node, timeout_sec=0.1)
            if (
                'gripper_l_joint1' in self._current_positions
                and 'gripper_r_joint1' in self._current_positions
            ):
                return
        self._node.get_logger().warn('joint_states 수신 타임아웃 (5초). 계속 진행.')

    def _send(self, side: str, gripper_position: float):
        """
        그리퍼 joint에 목표 위치를 발행.

        Parameters
        ----------
        side             : 'left' | 'right'
        gripper_position : 정규화 위치 (0.0 ~ 1.0)
        """
        gripper_position = max(0.0, min(1.0, gripper_position))

        if side == 'left':
            joint_names = self._left_joints
            publisher   = self._left_pub
        elif side == 'right':
            joint_names = self._right_joints
            publisher   = self._right_pub
        else:
            self._node.get_logger().error("side는 'left' 또는 'right' 여야 합니다.")
            return

        positions = []
        for joint in joint_names:
            if 'gripper' in joint:
                positions.append(float(gripper_position))
            else:
                positions.append(float(self._current_positions.get(joint, 0.0)))

        msg                        = JointTrajectory()
        msg.joint_names            = joint_names
        point                      = JointTrajectoryPoint()
        point.positions            = positions
        point.time_from_start.sec  = 1
        point.time_from_start.nanosec = 0
        msg.points.append(point)

        for _ in range(10):
            publisher.publish(msg)
            time.sleep(0.05)

        raw_approx = gripper_position * _RAW_MAX
        self._node.get_logger().info(
            f'gripper [{side}] → {gripper_position:.4f} (raw ≈ {raw_approx:.0f})'
        )

    def _get_ga(self):
        """GraspAssessment 인스턴스를 지연 생성 (순환 import 방지)."""
        if self._ga is None:
            from ai_worker_manipulation.skill_primitives.grasp_assessment import GraspAssessment
            self._ga = GraspAssessment(self._node)
        return self._ga

    # ── Public API ────────────────────────────────────────────────────────────

    def control(self, side: str, position: float):
        """
        그리퍼 위치를 직접 지정.

        Parameters
        ----------
        side     : 'left' | 'right' | 'both'
        position : 정규화 위치 0.0(열림) ~ 1.0(최대 닫힘)
        """
        if side == 'both':
            self._send('left',  position)
            self._send('right', position)
        else:
            self._send(side, position)

    def open(self, side: str = 'both'):
        """
        그리퍼를 완전히 엽니다. (Goal Position = 0)

        Parameters
        ----------
        side : 'left' | 'right' | 'both'
        """
        self._node.get_logger().info(f'[open] {side} → raw {_RAW_OPEN}')
        self.control(side, self.OPEN)

    def close(self, side: str = 'both'):
        """
        그리퍼를 닫습니다. (Goal Position = 740, 매뉴얼 기준 실질 닫힘)

        Parameters
        ----------
        side : 'left' | 'right' | 'both'
        """
        self._node.get_logger().info(f'[close] {side} → raw {_RAW_CLOSE}')
        self.control(side, self.CLOSED)

    def grip(self, side: str, object_name: str,
             stable_duration: float = 1.0) -> bool:
        """
        지정 오브젝트를 파지하고 grasp assessment 로 성공 여부를 판단.

        동작 순서:
          1. close(side) 호출
          2. 1.2초 대기 (그리퍼 도달 시간)
          3. GraspAssessment.assess_stable() 로 판단
          4. 결과 출력 및 반환

        Parameters
        ----------
        side            : 'left' | 'right'
        object_name     : LUT에 정의된 오브젝트 이름 (예: 'bottle')
        stable_duration : 파지 안정 유지 시간 (초), 기본 1.0

        Returns
        -------
        bool : 파지 성공 여부
        """
        if side not in ('left', 'right'):
            raise ValueError("grip() 의 side 는 'left' 또는 'right' 여야 합니다.")

        self._node.get_logger().info(
            f'[grip] {side}/{object_name} 파지 시작'
        )

        # Step 1: 닫기
        self.close(side)

        # Step 2: 그리퍼가 목표에 도달할 때까지 대기
        time.sleep(1.2)

        # Step 3: GraspAssessment 평가
        ga      = self._get_ga()
        success = ga.assess_stable(side, object_name, duration=stable_duration)

        # Step 4: 결과 출력
        if success:
            print(f'grasp success ({side}, {object_name})')
        else:
            print(f'grasp failed  ({side}, {object_name})')
            # 실패 시 열기
            self.open(side)

        return success

    def shutdown(self):
        self._node.destroy_node()
        if self._own_node:
            rclpy.shutdown()