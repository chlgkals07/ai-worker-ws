import time
import threading
from typing import Optional, Dict

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from ai_worker_manipulation.skill_primitives.grasp_assessment import GraspAssessment


# ══════════════════════════════════════════════════════════════════════════════
# ★ 사용자 조정 파라미터 ★
# ══════════════════════════════════════════════════════════════════════════════

# 그리퍼 속도 — trajectory time_from_start (초)
# 값이 작을수록 빠름. 너무 작으면 하드웨어가 못 따라감.
GRIPPER_SPEED: float = 1.0   # ← 여기서 속도를 변경하세요 (단위: 초)

# ══════════════════════════════════════════════════════════════════════════════
# ★ 컨트롤러 구성 선택 ★
# ══════════════════════════════════════════════════════════════════════════════
#
# [현재 상황]
#   ros2 control list_controllers 결과:
#     arm_l_controller  (JointTrajectoryController) — arm_l_joint1~7 + gripper_l_joint1 관리
#     arm_r_controller  (JointTrajectoryController) — arm_r_joint1~7 + gripper_r_joint1 관리
#     → 전용 gripper_controller 없음!
#
# [문제]
#   JointTrajectoryController 에 gripper joint 만 담은 메시지를 보내면
#   컨트롤러 YAML 에 allow_partial_joints_goal: true 가 없는 경우 무시/거부됨.
#
# [해결책 — USE_DEDICATED_GRIPPER_CONTROLLER 플래그]
#   False (기본값, 현재 상황):
#     arm_l/r_controller 토픽으로 전송.
#     arm joint 들은 현재 위치 유지(hold) + gripper joint 만 목표 위치로 전송.
#     allow_partial_joints_goal 설정 불필요 — 모든 joint 포함이므로 항상 수락됨.
#
#   True (전용 gripper controller 추가 후):
#     /gripper_l_controller/joint_trajectory
#     /gripper_r_controller/joint_trajectory 토픽 사용.
#     gripper joint 1개만 메시지에 포함.
#
USE_DEDICATED_GRIPPER_CONTROLLER: bool = False

# ══════════════════════════════════════════════════════════════════════════════
# 하드웨어 상수
# ══════════════════════════════════════════════════════════════════════════════

# 정규화 position (0.0 ~ 1.0)
POSITION_OPEN   = 0.98   # Open  목표 (raw 1130 / 1150 ≈ 0.98)
POSITION_CLOSED = 0.0    # Close 목표 (raw 0)

# position == 0 판정 허용 오차 (raw 단위 환산 시 ≈ 5/1150)
CLOSED_THRESHOLD = 0.005

MAX_GRASP_RETRY = 3

# ── Joint 이름 ─────────────────────────────────────────────────────────────────
# /joint_states 에서 확인된 실제 joint 이름
GRIPPER_JOINT = {
    'left' : 'gripper_l_joint1',
    'right': 'gripper_r_joint1',
}

# arm_l/r_controller 가 관리하는 arm joint (gripper 제외, 위치 유지용)
# 실제 URDF/컨트롤러 설정에 맞게 수정하세요.
ARM_JOINTS = {
    'left' : [
        'arm_l_joint1', 'arm_l_joint2', 'arm_l_joint3',
        'arm_l_joint4', 'arm_l_joint5', 'arm_l_joint6', 'arm_l_joint7',
    ],
    'right': [
        'arm_r_joint1', 'arm_r_joint2', 'arm_r_joint3',
        'arm_r_joint4', 'arm_r_joint5', 'arm_r_joint6', 'arm_r_joint7',
    ],
}

# ── 컨트롤러 토픽 ──────────────────────────────────────────────────────────────
# 전용 gripper controller 가 없으므로 arm controller 토픽 사용
CONTROLLER_TOPIC = {
    'left' : '/arm_l_controller/joint_trajectory',
    'right': '/arm_r_controller/joint_trajectory',
}

# 전용 gripper controller 추가 시 사용할 토픽
DEDICATED_GRIPPER_TOPIC = {
    'left' : '/gripper_l_controller/joint_trajectory',
    'right': '/gripper_r_controller/joint_trajectory',
}


# ══════════════════════════════════════════════════════════════════════════════
# GripperController
# ══════════════════════════════════════════════════════════════════════════════

class GripperController:
    """
    JointTrajectoryController 기반 그리퍼 고수준 제어기.

    [현재 하드웨어 구성]
    전용 gripper controller 가 없으며, gripper joint 들이
    arm_l/r_controller 에 포함되어 있습니다.
    따라서 그리퍼 명령 전송 시 arm joint 현재 위치를 함께 포함하여
    arm joint 가 움직이지 않도록 hold trajectory 를 구성합니다.

    Parameters
    ----------
    node : rclpy.node.Node — 외부 ROS2 노드
    """

    def __init__(self, node: Node):
        self._node = node
        self._assessment = GraspAssessment(node)
        self._moving_thread: Optional[threading.Thread] = None

        # /joint_states 에서 arm joint 현재 위치를 구독·저장
        # gripper send 시 arm hold position 으로 활용
        self._joint_positions: Dict[str, float] = {}
        self._joint_state_lock = threading.Lock()
        self._joint_state_sub = self._node.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_cb,
            10,
        )

        # /joint_states 수신 대기
        self._log('Waiting for /joint_states...')
        while rclpy.ok():
            with self._joint_state_lock:
                if self._joint_positions:
                    break
            rclpy.spin_once(self._node, timeout_sec=0.1)
        self._log('Joint states received.')

        # publisher 생성
        self._pubs = {}
        if USE_DEDICATED_GRIPPER_CONTROLLER:
            for side, topic in DEDICATED_GRIPPER_TOPIC.items():
                self._pubs[side] = self._node.create_publisher(
                    JointTrajectory, topic, 10
                )
            self._log('전용 gripper controller 모드로 초기화.')
        else:
            for side, topic in CONTROLLER_TOPIC.items():
                self._pubs[side] = self._node.create_publisher(
                    JointTrajectory, topic, 10
                )
            self._log(
                'arm controller 공유 모드로 초기화. '
                '(gripper 명령 시 arm joint hold trajectory 포함)'
            )

        self._node.get_logger().info('[GripperController] 초기화 완료.')

    # ── 콜백 ──────────────────────────────────────────────────────────────────
    def _joint_state_cb(self, msg: JointState):
        """
        /joint_states 수신 시 joint 위치 버퍼 갱신.
        arm joint hold position 계산에 사용.
        """
        with self._joint_state_lock:
            for name, pos in zip(msg.name, msg.position):
                self._joint_positions[name] = pos

    # ── 내부 헬퍼 ─────────────────────────────────────────────────────────────
    def _log(self, msg: str):
        self._node.get_logger().info(f'[GripperController] {msg}')

    def _err(self, msg: str):
        self._node.get_logger().error(f'[ERR][GripperController] {msg}')

    def _get_arm_hold_positions(self, side: str) -> Optional[list]:
        """
        arm joint 들의 현재 위치를 hold position 으로 반환.
        버퍼가 비어 있으면 None 반환 (아직 /joint_states 미수신).

        Parameters
        ----------
        side : 'left' | 'right'

        Returns
        -------
        list[float] | None
        """
        with self._joint_state_lock:
            positions = []
            for joint in ARM_JOINTS[side]:
                pos = self._joint_positions.get(joint)
                if pos is None:
                    self._err(
                        f'arm joint "{joint}" 위치 미확인 — '
                        '/joint_states 수신 전이거나 joint 이름 불일치.'
                    )
                    return None
                positions.append(pos)
            return positions

    def _make_duration(self) -> Duration:
        secs     = int(GRIPPER_SPEED)
        nanosecs = int((GRIPPER_SPEED - secs) * 1e9)
        return Duration(sec=secs, nanosec=nanosecs)

    def _send_position(self, side: str, gripper_position: float):
        """
        JointTrajectory 메시지를 생성해 컨트롤러로 전송.

        USE_DEDICATED_GRIPPER_CONTROLLER == False 인 경우:
          arm joint 들의 현재 위치(hold) + gripper 목표 위치를 함께 전송.
          이를 통해 arm controller 가 메시지를 수락하고
          arm joint 는 움직이지 않으며 gripper 만 이동.

        USE_DEDICATED_GRIPPER_CONTROLLER == True 인 경우:
          gripper joint 만 포함해 전용 gripper controller 토픽으로 전송.

        Parameters
        ----------
        side             : 'left' | 'right'
        gripper_position : 정규화 목표 위치 (0.0 ~ 1.0)
        """
        if side not in GRIPPER_JOINT:
            raise ValueError(f"side 는 'left' 또는 'right' 여야 합니다: {side}")

        msg = JointTrajectory()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        point = JointTrajectoryPoint()
        point.time_from_start = self._make_duration()

        if USE_DEDICATED_GRIPPER_CONTROLLER:
            # ── 전용 gripper controller: gripper joint 만 포함 ──────────────
            msg.joint_names = [GRIPPER_JOINT[side]]
            point.positions  = [float(gripper_position)]
        else:
            # ── arm controller 공유: arm joint hold + gripper joint 포함 ────
            arm_positions = self._get_arm_hold_positions(side)

            if arm_positions is None:
                # /joint_states 미수신 — arm joint 없이 gripper 만 전송 (fallback)
                # allow_partial_joints_goal: true 가 설정되어 있을 때만 동작함.
                self._err(
                    'arm joint 위치 미확인. gripper joint 만으로 전송 시도. '
                    '컨트롤러 YAML 에 allow_partial_joints_goal: true 가 필요할 수 있음.'
                )
                msg.joint_names = [GRIPPER_JOINT[side]]
                point.positions  = [float(gripper_position)]
            else:
                # arm joints (hold) + gripper joint (목표 위치)
                msg.joint_names = ARM_JOINTS[side] + [GRIPPER_JOINT[side]]
                point.positions  = arm_positions + [float(gripper_position)]

        msg.points = [point]
        self._pubs[side].publish(msg)

    def _get_present_position(self, side: str) -> Optional[float]:
        """
        현재 gripper joint 정규화 position 반환.

        1순위: GraspAssessment 내부 버퍼 (_positions) — assessment 구독 중일 때
        2순위: /joint_states 구독 버퍼 (_joint_positions) — 항상 사용 가능

        Parameters
        ----------
        side : 'left' | 'right'

        Returns
        -------
        float | None
        """
        joint = GRIPPER_JOINT[side]

        # GraspAssessment 버퍼 우선
        pos = self._assessment._positions.get(joint)
        if pos is not None:
            return pos

        # fallback: /joint_states 버퍼
        with self._joint_state_lock:
            return self._joint_positions.get(joint)

    # ══════════════════════════════════════════════════════════════════════════
    # Public API
    # ══════════════════════════════════════════════════════════════════════════

    def open(self, side: str):
        """
        그리퍼를 POSITION_OPEN 으로 열기.
        진행 중인 파지 평가 루프를 즉시 중단.

        Parameters
        ----------
        side : 'left' | 'right'
        """
        # 1. 평가 루프 즉시 중단
        self._assessment.stop()

        if self._moving_thread and self._moving_thread.is_alive():
            self._moving_thread.join(timeout=1.0)

        # 2. 그리퍼 열기
        self._send_position(side, POSITION_OPEN)
        self._log(f'open({side}) — position {POSITION_OPEN} 전송.')
        # Give the controller a bit of time to settle after the trajectory finishes
        time.sleep(GRIPPER_SPEED + 0.2)

    def close(self, side: str):
        """
        그리퍼를 POSITION_CLOSED 로 닫기.

        Parameters
        ----------
        side : 'left' | 'right'
        """
        self._send_position(side, POSITION_CLOSED)
        self._log(f'close({side}) — position {POSITION_CLOSED} 전송.')
        # Give the controller a bit of time to settle after the trajectory finishes
        time.sleep(GRIPPER_SPEED + 0.2)

    def grasp(self, side: str, object_name: str) -> bool:
        """
        물체 파지 시퀀스.

        순서:
          1. Close(side)
          2. assess_on_close() — 파지 평가 (1초 이상 조건 유지 시 성공)
             position == 0 이고 평가 실패 → Open + 재시도 (최대 3회)
             3회 실패 시 ERR 출력 후 False 반환
          3. 파지 성공 → goal_position 을 현재 위치의 절반으로 설정
          4. assess_moving() 을 별도 스레드로 실행 — 이동 중 슬립 감지

        Parameters
        ----------
        side        : 'left' | 'right'
        object_name : LUT 오브젝트 이름 (예: 'bottle')

        Returns
        -------
        bool : True = 파지 성공, False = 파지 실패
        """
        for attempt in range(1, MAX_GRASP_RETRY + 1):
            self._log(f'grasp({side}, {object_name}) — 시도 {attempt}/{MAX_GRASP_RETRY}')

            # ── 1. close ────────────────────────────────────────────────────
            self.close(side)

            # ── 2. 닫힘 중 파지 평가 ────────────────────────────────────────
            grasped = self._assessment.assess_on_close(side, object_name)

            # ── 3. position == 0 도달 여부 확인 ─────────────────────────────
            current_pos = self._get_present_position(side)
            reached_zero = (
                current_pos is not None and current_pos <= CLOSED_THRESHOLD
            )

            if reached_zero and not grasped:
                self._log('position == 0 도달, 파지 미감지 — open 후 재시도.')
                self.open(side)
                continue

            if grasped:
                # ── 4. 파지 성공 처리 ────────────────────────────────────
                self._log(f'파지 성공! ({side}/{object_name})')

                # 현재 위치의 절반으로 position 감소 (파지력 유지)
                if current_pos is not None and current_pos > CLOSED_THRESHOLD:
                    half_pos = current_pos / 2.0
                else:
                    # current_pos 가 0 이거나 미확인이면 절반 이동 불필요
                    half_pos = POSITION_CLOSED
                    self._log(
                        f'current_pos={current_pos} — half_pos 계산 불가, '
                        f'POSITION_CLOSED({POSITION_CLOSED}) 유지.'
                    )

                self._send_position(side, half_pos)
                self._log(
                    f'position → {half_pos:.3f} '
                    f'(현재 {current_pos:.3f} 의 절반)'
                )

                # ── 5. 이동 중 슬립 감지 (별도 스레드) ──────────────────
                self._moving_thread = threading.Thread(
                    target=self._moving_monitor,
                    args=(side, object_name),
                    daemon=True,
                )
                self._moving_thread.start()
                return True

            # open 명령 등으로 평가가 중단된 경우
            self._log('평가 중단 또는 실패.')
            self.open(side)

        # ── 3회 모두 실패 ────────────────────────────────────────────────────
        self._err(
            f'grasp({side}, {object_name}) — '
            f'{MAX_GRASP_RETRY}회 시도 모두 실패! Failure!'
        )
        return False

    # ── 이동 중 슬립 감지 스레드 ──────────────────────────────────────────────
    def _moving_monitor(self, side: str, object_name: str):
        """
        이동 중 슬립 감지 루프.
        슬립 감지 시 assess_moving() 내부에서 ERR 출력.
        JointTrajectoryController 는 Goal Current 직접 제어 불가이므로
        슬립 발생 시 Close() 재전송으로 파지력 유지를 시도.
        """
        self._assessment.assess_moving(
            side, object_name, goal_current=0.0
        )
        #?? intentional?