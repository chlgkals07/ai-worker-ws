"""
로봇 상태 모니터 노드.

다음 항목을 주기적으로 터미널에 출력:
  - CompetitionManager 상태 머신: IDLE/HOVERING/GRASPING/ATTACHED/RELEASING
  - 부착 물체 (competition_manager 보고 + MoveIt 보고 비교)
  - 그리퍼 상태 (좌/우): OPEN/CLOSED/MID 판정, position, effort
  - EEF 위치 (좌/우): base_link 기준 x y z
  - 팔 관절: 좌/우 전체 position + effort (rad)

구독 토픽:
  /joint_states              sensor_msgs/JointState
  /attached_collision_object moveit_msgs/AttachedCollisionObject  (MoveIt 내부)
  /attached_object           std_msgs/String                      (competition_manager)
  /cm_state                  std_msgs/String                      (competition_manager 상태)

사용:
  ros2 run ai_worker_manipulation robot_state_monitor

출력 주기 조정:
  PRINT_HZ 상수를 변경 (기본 2.0 Hz)
"""

import rclpy
from rclpy.node import Node
import tf2_ros
from datetime import datetime

from sensor_msgs.msg import JointState
from std_msgs.msg import String
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject

# ── 관절 이름 ─────────────────────────────────────────────────────────────────

GRIPPER_JOINT = {
    'right': 'gripper_r_joint1',
    'left':  'gripper_l_joint1',
}
EEF_LINK = {
    'right': 'end_effector_r_link',
    'left':  'end_effector_l_link',
}

# competition_manager_node.py와 동일한 임계값
GRIPPER_CLOSE_THRESHOLD = 0.35
GRIPPER_OPEN_THRESHOLD  = 0.25

PRINT_HZ = 2.0   # 터미널 출력 주기


class RobotStateMonitor(Node):

    def __init__(self):
        super().__init__('robot_state_monitor')

        self._joint_pos: dict[str, float] = {}
        self._joint_vel: dict[str, float] = {}
        self._joint_eff: dict[str, float] = {}

        # MoveIt /attached_collision_object 토픽 추적 (obj_id → link_name)
        self._moveit_attached: dict[str, str] = {}

        # competition_manager 발행 토픽
        self._cm_attached: str = ''      # /attached_object
        self._cm_state:    str = '──'    # /cm_state

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_subscription(JointState, '/joint_states', self._joint_cb, 10)
        self.create_subscription(
            AttachedCollisionObject, '/attached_collision_object',
            self._moveit_attached_cb, 10)
        self.create_subscription(String, '/attached_object', self._cm_attached_cb, 10)
        self.create_subscription(String, '/cm_state',        self._cm_state_cb,    10)
        self.create_timer(1.0 / PRINT_HZ, self._print_cb)

        self.get_logger().info(f'[RobotStateMonitor] 시작 — {PRINT_HZ:.0f}Hz 주기로 상태 출력')

    # ── 콜백 ─────────────────────────────────────────────────────────────────

    def _joint_cb(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self._joint_pos[name] = msg.position[i]
            if i < len(msg.velocity):
                self._joint_vel[name] = msg.velocity[i]
            if i < len(msg.effort):
                self._joint_eff[name] = msg.effort[i]

    def _moveit_attached_cb(self, msg: AttachedCollisionObject):
        obj_id = msg.object.id
        op     = msg.object.operation
        if op == CollisionObject.ADD:
            self._moveit_attached[obj_id] = msg.link_name
            self.get_logger().info(f'[MoveIt ATTACH] {obj_id} @ {msg.link_name}')
        elif op == CollisionObject.REMOVE:
            removed_link = self._moveit_attached.pop(obj_id, '?')
            self.get_logger().info(f'[MoveIt DETACH] {obj_id} @ {removed_link}')

    def _cm_attached_cb(self, msg: String):
        self._cm_attached = msg.data

    def _cm_state_cb(self, msg: String):
        self._cm_state = msg.data

    # ── 포맷 헬퍼 ─────────────────────────────────────────────────────────────

    def _gripper_str(self, side: str) -> str:
        joint = GRIPPER_JOINT[side]
        pos   = self._joint_pos.get(joint)
        if pos is None:
            return '── 미수신 ──'
        eff = self._joint_eff.get(joint, 0.0)
        if pos > GRIPPER_CLOSE_THRESHOLD:
            label = 'CLOSED'
        elif pos < GRIPPER_OPEN_THRESHOLD:
            label = 'OPEN  '
        else:
            label = 'MID   '
        return f'{label}  pos={pos:.3f}  eff={eff:+.4f}'

    def _eef_str(self, side: str) -> str:
        try:
            t  = self._tf_buffer.lookup_transform(
                'base_link', EEF_LINK[side], rclpy.time.Time())
            tr = t.transform.translation
            ro = t.transform.rotation
            return (f'x={tr.x:+.3f}  y={tr.y:+.3f}  z={tr.z:+.3f}'
                    f'   qx={ro.x:+.3f}  qy={ro.y:+.3f}'
                    f'  qz={ro.z:+.3f}  qw={ro.w:+.3f}')
        except Exception:
            return '── TF 미수신 ──'

    def _arm_joints_str(self, side: str) -> str:
        prefix = f'arm_{side[0]}_joint'
        pairs  = sorted(
            [(k, v) for k, v in self._joint_pos.items() if k.startswith(prefix)],
            key=lambda kv: kv[0])
        if not pairs:
            return '── 미수신 ──'
        vals = '  '.join(f'{v:+.3f}' for _, v in pairs)
        return f'[{vals}]'

    def _arm_efforts_str(self, side: str) -> str:
        prefix = f'arm_{side[0]}_joint'
        pairs  = sorted(
            [(k, v) for k, v in self._joint_eff.items() if k.startswith(prefix)],
            key=lambda kv: kv[0])
        if not pairs:
            return '── 미수신 ──'
        vals = '  '.join(f'{v:+.3f}' for _, v in pairs)
        return f'[{vals}]'

    def _attached_sync_str(self) -> str:
        """competition_manager와 MoveIt 사이의 attached 상태 일치 여부."""
        cm  = self._cm_attached
        mvt = set(self._moveit_attached.keys())
        if not cm and not mvt:
            return 'OK  (없음)'
        if cm and cm in mvt:
            return f'OK  {cm}'
        if cm and cm not in mvt:
            return f'MISMATCH  CM={cm!r}  MoveIt={list(mvt)}'
        if not cm and mvt:
            return f'MISMATCH  CM=없음  MoveIt={list(mvt)}'
        return '??'

    # ── 출력 ─────────────────────────────────────────────────────────────────

    def _print_cb(self):
        now = datetime.now().strftime('%H:%M:%S')
        W   = 62
        sep = '─' * W

        def row(label: str, val: str) -> str:
            inner = f'  {label}: {val}'
            return f'│{inner:<{W}}│'

        lines = [
            f'┌{sep}┐',
            f'│  ROBOT STATE  {now:<{W - 15}}│',
            f'├{sep}┤',
            f'│  COMPETITION MANAGER{" " * (W - 21)}│',
            row('State  ', self._cm_state),
            row('Attach ', self._attached_sync_str()),
            f'├{sep}┤',
            f'│  GRIPPER{" " * (W - 9)}│',
            row('R', self._gripper_str('right')),
            row('L', self._gripper_str('left')),
            f'├{sep}┤',
            f'│  END EFFECTOR  (base_link 기준){" " * (W - 33)}│',
            row('R', self._eef_str('right')),
            row('L', self._eef_str('left')),
            f'├{sep}┤',
            f'│  ARM JOINTS  pos (rad){" " * (W - 23)}│',
            row('R', self._arm_joints_str('right')),
            row('L', self._arm_joints_str('left')),
            f'├{sep}┤',
            f'│  ARM JOINTS  effort{" " * (W - 20)}│',
            row('R', self._arm_efforts_str('right')),
            row('L', self._arm_efforts_str('left')),
            f'└{sep}┘',
        ]

        print('\n'.join(lines) + '\n')


def main():
    rclpy.init()
    node = RobotStateMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()