"""
대회 환경 자동 관리 노드.

상태 머신:
  IDLE      — 물체 없음, hover 없음
  HOVERING  — EEF 근처 물체 감지 (노란 마커)
  GRASPING  — 그리퍼 닫힘 → GraspAssessment 진행 중
  ATTACHED  — 물체 attached (빨간 마커)
  RELEASING — 그리퍼 열림 → detach 처리 중

수신 토픽:
  /active_zone     std_msgs/String  → 'A'/'B'/'C'/'D' 구간 전환
  /attach_cmd      std_msgs/String  → '<obj_id> [arm:left|arm:right]' 수동 attach
  /detach_cmd      std_msgs/String  → '<obj_id>' 수동 detach
  /clear_env       std_msgs/String  → 환경 전체 제거

발행 토픽:
  /attached_object std_msgs/String  ← 현재 attached 물체 ID (없으면 '')

자동 attach/detach:
  gripper_X_joint1 > CLOSE_THRESHOLD (닫힘 엣지)
    → GraspAssessment.assess_on_close() 별도 스레드 실행
    → 검증 성공 시 attach (빨간 마커)
    → 검증 실패 시 attach 취소 (로그 출력)
  gripper_X_joint1 < OPEN_THRESHOLD (열림 엣지)
    → assessment.stop() 으로 검증 루프 즉시 중단
    → 현재 attached 물체 detach (낙하 위치 계산)
"""

import threading
import queue
from enum import Enum

import rclpy
import tf2_ros
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from ai_worker_manipulation.robot_interface.moveit_client import MoveItClient
from ai_worker_manipulation.skill_primitives.grasp_assessment import GraspAssessment
from ai_worker_manipulation.ai_worker_manipulation.mission_control.environment import (
    clear_all_objects,
    setup_zone_a, setup_zone_b, setup_zone_c, setup_zone_d,
    attach_object, detach_object,
    find_nearest_graspable,
    allow_zone_objects,
    EnvironmentVisualizer,
    OBJECT_LUT_NAME,
)

_CMD_ZONE          = 'zone'
_CMD_CLEAR         = 'clear'
_CMD_ATTACH        = 'attach'
_CMD_DETACH        = 'detach'
_CMD_ATTACH_AUTO   = 'attach_auto_r'
_CMD_DETACH_AUTO   = 'detach_auto_r'
_CMD_ATTACH_AUTO_L = 'attach_auto_l'
_CMD_DETACH_AUTO_L = 'detach_auto_l'

GRIPPER_CLOSE_THRESHOLD = 0.35   # 시뮬 그리퍼 최대 ~0.71 기준 완화
GRIPPER_OPEN_THRESHOLD  = 0.25

# Gazebo 시뮬에서는 그리퍼 effort가 0일 수 있어 assess_on_close가 항상 실패함.
# 실물 하드웨어 사용 시 True로 변경.
GRASP_ASSESSMENT_ENABLED = False

GRASP_DISTANCE = {
    'A': 0.15,
    'B': 0.30,
    'C': 0.15,
    'D': 0.20,
}
_GRASP_DISTANCE_DEFAULT = 0.15

# /attach_cmd arm 파싱 허용 값 (단축어 포함)
_ARM_ALIASES = {'l': 'left', 'r': 'right', 'left': 'left', 'right': 'right'}


class State(Enum):
    IDLE      = 'IDLE'
    HOVERING  = 'HOVERING'
    GRASPING  = 'GRASPING'
    ATTACHED  = 'ATTACHED'
    RELEASING = 'RELEASING'


class CompetitionManager:
    _SETUP_FN = {
        'A': setup_zone_a,
        'B': setup_zone_b,
        'C': setup_zone_c,
        'D': setup_zone_d,
    }

    def __init__(self, client: MoveItClient):
        self._client = client
        self._log    = client.node.get_logger()
        self._viz    = EnvironmentVisualizer(client.node)

        # GraspAssessment: 별도 노드로 생성 — assess_on_close()가 이 노드만 spin_once하므로
        # 메인 루프(client.node)와 스레드 경쟁 없음
        self._assessment_node = rclpy.create_node('grasp_assessment_node')
        self._assessment = GraspAssessment(self._assessment_node)

        self._state            = State.IDLE
        self._current_zone     = None
        self._attached_object: str | None = None
        self._attached_link:   str | None = None
        self._gripper_r_closed = False
        self._gripper_l_closed = False
        self._gripper_initialized = False

        self._pending: list = []
        self._hovered_l: str | None = None
        self._hovered_r: str | None = None

        # 파지 검증 스레드 (assess_on_close는 spin_once 블로킹 → 별도 스레드 필수)
        self._grasp_thread: threading.Thread | None = None
        self._grasp_result_q: queue.SimpleQueue = queue.SimpleQueue()

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, client.node)

        # 현재 attached 물체 ID 발행 — pick_and_place 등 외부 노드가 구독 가능
        self._attached_pub = client.node.create_publisher(String, '/attached_object', 10)
        # 상태 머신 현재 상태 발행 — robot_state_monitor 등이 구독
        self._state_pub    = client.node.create_publisher(String, '/cm_state', 10)

        self._switch_zone('A')

        client.node.create_subscription(String,     '/active_zone',  self._zone_cb,    10)
        client.node.create_subscription(String,     '/attach_cmd',   self._attach_cb,  10)
        client.node.create_subscription(String,     '/detach_cmd',   self._detach_cb,  10)
        client.node.create_subscription(String,     '/clear_env',    self._clear_cb,   10)
        client.node.create_subscription(JointState, '/joint_states', self._gripper_cb, 10)
        client.node.create_timer(0.5, self._republish_cb)

        self._log.info(
            'CompetitionManager 시작 — Zone A 초기화 완료\n'
            '  [자동] 그리퍼 닫힘 → GraspAssessment 검증 → 성공 시 attach\n'
            '  [자동] 그리퍼 열림 → 검증 중단 → detach\n'
            '  /active_zone     → 구간 전환 (A/B/C/D)\n'
            '  /attach_cmd      → 수동 attach  형식: "<obj_id> [arm:left|arm:right]"\n'
            '  /detach_cmd      → 수동 detach\n'
            '  /clear_env       → 환경 전체 제거\n'
            '  /attached_object ← 현재 attached 물체 ID 발행 (빈 문자열 = 없음)'
        )

    # ── 상태 전환 헬퍼 ──────────────────────────────────────────────────────────

    def _set_state(self, new_state: State):
        if new_state != self._state:
            self._log.info(f'[State] {self._state.value} → {new_state.value}')
            self._state = new_state
            msg = String()
            msg.data = new_state.value
            self._state_pub.publish(msg)

    def _publish_attached(self):
        """현재 attached 물체 ID를 /attached_object 에 즉시 발행."""
        msg = String()
        msg.data = self._attached_object or ''
        self._attached_pub.publish(msg)

    # ── 콜백: 큐에만 넣음 ──────────────────────────────────────────────────────

    def _zone_cb(self, msg: String):
        zone = msg.data.strip().upper()
        if zone not in self._SETUP_FN:
            self._log.warn(f'알 수 없는 구간: {zone}')
            return
        self._pending.append((_CMD_ZONE, zone))

    def _clear_cb(self, msg: String):
        self._pending.append((_CMD_CLEAR, None))

    def _attach_cb(self, msg: String):
        """형식: '<obj_id> [arm:left|arm:right|arm:l|arm:r]'. arm 미지정 시 기본값 right."""
        raw = msg.data.strip()
        arm = 'right'
        obj_id = raw
        if ' arm:' in raw:
            parts = raw.split(' arm:', 1)
            obj_id = parts[0].strip()
            raw_arm = parts[1].strip().lower()
            arm = _ARM_ALIASES.get(raw_arm)
            if arm is None:
                self._log.warn(f'[수동] 잘못된 arm 값 "{raw_arm}" — right으로 대체')
                arm = 'right'
        self._pending.append((_CMD_ATTACH, (obj_id, arm)))

    def _detach_cb(self, msg: String):
        self._pending.append((_CMD_DETACH, msg.data.strip()))

    def _republish_cb(self):
        if self._current_zone:
            self._viz.publish_zone(self._current_zone)

        # attached 물체 → 팔 따라 이동 (빨간색)
        if self._attached_object and self._attached_link:
            try:
                t = self._tf_buffer.lookup_transform(
                    'base_link', self._attached_link, rclpy.time.Time())
                self._viz.move_highlight(
                    self._attached_object,
                    t.transform.translation.x,
                    t.transform.translation.y,
                    t.transform.translation.z,
                )
            except Exception:
                pass

        # attach 없을 때 → 각 팔 최근접 물체 노란색 hover
        if self._current_zone and not self._attached_object:
            self._update_hover()
            # 그리퍼가 이미 닫힌 채로 물체에 접근한 경우 attach 트리거 (엣지 감지 보완)
            if self._hovered_l and self._gripper_l_closed:
                if not any(c[0] == _CMD_ATTACH_AUTO_L for c in self._pending):
                    self._pending.append((_CMD_ATTACH_AUTO_L, None))
            if self._hovered_r and self._gripper_r_closed:
                if not any(c[0] == _CMD_ATTACH_AUTO for c in self._pending):
                    self._pending.append((_CMD_ATTACH_AUTO, None))

        # /attached_object, /cm_state 주기적 발행 (구독자가 늦게 연결돼도 최신 상태 수신)
        self._publish_attached()
        s = String(); s.data = self._state.value
        self._state_pub.publish(s)

    def _update_hover(self):
        """각 팔 EEF 기준 GRASP_DISTANCE 이내 최근접 물체를 노란색으로 미리 표시."""
        COLOR_HOVER = (1.0, 1.0, 0.0, 1.0)
        threshold = GRASP_DISTANCE.get(self._current_zone, _GRASP_DISTANCE_DEFAULT)
        any_hover = False
        for side, link in (('l', 'end_effector_l_link'), ('r', 'end_effector_r_link')):
            try:
                t = self._tf_buffer.lookup_transform('base_link', link, rclpy.time.Time())
                pos = (t.transform.translation.x,
                       t.transform.translation.y,
                       t.transform.translation.z)
            except Exception:
                continue
            nearest_id, _ = find_nearest_graspable(
                self._client, self._current_zone, threshold, pos=pos,
                overrides=self._viz._overrides)
            old = self._hovered_l if side == 'l' else self._hovered_r
            if nearest_id != old:
                if old:
                    self._viz.restore_object(old)
                if nearest_id:
                    self._viz.highlight_object(nearest_id, COLOR_HOVER)
                if side == 'l':
                    self._hovered_l = nearest_id
                else:
                    self._hovered_r = nearest_id
            if nearest_id:
                any_hover = True

        # GRASPING/ATTACHED/RELEASING 중에는 hover 상태 전환 안 함
        if self._state == State.IDLE and any_hover:
            self._set_state(State.HOVERING)
        elif self._state == State.HOVERING and not any_hover:
            self._set_state(State.IDLE)

    def _gripper_cb(self, msg: JointState):
        names = msg.name

        if not self._gripper_initialized:
            # 첫 메시지: 현재 상태만 기록, attach 미발동 (엣지 감지 오작동 방지)
            if 'gripper_r_joint1' in names:
                self._gripper_r_closed = msg.position[names.index('gripper_r_joint1')] > GRIPPER_CLOSE_THRESHOLD
            if 'gripper_l_joint1' in names:
                self._gripper_l_closed = msg.position[names.index('gripper_l_joint1')] > GRIPPER_CLOSE_THRESHOLD
            
            if 'gripper_r_joint1' in names or 'gripper_l_joint1' in names:
                self._gripper_initialized = True
            return

        if 'gripper_r_joint1' in names:
            pos = msg.position[names.index('gripper_r_joint1')]
            if not self._gripper_r_closed and pos > GRIPPER_CLOSE_THRESHOLD:
                self._gripper_r_closed = True
                self._pending.append((_CMD_ATTACH_AUTO, None))
            elif self._gripper_r_closed and pos < GRIPPER_OPEN_THRESHOLD:
                self._gripper_r_closed = False
                self._pending.append((_CMD_DETACH_AUTO, None))

        if 'gripper_l_joint1' in names:
            pos = msg.position[names.index('gripper_l_joint1')]
            if not self._gripper_l_closed and pos > GRIPPER_CLOSE_THRESHOLD:
                self._gripper_l_closed = True
                self._pending.append((_CMD_ATTACH_AUTO_L, None))
            elif self._gripper_l_closed and pos < GRIPPER_OPEN_THRESHOLD:
                self._gripper_l_closed = False
                self._pending.append((_CMD_DETACH_AUTO_L, None))

    # ── 파지 검증 스레드 (Phase 1) ─────────────────────────────────────────────

    def _do_grasp(self, side_str: str, obj_id: str, link: str):
        """
        별도 스레드에서 실행. assess_on_close()가 spin_once를 내부 호출하므로
        메인 루프와 분리 필수.
        결과는 _grasp_result_q에 넣고, _check_grasp_results()에서 메인 루프가 처리.
        GRASP_ASSESSMENT_ENABLED=False 시 검증 생략 (Gazebo 시뮬용).
        """
        if not GRASP_ASSESSMENT_ENABLED:
            self._log.info(f'[GRASP] 파지 검증 스킵 (시뮬 모드) — {obj_id}')
            self._grasp_result_q.put((True, obj_id, link, side_str))
            return
        lut_name = OBJECT_LUT_NAME.get(obj_id, 'ETC')
        self._log.info(f'[GRASP] 파지 검증 시작 — {obj_id} (lut={lut_name}, side={side_str})')
        grasped = self._assessment.assess_on_close(side_str, lut_name)
        self._grasp_result_q.put((grasped, obj_id, link, side_str))

    def _check_grasp_results(self):
        """메인 루프에서 매 tick 호출. 검증 완료 시 attach 처리."""
        try:
            grasped, obj_id, link, side_str = self._grasp_result_q.get_nowait()
        except queue.Empty:
            return
        side_label = 'L' if side_str == 'left' else 'R'
        if not grasped:
            self._log.warn(f'[AUTO-{side_label}] 파지 검증 실패 — attach 취소 [{obj_id}]')
            self._set_state(State.IDLE)
            return
        if self._attached_object is not None:
            self._log.warn(f'[AUTO-{side_label}] 이미 attached 상태 — 중복 attach 생략')
            return
        attach_object(self._client, obj_id, link_name=link, viz=self._viz)
        self._attached_object = obj_id
        self._attached_link   = link
        self._set_state(State.ATTACHED)
        self._publish_attached()
        self._log.info(f'[AUTO-{side_label}] 파지 검증 성공 → Attached: {obj_id}')

    # ── 실제 실행: main loop에서 호출 ──────────────────────────────────────────

    def process_pending(self):
        while self._pending:
            cmd, arg = self._pending.pop(0)

            if cmd == _CMD_ZONE:
                if arg == self._current_zone:
                    self._log.info(f'Zone {arg} 이미 활성화됨 — 생략')
                else:
                    self._switch_zone(arg)

            elif cmd == _CMD_CLEAR:
                self._do_clear()

            elif cmd == _CMD_ATTACH:
                obj_id, arm = arg  # _attach_cb에서 파싱됨: arm = 'left' | 'right'
                link = f'end_effector_{arm[0]}_link'   # arm[0]: 'l' or 'r'
                attach_object(self._client, obj_id, link_name=link, viz=self._viz)
                self._attached_object = obj_id
                self._attached_link   = link
                self._set_state(State.ATTACHED)
                self._publish_attached()
                self._log.info(f'[수동] Attached: {obj_id} (arm={arm})')

            elif cmd == _CMD_DETACH:
                self._assessment.stop()
                link = self._attached_link or 'end_effector_r_link'
                drop = self._get_drop_pos(link)
                detach_object(self._client, arg, link_name=link, viz=self._viz, drop_pos=drop)
                if self._attached_object == arg:
                    self._attached_object = None
                    self._attached_link   = None
                    self._set_state(State.IDLE)
                    self._publish_attached()
                self._log.info(f'[수동] Detached: {arg}')

            elif cmd in (_CMD_ATTACH_AUTO, _CMD_ATTACH_AUTO_L):
                if self._attached_object is not None:
                    continue
                if self._grasp_thread is not None and self._grasp_thread.is_alive():
                    continue  # 파지 검증 이미 진행 중 — 중복 방지
                if not self._current_zone:
                    self._log.warn('[AUTO] 활성 구간 없음 — attach 생략')
                    continue

                link       = 'end_effector_l_link' if cmd == _CMD_ATTACH_AUTO_L else 'end_effector_r_link'
                side_str   = 'left'                if cmd == _CMD_ATTACH_AUTO_L else 'right'
                side_label = 'L'                   if cmd == _CMD_ATTACH_AUTO_L else 'R'
                threshold  = GRASP_DISTANCE.get(self._current_zone, _GRASP_DISTANCE_DEFAULT)

                # hover 사전 선택 물체 우선 (노란색으로 미리 표시됐던 것)
                hovered = self._hovered_l if cmd == _CMD_ATTACH_AUTO_L else self._hovered_r
                if hovered:
                    obj_id = hovered
                    if cmd == _CMD_ATTACH_AUTO_L:
                        self._hovered_l = None
                    else:
                        self._hovered_r = None
                else:
                    from ai_worker_manipulation.robot_interface.moveit_client import Arm
                    arm = Arm.LEFT if cmd == _CMD_ATTACH_AUTO_L else Arm.RIGHT
                    obj_id, _ = find_nearest_graspable(
                        self._client, self._current_zone, threshold, arm=arm,
                        overrides=self._viz._overrides)

                if obj_id:
                    self._set_state(State.GRASPING)
                    self._grasp_thread = threading.Thread(
                        target=self._do_grasp,
                        args=(side_str, obj_id, link),
                        daemon=True,
                    )
                    self._grasp_thread.start()
                    self._log.info(f'[AUTO-{side_label}] 파지 검증 스레드 시작 — {obj_id}')
                else:
                    self._log.info(f'[AUTO-{side_label}] 그리퍼 닫힘 — 활성 구간에 파지 물체 없음')

            elif cmd in (_CMD_DETACH_AUTO, _CMD_DETACH_AUTO_L):
                side_str   = 'left'  if cmd == _CMD_DETACH_AUTO_L else 'right'
                side_label = 'L'     if cmd == _CMD_DETACH_AUTO_L else 'R'
                # 검증 루프 중단 (열림 명령이 assess_on_close timeout보다 빠르게 처리)
                self._assessment.stop()
                self._set_state(State.RELEASING)
                if self._attached_object:
                    link = self._attached_link or 'end_effector_r_link'
                    drop = self._get_drop_pos(link)
                    detach_object(self._client, self._attached_object,
                                  link_name=link, viz=self._viz, drop_pos=drop)
                    self._log.info(f'[AUTO-{side_label}] Detached: {self._attached_object}')
                    self._attached_object = None
                    self._attached_link   = None
                    self._publish_attached()
                self._set_state(State.IDLE)

    def _get_drop_pos(self, link_name: str) -> tuple:
        """
        TF로 현재 EEF (x, y)를 얻어 낙하 좌표 반환 (전 구간 공통).
        - 현재 (x, y) 아래의 가장 높은 표면 위에 물체가 자유낙하
        - Zone C 한정: peg 근처(y ±8cm, x ±15cm)면 해당 peg 좌표로 스냅
        - fallback: 원래 오브젝트 z
        """
        from ai_worker_manipulation.ai_worker_manipulation.mission_control.environment import (
            _OBJECT_MARKER_MAP,
            ZONE_C_PEG_X, ZONE_C_PEG_Y_POSITIONS, ZONE_C_BOLT_Z,
            SURFACE_OBJECTS,
        )
        try:
            t = self._tf_buffer.lookup_transform('base_link', link_name, rclpy.time.Time())
            ex, ey = t.transform.translation.x, t.transform.translation.y
        except Exception:
            return (0.0, 0.0, 0.01)

        obj_half_h = 0.0
        fallback_z = 0.01
        if self._attached_object:
            m = _OBJECT_MARKER_MAP.get(self._attached_object)
            if m:
                obj_half_h = m.scale.z / 2.0
                fallback_z = m.pose.position.z

        if self._current_zone == 'C':
            for py in ZONE_C_PEG_Y_POSITIONS:
                if abs(ey - py) < 0.08 and abs(ex - ZONE_C_PEG_X) < 0.15:
                    return (ZONE_C_PEG_X, py, ZONE_C_BOLT_Z)

        best_top = None
        for obj_id in SURFACE_OBJECTS.get(self._current_zone, []):
            sm = _OBJECT_MARKER_MAP.get(obj_id)
            if sm is None:
                continue
            hx = sm.scale.x / 2.0
            hy = sm.scale.y / 2.0
            if abs(ex - sm.pose.position.x) <= hx and abs(ey - sm.pose.position.y) <= hy:
                top_z = sm.pose.position.z + sm.scale.z / 2.0
                if best_top is None or top_z > best_top:
                    best_top = top_z

        drop_z = (best_top + obj_half_h) if best_top is not None else fallback_z
        return (ex, ey, drop_z)

    def _switch_zone(self, zone: str):
        # 파지 검증 진행 중이면 즉시 중단
        if self._grasp_thread is not None and self._grasp_thread.is_alive():
            self._assessment.stop()
            self._grasp_thread.join(timeout=1.0)

        # 기존 attached 물체 먼저 detach (씬 전환 전 MoveIt 상태 정리)
        if self._attached_object:
            link = self._attached_link or 'end_effector_r_link'
            drop = self._get_drop_pos(link)
            detach_object(self._client, self._attached_object,
                          link_name=link, viz=self._viz, drop_pos=drop)
            self._log.info(f'[Zone 전환] 기존 attached 물체 제거: {self._attached_object}')
            self._attached_object = None
            self._attached_link   = None
            self._publish_attached()

        self._pending = [c for c in self._pending
                         if c[0] not in (_CMD_ATTACH_AUTO, _CMD_ATTACH_AUTO_L)]
        self._viz.clear()
        clear_all_objects(self._client)
        self._SETUP_FN[zone](self._client)
        self._viz.clear()
        self._viz.publish_zone(zone)
        self._current_zone = zone
        self._hovered_l = None
        self._hovered_r = None
        self._set_state(State.IDLE)
        self._allow_graspable(zone)
        self._log.info(f'Zone {zone} 초기화 완료 — 파지 물체 충돌 허용됨')

    def _allow_graspable(self, zone: str):
        """파지 대상 물체 충돌 허용 — environment.allow_zone_objects 위임."""
        allow_zone_objects(self._client, zone)

    def _do_clear(self):
        self._assessment.stop()
        # attached 물체 있으면 먼저 detach (clear_all_objects 전에 해야 MoveIt 씬 일관성 유지)
        if self._attached_object:
            link = self._attached_link or 'end_effector_r_link'
            detach_object(self._client, self._attached_object, link_name=link)
            self._attached_object = None
            self._attached_link   = None
            self._publish_attached()
        clear_all_objects(self._client)
        self._viz.clear()
        self._current_zone = None
        self._hovered_l = None
        self._hovered_r = None
        self._set_state(State.IDLE)
        self._log.info('환경 전체 제거 완료')


def main():
    client  = MoveItClient()
    manager = CompetitionManager(client)
    try:
        while rclpy.ok():
            rclpy.spin_once(client.node, timeout_sec=0.1)
            manager.process_pending()
            manager._check_grasp_results()   # 파지 검증 결과 처리
    except KeyboardInterrupt:
        pass
    finally:
        manager._assessment_node.destroy_node()
        client.shutdown()


if __name__ == '__main__':
    main()