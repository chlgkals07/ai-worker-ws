import time
import json
import os
from typing import Optional

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState

import ament_index_python.packages as ament

_PKG_DIR = ament.get_package_share_directory('ai_worker_manipulation')
LUT_PATH = os.path.join(_PKG_DIR, 'data', 'object_lut.json')


def _load_lut(path: str) -> dict:
    with open(path, 'r') as f:
        data = json.load(f)
    return data['objects']


POSITION_OPEN   = 0
POSITION_CLOSED = 1150

STABLE_DURATION = 1.0
STABLE_POLL_HZ  = 20


class GraspAssessment:

    def __init__(self, node: Node, lut_path: str = LUT_PATH, callback_group=None):
        self._node = node
        self._log  = node.get_logger()
        self._lut  = _load_lut(lut_path)

        cb = callback_group or ReentrantCallbackGroup()

        self._positions: dict[str, float] = {}
        self._efforts:   dict[str, float] = {}

        self._sub = self._node.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, 10,
            callback_group=cb,
        )
        self._log.info('[GraspAssessment] 초기화 완료. LUT 로드됨.')

    # ── 콜백 ──────────────────────────────────────────────────────────────────

    def _joint_state_cb(self, msg: JointState):
        for name, pos, eff in zip(msg.name, msg.position, msg.effort):
            self._positions[name] = pos
            self._efforts[name]   = eff

    # ── 내부 헬퍼 ─────────────────────────────────────────────────────────────

    def _joint_name(self, side: str) -> str:
        if side not in ('left', 'right'):
            raise ValueError(f"side는 'left' 또는 'right' 여야 합니다. 입력값: {side}")
        prefix = 'l' if side == 'left' else 'r'
        return f'gripper_{prefix}_joint1'

    def _get_position_raw(self, side: str) -> Optional[float]:
        joint = self._joint_name(side)
        norm  = self._positions.get(joint)
        if norm is None:
            return None
        return norm * POSITION_CLOSED

    def _get_effort_abs(self, side: str) -> float:
        joint = self._joint_name(side)
        return abs(self._efforts.get(joint, 0.0))

    # ── Public API ────────────────────────────────────────────────────────────

    def get_lut_objects(self) -> list[str]:
        return list(self._lut.keys())

    def assess(self, side: str, object_name: str) -> dict:
        if object_name not in self._lut:
            available = list(self._lut.keys())
            raise KeyError(
                f"오브젝트 '{object_name}' 가 LUT에 없습니다. "
                f"사용 가능: {available}"
            )

        entry      = self._lut[object_name]
        pos_thresh = float(entry['position_min'])
        eff_thresh = float(entry['effort_min'])

        pos = self._get_position_raw(side)
        eff = self._get_effort_abs(side)

        if pos is None:
            self._log.warn(f'[GraspAssessment] {side} 그리퍼 joint_states 미수신.')
            return {
                'is_grasping': False,
                'position_ok': False,
                'effort_ok':   False,
                'position':    None,
                'effort':      eff,
                'pos_thresh':  pos_thresh,
                'eff_thresh':  eff_thresh,
            }

        position_ok = pos < pos_thresh
        effort_ok   = eff > eff_thresh
        is_grasping = position_ok and effort_ok

        return {
            'is_grasping': is_grasping,
            'position_ok': position_ok,
            'effort_ok':   effort_ok,
            'position':    round(pos, 2),
            'effort':      round(eff, 4),
            'pos_thresh':  pos_thresh,
            'eff_thresh':  eff_thresh,
        }

    def assess_stable(self, side: str, object_name: str,
                      duration: float = STABLE_DURATION) -> bool:
        self._log.info(
            f'[GraspAssessment] {side}/{object_name} 안정성 평가 시작 '
            f'(목표: {duration}초 유지)'
        )

        stable_start: Optional[float] = None
        poll_interval = 1.0 / STABLE_POLL_HZ
        deadline = time.time() + duration * 5

        while time.time() < deadline:
            time.sleep(poll_interval)
            result = self.assess(side, object_name)

            if result['is_grasping']:
                if stable_start is None:
                    stable_start = time.time()
                    self._log.info(
                        f'[GraspAssessment] 조건 충족 시작 '
                        f"pos={result['position']:.1f} "
                        f"eff={result['effort']:.4f}"
                    )
                elapsed = time.time() - stable_start
                if elapsed >= duration:
                    self._log.info(
                        f'[GraspAssessment] ✅ 파지 안정 확인 ({elapsed:.2f}초 유지)'
                    )
                    return True
            else:
                if stable_start is not None:
                    self._log.info(
                        f'[GraspAssessment] 조건 불충족 → 타이머 리셋 '
                        f"position_ok={result['position_ok']} "
                        f"effort_ok={result['effort_ok']}"
                    )
                stable_start = None

        self._log.warn('[GraspAssessment] ❌ 안정성 조건 미충족 (시간 초과)')
        return False
