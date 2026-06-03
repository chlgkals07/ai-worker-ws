import sys
import rclpy
from rclpy.node import Node

from ai_worker_manipulation.robot_interface.moveit_client import MoveItClient, Arm, MoveResult


class ClientTest:

    def __init__(self, node: Node):
        self._node = node
        self._log  = node.get_logger()
        self._passed = 0
        self._failed = 0

    def _ok(self, name: str, detail: str = '') -> None:
        msg = f'[PASS] {name}'
        if detail:
            msg += f'  |  {detail}'
        self._log.info(msg)
        self._passed += 1

    def _fail(self, name: str, detail: str = '') -> None:
        msg = f'[FAIL] {name}'
        if detail:
            msg += f'  |  {detail}'
        self._log.error(msg)
        self._failed += 1

    def _check(self, name: str, ok: bool, detail: str = '') -> bool:
        (self._ok if ok else self._fail)(name, detail)
        return ok

    def _section(self, title: str) -> None:
        self._log.info('')
        self._log.info(f'--- {title} ---')

    def run(self) -> int:
        self._log.info('=' * 60)
        self._log.info('MoveItClient API Test')
        self._log.info('=' * 60)

        # ── 1. Initialization ──────────────────────────────────────────
        self._section('1. Initialization')
        client = None
        try:
            client = MoveItClient(self._node)
            self._ok('client created', 'action servers ready, joint states received')
        except RuntimeError as e:
            self._fail('client created', str(e))
            self._log.error('Cannot continue without a working client — is move_group running?')
            return self._summary()
        except Exception as e:
            self._fail('client created', f'unexpected exception: {e}')
            return self._summary()

        # ── 2. get_joints ──────────────────────────────────────────────
        self._section('2. get_joints')
        for arm in [Arm.RIGHT, Arm.LEFT]:
            joints = client.get_joints(arm)
            ok = joints is not None and len(joints) == 7
            detail = f'{[f"{j:.3f}" for j in joints]}' if ok else f'returned {joints}'
            self._check(f'get_joints [{arm.value}] — 7 floats', ok, detail)

            if ok:
                in_range = all(-7.0 < j < 7.0 for j in joints)
                self._check(
                    f'get_joints [{arm.value}] — values in plausible range',
                    in_range,
                    'all within ±7 rad' if in_range else f'suspicious values: {joints}'
                )

        # ── 3. get_pose ────────────────────────────────────────────────
        self._section('3. get_pose (FK)')
        for arm in [Arm.RIGHT, Arm.LEFT]:
            pose = client.get_pose(arm)
            ok = pose is not None
            if ok:
                p = pose.position
                detail = f'pos=({p.x:.3f}, {p.y:.3f}, {p.z:.3f})'
            else:
                detail = 'returned None — is move_group compute_fk service available?'
            self._check(f'get_pose [{arm.value}]', ok, detail)

        # ── 4. Input validation ────────────────────────────────────────
        self._section('4. Input validation')

        r = client.move_to_joints([0.0] * 6, arm=Arm.RIGHT)
        self._check(
            'move_to_joints rejects 6 values → INVALID',
            r == MoveResult.INVALID,
            f'got {r.value}'
        )

        r = client.move_to_joints([0.0] * 8, arm=Arm.RIGHT)
        self._check(
            'move_to_joints rejects 8 values → INVALID',
            r == MoveResult.INVALID,
            f'got {r.value}'
        )

        # ── 5. Destroy guard ───────────────────────────────────────────
        self._section('5. Destroy guard')
        client.destroy()
        self._ok('destroy() called')

        raised = False
        try:
            client.move_to_joints([0.0] * 7)
        except RuntimeError:
            raised = True
        except Exception as e:
            self._fail('destroy guard raises RuntimeError', f'unexpected exception: {e}')

        if raised:
            self._ok('move_to_joints raises RuntimeError after destroy()')

        raised = False
        try:
            client.get_joints()
        except RuntimeError:
            raised = True
        except Exception as e:
            self._fail('destroy guard on get_joints', f'unexpected exception: {e}')

        if raised:
            self._ok('get_joints raises RuntimeError after destroy()')

        return self._summary()

    def _summary(self) -> int:
        total = self._passed + self._failed
        self._log.info('')
        self._log.info('=' * 60)
        self._log.info(f'Results: {self._passed}/{total} passed    {self._failed} failed')
        self._log.info('=' * 60)
        return 0 if self._failed == 0 else 1


# ------------------------------------------------------------------


def main():
    rclpy.init()
    node = Node('test_moveit_client')
    test = ClientTest(node)
    try:
        code = test.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
