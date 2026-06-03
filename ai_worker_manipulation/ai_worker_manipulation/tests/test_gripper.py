import sys
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from ai_worker_manipulation.robot_interface.gripper_command import GripperCommand
from ai_worker_manipulation.robot_interface.gripper_controller import GripperInterface
from ai_worker_manipulation.skill_primitives.grasp_assessment import GraspAssessment


class GripperTest:

    def __init__(
        self,
        node: Node,
        gripper: GripperInterface,
        command: GripperCommand,
        assessment: GraspAssessment,
    ):
        self._node       = node
        self._log        = node.get_logger()
        self._gripper    = gripper
        self._command    = command
        self._assessment = assessment
        self._passed     = 0
        self._failed     = 0

    def _ok(self, name: str, detail: str = '') -> None:
        self._log.info(f'[PASS] {name}' + (f'  |  {detail}' if detail else ''))
        self._passed += 1

    def _fail(self, name: str, detail: str = '') -> None:
        self._log.error(f'[FAIL] {name}' + (f'  |  {detail}' if detail else ''))
        self._failed += 1

    def _check(self, name: str, ok: bool, detail: str = '') -> bool:
        (self._ok if ok else self._fail)(name, detail)
        return ok

    def _section(self, title: str) -> None:
        self._log.info('')
        self._log.info('─' * 60)
        self._log.info(f'  {title}')
        self._log.info('─' * 60)

    def _wait(self) -> None:
        self._gripper.wait_until_executed()
        self._gripper.wait_motion()

    def _pos(self, side: str) -> str:
        p = self._command.get_position(side)
        return f'{p:.3f}' if p is not None else 'None'

    def section_init(self) -> bool:
        self._section('1. Init — waiting for /joint_states')

        deadline = time.time() + 5.0
        while time.time() < deadline:
            if all(self._command.get_position(s) is not None for s in ('right', 'left')):
                break
            time.sleep(0.1)

        ok = all(self._command.get_position(s) is not None for s in ('right', 'left'))
        detail = (
            f'right={self._pos("right")}  left={self._pos("left")}' if ok
            else 'timed out — is the robot stack running?'
        )
        return self._check('joint states received', ok, detail)

    def section_right(self) -> None:
        self._section('2. Right gripper — open / close')

        self._log.info(f'[right] current position={self._pos("right")} '
                       f'is_open={self._gripper.is_open("right")} '
                       f'is_closed={self._gripper.is_closed("right")}')

        self._log.info('[right] opening...')
        self._gripper.open('right')
        self._wait()
        pos = self._command.get_position('right')
        self._check('open [right]', pos is not None and pos <= 0.2,
                    f'position={self._pos("right")}')

        self._log.info('[right] closing...')
        self._gripper.close('right')
        self._wait()
        pos = self._command.get_position('right')
        self._check('close [right]', pos is not None and pos >= 0.8,
                    f'position={self._pos("right")}')

        self._gripper.open('right')
        self._wait()

    def section_left(self) -> None:
        self._section('3. Left gripper — open / close')

        self._log.info(f'[left] current position={self._pos("left")} '
                       f'is_open={self._gripper.is_open("left")} '
                       f'is_closed={self._gripper.is_closed("left")}')

        self._log.info('[left] opening...')
        self._gripper.open('left')
        self._wait()
        pos = self._command.get_position('left')
        self._check('open [left]', pos is not None and pos <= 0.2,
                    f'position={self._pos("left")}')

        self._log.info('[left] closing...')
        self._gripper.close('left')
        self._wait()
        pos = self._command.get_position('left')
        self._check('close [left]', pos is not None and pos >= 0.8,
                    f'position={self._pos("left")}')

        self._gripper.open('left')
        self._wait()

    def section_both(self) -> None:
        self._section('4. Both grippers simultaneously — open / close')

        self._log.info('[both] opening...')
        self._gripper.open('both')
        self._wait()

        for side in ('right', 'left'):
            pos = self._command.get_position(side)
            self._check(f'open both — [{side}]', pos is not None and pos <= 0.2,
                        f'position={self._pos(side)}')

        self._log.info('[both] closing...')
        self._gripper.close('both')
        self._wait()

        for side in ('right', 'left'):
            pos = self._command.get_position(side)
            self._check(f'close both — [{side}]', pos is not None and pos >= 0.8,
                        f'position={self._pos(side)}')

        self._gripper.open('both')
        self._wait()

    def _run_assess(self, side: str) -> dict | None:
        try:
            return self._assessment.assess(side, 'ETC')
        except KeyError as e:
            self._fail('assess() ran', f'object_lut.json missing entry: {e}')
            return None
        except Exception as e:
            self._fail('assess() ran', f'unexpected error: {e}')
            return None

    def _log_assess(self, result: dict) -> None:
        self._log.info(
            f'[assessment] '
            f"pos={result['position']}  effort={result['effort']}  "
            f"pos_thresh={result['pos_thresh']}  eff_thresh={result['eff_thresh']}  "
            f"position_ok={result['position_ok']}  effort_ok={result['effort_ok']}  "
            f"is_grasping={result['is_grasping']}"
        )

    def section_grasp_assessment(self) -> None:
        self._section(
            '5. Grasp assessment\n'
            '   Part A: empty gripper  — works in sim + hardware\n'
            '   Part B: with object    — hardware only (sim effort=0 always)'
        )

        expected_keys = {'is_grasping', 'position_ok', 'effort_ok',
                         'position', 'effort', 'pos_thresh', 'eff_thresh'}

        self._log.info('[right] closing with no object...')
        self._gripper.close('right')
        self._wait()

        result = self._run_assess('right')
        if result is None:
            self._gripper.open('right')
            self._wait()
            return

        self._check('assess() returns correct keys',
                    expected_keys.issubset(result.keys()),
                    str(list(result.keys())))

        self._check('assess() is_grasping=False with empty gripper',
                    result['is_grasping'] is False,
                    f"is_grasping={result['is_grasping']}")

        self._log_assess(result)

        self._gripper.open('right')
        self._wait()

        self._log.info('')
        self._log.info('>>> HARDWARE ONLY — sim will fail (effort=0)')
        self._log.info('>>> Place an object in the RIGHT gripper, then press ENTER...')
        input()

        self._log.info('[right] closing around object...')
        self._gripper.close('right')
        self._wait()

        result = self._run_assess('right')
        if result is None:
            self._gripper.open('right')
            self._wait()
            return

        self._log_assess(result)

        self._check('assess() position_ok=True (object blocking close)',
                    result['position_ok'] is True,
                    f"position={result['position']}  pos_thresh={result['pos_thresh']}")

        self._check('assess() effort_ok=True (gripper squeezing)',
                    result['effort_ok'] is True,
                    f"effort={result['effort']}  eff_thresh={result['eff_thresh']}")

        self._check('assess() is_grasping=True with object',
                    result['is_grasping'] is True,
                    f"is_grasping={result['is_grasping']}")

        self._log.info('>>> Remove the object, then press ENTER...')
        input()

        self._gripper.open('right')
        self._wait()

    def run(self) -> int:
        self._log.info('=' * 60)
        self._log.info('Gripper Test — open / close / assessment')
        self._log.info('=' * 60)

        if not self.section_init():
            self._log.error('Aborting — no joint states.')
            return self._summary()

        self.section_right()
        self.section_left()
        self.section_both()
        self.section_grasp_assessment()

        return self._summary()

    def _summary(self) -> int:
        total = self._passed + self._failed
        self._log.info('')
        self._log.info('=' * 60)
        self._log.info(f'Results: {self._passed}/{total} passed    {self._failed} failed')
        self._log.info('=' * 60)
        return 0 if self._failed == 0 else 1


def main():
    rclpy.init()
    node = Node('test_gripper')

    cb = ReentrantCallbackGroup()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    command    = GripperCommand(node, callback_group=cb)
    gripper    = GripperInterface(node)
    assessment = GraspAssessment(node, callback_group=cb)

    test = GripperTest(node, gripper, command, assessment)

    try:
        code = test.run()
    except Exception as e:
        node.get_logger().error(f'Test aborted: {e}')
        code = 1
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(code)


if __name__ == '__main__':
    main()
