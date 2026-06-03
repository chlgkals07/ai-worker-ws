"""
Arm motion test — focused on move_to_pose with various positions and orientations.

How to add tests:
  - Add a pose to TEST_POSES / TEST_POSES_L: (x, y, z, qx, qy, qz, qw)
  - Run get_pose() first (Section 0) to find your robot's current workspace

Sections:
  0. Current state — prints FK pose and joint positions for both arms
  1. Pose sweep (right) — each pose in TEST_POSES tested with RRTConnect
  2. Pose sweep (left)  — each pose in TEST_POSES_L tested with RRTConnect

Usage:
    ros2 run ai_worker_manipulation test_arm_motion
"""

import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

from ai_worker_manipulation.robot_interface.moveit_client import MoveItClient, Arm, MoveResult


# ------------------------------------------------------------------
# Poses to test
# Fill these in based on your robot's actual workspace.
# Start conservative — small offsets from home FK pose.
# Format: 'label': (x, y, z, qx, qy, qz, qw)
# ------------------------------------------------------------------

TEST_POSES = {
    # All poses use base_link frame. READY_POSE = (0.060, -0.227, 0.675) is the
    # homing target; all tests start from there. Safe z >= 0.625 (lower needs
    # forward offset to keep joint2 away from its upper limit at 0.001 rad).
    # Quaternions: (qx, qy, qz, qw). Rotations are intrinsic around the listed axis.

    # ── Baseline ──────────────────────────────────────────────────────────────
    'ready':             (0.060, -0.227,  0.675, 0.000,  0.000,  0.000,  1.000),

    # ── Position sweep — identity orientation ─────────────────────────────────
    'forward_near':      (0.120, -0.227,  0.675, 0.000,  0.000,  0.000,  1.000),
    'forward_mid':       (0.200, -0.227,  0.675, 0.000,  0.000,  0.000,  1.000),
    'far_forward':       (0.250, -0.227,  0.675, 0.000,  0.000,  0.000,  1.000),
    'side_near':         (0.060, -0.327,  0.675, 0.000,  0.000,  0.000,  1.000),
    'side_far':          (0.060, -0.427,  0.675, 0.000,  0.000,  0.000,  1.000),
    'higher':            (0.060, -0.227,  0.775, 0.000,  0.000,  0.000,  1.000),
    'lower':             (0.100, -0.250,  0.625, 0.000,  0.000,  0.000,  1.000),
    'diagonal':          (0.150, -0.350,  0.675, 0.000,  0.000,  0.000,  1.000),
    'high_forward':      (0.150, -0.250,  0.775, 0.000,  0.000,  0.000,  1.000),
    'high_side':         (0.060, -0.350,  0.775, 0.000,  0.000,  0.000,  1.000),
    'lower_side':        (0.100, -0.350,  0.625, 0.000,  0.000,  0.000,  1.000),

    # ── Z-axis rotation (wrist yaw) ────────────────────────────────────────────
    'rot_z_neg90':       (0.060, -0.227,  0.675, 0.000,  0.000, -0.707,  0.707),
    'rot_z_neg45':       (0.060, -0.227,  0.675, 0.000,  0.000, -0.383,  0.924),
    'rot_z_45':          (0.060, -0.227,  0.675, 0.000,  0.000,  0.383,  0.924),
    'rot_z_90':          (0.060, -0.227,  0.675, 0.000,  0.000,  0.707,  0.707),

    # ── Y-axis rotation (EE pitch — tilt forward/back) ─────────────────────────
    'rot_y_30':          (0.060, -0.227,  0.675, 0.000,  0.259,  0.000,  0.966),
    'rot_y_45':          (0.060, -0.227,  0.675, 0.000,  0.383,  0.000,  0.924),
    'rot_y_90':          (0.060, -0.227,  0.675, 0.000,  0.707,  0.000,  0.707),

    # ── X-axis rotation (EE roll) ──────────────────────────────────────────────
    'rot_x_30':          (0.060, -0.227,  0.675, 0.259,  0.000,  0.000,  0.966),
    'rot_x_neg30':       (0.060, -0.227,  0.675,-0.259,  0.000,  0.000,  0.966),

    # ── Combined position + orientation (practical grasping scenarios) ─────────
    'fwd_rot_z45':       (0.150, -0.250,  0.675, 0.000,  0.000,  0.383,  0.924),
    'fwd_rot_z_neg45':   (0.150, -0.250,  0.675, 0.000,  0.000, -0.383,  0.924),
    'fwd_rot_z90':       (0.150, -0.250,  0.675, 0.000,  0.000,  0.707,  0.707),
    'fwd_rot_z_neg90':   (0.150, -0.250,  0.675, 0.000,  0.000, -0.707,  0.707),
    'fwd_rot_y30':       (0.150, -0.250,  0.675, 0.000,  0.259,  0.000,  0.966),
    'fwd_rot_x30':       (0.150, -0.250,  0.675, 0.259,  0.000,  0.000,  0.966),
    'side_rot_z90':      (0.060, -0.350,  0.675, 0.000,  0.000,  0.707,  0.707),
    'lower_rot_z45':     (0.100, -0.250,  0.625, 0.000,  0.000,  0.383,  0.924),
    'diagonal_rot_z45':  (0.150, -0.350,  0.675, 0.000,  0.000,  0.383,  0.924),
}

TEST_POSES_L = {
    'ready_l':           (0.060,  0.227,  0.675, 0.000,  0.000,  0.000,  1.000),
    'forward_near_l':    (0.120,  0.227,  0.675, 0.000,  0.000,  0.000,  1.000),
    'forward_mid_l':     (0.200,  0.227,  0.675, 0.000,  0.000,  0.000,  1.000),
    'side_near_l':       (0.060,  0.327,  0.675, 0.000,  0.000,  0.000,  1.000),
    'side_far_l':        (0.060,  0.427,  0.675, 0.000,  0.000,  0.000,  1.000),
    'higher_l':          (0.060,  0.227,  0.775, 0.000,  0.000,  0.000,  1.000),
    'lower_l':           (0.100,  0.250,  0.625, 0.000,  0.000,  0.000,  1.000),
    'diagonal_l':        (0.150,  0.350,  0.675, 0.000,  0.000,  0.000,  1.000),
    'rot_z_neg90_l':     (0.060,  0.227,  0.675, 0.000,  0.000, -0.707,  0.707),
    'rot_z_neg45_l':     (0.060,  0.227,  0.675, 0.000,  0.000, -0.383,  0.924),
    'rot_z_45_l':        (0.060,  0.227,  0.675, 0.000,  0.000,  0.383,  0.924),
    'rot_z_90_l':        (0.060,  0.227,  0.675, 0.000,  0.000,  0.707,  0.707),
    'rot_y_30_l':        (0.060,  0.227,  0.675, 0.000,  0.259,  0.000,  0.966),
    'rot_y_45_l':        (0.060,  0.227,  0.675, 0.000,  0.383,  0.000,  0.924),
    'rot_x_30_l':        (0.060,  0.227,  0.675, 0.259,  0.000,  0.000,  0.966),
    'fwd_rot_z45_l':     (0.150,  0.250,  0.675, 0.000,  0.000,  0.383,  0.924),
    'fwd_rot_y30_l':     (0.150,  0.250,  0.675, 0.000,  0.259,  0.000,  0.966),
}

_READY_POSE_XYZQ   = (0.060, -0.227, 0.675, 0.0, 0.0, 0.0, 1.0)
_READY_POSE_L_XYZQ = (0.060,  0.227, 0.675, 0.0, 0.0, 0.0, 1.0)

# WARNING: joint_limits.yaml sets arm_r_joint2 max_position=0.001, so all-zero config
# puts joint2 right at its upper limit — causes boundary singularities and planning failures.
HOME_R = [0.0, -0.5, 0.0,  0.5, 0.0, 0.0, 0.0]
HOME_L = [0.0,  0.5, 0.0, -0.5, 0.0, 0.0, 0.0]


# ------------------------------------------------------------------

def make_pose(x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0) -> Pose:
    p = Pose()
    p.position.x, p.position.y, p.position.z = x, y, z
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
    return p


# ------------------------------------------------------------------

class MotionTest:

    def __init__(self, client: MoveItClient, log):
        self._client = client
        self._log    = log
        self._passed = 0
        self._failed = 0

    def _ok(self, name: str, detail: str = '') -> None:
        self._log.info(f'[PASS] {name}' + (f'  |  {detail}' if detail else ''))
        self._passed += 1

    def _fail(self, name: str, detail: str = '') -> None:
        self._log.error(f'[FAIL] {name}' + (f'  |  {detail}' if detail else ''))
        self._failed += 1

    def _check(self, name: str, result: MoveResult,
               expected: MoveResult = MoveResult.SUCCEEDED) -> None:
        ok = result == expected
        detail = f'result={result.value}'
        (self._ok if ok else self._fail)(name, detail)

    def _section(self, title: str) -> None:
        self._log.info('')
        self._log.info(f'{"─" * 60}')
        self._log.info(f'  {title}')
        self._log.info(f'{"─" * 60}')

    def _home(self) -> bool:
        ready = make_pose(*_READY_POSE_XYZQ)
        pose  = self._client.get_pose(arm=Arm.RIGHT)
        if pose is not None:
            dx  = abs(pose.position.x - ready.position.x)
            dy  = abs(pose.position.y - ready.position.y)
            dz  = abs(pose.position.z - ready.position.z)
            dqx = abs(pose.orientation.x - ready.orientation.x)
            dqy = abs(pose.orientation.y - ready.orientation.y)
            dqz = abs(pose.orientation.z - ready.orientation.z)
            dqw = abs(pose.orientation.w - ready.orientation.w)
            if dx < 0.01 and dy < 0.01 and dz < 0.01 and dqx < 0.05 and dqy < 0.05 and dqz < 0.05 and dqw < 0.05:
                self._log.info('[home] arm_r already at ready pose — skipping')
                return True
        result = self._client.move_to_pose(ready, arm=Arm.RIGHT, velocity=0.2)
        if result != MoveResult.SUCCEEDED:
            self._log.error(f'[home] arm_r ready-pose move FAILED — result={result.value}')
            return False
        return True

    def _home_l(self) -> bool:
        ready = make_pose(*_READY_POSE_L_XYZQ)
        pose  = self._client.get_pose(arm=Arm.LEFT)
        if pose is not None:
            dx  = abs(pose.position.x - ready.position.x)
            dy  = abs(pose.position.y - ready.position.y)
            dz  = abs(pose.position.z - ready.position.z)
            dqx = abs(pose.orientation.x - ready.orientation.x)
            dqy = abs(pose.orientation.y - ready.orientation.y)
            dqz = abs(pose.orientation.z - ready.orientation.z)
            dqw = abs(pose.orientation.w - ready.orientation.w)
            if dx < 0.01 and dy < 0.01 and dz < 0.01 and dqx < 0.05 and dqy < 0.05 and dqz < 0.05 and dqw < 0.05:
                self._log.info('[home_l] arm_l already at ready pose — skipping')
                return True
        result = self._client.move_to_pose(ready, arm=Arm.LEFT, velocity=0.2)
        if result != MoveResult.SUCCEEDED:
            self._log.error(f'[home_l] arm_l ready-pose move FAILED — result={result.value}')
            return False
        return True

    def section_current_state(self) -> None:
        self._section('0. Current state (read-only, no motion)')

        for arm, label in ((Arm.RIGHT, 'arm_r'), (Arm.LEFT, 'arm_l')):
            joints = self._client.get_joints(arm=arm)
            if joints:
                jstr = ', '.join(f'{j:.3f}' for j in joints)
                self._log.info(f'{label} joints : [{jstr}]')
            else:
                self._log.warn(f'{label} joints : unavailable')

            pose = self._client.get_pose(arm=arm)
            if pose:
                p, o = pose.position, pose.orientation
                self._log.info(
                    f'{label} FK pose: pos=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) '
                    f'quat=({o.x:.3f}, {o.y:.3f}, {o.z:.3f}, {o.w:.3f})'
                )
            else:
                self._log.warn(f'{label} FK pose: unavailable')

    def section_pose_sweep(self) -> None:
        self._section(f'1. Pose sweep (right) — {len(TEST_POSES)} poses, RRTConnect')

        for label, (x, y, z, qx, qy, qz, qw) in TEST_POSES.items():
            if not self._home():
                self._fail(f'pose [{label}]', 'skipped — homing failed')
                continue
            pose = make_pose(x, y, z, qx, qy, qz, qw)
            result = self._client.move_to_pose(pose, arm=Arm.RIGHT, velocity=0.1)
            self._check(f'pose [{label}]', result)

    def section_pose_sweep_left(self) -> None:
        self._section(f'2. Pose sweep (left) — {len(TEST_POSES_L)} poses, RRTConnect')

        for label, (x, y, z, qx, qy, qz, qw) in TEST_POSES_L.items():
            if not self._home_l():
                self._fail(f'pose_l [{label}]', 'skipped — homing failed')
                continue
            pose = make_pose(x, y, z, qx, qy, qz, qw)
            result = self._client.move_to_pose(pose, arm=Arm.LEFT, velocity=0.1)
            self._check(f'pose_l [{label}]', result)

    def run(self) -> int:
        self._log.info('=' * 60)
        self._log.info('Arm Motion Test — move_to_pose')
        self._log.info('=' * 60)

        self.section_current_state()
        self.section_pose_sweep()
        self.section_pose_sweep_left()

        self._log.info('')
        self._log.info('--- Final: return to home joints ---')
        self._client.move_to_joints(HOME_R, arm=Arm.RIGHT, velocity=0.1)
        self._client.move_to_joints(HOME_L, arm=Arm.LEFT, velocity=0.1)

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
    node   = Node('test_arm_motion')
    client = MoveItClient(node)
    test   = MotionTest(client, node.get_logger())
    try:
        code = test.run()
    except Exception as e:
        node.get_logger().error(f'Test aborted: {e}')
        code = 1
    finally:
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
