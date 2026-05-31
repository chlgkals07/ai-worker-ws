"""
Quick TF check — transforms raw GPD poses to base_link and prints results.

Change CAMERA_FRAME to match your setup if the transform fails.
Likely candidates: 'camera_r_link', 'camera_l_link', 'zedm_camera_center'

Usage:
    ros2 run ai_worker_manipulation test_tf_gpd
"""

import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation

from ai_worker_manipulation.robot_interface.tf_transformer import tf_transformer


# ── Change this if transform fails ────────────────────────────────────────────
# Try in order: camera_r_optical_frame, camera_l_link, zedm_camera_center
# If all fail or give bad results, GPD may already be in base_link (no transform needed)
CAMERA_FRAME = 'camera_r_optical_frame'
TARGET_FRAME = 'base_link'

# ── Raw GPD candidates (in camera frame) ──────────────────────────────────────
# Format: (label, x, y, z, qx, qy, qz, qw)
GPD_POSES = [
    ('Grasp 0 (score=-1492)', 0.5073, -0.1971, 0.9444,  0.7780,  0.5259, -0.3392,  0.0556),
    ('Grasp 1 (score=-1977)', 0.5422, -0.2481, 0.8984,  0.6434,  0.0768, -0.6208, -0.4412),
    ('Grasp 2 (score=-2223)', 0.5363, -0.2282, 0.9197,  0.6680, -0.1093, -0.6092, -0.4132),
]

# Workspace sanity check bounds (base_link frame, from test_arm_motion.py)
X_MIN, X_MAX = 0.0,   0.35
Y_MIN, Y_MAX = -0.55, 0.55
Z_MIN, Z_MAX = 0.60,  0.85
# ──────────────────────────────────────────────────────────────────────────────


def rpy_str(qx, qy, qz, qw) -> str:
    rpy = Rotation.from_quat([qx, qy, qz, qw]).as_euler('xyz', degrees=True)
    return f'R={rpy[0]:.1f}°  P={rpy[1]:.1f}°  Y={rpy[2]:.1f}°'


def in_workspace(x, y, z) -> bool:
    return X_MIN <= x <= X_MAX and Y_MIN <= y <= Y_MAX and Z_MIN <= z <= Z_MAX


def main():
    rclpy.init()
    node = Node('test_tf_gpd')
    log  = node.get_logger()
    tf   = tf_transformer(node)

    log.info('=' * 60)
    log.info(f'TF GPD Check  {CAMERA_FRAME} → {TARGET_FRAME}')
    log.info('=' * 60)

    # Wait for TF to become available
    log.info(f'Waiting for transform {CAMERA_FRAME} → {TARGET_FRAME}...')
    deadline = time.time() + 5.0
    available = False
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if tf.is_transform_available(CAMERA_FRAME, TARGET_FRAME):
            available = True
            break

    if not available:
        log.error(
            f'Transform {CAMERA_FRAME} → {TARGET_FRAME} not available after 5s.\n'
            f'Try: CAMERA_FRAME = "camera_l_link" or "zedm_camera_center"'
        )
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    log.info('Transform available. Converting GPD poses...\n')

    for label, x, y, z, qx, qy, qz, qw in GPD_POSES:
        log.info(f'── {label} ──')
        log.info(f'  Camera frame : pos=({x:.3f}, {y:.3f}, {z:.3f})  '
                 f'{rpy_str(qx, qy, qz, qw)}')

        ps              = PoseStamped()
        ps.header.frame_id    = CAMERA_FRAME
        ps.header.stamp       = rclpy.time.Time().to_msg()  # Time(0) = use latest available
        ps.pose.position.x    = x
        ps.pose.position.y    = y
        ps.pose.position.z    = z
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw

        result = tf.transform_pose(ps, TARGET_FRAME)

        if result is None:
            log.error('  base_link    : transform FAILED')
            continue

        p, o = result.position, result.orientation
        in_ws = in_workspace(p.x, p.y, p.z)

        log.info(
            f'  base_link    : pos=({p.x:.3f}, {p.y:.3f}, {p.z:.3f})  '
            f'{rpy_str(o.x, o.y, o.z, o.w)}'
        )
        log.info(f'  Workspace    : {"✓ IN RANGE" if in_ws else "✗ OUT OF RANGE"}  '
                 f'(x:[{X_MIN},{X_MAX}] y:[{Y_MIN},{Y_MAX}] z:[{Z_MIN},{Z_MAX}])')
        log.info('')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
