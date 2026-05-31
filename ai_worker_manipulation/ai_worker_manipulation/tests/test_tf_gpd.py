"""
TF GPD Check with RViz visualization.

Prints before/after coordinates and publishes markers to RViz:
  /gpd_debug/before  — RED arrows in camera frame (as GPD gave them)
  /gpd_debug/after   — GREEN arrows in base_link  (after transform)

Arrow direction = gripper approach direction.
Bright green = in workspace, dark green = out of workspace.

Usage:
    ros2 run ai_worker_manipulation test_tf_gpd

RViz setup:
    Add → By topic → /gpd_debug/before  (MarkerArray)
    Add → By topic → /gpd_debug/after   (MarkerArray)
    Fixed Frame = base_link
"""

import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from scipy.spatial.transform import Rotation

from ai_worker_manipulation.robot_interface.tf_transformer import tf_transformer


# ── Change this if transform fails ────────────────────────────────────────────
# Try in order: camera_r_optical_frame, camera_r_link, camera_l_link, zedm_camera_center
# If all fail or give bad results, GPD may already be in base_link (no transform needed)
CAMERA_FRAME = 'camera_r_optical_frame'
TARGET_FRAME = 'base_link'

# ── Raw GPD candidates (copy-paste from /gpd/grasp_poses topic) ───────────────
# Format: (label, x, y, z, qx, qy, qz, qw)
GPD_POSES = [
    ('Grasp 0 (score=-1492)', 0.5073, -0.1971, 0.9444,  0.7780,  0.5259, -0.3392,  0.0556),
    ('Grasp 1 (score=-1977)', 0.5422, -0.2481, 0.8984,  0.6434,  0.0768, -0.6208, -0.4412),
    ('Grasp 2 (score=-2223)', 0.5363, -0.2282, 0.9197,  0.6680, -0.1093, -0.6092, -0.4132),
]

# Workspace sanity check bounds (base_link frame)
X_MIN, X_MAX = 0.0,   0.35
Y_MIN, Y_MAX = -0.55, 0.55
Z_MIN, Z_MAX = 0.60,  0.85
# ──────────────────────────────────────────────────────────────────────────────


def rpy_str(qx, qy, qz, qw) -> str:
    rpy = Rotation.from_quat([qx, qy, qz, qw]).as_euler('xyz', degrees=True)
    return f'R={rpy[0]:.1f}°  P={rpy[1]:.1f}°  Y={rpy[2]:.1f}°'


def in_workspace(x, y, z) -> bool:
    return X_MIN <= x <= X_MAX and Y_MIN <= y <= Y_MAX and Z_MIN <= z <= Z_MAX


def make_arrow(frame_id, idx, x, y, z, qx, qy, qz, qw, r, g, b) -> Marker:
    m = Marker()
    m.header.frame_id = frame_id
    m.ns = frame_id
    m.id = idx * 2
    m.type = Marker.ARROW
    m.action = Marker.ADD
    m.pose.position.x = x
    m.pose.position.y = y
    m.pose.position.z = z
    m.pose.orientation.x = qx
    m.pose.orientation.y = qy
    m.pose.orientation.z = qz
    m.pose.orientation.w = qw
    m.scale.x = 0.12  # shaft length
    m.scale.y = 0.02  # shaft diameter
    m.scale.z = 0.03  # head diameter
    m.color.r = r
    m.color.g = g
    m.color.b = b
    m.color.a = 0.9
    m.lifetime.sec = 2
    return m


def make_text(frame_id, idx, x, y, z, text) -> Marker:
    m = Marker()
    m.header.frame_id = frame_id
    m.ns = frame_id + '_label'
    m.id = idx * 2 + 1
    m.type = Marker.TEXT_VIEW_FACING
    m.action = Marker.ADD
    m.pose.position.x = x
    m.pose.position.y = y
    m.pose.position.z = z + 0.07
    m.pose.orientation.w = 1.0
    m.scale.z = 0.04
    m.color.r = 1.0
    m.color.g = 1.0
    m.color.b = 1.0
    m.color.a = 1.0
    m.text = text
    m.lifetime.sec = 2
    return m


def main():
    rclpy.init()
    node = Node('test_tf_gpd')
    log  = node.get_logger()
    tf   = tf_transformer(node)

    pub_before = node.create_publisher(MarkerArray, '/gpd_debug/before', 10)
    pub_after  = node.create_publisher(MarkerArray, '/gpd_debug/after',  10)

    log.info('=' * 60)
    log.info(f'TF GPD Check  {CAMERA_FRAME} → {TARGET_FRAME}')
    log.info('=' * 60)

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
            f'Transform not available after 5s.\n'
            f'Try: CAMERA_FRAME = "camera_l_link" or "zedm_camera_center"'
        )
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    log.info('Transform available.\n')

    # ── Print before/after for every pose ─────────────────────────────────────
    results = []
    for i, (label, x, y, z, qx, qy, qz, qw) in enumerate(GPD_POSES):
        log.info(f'── {label} ──')
        log.info(f'  BEFORE [{CAMERA_FRAME}]')
        log.info(f'    pos = ({x:.3f}, {y:.3f}, {z:.3f})')
        log.info(f'    ori = {rpy_str(qx, qy, qz, qw)}')

        ps = PoseStamped()
        ps.header.frame_id    = CAMERA_FRAME
        ps.header.stamp       = rclpy.time.Time().to_msg()
        ps.pose.position.x    = x
        ps.pose.position.y    = y
        ps.pose.position.z    = z
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw

        result = tf.transform_pose(ps, TARGET_FRAME)

        if result is None:
            log.error(f'  AFTER  [{TARGET_FRAME}] : transform FAILED')
            log.info('')
            results.append((i, label, x, y, z, qx, qy, qz, qw, None))
            continue

        p, o = result.position, result.orientation
        in_ws = in_workspace(p.x, p.y, p.z)

        log.info(f'  AFTER  [{TARGET_FRAME}]')
        log.info(f'    pos = ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})')
        log.info(f'    ori = {rpy_str(o.x, o.y, o.z, o.w)}')
        log.info(f'    {"✓ IN WORKSPACE" if in_ws else "✗ OUT OF WORKSPACE"}')
        log.info('')
        results.append((i, label, x, y, z, qx, qy, qz, qw, (p, o)))

    # ── RViz publish loop ──────────────────────────────────────────────────────
    log.info('Publishing markers to RViz. Ctrl+C to stop.')
    log.info('  RED   arrows = before (camera frame)')
    log.info('  GREEN arrows = after  (base_link)  bright=in workspace  dim=out\n')

    while rclpy.ok():
        now = node.get_clock().now().to_msg()
        before_markers = MarkerArray()
        after_markers  = MarkerArray()

        for i, label, x, y, z, qx, qy, qz, qw, after in results:
            # RED arrow — camera frame (before)
            m = make_arrow(CAMERA_FRAME, i, x, y, z, qx, qy, qz, qw, 1.0, 0.2, 0.2)
            m.header.stamp = now
            t = make_text(CAMERA_FRAME, i, x, y, z,
                          f'{label}\n({x:.2f}, {y:.2f}, {z:.2f})')
            t.header.stamp = now
            before_markers.markers.append(m)
            before_markers.markers.append(t)

            if after is not None:
                p, o = after
                in_ws = in_workspace(p.x, p.y, p.z)
                # Bright green = in workspace, dim green = out
                g_val = 1.0 if in_ws else 0.4
                m2 = make_arrow(TARGET_FRAME, i, p.x, p.y, p.z,
                                o.x, o.y, o.z, o.w, 0.1, g_val, 0.1)
                m2.header.stamp = now
                ws_label = '✓' if in_ws else '✗'
                t2 = make_text(TARGET_FRAME, i, p.x, p.y, p.z,
                               f'{label} {ws_label}\n({p.x:.2f}, {p.y:.2f}, {p.z:.2f})')
                t2.header.stamp = now
                after_markers.markers.append(m2)
                after_markers.markers.append(t2)

        pub_before.publish(before_markers)
        pub_after.publish(after_markers)
        rclpy.spin_once(node, timeout_sec=0.5)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
