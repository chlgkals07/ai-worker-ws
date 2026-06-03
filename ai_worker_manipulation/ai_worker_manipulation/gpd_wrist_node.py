#!/usr/bin/env python3
"""
GPD grasp detection node for wrist camera.

perception_2d_to_pcd_wrist 패키지의 wrist_grasp_pcd_node가 발행하는
정제된 PointCloud2를 받아 GPD를 실행하고 grasp poses를 발행합니다.

Subscribe:
  /perception/wrist/target_pcd/<class_name>  (PointCloud2, base_link 기준)

Publish:
  /gpd/grasp_poses  (PoseArray, base_link 기준)
"""

import os
import re
import subprocess
import tempfile

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2


# ---------------------------------------------------------------------------
# Utilities
# ---------------------------------------------------------------------------

def pointcloud2_to_xyz(msg: PointCloud2) -> np.ndarray:
    """PointCloud2 → (N, 3) float64, NaN 제거."""
    pts = list(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))
    if not pts:
        return np.zeros((0, 3), dtype=np.float64)
    return np.array(pts, dtype=np.float64)


def write_pcd(path: str, points: np.ndarray):
    """(N, 3) float32 배열을 binary PCD v0.7 파일로 저장."""
    pts = points.astype(np.float32)
    n = len(pts)
    header = (
        f"# .PCD v0.7 - Point Cloud Data file\n"
        f"VERSION 0.7\n"
        f"FIELDS x y z\n"
        f"SIZE 4 4 4\n"
        f"TYPE F F F\n"
        f"COUNT 1 1 1\n"
        f"WIDTH {n}\n"
        f"HEIGHT 1\n"
        f"VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        f"DATA binary\n"
    )
    with open(path, 'wb') as f:
        f.write(header.encode())
        f.write(pts.tobytes())


def make_temp_config(base_cfg_path: str, view_point: np.ndarray) -> str:
    """GPD cfg를 복사하고 camera_position을 view_point로 교체해 임시 파일 반환."""
    with open(base_cfg_path, 'r') as f:
        content = f.read()
    new_pos = f'{view_point[0]:.6f} {view_point[1]:.6f} {view_point[2]:.6f}'
    content = re.sub(r'camera_position\s*=.*', f'camera_position = {new_pos}', content)
    tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.cfg', delete=False, prefix='gpd_')
    tmp.write(content)
    tmp.close()
    return tmp.name


def parse_gpd_output(stdout: str) -> list[dict]:
    """GPD stdout → grasp dict 리스트 (score, position, approach, binormal, axis)."""
    grasps: list[dict] = []
    cur: dict = {}

    def _xyz(line: str, key: str) -> np.ndarray:
        try:
            rest = line.split(key + ':')[1]
            parts = rest.strip().replace('x=', '').replace('y=', '').replace('z=', '')
            vals = [v.strip().rstrip(',') for v in parts.split()]
            return np.array([float(vals[0]), float(vals[1]), float(vals[2])])
        except Exception:
            return np.zeros(3)

    for line in stdout.splitlines():
        if 'Grasp' in line and 'score:' in line:
            if cur:
                grasps.append(cur)
            score = float(line.split('score:')[1].replace(')', '').strip())
            cur = {'score': score}
        elif cur:
            if 'position:' in line:
                cur['position'] = _xyz(line, 'position')
            elif 'approach:' in line:
                cur['approach'] = _xyz(line, 'approach')
            elif 'binormal:' in line:
                cur['binormal'] = _xyz(line, 'binormal')
            elif 'axis:' in line:
                cur['axis'] = _xyz(line, 'axis')

    if cur:
        grasps.append(cur)
    return grasps


def grasp_to_pose(grasp: dict) -> Pose:
    """grasp dict → geometry_msgs/Pose."""
    pose = Pose()

    pos = grasp.get('position', np.zeros(3))
    pose.position.x = float(pos[0])
    pose.position.y = float(pos[1])
    pose.position.z = float(pos[2])

    approach = grasp.get('approach', np.array([1.0, 0.0, 0.0]))
    binormal = grasp.get('binormal', np.array([0.0, 1.0, 0.0]))
    axis     = grasp.get('axis',     np.array([0.0, 0.0, 1.0]))

    R = np.column_stack([approach, binormal, axis])
    R, _ = np.linalg.qr(R)
    if np.linalg.det(R) < 0:
        R[:, 2] *= -1

    quat = Rotation.from_matrix(R).as_quat()  # [x, y, z, w]
    pose.orientation.x = float(quat[0])
    pose.orientation.y = float(quat[1])
    pose.orientation.z = float(quat[2])
    pose.orientation.w = float(quat[3])
    return pose


# ---------------------------------------------------------------------------
# ROS2 Node
# ---------------------------------------------------------------------------

class GpdWristNode(Node):
    def __init__(self):
        super().__init__('gpd_wrist_node')

        self.declare_parameter('class_name',  'target')
        self.declare_parameter('gpd_dir',     '/root/ros2_ws/src/ai_worker/gpd')
        self.declare_parameter('gpd_config',  'cfg/eigen_params.cfg')
        self.declare_parameter('camera_frame', 'camera_right_color_optical_frame')
        self.declare_parameter('base_frame',  'base_link')
        self.declare_parameter('gpd_timeout', 60.0)

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        class_name = self.get_parameter('class_name').value
        topic = f'/perception/wrist/target_pcd/{class_name}'

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(PointCloud2, topic, self._on_cloud, qos)

        self.grasp_pub = self.create_publisher(PoseArray, '/gpd/grasp_poses', 10)

        self.get_logger().info(f'Subscribing: {topic}')
        self.get_logger().info('GPD wrist node ready.')

    # ------------------------------------------------------------------
    def _get_camera_position(self) -> np.ndarray:
        """카메라 프레임의 base_link 기준 위치를 TF에서 조회."""
        camera_frame = self.get_parameter('camera_frame').value
        base_frame   = self.get_parameter('base_frame').value
        try:
            t = self.tf_buffer.lookup_transform(
                base_frame, camera_frame, rclpy.time.Time())
            tr = t.transform.translation
            return np.array([tr.x, tr.y, tr.z])
        except tf2_ros.TransformException as e:
            self.get_logger().warn(
                f'TF lookup failed ({camera_frame} → {base_frame}): {e}\n'
                f'  camera_position = [0, 0, 0] 로 fallback')
            return np.zeros(3)

    # ------------------------------------------------------------------
    def _on_cloud(self, msg: PointCloud2):
        pts = pointcloud2_to_xyz(msg)
        self.get_logger().info(f'Cloud received: {len(pts)} pts')

        if len(pts) == 0:
            self.get_logger().warn('Empty cloud, skipping.')
            return

        camera_pos = self._get_camera_position()
        grasps = self._run_gpd(pts, camera_pos)

        self.get_logger().info(f'GPD detected {len(grasps)} grasps.')
        for i, g in enumerate(grasps):
            pos = g.get('position', np.zeros(3))
            self.get_logger().info(
                f'  [{i}] score={g["score"]:.3f}  '
                f'pos=({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})')

        self._publish(grasps)

    # ------------------------------------------------------------------
    def _run_gpd(self, points: np.ndarray, camera_pos: np.ndarray) -> list[dict]:
        gpd_dir    = self.get_parameter('gpd_dir').value
        config_rel = self.get_parameter('gpd_config').value
        config_abs = os.path.join(gpd_dir, config_rel)
        timeout    = self.get_parameter('gpd_timeout').value

        tmp_pcd = tempfile.NamedTemporaryFile(suffix='.pcd', delete=False, prefix='gpd_in_')
        tmp_pcd.close()
        tmp_cfg = None

        try:
            write_pcd(tmp_pcd.name, points)
            tmp_cfg = make_temp_config(config_abs, camera_pos)

            result = subprocess.run(
                ['./build/detect_grasps', tmp_cfg, tmp_pcd.name],
                cwd=gpd_dir,
                capture_output=True,
                text=True,
                timeout=timeout,
                env={**os.environ, 'LIBGL_ALWAYS_SOFTWARE': '1'},
            )

            if result.returncode != 0:
                self.get_logger().error(f'GPD exited with code {result.returncode}')
                self.get_logger().error(result.stderr[-500:])
                return []

            return parse_gpd_output(result.stdout)

        except subprocess.TimeoutExpired:
            self.get_logger().error(f'GPD timed out after {timeout}s.')
            return []
        except FileNotFoundError:
            self.get_logger().error(f'GPD binary not found: {gpd_dir}/build/detect_grasps')
            return []
        finally:
            os.unlink(tmp_pcd.name)
            if tmp_cfg:
                os.unlink(tmp_cfg)

    # ------------------------------------------------------------------
    def _publish(self, grasps: list[dict]):
        msg = PoseArray()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('base_frame').value
        msg.poses = [grasp_to_pose(g) for g in grasps if 'position' in g]
        self.grasp_pub.publish(msg)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    rclpy.init()
    node = GpdWristNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
