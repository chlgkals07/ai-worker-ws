#!/usr/bin/env python3
"""
gpd_grasp_publisher.py
-----------------------
PointCloud2 토픽을 수신하여 GPD를 실행하고
grasp pose를 PoseStamped / PoseArray 토픽으로 발행.

Subscribe : /wrist_camera/cloud   (sensor_msgs/PointCloud2)
Publish   : /gpd/best_grasp       (geometry_msgs/PoseStamped)  — score 1위
            /gpd/grasps           (geometry_msgs/PoseArray)    — 필터 후 전체

Usage:
  ros2 run ai_worker_manipulation gpd_grasp_publisher
"""

import os
import re
import subprocess
import tempfile
import threading

import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

GPD_DIR = "/root/ros2_ws/src/ai_worker/gpd"
GPD_CFG = "cfg/eigen_params.cfg"

APPROACH_SIM_THRESHOLD = 0.6   # approach-카메라 방향 cosine similarity
POSITION_RADIUS        = 0.10  # grasp position이 물체 중심에서 허용되는 최대 거리 (m)
MAX_GRASPS             = 8     # 발행할 최대 grasp 수


class GpdGraspPublisher(Node):

    def __init__(self):
        super().__init__('gpd_grasp_publisher')

        self._pub_best  = self.create_publisher(PoseStamped, '/gpd/best_grasp',   10)
        self._pub_all   = self.create_publisher(PoseArray,   '/gpd/grasps',       10)
        self._pub_poses = self.create_publisher(PoseArray,   '/gpd/grasp_poses',  10)

        self._sub = self.create_subscription(
            PointCloud2, '/wrist_camera/cloud', self._cloud_cb, 10,
        )

        self._running = False  # GPD 실행 중이면 새 클라우드 무시

        self.get_logger().info('[GpdGraspPublisher] 초기화 완료')
        self.get_logger().info('  subscribe : /wrist_camera/cloud')
        self.get_logger().info('  publish   : /gpd/best_grasp, /gpd/grasps, /gpd/grasp_poses')

    # ── Callback ──────────────────────────────────────────────────────────────

    def _cloud_cb(self, msg: PointCloud2):
        if self._running:
            return
        self._running = True
        threading.Thread(target=self._process, args=(msg,), daemon=True).start()

    # ── Processing pipeline ───────────────────────────────────────────────────

    def _process(self, msg: PointCloud2):
        try:
            pcd = self._to_o3d(msg)

            if len(pcd.points) < 50:
                self.get_logger().warn(f'포인트 수 부족 ({len(pcd.points)}개), GPD 스킵')
                return

            centroid   = np.asarray(pcd.points).mean(axis=0)
            camera_pos = centroid + np.array([0.0, -0.3, 0.4])  # 손목 카메라 근사 위치

            grasps = self._run_gpd(pcd, camera_pos)
            if not grasps:
                return

            grasps = self._filter(grasps, camera_pos, centroid)
            if not grasps:
                self.get_logger().warn('필터 후 유효한 grasp 없음')
                return

            self._publish(grasps, msg.header.stamp)

        finally:
            self._running = False

    # ── PointCloud2 → Open3D ──────────────────────────────────────────────────

    def _to_o3d(self, msg: PointCloud2) -> o3d.geometry.PointCloud:
        pts = np.array([
            [p[0], p[1], p[2]]
            for p in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        ])
        pcd = o3d.geometry.PointCloud()
        if len(pts) > 0:
            pcd.points = o3d.utility.Vector3dVector(pts)
        return pcd

    # ── GPD ───────────────────────────────────────────────────────────────────

    def _run_gpd(self, pcd: o3d.geometry.PointCloud, camera_pos: np.ndarray) -> list[dict]:
        tmp_pcd = tempfile.NamedTemporaryFile(suffix='.pcd', delete=False, prefix='gpd_pub_')
        tmp_pcd.close()
        tmp_cfg = None

        try:
            o3d.io.write_point_cloud(tmp_pcd.name, pcd)
            tmp_cfg = self._make_temp_config(camera_pos)

            self.get_logger().info(
                f'[GPD] 실행 중 - 포인트 {len(pcd.points)}개, '
                f'camera_pos={np.round(camera_pos, 3)}'
            )
            result = subprocess.run(
                ['./build/detect_grasps', tmp_cfg, tmp_pcd.name],
                cwd=GPD_DIR,
                capture_output=True,
                text=True,
                timeout=120,
                env={**os.environ, 'LIBGL_ALWAYS_SOFTWARE': '1'},
            )
        finally:
            os.unlink(tmp_pcd.name)
            if tmp_cfg:
                os.unlink(tmp_cfg)

        if result.returncode != 0:
            self.get_logger().error(f'GPD 실패: returncode={result.returncode}')
            self.get_logger().error(result.stderr[-300:])
            return []

        return self._parse(result.stdout)

    def _make_temp_config(self, view_point: np.ndarray) -> str:
        base_cfg = os.path.join(GPD_DIR, GPD_CFG)
        with open(base_cfg) as f:
            content = f.read()
        new_pos = f'{view_point[0]:.6f} {view_point[1]:.6f} {view_point[2]:.6f}'
        content = re.sub(r'camera_position\s*=.*', f'camera_position = {new_pos}', content)
        tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.cfg', delete=False, prefix='gpd_cfg_')
        tmp.write(content)
        tmp.close()
        return tmp.name

    def _parse(self, stdout: str) -> list[dict]:
        grasps, cur = [], {}

        def _xyz(line, key):
            try:
                rest  = line.split(key + ':')[1]
                parts = rest.strip().replace('x=', '').replace('y=', '').replace('z=', '')
                vals  = [v.strip().rstrip(',') for v in parts.split()]
                return np.array([float(vals[0]), float(vals[1]), float(vals[2])])
            except Exception:
                return np.zeros(3)

        for line in stdout.splitlines():
            if 'Grasp' in line and 'score:' in line:
                if cur:
                    grasps.append(cur)
                cur = {'score': float(line.split('score:')[1].replace(')', '').strip())}
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

    # ── Filtering ─────────────────────────────────────────────────────────────

    def _filter(self, grasps: list[dict],
                camera_pos: np.ndarray,
                centroid: np.ndarray) -> list[dict]:

        # 1) approach-카메라 방향 cosine similarity
        cam_dir = centroid - camera_pos
        cam_dir /= np.linalg.norm(cam_dir) + 1e-9
        grasps = [
            g for g in grasps
            if 'approach' in g and
            float(np.dot(cam_dir,
                         g['approach'] / (np.linalg.norm(g['approach']) + 1e-9)))
            >= APPROACH_SIM_THRESHOLD
        ]

        # 2) approach X >= 0 (로봇 전방에서 진입하는 방향만)
        grasps = [g for g in grasps if g.get('approach', np.zeros(3))[0] >= 0]

        # 3) grasp position이 물체 중심 반경 이내
        grasps = [
            g for g in grasps
            if 'position' in g and
            float(np.linalg.norm(g['position'] - centroid)) <= POSITION_RADIUS
        ]

        # 4) score 내림차순 정렬, 상위 MAX_GRASPS 개
        grasps = sorted(grasps, key=lambda g: g['score'], reverse=True)[:MAX_GRASPS]

        self.get_logger().info(f'필터 후 유효 grasp: {len(grasps)}개')
        return grasps

    # ── Pose conversion ───────────────────────────────────────────────────────

    def _grasp_to_quaternion(self, grasp: dict) -> np.ndarray:
        """approach / binormal / axis → quaternion [x, y, z, w]"""
        approach = grasp.get('approach', np.array([1.0, 0.0, 0.0]))
        binormal = grasp.get('binormal', np.array([0.0, 1.0, 0.0]))
        axis     = grasp.get('axis',     np.array([0.0, 0.0, 1.0]))

        R = np.column_stack([approach, binormal, axis])
        R, _ = np.linalg.qr(R)
        if np.dot(R[:, 0], approach) < 0:
            R[:, 0] *= -1
        if np.linalg.det(R) < 0:
            R[:, 2] *= -1

        return Rotation.from_matrix(R).as_quat()  # [x, y, z, w]

    def _to_pose_msg(self, grasp: dict) -> Pose:
        pos  = grasp['position']
        quat = self._grasp_to_quaternion(grasp)

        p = Pose()
        p.position.x    = float(pos[0])
        p.position.y    = float(pos[1])
        p.position.z    = float(pos[2])
        p.orientation.x = float(quat[0])
        p.orientation.y = float(quat[1])
        p.orientation.z = float(quat[2])
        p.orientation.w = float(quat[3])
        return p

    # ── Publishing ────────────────────────────────────────────────────────────

    def _publish(self, grasps: list[dict], stamp):
        frame_id = 'base_link'

        # PoseArray (전체)
        arr = PoseArray()
        arr.header.stamp    = stamp
        arr.header.frame_id = frame_id
        for g in grasps:
            if 'position' in g:
                arr.poses.append(self._to_pose_msg(g))
        self._pub_all.publish(arr)

        # PoseArray — orchestrator topic alias
        poses_msg = PoseArray()
        poses_msg.header.frame_id = frame_id
        poses_msg.header.stamp    = stamp
        poses_msg.poses = arr.poses
        self._pub_poses.publish(poses_msg)

        # PoseStamped (best)
        best = PoseStamped()
        best.header.stamp    = stamp
        best.header.frame_id = frame_id
        best.pose = arr.poses[0]
        self._pub_best.publish(best)

        pos = grasps[0]['position']
        self.get_logger().info(
            f'[발행] best grasp - score={grasps[0]["score"]:.3f}  '
            f'pos=({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})'
        )


def main():
    rclpy.init()
    node = GpdGraspPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
