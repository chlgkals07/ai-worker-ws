#!/usr/bin/env python3
"""
실시간 저장된 wrist PCD 데이터 → GPD grasp detection + Open3D 시각화

Usage:
  # 최신 wrist_outputs 폴더 자동 탐지
  python3 test_gpd_wrist_live.py

  # 폴더 직접 지정
  python3 test_gpd_wrist_live.py --data /root/ros2_ws/src/ai_worker/wrist_outputs_20260601_XXXXXX

  # 단일 프레임 지정
  python3 test_gpd_wrist_live.py --frame 3

  # sequential 모드 (N키 = 다음 프레임)
  python3 test_gpd_wrist_live.py --mode sequential

  # GPD 없이 포인트 클라우드만 보기
  python3 test_gpd_wrist_live.py --no-gpd
"""

import argparse
import csv
import glob
import os
import subprocess
import tempfile

import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation

AI_WORKER_DIR = "/root/ros2_ws/src/ai_worker"
GPD_DIR       = os.path.join(AI_WORKER_DIR, "gpd")
GPD_CFG       = "cfg/eigen_params.cfg"


def find_latest_output_dir() -> str:
    """wrist_outputs_* 폴더 중 가장 최신 것을 반환."""
    candidates = sorted(glob.glob(os.path.join(AI_WORKER_DIR, "wrist_outputs_*")))
    if not candidates:
        raise FileNotFoundError(f"wrist_outputs_* 폴더 없음: {AI_WORKER_DIR}")
    latest = candidates[-1]
    # 세션 서브폴더가 있으면 그 안으로 진입
    subdirs = [d for d in glob.glob(os.path.join(latest, "*")) if os.path.isdir(d)]
    if subdirs and not os.path.isdir(os.path.join(latest, "mask_cloud")):
        latest = sorted(subdirs)[-1]
    print(f"[데이터] 자동 탐지: {latest}")
    return latest


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------

def load_pcd(path: str) -> o3d.geometry.PointCloud:
    return o3d.io.read_point_cloud(path)


def load_pose_csv(path: str) -> dict:
    poses = {}
    if not os.path.exists(path):
        return poses
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            idx = int(row['idx'])
            poses[idx] = {
                'position': np.array([float(row['x']), float(row['y']), float(row['z'])]),
                'quat':     np.array([float(row['qx']), float(row['qy']),
                                      float(row['qz']), float(row['qw'])]),
            }
    return poses


def sorted_pcd_files(pcd_dir: str) -> list:
    return sorted(glob.glob(os.path.join(pcd_dir, "frame_*.pcd")))


# ---------------------------------------------------------------------------
# Grasp filters
# ---------------------------------------------------------------------------

def filter_by_approach_x(grasps: list) -> list:
    valid = [g for g in grasps if g.get('approach', np.zeros(3))[0] >= 0]
    print(f"[필터] approach X >= 0: {len(valid)}/{len(grasps)}개 통과")
    return valid


def filter_by_approach(grasps: list, camera_pos: np.ndarray,
                       object_center: np.ndarray, threshold: float = 0.6) -> list:
    cam_dir = object_center - camera_pos
    cam_dir = cam_dir / (np.linalg.norm(cam_dir) + 1e-9)

    valid = []
    for g in grasps:
        if 'approach' not in g:
            continue
        approach = g['approach'] / (np.linalg.norm(g['approach']) + 1e-9)
        cos_sim = float(np.dot(cam_dir, approach))
        g['cos_sim'] = cos_sim
        if cos_sim >= threshold:
            valid.append(g)

    print(f"[필터] approach 유사도 >= {threshold}: {len(valid)}/{len(grasps)}개 통과")
    return valid


def filter_grasps_by_position(grasps: list, object_center: np.ndarray,
                               radius: float = 0.10) -> list:
    valid = [g for g in grasps
             if 'position' in g and
             float(np.linalg.norm(g['position'] - object_center)) <= radius]
    print(f"[필터] grasp position <= {radius}m: {len(valid)}/{len(grasps)}개 통과")
    return valid


# ---------------------------------------------------------------------------
# GPD
# ---------------------------------------------------------------------------

def run_gpd(pcd: o3d.geometry.PointCloud):
    centroid = np.asarray(pcd.points).mean(axis=0)
    camera_pos = centroid + np.array([0.0, -0.3, 0.4])

    tmp_pcd = tempfile.NamedTemporaryFile(suffix='.pcd', delete=False, prefix='gpd_live_')
    tmp_pcd.close()
    tmp_cfg = None

    try:
        o3d.io.write_point_cloud(tmp_pcd.name, pcd)
        tmp_cfg = _make_temp_config(os.path.join(GPD_DIR, GPD_CFG), camera_pos)

        print(f"[GPD] 포인트 수: {len(pcd.points)}, camera_pos={np.round(camera_pos, 3)}")
        print("[GPD] 실행 중...")

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
        print(f"[GPD ERROR] returncode={result.returncode}")
        print(result.stderr[-500:])
        return [], camera_pos

    for line in result.stdout.split('\n'):
        if 'Selected grasps' in line:
            print(line)

    grasps = _parse_gpd_output(result.stdout)
    return grasps, camera_pos


def _make_temp_config(base_cfg: str, view_point: np.ndarray) -> str:
    import re
    with open(base_cfg) as f:
        content = f.read()
    new_pos = f'{view_point[0]:.6f} {view_point[1]:.6f} {view_point[2]:.6f}'
    content = re.sub(r'camera_position\s*=.*', f'camera_position = {new_pos}', content)
    tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.cfg', delete=False, prefix='gpd_cfg_')
    tmp.write(content)
    tmp.close()
    return tmp.name


def _parse_gpd_output(stdout: str) -> list:
    grasps, cur = [], {}

    def _xyz(line, key):
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


# ---------------------------------------------------------------------------
# Visualization
# ---------------------------------------------------------------------------

def grasp_to_quaternion(grasp: dict) -> np.ndarray:
    approach = grasp.get('approach', np.array([1.0, 0.0, 0.0]))
    axis     = grasp.get('axis',     np.array([0.0, 0.0, 1.0]))
    approach = approach / (np.linalg.norm(approach) + 1e-9)
    axis     = axis     / (np.linalg.norm(axis)     + 1e-9)
    binormal = np.cross(axis, approach)
    binormal = binormal / (np.linalg.norm(binormal) + 1e-9)
    axis     = np.cross(approach, binormal)
    R = np.column_stack([approach, binormal, axis])
    return Rotation.from_matrix(R).as_quat()


def make_grasp_frame(grasp: dict, scale=0.04) -> o3d.geometry.TriangleMesh:
    pos      = grasp.get('position', np.zeros(3))
    approach = grasp.get('approach', np.array([1.0, 0.0, 0.0]))
    axis     = grasp.get('axis',     np.array([0.0, 0.0, 1.0]))
    approach = approach / (np.linalg.norm(approach) + 1e-9)
    axis     = axis     / (np.linalg.norm(axis)     + 1e-9)
    binormal = np.cross(axis, approach)
    binormal = binormal / (np.linalg.norm(binormal) + 1e-9)
    axis     = np.cross(approach, binormal)
    R = np.column_stack([approach, binormal, axis])
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=scale)
    frame.rotate(R, center=np.zeros(3))
    frame.translate(pos)
    return frame


def make_gt_sphere(position: np.ndarray, radius=0.015) -> o3d.geometry.TriangleMesh:
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
    sphere.translate(position)
    sphere.paint_uniform_color([1.0, 0.9, 0.0])
    sphere.compute_vertex_normals()
    return sphere


def show_frame(pcd, grasps, gt_pose, title):
    geoms = [pcd]
    for i, g in enumerate(grasps):
        if 'position' not in g:
            continue
        geoms.append(make_grasp_frame(g))
        quat = grasp_to_quaternion(g)
        pos  = g['position']
        print(f"  Grasp {i}: score={g['score']:.3f}")
        print(f"    position:    x={pos[0]:.4f}  y={pos[1]:.4f}  z={pos[2]:.4f}")
        print(f"    orientation: x={quat[0]:.4f}  y={quat[1]:.4f}  z={quat[2]:.4f}  w={quat[3]:.4f}")

    if gt_pose is not None:
        geoms.append(make_gt_sphere(gt_pose['position']))
        print(f"  GT position: {np.round(gt_pose['position'], 3)}")

    print()
    print("  [좌표축] GPD grasp pose (RGB = XYZ)")
    if gt_pose is not None:
        print("  [노란 구] Ground truth target position")
    print("  마우스 드래그: 회전 / 스크롤: 줌 / Q: 종료")

    centroid = np.asarray(pcd.points).mean(axis=0)
    o3d.visualization.draw_geometries(
        geoms,
        window_name=title,
        width=1280, height=720,
        lookat=centroid,
        up=[0.0, 0.0, 1.0],
        front=[0.0, -1.0, 0.3],
        zoom=0.5,
    )


# ---------------------------------------------------------------------------
# Modes
# ---------------------------------------------------------------------------

def run_single(files, poses, frame_idx, use_gpd, folder_name):
    idx  = frame_idx % len(files)
    path = files[idx]
    print(f"\n[프레임 {idx:03d}] {os.path.basename(path)}")

    pcd = load_pcd(path)
    gt  = poses.get(idx)
    pts = np.asarray(pcd.points)
    print(f"  포인트 수: {len(pts)}")

    if len(pts) < 50:
        print("  [경고] 포인트가 너무 적어 GPD 스킵")
        use_gpd = False

    grasps = []
    if use_gpd:
        centroid = pts.mean(axis=0)
        grasps, camera_pos = run_gpd(pcd)
        grasps = filter_by_approach(grasps, camera_pos, centroid)
        grasps = filter_by_approach_x(grasps)
        if gt is not None:
            grasps = filter_grasps_by_position(grasps, gt['position'], radius=0.10)
        grasps = sorted(grasps, key=lambda g: g['score'], reverse=True)[:8]
        print(f"\n[결과] 유효 grasp: {len(grasps)}개 (최대 8개, score 순)")

    title = f"{folder_name} — frame {idx:03d}  |  GPD {len(grasps)} grasps"
    show_frame(pcd, grasps, gt, title)


def run_sequential(files, poses, use_gpd, folder_name):
    print(f"[sequential] {len(files)}프레임  |  N=다음 프레임 / Q=종료")

    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(window_name=f"{folder_name} — Sequential", width=1280, height=720)

    state = {"idx": 0, "geoms": []}

    def _load_geoms(idx):
        pcd  = load_pcd(files[idx])
        gt   = poses.get(idx)
        geoms = [pcd]
        if use_gpd and len(pcd.points) >= 50:
            centroid = np.asarray(pcd.points).mean(axis=0)
            grasps, camera_pos = run_gpd(pcd)
            grasps = filter_by_approach(grasps, camera_pos, centroid)
            grasps = filter_by_approach_x(grasps)
            if gt is not None:
                grasps = filter_grasps_by_position(grasps, gt['position'], radius=0.10)
            grasps = sorted(grasps, key=lambda g: g['score'], reverse=True)[:8]
            print(f"  frame {idx:03d}: {len(grasps)} grasps ({len(pcd.points)} pts)")
            for g in grasps:
                if 'position' in g:
                    geoms.append(make_grasp_frame(g))
        if gt is not None:
            geoms.append(make_gt_sphere(gt['position']))
        return geoms

    def next_frame(vis):
        for g in state["geoms"]:
            vis.remove_geometry(g, reset_bounding_box=False)
        state["idx"] = (state["idx"] + 1) % len(files)
        state["geoms"] = _load_geoms(state["idx"])
        for g in state["geoms"]:
            vis.add_geometry(g, reset_bounding_box=False)
        vis.update_renderer()
        return False

    state["geoms"] = _load_geoms(0)
    for g in state["geoms"]:
        vis.add_geometry(g)

    vis.register_key_callback(ord("N"), next_frame)
    vis.register_key_callback(ord("n"), next_frame)

    opt = vis.get_render_option()
    opt.point_size = 2.5
    opt.background_color = np.array([0.1, 0.1, 0.1])

    vis.run()
    vis.destroy_window()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--data',   type=str, default=None,
                        help='wrist_outputs 폴더 경로 (미지정 시 최신 폴더 자동 탐지)')
    parser.add_argument('--frame',  type=int, default=0)
    parser.add_argument('--mode',   choices=['single', 'sequential'], default='single')
    parser.add_argument('--no-gpd', action='store_true', help='GPD 없이 시각화만')
    args = parser.parse_args()

    base_dir = args.data if args.data else find_latest_output_dir()
    pcd_dir  = os.path.join(base_dir, "mask_cloud")
    pose_csv = os.path.join(base_dir, "pose", "target_pose.csv")
    folder_name = os.path.basename(base_dir)

    files = sorted_pcd_files(pcd_dir)
    if not files:
        print(f"[ERROR] PCD 파일 없음: {pcd_dir}")
        return
    print(f"PCD 파일 {len(files)}개 발견  ({base_dir})")

    poses = load_pose_csv(pose_csv)
    print(f"Ground truth pose {len(poses)}개 로드")

    use_gpd = not args.no_gpd

    if args.mode == 'sequential':
        run_sequential(files, poses, use_gpd, folder_name)
    else:
        run_single(files, poses, args.frame, use_gpd, folder_name)


if __name__ == '__main__':
    main()
