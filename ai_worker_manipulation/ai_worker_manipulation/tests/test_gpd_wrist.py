#!/usr/bin/env python3
"""
Wrist camera PCD → GPD grasp detection + Open3D visualization.

Usage:
  # 단일 프레임
  python3 test_gpd_wrist.py --frame 0

  # 전체 프레임 합산 (더 촘촘한 포인트)
  python3 test_gpd_wrist.py --combine
"""

import argparse
import glob
import os
import subprocess
import tempfile

import numpy as np
import open3d as o3d

PCD_DIR = "/root/ros2_ws/src/ai_worker/wrist_pointcloud_16UC1/pcd"
GPD_DIR = "/root/ros2_ws/src/ai_worker/gpd"
GPD_CFG = "cfg/eigen_params.cfg"
MM_TO_M = 0.001

# depth optical frame: 카메라가 원점에서 +Z를 향함
CAMERA_POSITION = np.array([0.0, 0.0, 0.0])


# ---------------------------------------------------------------------------
# Point cloud helpers
# ---------------------------------------------------------------------------

def load_pcd_meters(path: str) -> o3d.geometry.PointCloud:
    pcd = o3d.io.read_point_cloud(path)
    pts = np.asarray(pcd.points) * MM_TO_M
    pcd.points = o3d.utility.Vector3dVector(pts)
    return pcd


def prepare_pcd(files: list[str]) -> o3d.geometry.PointCloud:
    all_pts = []
    for f in files:
        pcd = load_pcd_meters(f)
        all_pts.append(np.asarray(pcd.points))

    merged = o3d.geometry.PointCloud()
    merged.points = o3d.utility.Vector3dVector(np.vstack(all_pts))
    merged, _ = merged.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    merged = merged.voxel_down_sample(voxel_size=0.003)
    return merged


# ---------------------------------------------------------------------------
# GPD
# ---------------------------------------------------------------------------

def run_gpd(pcd: o3d.geometry.PointCloud) -> list[dict]:
    tmp = tempfile.NamedTemporaryFile(suffix=".pcd", delete=False, prefix="gpd_wrist_")
    tmp.close()
    o3d.io.write_point_cloud(tmp.name, pcd)

    print(f"\n[GPD] 포인트 수: {len(pcd.points)}")
    print("[GPD] 실행 중...")

    try:
        result = subprocess.run(
            ["./build/detect_grasps", GPD_CFG, tmp.name],
            cwd=GPD_DIR,
            capture_output=True,
            text=True,
            timeout=120,
            env={**os.environ, "LIBGL_ALWAYS_SOFTWARE": "1"},
        )
    finally:
        os.unlink(tmp.name)

    if result.returncode != 0:
        print(f"[GPD ERROR] returncode={result.returncode}")
        print(result.stderr[-800:])
        return []

    # stdout 에서 selected grasps 이후만 출력
    printing = False
    for line in result.stdout.split("\n"):
        if "Selected grasps" in line:
            printing = True
        if printing:
            print(line)

    return parse_gpd_output(result.stdout)


def parse_gpd_output(stdout: str) -> list[dict]:
    grasps = []
    cur = {}

    def _xyz(line: str, key: str) -> np.ndarray:
        # "position: x=0.123, y=0.456, z=0.789" 형식
        try:
            rest = line.split(key + ":")[1]
            parts = rest.strip().replace("x=", "").replace("y=", "").replace("z=", "")
            vals = [v.strip().rstrip(",") for v in parts.split()]
            return np.array([float(vals[0]), float(vals[1]), float(vals[2])])
        except Exception:
            return np.zeros(3)

    for line in stdout.splitlines():
        if "Grasp" in line and "score:" in line:
            if cur:
                grasps.append(cur)
            score = float(line.split("score:")[1].replace(")", "").strip())
            cur = {"score": score}
        elif cur:
            if "position:" in line:
                cur["position"] = _xyz(line, "position")
            elif "approach:" in line:
                cur["approach"] = _xyz(line, "approach")
            elif "binormal:" in line:
                cur["binormal"] = _xyz(line, "binormal")
            elif "axis:" in line:
                cur["axis"] = _xyz(line, "axis")

    if cur:
        grasps.append(cur)
    return grasps


# ---------------------------------------------------------------------------
# Visualization
# ---------------------------------------------------------------------------

def colorize_by_depth(pcd: o3d.geometry.PointCloud):
    pts = np.asarray(pcd.points)
    z = pts[:, 2]
    z_norm = (z - z.min()) / (z.max() - z.min() + 1e-9)
    colors = np.stack([z_norm, 1.0 - np.abs(z_norm - 0.5) * 2, 1.0 - z_norm], axis=1)
    pcd.colors = o3d.utility.Vector3dVector(colors)


def make_grasp_frame(grasp: dict, scale: float = 0.05) -> o3d.geometry.TriangleMesh:
    pos      = grasp.get("position", np.zeros(3))
    approach = grasp.get("approach", np.array([1, 0, 0]))
    binormal = grasp.get("binormal", np.array([0, 1, 0]))
    axis     = grasp.get("axis",     np.array([0, 0, 1]))

    R = np.column_stack([approach, binormal, axis])
    R, _ = np.linalg.qr(R)
    if np.linalg.det(R) < 0:
        R[:, 2] *= -1

    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=scale)
    frame.rotate(R, center=np.zeros(3))
    frame.translate(pos)
    return frame


def visualize(pcd: o3d.geometry.PointCloud, grasps: list[dict], title: str):
    colorize_by_depth(pcd)
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))

    geoms = [pcd]

    for i, g in enumerate(grasps):
        if "position" not in g:
            continue
        frame = make_grasp_frame(g, scale=0.04)
        geoms.append(frame)
        print(f"  Grasp {i}: score={g['score']:.3f}  pos={np.round(g['position'], 3)}")

    print(f"\n[시각화] grasp {len(grasps)}개 표시 중...")
    print("  마우스 드래그: 회전 / 스크롤: 줌 / Q: 종료")

    o3d.visualization.draw_geometries(
        geoms,
        window_name=title,
        width=1280, height=720,
    )


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--frame", type=int, default=0,
                        help="단일 프레임 인덱스 (0~59)")
    parser.add_argument("--combine", action="store_true",
                        help="전체 프레임 합산 사용")
    args = parser.parse_args()

    files = sorted(glob.glob(os.path.join(PCD_DIR, "*.pcd")))
    if not files:
        print(f"[ERROR] PCD 파일 없음: {PCD_DIR}")
        return

    if args.combine:
        print(f"[모드] 전체 {len(files)}프레임 합산")
        target_files = files
        title = f"Wrist PCD (combined {len(files)} frames)"
    else:
        idx = args.frame % len(files)
        target_files = [files[idx]]
        title = f"Wrist PCD frame {idx:03d}: {os.path.basename(files[idx])}"
        print(f"[모드] 단일 프레임: {os.path.basename(files[idx])}")

    pcd = prepare_pcd(target_files)
    print(f"[PCD] 전처리 완료, 포인트 수: {len(pcd.points)}")

    grasps = run_gpd(pcd)
    print(f"\n[결과] GPD detected {len(grasps)} grasps")

    if not grasps:
        print("[경고] grasp가 없어 포인트 클라우드만 표시합니다.")

    visualize(pcd, grasps, title)


if __name__ == "__main__":
    main()
