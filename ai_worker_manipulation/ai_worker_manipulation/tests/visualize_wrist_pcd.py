#!/usr/bin/env python3
"""
Wrist pointcloud visualizer.
  - 전체 60프레임 합쳐서 보기 (combined)
  - 또는 프레임별로 순서대로 보기 (sequential)
실행: python3 visualize_wrist_pcd.py [--mode combined|sequential]
"""

import argparse
import glob
import os
import numpy as np
import open3d as o3d

PCD_DIR = "/root/ros2_ws/src/ai_worker/wrist_pointcloud_16UC1/pcd"
MM_TO_M = 0.001


def load_pcd_meters(path: str) -> o3d.geometry.PointCloud:
    pcd = o3d.io.read_point_cloud(path)
    pts = np.asarray(pcd.points) * MM_TO_M
    pcd.points = o3d.utility.Vector3dVector(pts)
    return pcd


def colorize_by_height(pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
    pts = np.asarray(pcd.points)
    z = pts[:, 2]
    z_norm = (z - z.min()) / (z.max() - z.min() + 1e-9)
    colors = np.zeros((len(pts), 3))
    # 파란색(낮음) → 초록 → 빨간색(높음)
    colors[:, 0] = z_norm                  # R
    colors[:, 1] = 1.0 - np.abs(z_norm - 0.5) * 2  # G (middle peak)
    colors[:, 2] = 1.0 - z_norm            # B
    pcd.colors = o3d.utility.Vector3dVector(colors)
    return pcd


def show_combined(files: list[str]):
    print(f"[combined] {len(files)}개 프레임 로딩 중...")
    all_pts = []
    for f in files:
        pcd = load_pcd_meters(f)
        all_pts.append(np.asarray(pcd.points))

    merged = o3d.geometry.PointCloud()
    merged.points = o3d.utility.Vector3dVector(np.vstack(all_pts))
    merged, _ = merged.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    merged = merged.voxel_down_sample(voxel_size=0.003)
    merged.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
    colorize_by_height(merged)

    print(f"[combined] 포인트 수: {len(merged.points)}")
    o3d.visualization.draw_geometries(
        [merged],
        window_name="Wrist PointCloud — Combined (all frames)",
        width=1280, height=720,
        point_show_normal=False,
    )


def show_sequential(files: list[str]):
    print(f"[sequential] {len(files)}개 프레임, 키보드: N=다음 / Q=종료")

    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(window_name="Wrist PointCloud — Sequential",
                      width=1280, height=720)

    state = {"idx": 0, "geom": None}

    def next_frame(vis):
        state["idx"] = (state["idx"] + 1) % len(files)
        pcd = load_pcd_meters(files[state["idx"]])
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
        colorize_by_height(pcd)
        state["geom"].points  = pcd.points
        state["geom"].normals = pcd.normals
        state["geom"].colors  = pcd.colors
        vis.update_geometry(state["geom"])
        vis.update_renderer()
        print(f"  frame {state['idx']:03d}: {os.path.basename(files[state['idx']])}")
        return False

    # 첫 프레임 로드
    first = load_pcd_meters(files[0])
    first.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
    colorize_by_height(first)
    state["geom"] = first
    vis.add_geometry(first)

    vis.register_key_callback(ord("N"), next_frame)
    vis.register_key_callback(ord("n"), next_frame)

    opt = vis.get_render_option()
    opt.point_size = 2.0
    opt.background_color = np.array([0.1, 0.1, 0.1])

    print(f"  frame 000: {os.path.basename(files[0])}")
    vis.run()
    vis.destroy_window()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["combined", "sequential"],
                        default="combined")
    args = parser.parse_args()

    files = sorted(glob.glob(os.path.join(PCD_DIR, "*.pcd")))
    if not files:
        print(f"[ERROR] PCD 파일 없음: {PCD_DIR}")
        return
    print(f"파일 {len(files)}개 발견")

    if args.mode == "sequential":
        show_sequential(files)
    else:
        show_combined(files)


if __name__ == "__main__":
    main()
