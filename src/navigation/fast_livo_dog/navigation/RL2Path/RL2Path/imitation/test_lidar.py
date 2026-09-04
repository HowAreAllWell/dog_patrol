#!/usr/bin/env python3
"""Multi-LiDAR point‑cloud collection script for Isaac Sim.

**Change log (2025‑07‑05)**
- *Bug fix*: moved the final progress print **before** closing the HDF5 file
  (h5py invalidated the dataset ID, causing `RuntimeError: Unable to synchronously get dataspace`).
- Added explicit `flush()` before `close()` to guarantee all buffers reach disk.

Everything else remains identical to the previous version.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path
from typing import List

import numpy as np
import h5py

try:
    import tomllib  # Python 3.11+
except ModuleNotFoundError:  # ≤3.10
    import tomli as tomllib  # type: ignore

from RL2Path.nav.cost_map_2d import *

cfg = CostmapCfg()
cm = Costmap_2d(cfg)

# ────────────────────────────────────────────────────────────────────────────────
# Helper functions for HDF5 incremental writing
# ────────────────────────────────────────────────────────────────────────────────

def _init_h5(file_path: Path):
    """Create/overwrite an HDF5 file and return the expandable datasets."""
    h5f = h5py.File(file_path, "w")
    vlen = h5py.vlen_dtype(np.float32)
    d_points = h5f.create_dataset("points", shape=(0,), maxshape=(None,), dtype=vlen, compression="gzip")
    d_indices = h5f.create_dataset("path_index", shape=(0,), maxshape=(None,), dtype=np.int64, compression="gzip")
    return h5f, d_points, d_indices


def _flush_h5(buf_pts: List[np.ndarray], buf_idx: List[int], d_pts, d_idx):
    if not buf_pts:
        return
    n = len(buf_pts)
    start = d_pts.shape[0]
    d_pts.resize(start + n, axis=0)
    d_idx.resize(start + n, axis=0)
    d_pts[start:] = buf_pts
    d_idx[start:] = buf_idx
    buf_pts.clear()
    buf_idx.clear()

def visualize_costmap_and_path(costmap: np.ndarray,
                               params,
                               lpath: np.ndarray | None = None,
                               title: str | None = None,
                               show_goal: bool = False):
    """
    costmap : (H, W) numpy array, 0~1
    lpath   : (N, 2) local-XY path
    params  : CostmapCfg, 需包含 resolution / size_x / size_y
    """
    # —— 把物理坐标 (m) → 栅格坐标 (pixel) ——————————
    # print("costmap",costmap)
    # print("lpath",lpath)
    costmap = costmap.detach().cpu().numpy() if isinstance(costmap, torch.Tensor) else costmap
    costmap = costmap.squeeze()

    # # ① 先沿 x 轴对称——上下翻转
    # cm_flip_x = costmap[::-1, :]        # 等价于 np.flipud(img)

    # # # ② 再沿 y 轴对称——左右翻转
    # costmap = cm_flip_x[:, ::-1]  # 等价于 np.fliplr(img_flip_x)

    # costmap = 0.5 * (costmap + costmap[::-1, ::-1])

    r  = params.resolution           # m per cell
    H, W = costmap.shape
    cx, cy = W/2.0, H/2.0            # 物理(0,0) 对应像素中心

    # —— 画图 ————————————————————————————————
    plt.figure(figsize=(6, 6))
    plt.imshow(costmap,
               cmap='gray_r',
               origin='lower',
               extent=[-cx*r, (W-cx)*r, -cy*r, (H-cy)*r])   # 坐标轴直接用 [m]
    if lpath is not None:
        plt.plot(lpath[:, 0], lpath[:, 1], 'r-', lw=2, label='local path')
    plt.scatter(0, 0, c='g', s=60, marker='*', label='start')
    if show_goal:
        plt.scatter(lpath[-1, 0], lpath[-1, 1], c='b', s=50, label='goal')
    plt.axis('equal')
    plt.xlabel('x  [m]')
    plt.ylabel('y  [m]')
    if title: plt.title(title)
    plt.legend()
    plt.tight_layout()
    plt.savefig(f"./logs/plt/costmap_path_{title or 'default'}.png")
    plt.close()

# ────────────────────────────────────────────────────────────────────────────────
# Main scanner class
# ────────────────────────────────────────────────────────────────────────────────

class MultiLidarScanner:
    """Drive N RotatingLidarPhysX sensors in parallel and write clouds → HDF5."""

    def __init__(self, cfg: dict, gpu_id: int):
        self.cfg = cfg
        self.gpu_id = gpu_id
        self.world = None
        self.lidars: List["RotatingLidarPhysX"] = []
        self.lidar_iface = None
        self.step = 0
        self.total_paths = 0
        self.buf_pts: List[np.ndarray] = []
        self.buf_idx: List[int] = []
        self.h5f = None
        self.d_points = None
        self.d_indices = None

    # ---------------------------------------------------------------------
    # Scene and sensors
    # ---------------------------------------------------------------------
    def setup(self):
        # Import Isaac Sim only after SimulationApp exists
        from omni.isaac.kit import SimulationApp

        headless = os.environ.get("DISPLAY", "") == ""
        self.sim_app = SimulationApp({"headless": False})

        # Late imports that require the app
        from omni.isaac.core.utils.stage import add_reference_to_stage
        from omni.isaac.core.world import World
        from omni.isaac.sensor import RotatingLidarPhysX
        from isaacsim.sensors.physx import _range_sensor
        from omni.physx.scripts import utils as physx_utils
        import omni.timeline

        self.timeline = omni.timeline.get_timeline_interface()

        # World
        self.world = World(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0, stage_units_in_meters=1.0)
        self.world.scene.add_ground_plane()

        # Environment mesh
        add_reference_to_stage(self.cfg["paths"]["usd_mesh"], "/World/Environment")
        env_prim = self.world.stage.GetPrimAtPath("/World/Environment")
        physx_utils.setCollider(env_prim, approximationShape="convexHull")

        # LiDARs
        lidar_cfg = self.cfg["lidar"]
        count = int(lidar_cfg["count"])
        base_name = lidar_cfg["name"]
        for i in range(count):
            prim_path = f"/World/{base_name}_{i}"
            lidar = RotatingLidarPhysX(
                prim_path=prim_path,
                name=f"{base_name}_{i}",
                translation=lidar_cfg["translation"],
                rotation_frequency=lidar_cfg["rotation_frequency"],
                fov=lidar_cfg["fov"],
                resolution=lidar_cfg["resolution"],
                valid_range=lidar_cfg["valid_range"],
            )
            self.world.scene.add(lidar)
            self.lidars.append(lidar)

        # Paths
        paths_npz = np.load(self.cfg["paths"]["path_dataset"], allow_pickle=True)
        self.paths: np.ndarray = paths_npz["paths_pc"]
        sample_n = int(self.cfg["process"].get("sample_n", -1))
        if sample_n > 0:
            self.paths = self.paths[:sample_n]
        self.total_paths = len(self.paths)

        # HDF5
        # out_path = Path(self.cfg["paths"]["lidar_dataset"]).expanduser()
        # out_path.parent.mkdir(parents=True, exist_ok=True)
        # self.h5f, self.d_points, self.d_indices = _init_h5(out_path)

        # Finalise
        self.world.reset()
        self.lidar_iface = _range_sensor.acquire_lidar_sensor_interface()

    # ---------------------------------------------------------------------
    # Main loop
    # ---------------------------------------------------------------------
    def run(self):
        save_interval = int(self.cfg["collect"]["save_interval"])
        lidar_height = float(self.cfg["collect"]["lidar_height"])
        count = len(self.lidars)

        start_time = time.time()
        while self.sim_app.is_running() and self.step < self.total_paths:
            
            if not self.timeline.is_playing():
                self.sim_app.update()
                continue 
            
            # Move each lidar to its next path sample
            for i, sensor in enumerate(self.lidars):
                p_idx = self.step + i
                if p_idx >= self.total_paths:
                    break
                x, y = self.paths[p_idx][0]
                sensor.set_world_pose(
                    position=np.array([x, y, lidar_height], dtype=np.float32),
                    orientation=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
                )

            # Step world (render=True so sensors update)
            self.sim_app.update()
            self.world.step(render=True)

            # Collect clouds
            for i, sensor in enumerate(self.lidars):
                p_idx = self.step + i
                if p_idx >= self.total_paths:
                    break
                pts = self.lidar_iface.get_point_cloud_data(sensor.prim_path)
                if pts.ndim == 3 and pts.shape[1] > 0:
                    pts_xy = pts[:, 0, :2]
                    self.buf_pts.append(pts_xy.astype(np.float32).reshape(-1))
                else:
                    self.buf_pts.append(np.empty((0,), dtype=np.float32))
                self.buf_idx.append(p_idx)

            print(pts_xy.shape)

            cm.generate(pts_xy[np.newaxis, :])
            cm.inflate()
            visualize_costmap_and_path(cm.inflated_costmaps[0], params=cfg, title=f"step_{self.step}")
            

            self.step += count
            # if len(self.buf_pts) >= save_interval:
            #     _flush_h5(self.buf_pts, self.buf_idx, self.d_points, self.d_indices)

            # if self.step % max(1, self.total_paths // 100) == 0:
            #     pct = self.step / self.total_paths * 100
            #     print(f"Progress: {pct:5.1f}%  ({self.step}/{self.total_paths})")

        # Final flush and summary (bug‑fix – compute before closing file!)
        # _flush_h5(self.buf_pts, self.buf_idx, self.d_points, self.d_indices)
        # n_samples = self.d_points.shape[0]
        # fname = self.h5f.filename
        # self.h5f.flush()
        # self.h5f.close()
        # print(f"Collection complete – {n_samples} samples written to {fname}")

        # Shutdown Isaac Sim
        self.sim_app.close()

# ────────────────────────────────────────────────────────────────────────────────
# CLI
# ────────────────────────────────────────────────────────────────────────────────

def _parse_args():
    p = argparse.ArgumentParser("Multi‑LiDAR collector – Isaac Sim")
    p.add_argument("--config", required=True, help="TOML config file")
    p.add_argument("--gpu", type=int, default=0, help="CUDA device index")
    return p.parse_args()


def main():
    args = _parse_args()
    os.environ["CUDA_VISIBLE_DEVICES"] = str(args.gpu)
    with open(args.config, "rb") as f:
        cfg = tomllib.load(f)
    scanner = MultiLidarScanner(cfg, args.gpu)
    scanner.setup()
    scanner.run()


if __name__ == "__main__":
    sys.exit(main())
