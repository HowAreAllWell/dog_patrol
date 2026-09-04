# -*- coding: utf-8 -*-
"""
Integrated 2 D / 3 D dataset pre‑processing script (updated)
-----------------------------------------------------------

**Change log (2025‑07‑03)**
- *Removed* the additional rotation that forced the goal to be in front of the robot.  
  We now **only translate** the global path into the local frame (origin at the first
  path point) and then perform normalisation.
- `local_transform_2d` / `local_transform_3d` signatures simplified: **yaw argument
  removed**.
- Call‑sites in `process_2d` / `process_3d` updated accordingly.
- Minor doc‑string updates and in‑code comments to reflect the new behaviour.

The rest of the pipeline (cost‑map generation, in‑bounds checks, action/key‑point
extraction, I/O) is unchanged.
"""

import argparse
from pathlib import Path
from isaaclab.app import AppLauncher

ap = argparse.ArgumentParser("Unified 2D/3D dataset pre‑processor (updated)")
ap.add_argument("--config", type=str, default="source/RL2Path/config/imitation.toml", help="config file path")

import h5py
import torch
import tomli
import pickle
from typing import List, Tuple
import matplotlib.pyplot as plt

import numpy as np
from tqdm import tqdm

# ─── Cost‑map back‑end ──────────────────────────────────────────────────────
# Both 2 D & 3 D use the same API from RL2Path (2 D simply has z‑min == z‑max)
from RL2Path.nav.cost_map_2d import Costmap_2d, CostmapCfg
from RL2Path.nav.grid_2d import Grid

# ─────────────────────────────────────────────────────────────────────────────
# Utility functions – agnostic to 2 D / 3 D wherever possible
# ─────────────────────────────────────────────────────────────────────────────

def extract_key_points_combined(path: np.ndarray,
                                num_key_points: int,
                                threshold_distance: float) -> np.ndarray:
    """Return *num_key_points + 1* key‑points **including** start & goal."""
    if path.shape[0] == 0:
        return path.copy()

    distances = np.linalg.norm(path - path[0], axis=1)
    total_distance = distances[-1]
    key_points = [path[0]]

    if total_distance >= threshold_distance:  # distance‑based sampling
        thresholds = np.linspace(threshold_distance / num_key_points,
                                 total_distance, num_key_points)
        ptr = 0
        for th in thresholds:
            while ptr < len(distances) and distances[ptr] < th:
                ptr += 1
            key_points.append(path[min(ptr, len(path) - 1)])
    else:                                      # uniform along arclength
        if len(path) < 2:
            return path.copy()
        seg_len = np.linalg.norm(np.diff(path, axis=0), axis=1)
        cum_len = np.insert(np.cumsum(seg_len), 0, 0.0)
        targets = np.linspace(0.0, cum_len[-1], num_key_points + 1)[1:]
        ptr = 0
        for t in targets:
            while ptr < len(seg_len) and cum_len[ptr + 1] < t:
                ptr += 1
            if ptr >= len(seg_len):
                key_points.append(path[-1])
            else:
                ratio = (t - cum_len[ptr]) / seg_len[ptr]
                key = path[ptr] + ratio * (path[ptr + 1] - path[ptr])
                key_points.append(key)

    key_points[-1] = path[-1]  # make sure goal is exact
    return np.asarray(key_points)


# ─── 2 D specific helpers ───────────────────────────────────────────────────

def to_polar(pt: np.ndarray) -> Tuple[float, float]:
    r = np.hypot(pt[0], pt[1])
    theta = np.arctan2(pt[1], pt[0])
    return r, theta


def normalize_polar(pt: Tuple[float, float], max_r: float, theta_range: Tuple[float, float]) -> np.ndarray:
    r, th = pt
    return np.array([r / max_r, (th - theta_range[0]) / (theta_range[1] - theta_range[0])])


def normalize_action_2d(action: np.ndarray, max_r: float, theta_range: Tuple[float, float]) -> np.ndarray:
    n = action.shape[0]
    r_min = np.zeros(n)
    r_max = np.linspace(0.0, max_r, n + 1)[1:]
    th_min, th_max = theta_range
    r_norm = (action[:, 0] - r_min) / (r_max - r_min)
    th_norm = (action[:, 1] - th_min) / (th_max - th_min)
    return np.stack([r_norm, th_norm], axis=-1)


def obs_2d(costmap: np.ndarray, goal_local: Tuple[float, float],
           lethal: float, max_r: float, theta_range) -> np.ndarray:
    return np.concatenate([
        costmap.flatten() / lethal,
        normalize_polar(goal_local, max_r, theta_range)
    ])

# **UPDATED** – translation‑only local frame (no rotation)

def local_transform_2d(path: np.ndarray) -> np.ndarray:
    origin = path[0][:2]  # 初始位置
    delta = path[:, :2] - origin  # 平移使得起点为原点
    return delta  

def in_bounds_2d(local_path: np.ndarray, params: CostmapCfg) -> bool:
    x, y = local_path[:, 0], local_path[:, 1]
    return (x >= params.x_min).all() and (x <= params.x_max).all() and \
           (y >= params.y_min).all() and (y <= params.y_max).all()

# ─── 3 D specific helpers ───────────────────────────────────────────────────

def to_spherical(pt: np.ndarray) -> Tuple[float, float, float]:
    r = np.linalg.norm(pt)
    theta = np.arctan2(pt[1], pt[0])
    phi = np.arctan2(pt[2], np.hypot(pt[0], pt[1]))
    return r, theta, phi


def normalize_spherical(pt: Tuple[float, float, float], max_r: float) -> np.ndarray:
    r, th, ph = pt
    return np.array([r / max_r,
                     (th + np.pi) / (2 * np.pi),
                     (ph + np.pi / 2) / np.pi])


def normalize_action_3d(action: np.ndarray, max_r: float) -> np.ndarray:
    n = action.shape[0]
    r_max = np.linspace(0.0, max_r, n + 1)[1:]
    r_norm = (action[:, 0] - 0.0) / (r_max - 0.0)
    th_norm = (action[:, 1] + np.pi) / (2 * np.pi)
    ph_norm = (action[:, 2] + np.pi / 2) / np.pi
    return np.stack([r_norm, th_norm, ph_norm], axis=-1)


def obs_3d(costmap: np.ndarray, goal_local: Tuple[float, float, float],
           lethal: float, max_r: float) -> np.ndarray:
    return np.concatenate([
        costmap.flatten() / lethal,
        normalize_spherical(goal_local, max_r)
    ])


# **UPDATED** – translation‑only local frame (no rotation)

def local_transform_3d(path: np.ndarray) -> np.ndarray:
    """Translate the global 3‑D path so that the first point is the origin."""
    origin = path[0][:3]
    return path[:, :3] - origin


def in_bounds_3d(local_path: np.ndarray, params: CostmapCfg) -> bool:
    x, y, z = local_path[:, 0], local_path[:, 1], local_path[:, 2]
    return (x >= params.x_min).all() and (x <= params.x_max).all() and \
           (y >= params.y_min).all() and (y <= params.y_max).all() and \
           (z >= params.z_min).all() and (z <= params.z_max).all()

# ─────────────────────────────────────────────────────────────────────────────
# Main processing pipelines
# ─────────────────────────────────────────────────────────────────────────────

def visualize_costmap_and_path(costmap: np.ndarray,
                               lpath:   np.ndarray,
                               params:  CostmapCfg,
                               title:   str | None = None,
                               show_goal: bool = True):
    """
    costmap : (H, W) uint8   —— 已经是 [0, x_max] × [y_min, y_max] 的矩形网格
    lpath   : (N, 2) float32 —— 已经在同一个局部坐标系（米）
    params  : CostmapCfg     —— 只用到 x_max / y_min / y_max / resolution
    """
    costmap = costmap.squeeze()                 # 去掉 (1, H, W) 之类多余维度

    # —— 世界坐标范围（单位：米） ————————————————
    x_min, x_max = params.x_min, params.x_max   # 现为 0.0, 2.2
    y_min, y_max = params.y_min, params.y_max   # 仍为 -1.1, 1.1

    plt.figure(figsize=(6, 6))
    plt.imshow(costmap,
               cmap='gray_r',
               origin='lower',                  # 行 0 放在 y_min（图像下边）
               extent=[x_min, x_max, y_min, y_max])

    # —— 叠加路径 / 起终点 ————————————————
    plt.plot(lpath[:, 0], lpath[:, 1], 'r-', lw=2, label='local path')
    plt.scatter(0.0, 0.0, c='g', s=60, marker='*', label='start')
    if show_goal:
        plt.scatter(lpath[-1, 0], lpath[-1, 1], c='b', s=50, label='goal')

    plt.axis('equal')
    plt.xlabel('x [m]  (forward)')
    plt.ylabel('y [m]  (left ↔ right)')
    if title:
        plt.title(title)
    plt.legend()
    plt.tight_layout()
    plt.savefig(f"./logs/plt/costmap_path_{title or 'default'}.png")


def rotate_xy(arr: np.ndarray, theta: float) -> np.ndarray:
    """
    将数组 arr 的前两列 (x,y) 绕原点旋转 theta，
    其他列（z、强度等）保持不变。
    """
    c, s = np.cos(theta), np.sin(theta)
    rot = np.array([[c, -s],
                    [s,  c]], dtype=arr.dtype)
    arr_xy = arr[..., :2]          # (..., 2)
    arr[..., :2] = arr_xy @ rot.T  # 就地修改，保持 shape
    return arr

def process_2d(
    lidar_dataset: List[np.ndarray],          # (B, N, {2|3})
    path_dataset:  List[np.ndarray],
    idx:            np.ndarray | List[int],   # 与 lidar_dataset 一一对应
    action_dim:     int,
    params:         CostmapCfg,
    max_dist:       float,
    theta_range: Tuple[float, float] = (-np.pi/3, np.pi/3),
    action_theta_range: Tuple[float, float] = (-np.pi/2, np.pi/2)
) -> Tuple[List[np.ndarray], List[np.ndarray]]:
    """
    处理 2‑D 数据：生成 costmap → 观测向量 → 动作向量。
    关键更新：
      1. 先把每条路径平移到局部坐标系；
      2. 计算旋角 θ = (-θ_goal) + 𝒰(-π/3, π/3)，使终点≈正前方并加扰动；
      3. 对路径和对应 LiDAR 点云同时旋转 θ；
      4. 之后所有步骤（in‑bounds / inflate / 归一化）保持不变。
    """
    cm = Costmap_2d(params=params)

    # ————————————————————————
    # ① 同步旋转 LiDAR 点云，使其与后续局部路径对齐
    # ————————————————————————
    thetas, lidar_rot, valid_idx = [], [], []

    for i, pc in enumerate(lidar_dataset):
        path_idx  = int(idx[i]) if idx is not None else i
        gpath     = np.asarray(path_dataset[path_idx])
        lpath0    = local_transform_2d(gpath)          # 仅平移

        if lpath0.shape[0] < 2:                        # 极端异常，跳过
            continue

        θ_goal  = np.arctan2(lpath0[-1, 1], lpath0[-1, 0])   # 终点方位角
        θ_align = -θ_goal                                     # 对齐到 +x
        θ_noise = np.random.uniform(theta_range[0], theta_range[1])        # ±60° 扰动
        θ       = θ_align + θ_noise

        thetas.append(θ)
        lidar_rot.append(rotate_xy(pc.copy(), θ))      # 点云旋转
        valid_idx.append(path_idx)

    if not lidar_rot:                                  # 所有样本都被过滤
        return [], []

    lidar_tensor = torch.from_numpy(
        np.stack(lidar_rot, axis=0)
    ).float().to(cm.device)

    # ————————————————————————
    # ② 生成 & 膨胀 cost‑map
    # ————————————————————————
    cm.generate(lidar_tensor)
    inflated_list = cm.inflate()

    # ————————————————————————
    # ③ 逐样本提取观测 / 动作
    # ————————————————————————
    obss, acts = [], []

    for i, path_idx in enumerate(valid_idx):
        gpath = np.asarray(path_dataset[path_idx])

        #   · 平移 → 旋转（同 LiDAR）
        lpath = local_transform_2d(gpath)
        lpath = rotate_xy(lpath.copy(), thetas[i])

        if not in_bounds_2d(lpath, params):            # 超出前半平面
            continue

        inflated = inflated_list[i].cpu().numpy()

        #   · 观测向量
        goal_local = to_polar(lpath[-1])
        obs = obs_2d(inflated, goal_local,
                     params.lethal_obstacle, max_dist, action_theta_range)
        obss.append(obs)

        #   · 动作向量（关键点 → 极坐标归一化）
        kp_mid    = lpath[1:-1]                         # 去掉起/终点
        polar_mid = np.array([to_polar(p) for p in kp_mid])
        act       = normalize_action_2d(polar_mid, max_dist, action_theta_range)
        acts.append(act)
        
        # visualize_costmap_and_path(inflated, lpath, params, len(obss))

    return obss, acts


def process_3d(lidar_dataset: List[Tuple[np.ndarray, np.ndarray, float]],
               path_dataset: List[np.ndarray],
               action_dim: int,
               params: CostmapCfg,
               max_dist: float) -> Tuple[List[np.ndarray], List[np.ndarray]]:
    costmaps, obss, acts = [], [], []
    cm = Costmap_2d(params=params)

    for (pose, cloud, _), gpath in tqdm(zip(lidar_dataset, path_dataset),
                                         total=len(path_dataset),
                                         desc="3D samples"):
        pts = np.asarray(cloud)
        ranges = np.linalg.norm(pts, axis=1)
        cm.generate(pts, ranges)
        inflated = cm.inflate()
        if torch.is_tensor(inflated):          
            inflated = inflated.detach().cpu().numpy()  
        costmaps.append(inflated)

        lpath = local_transform_3d(gpath)
        if not in_bounds_3d(lpath, params):
            continue

        goal_local = to_spherical(lpath[-1])
        obs = obs_3d(inflated, goal_local, params.lethal_obstacle, max_dist)
        obss.append(obs)

        kp = extract_key_points_combined(lpath, action_dim + 1, max_dist - 0.05)
        kp_mid = kp[1:]
        sph_mid = np.array([to_spherical(p) for p in kp_mid])
        act = normalize_action_3d(sph_mid, max_dist)
        acts.append(act)

    return obss, acts

# ─────────────────────────────────────────────────────────────────────────────
# CLI entry-point
# ─────────────────────────────────────────────────────────────────────────────
def main():
    # ── Load configuration ─────────────────────────────
    args = ap.parse_args()
    config_path = Path(args.config)
    with open(config_path, "rb") as f:
        config = tomli.load(f)

    paths_cfg   = config["paths"]
    process_cfg = config["process"]
    sample_n    = process_cfg.get("sample_n", -1)
    sample_n    = sample_n if sample_n > 0 else None
    costmap_params = config["costmap"]
    imitation_params = config.get("imitation", {})

    lidar_dataset_path = Path(paths_cfg["lidar_dataset"])
    path_dataset_path  = Path(paths_cfg["path_dataset"])
    output_path        = Path(paths_cfg["output"])

    print(f"[INFO] Loading LiDAR dataset from: {lidar_dataset_path}")

    # ── Load LiDAR dataset ─────────────────────────────
    if lidar_dataset_path.suffix == ".npz":
        with np.load(lidar_dataset_path, allow_pickle=True, mmap_mode="r") as data:
            arr = data["paths"]
            lidar_ds = arr[:].tolist()
        print(f"[INFO] Loaded {len(lidar_ds)} LiDAR samples from .npz file")

    elif lidar_dataset_path.suffix == ".h5":
        with h5py.File(lidar_dataset_path, "r") as f:
            pts_flat = f["points"][:]
            idx      = f["path_index"][:]
        pts_flat = np.stack(pts_flat).reshape(len(pts_flat), -1, 2)
        print(f"[INFO] Loaded {pts_flat.shape[0]} LiDAR samples from .h5 file")

    else:  # assume .pkl
        with open(lidar_dataset_path, "rb") as f:
            all_lidar = pickle.load(f)
        lidar_ds = all_lidar[:]
        if len(all_lidar) > len(lidar_ds):
            print("[WARN] .pkl memory peak cannot be saved, but processing time reduced")
        print(f"[INFO] Loaded {len(lidar_ds)} LiDAR samples from .pkl file")

    # ── Load Path dataset ──────────────────────────────
    print(f"[INFO] Loading path dataset from: {path_dataset_path}")
    if path_dataset_path.suffix == ".npz":
        paths = np.load(path_dataset_path, allow_pickle=True)["paths_key"].tolist()
    elif path_dataset_path.suffix == ".h5":
        with h5py.File(path_dataset_path, "r") as f:
            paths = f["paths"][:].tolist()
    else:
        with open(path_dataset_path, "rb") as f:
            paths = pickle.load(f)
    print(f"[INFO] Loaded {len(paths)} paths")

    # ── Validate data ─────────────────────────────────
    # if lidar_dataset_path.suffix == ".h5":
    #     assert pts_flat.shape[0] == len(paths), f"[ERROR] Mismatch: {pts_flat.shape[0]} vs {len(paths)}"
    #     print(f"[INFO] Dataset sizes match: {pts_flat.shape[0]} samples")
    # else:
    #     assert len(lidar_ds) == len(paths), f"[ERROR] Mismatch: {len(lidar_ds)} vs {len(paths)}"
    #     print(f"[INFO] Dataset sizes match: {len(lidar_ds)} samples")

    # ── Process ────────────────────────────────────────
    mode       = process_cfg.get("mode", "2d")
    action_dim = process_cfg.get("action_dim", 12)
    max_d      = process_cfg.get("max_distance", 1.0)  # 预定义常量
    params     = CostmapCfg(
            x_min=costmap_params["x_min"],
            x_max=costmap_params["x_max"],
            y_min=costmap_params["y_min"],
            y_max=costmap_params["y_max"],
        )         
    theta_range = costmap_params.get("theta_range", (-np.pi/3, np.pi/3))
    theta_range[0] = np.deg2rad(theta_range[0])  # 转换为弧度
    theta_range[1] = np.deg2rad(theta_range[1])  # 转换为弧度
    print(f"[INFO] observation theta range: {theta_range}")
    
    action_theta_range = imitation_params.get("action_theta_range", (-np.pi/2, np.pi/2))
    action_theta_range[0] = np.deg2rad(action_theta_range[0])  # 转换为弧度
    action_theta_range[1] = np.deg2rad(action_theta_range[1])  # 转换为弧度
    print(f"[INFO] action theta range: {action_theta_range}")

    print(f"[INFO] Starting processing in {mode.upper()} mode...")

    if mode == "2d":
        obss, acts = process_2d(pts_flat, paths, idx,
                                action_dim=action_dim // 2,
                                params=params, max_dist=max_d, theta_range=theta_range, action_theta_range=action_theta_range)
    else:
        obss, acts = process_3d(lidar_ds, paths,
                                action_dim=action_dim // 3,
                                params=params, max_dist=max_d)

    print(f"[INFO] Processed {len(obss)} valid samples")

    # ── Save ──────────────────────────────────────────
    
    print(f"[INFO] Saving processed transitions to: {output_path}")
    if output_path.suffix == ".h5":
        with h5py.File(output_path, "w") as f:
            f.create_dataset("observations", data=np.array(obss, dtype=np.float32), compression="gzip")
            f.create_dataset("actions", data=np.array(acts, dtype=np.float32), compression="gzip")
    else:
        with open(output_path, "wb") as f:
            pickle.dump([obss, acts], f)
    print(f"[SUCCESS] Saved {len(obss)} transitions to {output_path}")

if __name__ == "__main__":
    main()