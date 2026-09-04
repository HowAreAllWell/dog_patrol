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

ACTION_MAX_DISTANCE = 1.0  #   3 D: sphere radius; 2 D: circle radius
ACTION_DIM_DEFAULT = 12    #   Number of DOFs; will be /2 (2 D) or /3 (3 D)

ap = argparse.ArgumentParser("Unified 2D/3D dataset pre‑processor (updated)")
ap.add_argument("--mode", choices=["2d", "3d"], required=True)
ap.add_argument("--lidar_dataset", type=Path, required=True)
ap.add_argument("--path_dataset",  type=Path, required=True)
ap.add_argument("--output",        type=Path, required=True)
ap.add_argument("--action_dim",    type=int, default=ACTION_DIM_DEFAULT)
ap.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")
ap.add_argument(
    "--sample_n", type=int, default=-1,
    help="仅取前 n 条样本做测试；<=0 表示使用全部数据"
)

# append AppLauncher cli args
# AppLauncher.add_app_launcher_args(ap)
# parse the arguments
args_cli = ap.parse_args()

# launch omniverse app
# app_launcher = AppLauncher(args_cli)
# simulation_app = app_launcher.app

import h5py
import torch
import pickle
from typing import List, Tuple
import matplotlib.pyplot as plt

import numpy as np
from tqdm import tqdm

# ─── Cost‑map back‑end ──────────────────────────────────────────────────────
# Both 2 D & 3 D use the same API from RL2Path (2 D simply has z‑min == z‑max)
from RL2Path.nav.cost_map_2d import Costmap_2d, CostmapCfg
from stable_baselines3 import PPO

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

def to_cartesian(pt: np.ndarray) -> Tuple[float, float]:
    r, theta = pt
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    return x, y

def normalize_polar(pt: Tuple[float, float], max_r: float) -> np.ndarray:
    r, th = pt
    return np.array([r / max_r, (th + np.pi) / (2 * np.pi)])


def normalize_action_2d(action: np.ndarray, max_r: float) -> np.ndarray:
    n = action.shape[0]
    r_min = np.zeros(n)
    r_max = np.linspace(0.0, max_r, n + 1)[1:]
    th_min, th_max = -np.pi, np.pi
    r_norm = (action[:, 0] - r_min) / (r_max - r_min)
    th_norm = (action[:, 1] - th_min) / (th_max - th_min)
    return np.stack([r_norm, th_norm], axis=-1)

def denormalize_action_2d(action: np.ndarray, max_r: float) -> np.ndarray:
    """
    [已给出的反归一化] 由 (r_norm, θ_norm) 得到 (r, θ)，支持批量维。
    """
    if action.shape[-1] == 2:                       # 已配对
        point_num = action.shape[-2]
        pair = action
    else:                                            # 扁平化
        point_num = action.shape[-1] // 2
        pair = action.reshape(*action.shape[:-1], point_num, 2)

    theta_min, theta_max = -np.pi, np.pi
    r_min = 0.0

    r_max_seq = np.linspace(0.0, float(max_r), point_num + 1)[1:]
    r_max_seq = r_max_seq.reshape(*([1] * (pair.ndim - 2)), point_num)   # broadcast

    r     = pair[..., 0] * (r_max_seq - r_min) + r_min
    theta = pair[..., 1] * (theta_max - theta_min) + theta_min
    return np.stack((r, theta), axis=-1)               # (..., point_num, 2)

def denormalize_point(point: np.ndarray, max_r: float) -> Tuple[float, float]:
    """
    [已给出的反归一化] 由 (r_norm, θ_norm) 得到 (r, θ)，单点输入。
    """
    theta_min, theta_max = -np.pi, np.pi
    r_min = 0.0

    r_max = float(max_r)
    r = point[0] * (r_max - r_min) + r_min
    theta = point[1] * (theta_max - theta_min) + theta_min
    return r, theta

def obs_2d(costmap: np.ndarray, goal_local: Tuple[float, float],
           lethal: float, max_r: float) -> np.ndarray:
    return np.concatenate([
        costmap.flatten() / lethal,
        normalize_polar(goal_local, max_r)
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
                               lpath: np.ndarray,
                               params,
                               goal: np.ndarray | None = None,
                               controll_points = None,
                               title: str | None = None,
                               show_goal: bool = True):
    """
    costmap : (H, W) numpy array, 0~1
    lpath   : (N, 2) local-XY path
    params  : CostmapCfg, 需包含 resolution / size_x / size_y
    """
    # —— 把物理坐标 (m) → 栅格坐标 (pixel) ——————————
    # print("costmap",costmap)
    # print("lpath",lpath)
    costmap = costmap.squeeze()

    # # ① 先沿 x 轴对称——上下翻转
    cm_flip_x = costmap[::-1, :]        # 等价于 np.flipud(img)

    # # ② 再沿 y 轴对称——左右翻转
    costmap = cm_flip_x[:, ::-1]  # 等价于 np.fliplr(img_flip_x)

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
    if controll_points is not None:
        plt.scatter(controll_points[:, 0], controll_points[:, 1], c="y", s=60, marker='o', label='local path points')
    plt.plot(lpath[:, 0], lpath[:, 1], 'r-', lw=2, label='local path')
    plt.scatter(0, 0, c='g', s=60, marker='*', label='start')
    if show_goal:
        if goal is None:
            plt.scatter(lpath[-1, 0], lpath[-1, 1], c='b', s=50, label='goal')
        else:
            plt.scatter(goal[0], goal[1], c='b', s=50, label='goal')
    plt.axis('equal')
    plt.xlabel('x  [m]')
    plt.ylabel('y  [m]')
    if title: plt.title(title)
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

def process_2d(lidar_dataset: List[Tuple[np.ndarray]],
               path_dataset: List[np.ndarray],
               idx, 
               action_dim: int,
               params: CostmapCfg,
               max_dist: float) -> Tuple[List[np.ndarray], List[np.ndarray]]:
    costmaps, obss, acts = [], [], []
    cm = Costmap_2d(params=params)
    
    obss = []
    acts = []
    
    thetas = np.random.uniform(-np.pi, np.pi, size=len(lidar_dataset))
    lidar_dataset = [
        rotate_xy(pc.copy(), th)     # copy 防止原数据被覆写
        for pc, th in zip(lidar_dataset, thetas)
    ]
    
    lidar_dataset  = np.stack(lidar_dataset, axis=0)             # (B, N, 3)
    lidar_dataset = torch.from_numpy(lidar_dataset).float().to(cm.device)
    
    print("[INFO] Generating costmaps")
    cm.generate(lidar_dataset)
    inflated_list = cm.inflate()
    
    print("[INFO] Costmaps generated, processing observation...")
    
    for i in tqdm(range(len(lidar_dataset)), desc="Processing 2D samples"):
        path_idx = idx[i]
        gpath = np.array(path_dataset[int(path_idx)])  # 获取全局路径
        
        # ── local path + cleaning ──────────────────────────────────────────
        lpath = local_transform_2d(gpath)
        lpath = rotate_xy(lpath.copy(), thetas[i])
        if not in_bounds_2d(lpath, params):
            print(f"[WARN] Sample {i} out of bounds, skipping...")
            continue  # drop sample
        
        inflated = inflated_list[i].cpu().numpy()  

        # ── observation ----------------------------------------------------
        goal_local = to_polar(lpath[-1])
        obs = obs_2d(inflated, goal_local, params.lethal_obstacle, max_dist)
        obss.append(obs)

        # ── action ---------------------------------------------------------
        # kp = extract_key_points_combined(lpath, action_dim, max_dist - 0.05)
        # kp_mid = kp[1:]                       # keep end‑point
        # # print(kp_mid.shape)
        # polar_mid = np.array([to_polar(p) for p in kp_mid])
        kp_mid = lpath[1:]  # keep end‑point
        polar_mid = np.array([to_polar(p) for p in kp_mid])
        act = normalize_action_2d(polar_mid, max_dist)
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
    args = ap.parse_args()

    print(f"[INFO] Loading lidar dataset from: {args.lidar_dataset}")
    # ── load datasets ──────────────────────────────────────────────────────
    n_samp = args.sample_n if args.sample_n and args.sample_n > 0 else None  # NEW

    # ── .npz ──────────────────────────────────────────────────────────────
    if args.lidar_dataset.suffix == ".npz":
        with np.load(args.lidar_dataset, allow_pickle=True, mmap_mode='r') as data:  # mmap 避免整块拷贝
            arr = data["paths"]
            lidar_ds = arr[:n_samp].tolist() if n_samp else arr.tolist()            # NEW
        print(f"[INFO] Loaded {len(lidar_ds)} LiDAR samples from .npz file")

    elif args.lidar_dataset.suffix == ".h5":
        lidar_ds = []
        with h5py.File(args.lidar_dataset, "r") as f:
            pts_flat = f["points"][:40]       # N × (var) ― 每行是一维 float32
            idx      = f["path_index"][:40]   # N
            
        pts_flat = np.stack(pts_flat)
        pts_flat = pts_flat.reshape(len(pts_flat), -1, 2)
        
        print(pts_flat.shape)

        print(f"[INFO] Loaded {pts_flat.shape[0]} LiDAR samples from .h5 file")
    # ── .pkl ───────────────────────────────────────────────────────────────
    else:
        with open(args.lidar_dataset, "rb") as f:
            all_lidar = pickle.load(f)
        lidar_ds = all_lidar[:n_samp] if n_samp else all_lidar            # NEW
        if n_samp and len(all_lidar) > len(lidar_ds):
            print("[WARN] .pkl needs full deserialization before slicing,"
                "加载高峰内存无法省，但仍可节省后续处理时间。")
        print(f"[INFO] Loaded {len(lidar_ds)} LiDAR samples from .pkl file")


    print(f"[INFO] Loading path dataset from: {args.path_dataset}")
    if args.path_dataset.suffix == ".npz":
        paths = np.load(args.path_dataset, allow_pickle=True)["paths_pc"].tolist()
        print(f"[INFO] Loaded {len(paths)} paths from .npz file")
    elif args.path_dataset.suffix == ".h5":
        with h5py.File(args.path_dataset, "r") as f:
            paths = f["paths"][:].tolist()
        print(f"[INFO] Loaded {len(paths)} paths from .h5 file")
    else:
        with open(args.path_dataset, "rb") as f:
            paths = pickle.load(f)
        print(f"[INFO] Loaded {len(paths)} paths from .pkl file")
    
    # if args.sample_n is not None and args.sample_n > 0:
    #     paths = paths[:args.sample_n]

    # assert pts_flat.shape[0] == len(paths), f"[ERROR] LiDAR & Path dataset length mismatch: {pts_flat.shape[0]} vs {len(paths)}"
    print(f"[INFO] Dataset sizes match: {pts_flat.shape[0]} samples")

    params = CostmapCfg()  # default params; adjust if needed
    max_d = 1.8

    print(f"[INFO] Starting processing in {args.mode.upper()} mode...")

    if args.mode == "2d":
        obss, acts = process_2d(pts_flat, paths, idx, 
                                action_dim=args.action_dim // 2,
                                params=params, max_dist=max_d)
    else:
        obss, acts = process_3d(lidar_ds, paths,
                                action_dim=args.action_dim // 3,
                                params=params, max_dist=max_d)
    
    model = PPO.load("outputs/ckpts/tcnn_v0/model_20.zip", device="cuda")
    
    print(len(obss))
    
    for i in range(40):
        print(i)
        obs = obss[i]
        obs = np.concatenate([
            np.tile(obs[:2500], 3),
            obs[2500:]
        ]).astype(np.float32)
        # print(obs[2500:])
        
        key_points, _  = model.predict(obs, deterministic=True)
        
        costmap = obs[:2500].reshape(50, 50) * params.lethal_obstacle
        
        goal = denormalize_point(obs[7500:], max_d)
        goal = to_cartesian(goal)
        
        denormal_points = denormalize_action_2d(key_points, max_d)
        cart_points = np.array([to_cartesian(p) for p in denormal_points])
        control_points = np.vstack(([0, 0], cart_points))
        
        from scipy.interpolate import CubicSpline
        time_points = np.linspace(0, 1, len(control_points))
        spline_x = CubicSpline(time_points, control_points[:, 0])
        spline_y = CubicSpline(time_points, control_points[:, 1])
        path = np.column_stack((spline_x(np.arange(0, 1.05, 0.05)), spline_y(np.arange(0, 1.05, 0.05))))
        
        
        visualize_costmap_and_path(costmap, path, params, controll_points=control_points, goal=goal, title=f"2D Path_{i}")
    
    # print(f"[INFO] Processed {len(obss)} valid samples")

    # print(f"[INFO] Saving processed transitions to: {args.output}")
    # if args.output.suffix == ".h5":
    #     with h5py.File(args.output, "w") as f:
    #         f.create_dataset("observations", data=np.array(obss, dtype=np.float32), compression="gzip")
    #         f.create_dataset("actions", data=np.array(acts, dtype=np.float32), compression="gzip")
    #     print(f"[SUCCESS] Saved {len(obss)} transitions to HDF5 -> {args.output}")
    # else:
    #     with open(args.output, "wb") as f:
    #         pickle.dump([obss, acts], f)
    #     print(f"[SUCCESS] Saved {len(obss)} transitions to Pickle -> {args.output}")

if __name__ == "__main__":
    main()
