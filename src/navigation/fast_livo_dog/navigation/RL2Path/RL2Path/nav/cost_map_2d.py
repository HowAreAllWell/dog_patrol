# costmap.py

import torch
import numpy as np
import matplotlib.pyplot as plt
# from isaaclab.utils import configclass
from dataclasses import dataclass

# @configclass
@dataclass
class CostmapCfg:
    """
    包含生成和膨胀代价地图所需的所有参数。
    """
    x_min: float = 0.0
    x_max: float = 2.0
    y_min: float = -1.0
    y_max: float = 1.0
    map_size: int = 50  # 代价地图的分辨率（map_size x map_size）
    r_max: float = 12.0
    r_min: float = 0.15

    resolution: float = (x_max - x_min) / map_size 

    lethal_obstacle: int = 255
    inscribed_obstacle: int = 200

    inscribed_radius: float = 0.15
    inflation_radius: float = 0.45
    cost_scaling_factor: float = 4.0


class Costmap_2d:
    r"""同时为 **多个 agent** 生成 / 膨胀 2-D costmap（全部基于 **torch**）。"""

    # -------------------------------------------------------------- #
    # 初始化
    # -------------------------------------------------------------- #
    def __init__(
        self,
        params: CostmapCfg = CostmapCfg(),
        device: str | torch.device = "cuda"
    ):
        self.params = params
        self.device = torch.device(device)
        self.resolution = params.resolution

        self.costmaps: torch.Tensor | None = None       # (B, H, W), uint8
        self.inflated_costmaps: torch.Tensor | None = None

    # -------------------------------------------------------------- #
    # 工具函数
    # -------------------------------------------------------------- #
    def _world_to_map(self, pts: torch.Tensor) -> torch.Tensor:
        """(N,2) local-coord → (N,2) 整型网格索引（x,y）"""
        idx = torch.floor(
            (pts - torch.tensor([self.params.x_min, self.params.y_min], device=pts.device))
            / self.resolution
        ).long()
        return idx
    
    def _to_tensor(self, arr, dtype=torch.float32):
        if isinstance(arr, torch.Tensor):
            return arr.to(self.device, dtype=dtype)
        elif isinstance(arr, np.ndarray):
            return torch.from_numpy(arr).to(self.device, dtype=dtype)
        else:
            raise TypeError(f"Unsupported type: {type(arr)}")

    def _compute_cost(self, dist: torch.Tensor) -> torch.Tensor:
        """距离 → cost（指数衰减模型）。"""
        p = self.params
        cost = torch.zeros_like(dist, dtype=torch.float32)

        mask0 = dist == 0
        mask1 = (dist > 0) & (dist <= p.inscribed_radius)
        mask2 = (dist > p.inscribed_radius) & (dist <= p.inflation_radius)

        cost[mask0] = p.lethal_obstacle
        cost[mask1] = p.inscribed_obstacle
        cost[mask2] = p.inscribed_obstacle * torch.exp(
            -p.cost_scaling_factor * (dist[mask2] - p.inscribed_radius)
        )
        return cost.clamp(0, p.inscribed_obstacle).to(torch.uint8)

    def _grid_coords(self, map_size: int, device: torch.device) -> torch.Tensor:
        """生成 (H*W,2) 网格坐标张量（x,y），缓存用。"""
        y, x = torch.meshgrid(
            torch.arange(map_size, device=device),
            torch.arange(map_size, device=device),
            indexing="ij",
        )
        return torch.stack([x.flatten(), y.flatten()], dim=1).float()  # (HW,2)

    # -------------------------------------------------------------- #
    # 生成原始 costmap（批量）
    # -------------------------------------------------------------- #
    def generate(
        self,
        points: torch.Tensor,        # (B,N,D≥2). D≥3 时自动忽略 Z
        ranges: torch.Tensor | None = None   # (B,N) 或 None
    ) -> torch.Tensor:
        if points.ndim != 3:
            raise ValueError("points 必须是 (B,N,D)")
        if points.shape[-1] < 2:
            raise ValueError("points 的最后一维至少要有 x,y")
        
        points = self._to_tensor(points, dtype=torch.float32)

        points = points.to(self.device)
        B, N, _ = points.shape
        map_size = self.params.map_size

        pts_xy = points[..., :2]                     # 仅取 x,y
        if ranges is None:
            ranges = pts_xy.norm(dim=-1)             # xy 平面距离
        
        ranges = self._to_tensor(ranges, dtype=torch.float32)
        ranges = ranges.to(self.device)

        costmaps = torch.zeros(
            (B, map_size, map_size), dtype=torch.uint8, device=self.device
        )

        for b in range(B):
            valid = (ranges[b] >= self.params.r_min) & (ranges[b] <= self.params.r_max) # ranges[b] < self.params.r_max
            valid_pts = pts_xy[b][valid]
            if valid_pts.numel() == 0:
                continue

            idx = self._world_to_map(valid_pts)      # 离散化
            in_bounds = (
                (idx[:, 0] >= 0) & (idx[:, 0] < map_size) &
                (idx[:, 1] >= 0) & (idx[:, 1] < map_size)
            )
            idx = idx[in_bounds]
            if idx.numel() == 0:
                continue

            costmaps[b][idx[:, 1], idx[:, 0]] = self.params.lethal_obstacle

        self.costmaps = costmaps
        return costmaps

    # -------------------------------------------------------------- #
    # 膨胀（批量，纯 torch 实现）
    # -------------------------------------------------------------- #
    def inflate(self) -> torch.Tensor:
        if self.costmaps is None:
            raise RuntimeError("请先调用 generate() 生成 costmap")

        B, H, W = self.costmaps.shape
        inflated = self.costmaps.clone()
        grid_xy = self._grid_coords(H, self.device)          # (HW,2) x,y
        grid_xy_m = grid_xy * self.resolution                # 转米

        for b in range(B):
            obst_mask = self.costmaps[b] == self.params.lethal_obstacle
            if not obst_mask.any():
                continue

            obst_idx = obst_mask.nonzero(as_tuple=False).float()  # (M,2) y,x
            obst_xy = torch.stack([obst_idx[:, 1], obst_idx[:, 0]], dim=1)  # x,y
            obst_xy_m = obst_xy * self.resolution

            # 距离矩阵 (HW,M) → 每格到最近障碍的距离
            min_dist = torch.cdist(grid_xy_m, obst_xy_m, p=2).min(dim=1).values
            min_dist = min_dist.view(H, W)

            mask = min_dist <= self.params.inflation_radius
            if mask.any():
                inflated[b][mask] = torch.maximum(
                    inflated[b][mask],
                    self._compute_cost(min_dist[mask])
                )

        self.inflated_costmaps = inflated
        return inflated

    # -------------------------------------------------------------- #
    # 查询接口 & 可视化
    # -------------------------------------------------------------- #
    def get_costmap(self, agent_idx: int | None = None):
        if self.costmaps is None:
            return None
        return self.costmaps if agent_idx is None else self.costmaps[agent_idx]

    def get_inflated_costmap(self, agent_idx: int | None = None):
        if self.inflated_costmaps is None:
            return None
        return (
            self.inflated_costmaps
            if agent_idx is None else self.inflated_costmaps[agent_idx]
        )

    def debug_plot(
        self,
        agent_idx: int = 0,
        inflated: bool = False,
        actions: torch.Tensor | None = None,  # (B,2) 或 None
        ax=None,
        goals: torch.Tensor | None = None,
        **imshow_kwargs
    ):
        costmap = self.inflated_costmaps[agent_idx].detach().cpu().numpy()
        
        costmap = costmap.squeeze()                 # 去掉 (1, H, W) 之类多余维度
        
        x_min, x_max = self.params.x_min, self.params.x_max   # 现为 0.0, 2.2
        y_min, y_max = self.params.y_min, self.params.y_max   # 仍为 -1.1, 1.1

        # —— 画图 ————————————————————————————————
        plt.figure(figsize=(6, 6))
        plt.imshow(costmap,
                    cmap='gray_r',
                    origin='lower',                  # 行 0 放在 y_min（图像下边）
                    extent=[x_min, x_max, y_min, y_max])
        
        if goals is not None:
            goals = goals.detach().cpu().numpy() if goals is not None else None
            plt.scatter(goals[agent_idx, 0], goals[agent_idx, 1], c='r', s=60, marker='*', label='goal')
            
        if actions is not None:
            actions = actions.detach().cpu().numpy()
            action = actions[agent_idx]
            plt.plot(action[:, 0], action[:, 1], 'g-', lw=2, label='local path')
        
        plt.axis('equal')
        plt.xlabel('x  [m]')
        plt.ylabel('y  [m]')
        plt.legend()
        plt.tight_layout()
        import time
        plt.savefig(f"./log/plt/{time.time()}_costmap_{agent_idx}_{'inflated' if inflated else 'raw'}.png")
        
        
if __name__ == "__main__":
    # 世界坐标
    p = torch.tensor([[0.00, 0.00],      # 原点
                    [1.92, 1.92],      # 接近右上角
                    [-1.99, -1.99]])   # 接近左下角
    cm = Costmap_2d()        # 使用默认参数
    print(cm._world_to_map(p))