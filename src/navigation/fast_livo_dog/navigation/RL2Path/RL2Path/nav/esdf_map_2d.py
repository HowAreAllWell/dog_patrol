# costmap.py
# -*- coding: utf-8 -*-

import os
import time
import math
import torch
import numpy as np
import matplotlib.pyplot as plt
import torch.nn.functional as F

from dataclasses import dataclass as configclass

@configclass
class CostmapCfg:
    """
    仅用于 2D ESDF（单位：米）的地图配置与可视化参数。
    注意：实际分辨率在 __init__ 里根据 (x_min..x_max, map_size) 动态计算。
    """
    x_min: float = 0.0
    x_max: float = 2.0
    y_min: float = -1.0
    y_max: float = 1.0
    map_size: int = 50       # H=W=map_size
    r_max: float = 12.0

    # 运行时会重算：resolution = (x_max - x_min) / map_size
    resolution: float = (x_max - x_min) / map_size

    # 下面这些仅为兼容旧接口的字段（不再用于代价值映射）
    lethal_obstacle: int = 255
    inscribed_obstacle: int = 200
    inscribed_radius: float = 0.15
    inflation_radius: float = 0.45
    cost_scaling_factor: float = 4.0

    # ESDF 相关
    signed: bool = True                  # 是否输出有符号距离（障碍内为负）
    esdf_clip: float = 2.0               # 可视化/裁剪上限（米），建议 1~2m

    # ========= 新增：快速模式（分层 EDT + 插值 + 可选窄带精修） =========
    fast_mode: bool = True               # 默认开启快速路径
    ds_factor: int = 8                   # 下采样因子（>=1；1 表示不降采样）
    refine_band_m: float = 0.0           # 窄带宽度（米）；<=0 关闭窄带精修
    refine_margin_cells: int = 2         # 窄带包围框额外外扩的像素（原始分辨率格子数）


class Costmap_2d:
    r"""批量 2-D ESDF 计算器。接口不变，但支持快速模式（分层 + 插值 + 可选窄带精修）。"""

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

        # 分辨率（要求 x、y 尺度一致）
        self.params.resolution = (self.params.x_max - self.params.x_min) / float(self.params.map_size)
        self.resolution = self.params.resolution
        res_y = (self.params.y_max - self.params.y_min) / float(self.params.map_size)
        if abs(res_y - self.resolution) > 1e-6:
            raise ValueError(
                "当前实现要求等轴网格（dx==dy）。请设置 (x_max-x_min)/(y_max-y_min)==1。"
            )

        # 兼容字段：
        # - costmaps: 仍保存“障碍占据”布尔图（uint8, 255=障碍, 0=非障碍），用于可视化与 ESDF 输入
        # - inflated_costmaps: 现在直接存放 ESDF（float32，米）
        self.costmaps: torch.Tensor | None = None          # (B,H,W) uint8，占据（障碍=255）
        self.inflated_costmaps: torch.Tensor | None = None # (B,H,W) float32，ESDF（米）

    # -------------------------------------------------------------- #
    # 工具函数
    # -------------------------------------------------------------- #
    def _world_to_map(self, pts: torch.Tensor) -> torch.Tensor:
        """(N,2) local-coord → (N,2) 整型网格索引（x,y）"""
        idx = torch.floor(
            (pts - torch.tensor([self.params.x_min, self.params.y_min], device=pts.device, dtype=pts.dtype))
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

    # ======== Batched 1D EDT（Felzenszwalb-Huttenlocher 的批量化版本） ========
    @staticmethod
    def _edt_1d_batched(f: torch.Tensor) -> torch.Tensor:
        """
        f: (M,N) float64，一维代价：目标处=0，其它=+INF(大数)
        返回：一维平方欧氏距离（float64），大小为 (M,N)

        说明：
        - 将多条“线”（M 条，每条长度 N）一起计算，避免对每条线单独调用。
        - 在 q 维度上保留标量循环（长度 N），但每一步都对 M 条线矢量化更新。
        """
        assert f.ndim == 2
        device = f.device
        M, N = f.shape
        INF = torch.finfo(torch.float64).max / 4.0

        v = torch.empty((M, N), dtype=torch.int64, device=device)
        z = torch.empty((M, N + 1), dtype=torch.float64, device=device)
        k = torch.zeros((M,), dtype=torch.int64, device=device)

        v[:, 0] = 0
        z[:, 0] = -INF
        z[:, 1] = +INF

        def compute_s(q: int, k_cur: torch.Tensor) -> torch.Tensor:
            idx_vk = v.gather(1, k_cur.view(M, 1)).squeeze(1)                 # (M,)
            fv_k = f.gather(1, idx_vk.view(M, 1)).squeeze(1)                  # (M,)
            q64 = torch.tensor(float(q), dtype=torch.float64, device=device)
            num = (f[:, q] + q64 * q64) - (fv_k + idx_vk.to(torch.float64) ** 2)
            den = (2.0 * q64) - 2.0 * idx_vk.to(torch.float64)
            return num / den

        for q in range(1, N):
            s = compute_s(q, k)
            z_k = z.gather(1, k.view(M, 1)).squeeze(1)
            cond = s <= z_k
            while cond.any():
                k = torch.where(cond, k - 1, k)
                s = compute_s(q, k)
                z_k = z.gather(1, k.view(M, 1)).squeeze(1)
                cond = s <= z_k

            k_next = k + 1
            v.scatter_(1, k_next.view(M, 1), torch.full((M, 1), q, dtype=torch.int64, device=device))
            z.scatter_(1, k_next.view(M, 1), s.view(M, 1))
            z.scatter_(1, (k_next + 1).view(M, 1), torch.full((M, 1), +INF, dtype=torch.float64, device=device))
            k = k_next

        d = torch.empty((M, N), dtype=torch.float64, device=device)
        k2 = torch.zeros((M,), dtype=torch.int64, device=device)
        for q in range(N):
            z_next = z.gather(1, (k2 + 1).view(M, 1)).squeeze(1)
            cond = z_next < float(q)
            while cond.any():
                k2 = torch.where(cond, k2 + 1, k2)
                z_next = z.gather(1, (k2 + 1).view(M, 1)).squeeze(1)
                cond = z_next < float(q)
            idx_vk2 = v.gather(1, k2.view(M, 1)).squeeze(1)
            dq = (float(q) - idx_vk2.to(torch.float64))
            d[:, q] = dq * dq + f.gather(1, idx_vk2.view(M, 1)).squeeze(1)
        return d

    def _edt_2d_to_targets_batched(self, targets_mask: torch.Tensor, cell_size: float | None = None) -> torch.Tensor:
        """
        2D 精确 EDT（批量）：到目标集合（True 的格子）的欧氏距离（米，float32）。
        targets_mask: (B,H,W) bool，True=目标（例如障碍或自由）
        要求：每个 batch 至少一个 True（上层会筛掉全 False 的 batch）
        cell_size: 每个格子的物理尺寸（米）。None 时用 self.resolution。
        """
        assert targets_mask.ndim == 3
        B, H, W = targets_mask.shape
        device = targets_mask.device
        INF = torch.finfo(torch.float64).max / 4.0
        cell = self.resolution if cell_size is None else float(cell_size)

        f = torch.where(
            targets_mask,
            torch.tensor(0.0, dtype=torch.float64, device=device),
            torch.tensor(INF, dtype=torch.float64, device=device),
        )  # (B,H,W) float64

        # 列变换：把 (B,H,W) 视作 M=B*W 条长度 H 的线
        f_cols = f.permute(0, 2, 1).contiguous().view(B * W, H)  # (M1=BW, N1=H)
        D_cols = self._edt_1d_batched(f_cols).view(B, W, H).permute(0, 2, 1).contiguous()  # -> (B,H,W)

        # 行变换：把 (B,H,W) 视作 M=B*H 条长度 W 的线
        f_rows = D_cols.view(B * H, W)  # (M2=BH, N2=W)
        D = self._edt_1d_batched(f_rows).view(B, H, W)  # (B,H,W) squared distance in grid units

        return torch.sqrt(D).to(torch.float32) * cell  # 转米

    # -------------------------------------------------------------- #
    # 生成“障碍占据”栅格（批量）：返回 uint8（255=障碍），完全向量化
    # -------------------------------------------------------------- #
    def generate(
        self,
        points: torch.Tensor,        # (B,N,D>=2). D>=3 时自动忽略 Z
        ranges: torch.Tensor | None = None   # (B,N) 或 None
    ) -> torch.Tensor:
        if points.ndim != 3:
            raise ValueError("points 必须是 (B,N,D)")
        if points.shape[-1] < 2:
            raise ValueError("points 的最后一维至少要有 x,y")
        
        points = self._to_tensor(points, dtype=torch.float32)
        B, N, _ = points.shape
        map_size = self.params.map_size
        H = W = map_size

        pts_xy = points[..., :2]
        if ranges is None:
            ranges = pts_xy.norm(dim=-1)
        ranges = self._to_tensor(ranges, dtype=torch.float32)

        # 有效性与越界裁剪（全部向量化）
        valid = ranges < self.params.r_max  # (B,N)
        idx = torch.floor(
            (pts_xy - torch.tensor([self.params.x_min, self.params.y_min], device=self.device))
            / self.resolution
        ).long()  # (B,N,2)
        ix = idx[..., 0]
        iy = idx[..., 1]
        inb = (ix >= 0) & (ix < W) & (iy >= 0) & (iy < H)
        keep = valid & inb  # (B,N)

        if not keep.any():
            occ_maps = torch.zeros((B, H, W), dtype=torch.uint8, device=self.device)
            self.costmaps = occ_maps
            return occ_maps

        # 扁平化一次性写入占据
        b_grid = torch.arange(B, device=self.device).view(B, 1).expand(B, N)  # (B,N)
        b_sel = b_grid[keep]   # (K,)
        x_sel = ix[keep]       # (K,)
        y_sel = iy[keep]       # (K,)

        linear = (b_sel * (H * W) + y_sel * W + x_sel).long()  # (K,)

        occ_flat = torch.zeros(B * H * W, dtype=torch.uint8, device=self.device)
        occ_flat[linear] = 255
        occ_maps = occ_flat.view(B, H, W)

        self.costmaps = occ_maps
        return occ_maps

    # ====================== 内部：分层快速 ESDF ====================== #
    def _inflate_fast(self) -> torch.Tensor:
        """
        分层 EDT + 双线性插值 + 可选窄带精修（局部精确 EDT）。
        返回：esdf (B,H,W) float32 [m]
        """
        if self.costmaps is None:
            raise RuntimeError("请先调用 generate() 生成障碍占据图")

        p = self.params
        B, H, W = self.costmaps.shape
        device = self.device

        # 原始障碍布尔
        obst_fine = (self.costmaps == 255)  # (B,H,W) bool
        has_obst = obst_fine.view(B, -1).any(dim=1)       # (B,)
        has_free = (~obst_fine).view(B, -1).any(dim=1)    # (B,)

        # 初值：全自由（+clip）
        esdf = torch.full((B, H, W), fill_value=p.esdf_clip, dtype=torch.float32, device=device)

        # 如果无障碍（全自由），可直接返回 +clip
        if (~has_obst).all():
            self.inflated_costmaps = esdf
            return esdf

        # 1) 下采样障碍图（max-pool，保证子块内有障碍则保留）
        s = max(int(p.ds_factor), 1)
        if s == 1:
            obst_ds = obst_fine
            Hs, Ws = H, W
        else:
            # 转成 NCHW 的 C=1 再 pool
            x = obst_fine.float().unsqueeze(1)  # (B,1,H,W)
            # ceil_mode=True 保留边缘
            x_ds = F.max_pool2d(x, kernel_size=s, stride=s, ceil_mode=True)
            obst_ds = (x_ds.squeeze(1) > 0.5)   # (B,Hs,Ws)
            Hs, Ws = obst_ds.shape[-2:]

        cell_coarse = self.resolution * s  # 低分辨率格子代表的物理米数

        # 2) 低分辨率精确 EDT（到障碍、到自由）
        esdf_coarse_pos = torch.full((B, Hs, Ws), p.esdf_clip, dtype=torch.float32, device=device)
        d_to_obst = None
        idx_obst = torch.nonzero(obst_ds.view(B, -1).any(dim=1)).squeeze(1)
        if idx_obst.numel() > 0:
            d_to_obst = self._edt_2d_to_targets_batched(obst_ds[idx_obst], cell_size=cell_coarse)  # (B',Hs,Ws)
            esdf_coarse_pos[idx_obst] = d_to_obst

        esdf_coarse_neg = None
        if p.signed:
            esdf_coarse_neg = torch.full((B, Hs, Ws), p.esdf_clip, dtype=torch.float32, device=device)
            idx_both = torch.nonzero(
                obst_ds.view(B, -1).any(dim=1) & (~obst_ds).view(B, -1).any(dim=1)
            ).squeeze(1)
            if idx_both.numel() > 0:
                d_to_free = self._edt_2d_to_targets_batched((~obst_ds[idx_both]), cell_size=cell_coarse)
                # 仅用于障碍内（负号）
                esdf_coarse_neg[idx_both] = d_to_free

        # 3) 双线性插值回原分辨率
        def upsample(m: torch.Tensor) -> torch.Tensor:
            if (m.shape[-2], m.shape[-1]) == (H, W):
                return m
            return F.interpolate(m.unsqueeze(1), size=(H, W), mode="bilinear", align_corners=False).squeeze(1)

        esdf_pos_up = upsample(esdf_coarse_pos)
        if p.signed:
            esdf_neg_up = upsample(esdf_coarse_neg)
            esdf = torch.where(obst_fine, -esdf_neg_up, esdf_pos_up)
        else:
            esdf = esdf_pos_up

        # 4) 可选：窄带精修（仅在 |ESDF| ≤ band 的区域做局部精确 EDT）
        band = float(p.refine_band_m)
        if band > 0.0:
            # 以插值后的绝对值为锚，找到需要精修的像素
            refine_mask = (esdf.abs() <= band)  # (B,H,W) bool
            # 对每个 batch 提取最小包围框，做局部精确 EDT，再回填
            for b in range(B):
                if not refine_mask[b].any():
                    continue
                yx = torch.nonzero(refine_mask[b], as_tuple=False)
                y_min = int(torch.min(yx[:, 0]).item())
                y_max = int(torch.max(yx[:, 0]).item())
                x_min = int(torch.min(yx[:, 1]).item())
                x_max = int(torch.max(yx[:, 1]).item())

                # 包围框外扩 margin，避免边缘截断
                pad = max(int(p.refine_margin_cells), 0)
                y0 = max(0, y_min - pad); y1 = min(H - 1, y_max + pad)
                x0 = max(0, x_min - pad); x1 = min(W - 1, x_max + pad)
                # 变成切片上界式
                y1s = y1 + 1; x1s = x1 + 1

                obst_patch = obst_fine[b:b+1, y0:y1s, x0:x1s]  # (1,h,w)
                has_ob = obst_patch.reshape(-1).any().item()
                has_fr = (~obst_patch).view(-1).any().item()

                # 如果两个集合都存在，才能合成有符号距离；否则跳过（插值值已在）
                if not has_ob and not has_fr:
                    continue

                # 精确 EDT（局部）：
                esdf_patch = torch.empty_like(obst_patch, dtype=torch.float32)
                if p.signed:
                    # 到障碍（自由端正）与到自由（障碍端负）
                    if has_ob:
                        d_ob = self._edt_2d_to_targets_batched(obst_patch, cell_size=self.resolution)   # (1,h,w)
                    else:
                        d_ob = torch.full_like(obst_patch, p.esdf_clip, dtype=torch.float32)
                    if has_fr:
                        d_fr = self._edt_2d_to_targets_batched((~obst_patch), cell_size=self.resolution)
                    else:
                        d_fr = torch.full_like(obst_patch, p.esdf_clip, dtype=torch.float32)
                    esdf_patch = torch.where(obst_patch, -d_fr, d_ob)
                else:
                    # 只要到障碍距离
                    if has_ob:
                        d_ob = self._edt_2d_to_targets_batched(obst_patch, cell_size=self.resolution)
                    else:
                        d_ob = torch.full_like(obst_patch, p.esdf_clip, dtype=torch.float32)
                    esdf_patch = d_ob

                # 仅把 refine_mask 覆盖区域回填（避免窄带外不必要改变）
                submask = refine_mask[b:b+1, y0:y1s, x0:x1s]
                esdf[b:b+1, y0:y1s, x0:x1s] = torch.where(
                    submask, esdf_patch, esdf[b:b+1, y0:y1s, x0:x1s]
                )

        # 裁剪到可视化/安全上限
        esdf = torch.clamp(esdf, min=-p.esdf_clip, max=p.esdf_clip)
        return esdf

    # -------------------------------------------------------------- #
    # inflate()：计算 ESDF（米），写入 inflated_costmaps（批量）
    # -------------------------------------------------------------- #
    def inflate(self, fast: bool | None = None) -> torch.Tensor:
        """
        计算 ESDF（米）。默认走快速模式（可在 CostmapCfg.fast_mode 或参数 fast 控制）。
        fast=None -> 使用 cfg.fast_mode；fast=True -> 分层+插值(+窄带)；fast=False -> 全图精确
        """
        if self.costmaps is None:
            raise RuntimeError("请先调用 generate() 生成障碍占据图")

        use_fast = self.params.fast_mode if fast is None else bool(fast)

        if not use_fast:
            # ===== 原精确路径（全图） =====
            p = self.params
            B, H, W = self.costmaps.shape
            device = self.device

            obst = (self.costmaps == 255)  # (B,H,W) bool
            has_obst = obst.view(B, -1).any(dim=1)       # (B,)
            has_free = (~obst).view(B, -1).any(dim=1)    # (B,)

            esdf = torch.full((B, H, W), fill_value=p.esdf_clip, dtype=torch.float32, device=device)

            if has_obst.any():
                idx_obst = torch.nonzero(has_obst).squeeze(1)
                d_to_obst = self._edt_2d_to_targets_batched(obst[idx_obst])  # (B',H,W)
                esdf[idx_obst] = d_to_obst

                if p.signed:
                    idx_both = torch.nonzero(has_obst & has_free).squeeze(1)
                    if idx_both.numel() > 0:
                        d_to_free = self._edt_2d_to_targets_batched((~obst[idx_both]))  # (B'',H,W)
                        mask_both_obst = obst[idx_both]  # (B'',H,W) True=障碍
                        esdf[idx_both][mask_both_obst] = -d_to_free[mask_both_obst]

                    idx_all_obst = torch.nonzero(has_obst & (~has_free)).squeeze(1)
                    if idx_all_obst.numel() > 0:
                        esdf[idx_all_obst] = -p.esdf_clip

            self.inflated_costmaps = esdf
            return esdf

        # ===== 快速路径 =====
        esdf = self._inflate_fast()
        self.inflated_costmaps = esdf
        return esdf

    # -------------------------------------------------------------- #
    # 查询接口（保持不变：名字不变；含义更新：inflated=ESDF）
    # -------------------------------------------------------------- #
    def get_costmap(self, agent_idx: int | None = None):
        """返回障碍占据图（uint8，255=障碍）。"""
        if self.costmaps is None:
            return None
        return self.costmaps if agent_idx is None else self.costmaps[agent_idx]

    def get_inflated_costmap(self, agent_idx: int | None = None):
        """返回 ESDF（float32，单位：米）。"""
        if self.inflated_costmaps is None:
            return None
        return self.inflated_costmaps if agent_idx is None else self.inflated_costmaps[agent_idx]

    # -------------------------------------------------------------- #
    # 可视化（ETH 风格：红→黄→绿；障碍=黑色等高线）
    # -------------------------------------------------------------- #
    def debug_plot(
        self,
        agent_idx: int = 0,
        inflated: bool = False,
        actions: torch.Tensor | None = None,  # (B,2) 或 None
        ax=None,
        goals: torch.Tensor | None = None,
        **imshow_kwargs
    ):
        x_min, x_max = self.params.x_min, self.params.x_max
        y_min, y_max = self.params.y_min, self.params.y_max
        os.makedirs("./logs/plt", exist_ok=True)

        if inflated:
            if self.inflated_costmaps is None:
                raise RuntimeError("请先调用 inflate() 计算 ESDF")
            esdf = self.inflated_costmaps[agent_idx].detach().cpu().numpy()
            dmax = float(self.params.esdf_clip) if self.params.esdf_clip > 0 else float(esdf.max() if np.isfinite(esdf).any() else 1.0)

            esdf_pos = esdf.copy()

            plt.figure(figsize=(6, 6))
            im = plt.imshow(
                esdf_pos,
                cmap="plasma",
                vmin=0.0,
                vmax=dmax,
                origin="lower",
                extent=[x_min, x_max, y_min, y_max],
                **imshow_kwargs,
            )

            if self.costmaps is not None:
                obst = (self.costmaps[agent_idx].detach().cpu().numpy() == 255)
                if obst.any():
                    plt.contour(
                        obst.astype(np.uint8),
                        levels=[0.5],
                        colors="black",
                        linewidths=1.0,
                        origin="lower",
                        extent=[x_min, x_max, y_min, y_max],
                    )

            if goals is not None:
                goals = goals.detach().cpu().numpy()
                plt.scatter(goals[agent_idx, 0], goals[agent_idx, 1], c="k", s=60, marker="*", label="goal")
            if actions is not None:
                actions = actions.detach().cpu().numpy()
                action = actions[agent_idx]
                plt.plot(action[:, 0], action[:, 1], "-", lw=2, label="path")

            plt.xlabel("x  [m]"); plt.ylabel("y  [m]")
            plt.title("ESDF (ETH-style: red→yellow→green)")
            plt.colorbar(im, shrink=0.82, label="distance to nearest obstacle [m]")
            plt.axis("equal"); plt.tight_layout()
            fname = f"./logs/plt/{int(time.time())}_esdf_{agent_idx}.png"
            plt.savefig(fname, dpi=160)
            plt.close()
            return

        if self.costmaps is None:
            raise RuntimeError("请先调用 generate() 生成占据图")
        occ = self.costmaps[agent_idx].detach().cpu().numpy().squeeze()

        plt.figure(figsize=(6, 6))
        plt.imshow(
            occ,
            cmap="gray_r",
            origin="lower",
            extent=[x_min, x_max, y_min, y_max],
            **imshow_kwargs,
        )
        if goals is not None:
            goals = goals.detach().cpu().numpy()
            plt.scatter(goals[agent_idx, 0], goals[agent_idx, 1], c="r", s=60, marker="*", label="goal")
        if actions is not None:
            actions = actions.detach().cpu().numpy()
            action = actions[agent_idx]
            plt.plot(action[:, 0], action[:, 1], "g-", lw=2, label="path")
        plt.xlabel("x  [m]"); plt.ylabel("y  [m]")
        plt.title("obstacles (uint8)")
        plt.axis("equal"); plt.tight_layout()
        fname = f"./logs/plt/{int(time.time())}_obstacles_{agent_idx}.png"
        plt.savefig(fname, dpi=160)
        plt.close()

    # ======== 网格中心（单位：米，局部坐标系） ========
    def _grid_centers_m(self, map_size: int, device: torch.device) -> torch.Tensor:
        """返回 (HW,2) 的网格中心坐标（x,y），保留以备扩展。"""
        p = self.params
        y, x = torch.meshgrid(
            torch.arange(map_size, device=device),
            torch.arange(map_size, device=device),
            indexing="ij",
        )
        xs = p.x_min + (x.float() + 0.5) * self.resolution
        ys = p.y_min + (y.float() + 0.5) * self.resolution
        return torch.stack([xs.flatten(), ys.flatten()], dim=1)  # (HW,2)

# ============================== #
#             TESTS              #
# ============================== #
if __name__ == "__main__":
    torch.manual_seed(0)
    cfg = CostmapCfg()
    # 可切换：cfg.fast_mode=True/False；ds_factor、refine_band_m 可调
    cm = Costmap_2d(cfg, device="cpu")  # 小图 CPU 即可；上 GPU 也能跑

    H = W = cfg.map_size

    def make_points_case(case: int):
        """
        返回 (1,N,2) points。
        case 0: 单点障碍
        case 1: 一道垂直“墙”
        case 2: 两团稀疏散点
        """
        if case == 0:
            pts = np.array([[ (cfg.x_min + cfg.x_max) * 0.5,
                               (cfg.y_min + cfg.y_max) * 0.5 ]], dtype=np.float32)
        elif case == 1:
            xs = np.full(25, (cfg.x_min + cfg.x_max) * 0.7, dtype=np.float32)
            ys = np.linspace(cfg.y_min + 0.1, cfg.y_max - 0.1, 25, dtype=np.float32)
            pts = np.stack([xs, ys], axis=1)
        else:
            n1 = 30; n2 = 30
            c1 = np.array([cfg.x_min + 0.5, 0.0], dtype=np.float32)
            c2 = np.array([cfg.x_max - 0.6, 0.4], dtype=np.float32)
            pts1 = c1 + 0.05 * np.random.randn(n1, 2).astype(np.float32)
            pts2 = c2 + 0.05 * np.random.randn(n2, 2).astype(np.float32)
            pts = np.concatenate([pts1, pts2], axis=0)
        return torch.from_numpy(pts)[None, ...]  # (1,N,2)

    for case in range(3):
        points = make_points_case(case)
        cm.generate(points)

        # 快速 vs 精确（可注释其一对比）
        esdf_fast = cm.inflate(fast=True)
        cm.debug_plot(agent_idx=0, inflated=True)
        plt.close()
        cm.debug_plot(agent_idx=0, inflated=False)
        plt.close()

        esdf_exact = cm.inflate(fast=False)  # 作为对照
        es = esdf_fast[0].numpy()
        print(f"[fast case {case}] ESDF stats (m): min={es.min():.3f}  max={es.max():.3f}  mean={es.mean():.3f}")
        es2 = esdf_exact[0].numpy()
        print(f"[exact case {case}] ESDF stats (m): min={es2.min():.3f}  max={es2.max():.3f}  mean={es2.mean():.3f}")