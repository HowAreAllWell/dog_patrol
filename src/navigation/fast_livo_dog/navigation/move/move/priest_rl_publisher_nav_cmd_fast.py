#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Fast variant of priest_rl_publisher_omni_nav_cmd.py.

This file leaves the original node untouched. It subclasses the original node
and replaces only two hot paths at runtime:
  1. scipy CubicSpline batch resampling -> vectorized torch resampling.
  2. exact/fast ESDF inflate -> approximate multi-ring torch max-pool ESDF.

Run directly with:
  python3 src/move/move/priest_rl_publisher_nav_cmd_fast.py
"""

from __future__ import annotations

import math
from typing import Optional

import numpy as np
import rclpy
import torch
import torch.nn.functional as F

try:
    import move.priest_rl_publisher_nav_cmd as base
except ModuleNotFoundError:
    import priest_rl_publisher_nav_cmd as base


class FastRLLocalPlannerNodeROS2(base.RLLocalPlannerNodeROS2):
    def __init__(self):
        self._original_spline_fn = base.batch_cubic_spline_interpolation
        super().__init__()

        self.declare_parameter("fast_nav.enable_fast_spline", True)
        self.declare_parameter("fast_nav.spline_mode", "catmull_rom")  # catmull_rom / linear / original
        self.declare_parameter("fast_nav.spline_max_points", 0)         # 0 keeps requested num_points

        self.declare_parameter("fast_nav.max_scan_points", 512)         # <=0 disables scan decimation
        self.declare_parameter("fast_nav.scan_select", "closest")       # closest / stride

        self.declare_parameter("fast_nav.costmap_mode", "approx")       # approx / original
        self.declare_parameter("fast_nav.costmap_reuse_frames", 1)       # 1 disables reuse
        self.declare_parameter("fast_nav.costmap_radii_cells", "1,2,4,8,16,32,64")

        self.fast_spline_enabled = bool(
            self.get_parameter("fast_nav.enable_fast_spline").get_parameter_value().bool_value
        )
        self.fast_spline_mode = (
            self.get_parameter("fast_nav.spline_mode").get_parameter_value().string_value
            or "catmull_rom"
        ).lower()
        self.fast_spline_max_points = int(
            self.get_parameter("fast_nav.spline_max_points").get_parameter_value().integer_value
        )
        self.fast_max_scan_points = int(
            self.get_parameter("fast_nav.max_scan_points").get_parameter_value().integer_value
        )
        self.fast_scan_select = (
            self.get_parameter("fast_nav.scan_select").get_parameter_value().string_value
            or "closest"
        ).lower()
        self.fast_costmap_mode = (
            self.get_parameter("fast_nav.costmap_mode").get_parameter_value().string_value
            or "approx"
        ).lower()
        self.fast_costmap_reuse_frames = max(
            1,
            int(self.get_parameter("fast_nav.costmap_reuse_frames").get_parameter_value().integer_value),
        )
        self.fast_costmap_radii_cells = self._parse_radii(
            self.get_parameter("fast_nav.costmap_radii_cells").get_parameter_value().string_value
        )

        base.batch_cubic_spline_interpolation = self._spline_resample_dispatch

        self._original_costmap_generate = self.costmap.generate
        self._original_costmap_inflate = self.costmap.inflate
        self._costmap_call_count = 0
        self._reused_costmap_this_frame = False
        self._cached_costmaps: Optional[torch.Tensor] = None
        self._cached_inflated_costmaps: Optional[torch.Tensor] = None
        self.costmap.generate = self._costmap_generate_dispatch
        self.costmap.inflate = self._costmap_inflate_dispatch

        self.get_logger().debug(
            "[fast_nav] enabled: "
            f"spline={self.fast_spline_enabled}/{self.fast_spline_mode}, "
            f"spline_max_points={self.fast_spline_max_points}, "
            f"max_scan_points={self.fast_max_scan_points}, "
            f"costmap_mode={self.fast_costmap_mode}, "
            f"costmap_reuse_frames={self.fast_costmap_reuse_frames}, "
            f"costmap_radii_cells={self.fast_costmap_radii_cells}"
        )

    @staticmethod
    def _parse_radii(value: str) -> tuple[int, ...]:
        radii = []
        for item in (value or "").split(","):
            item = item.strip()
            if not item:
                continue
            try:
                radius = int(item)
            except ValueError:
                continue
            if radius > 0:
                radii.append(radius)
        return tuple(radii) or (1, 2, 4, 8, 16, 32, 64)

    def _scan_points_ego(self) -> Optional[np.ndarray]:
        pts = super()._scan_points_ego()
        if pts is None:
            return None
        max_points = int(self.fast_max_scan_points)
        if max_points <= 0 or pts.shape[0] <= max_points:
            return pts

        if self.fast_scan_select == "stride":
            step = max(1, int(math.ceil(pts.shape[0] / max_points)))
            return pts[::step][:max_points].astype(np.float32, copy=False)

        dist2 = np.einsum("ij,ij->i", pts, pts)
        idx = np.argpartition(dist2, max_points - 1)[:max_points]
        idx.sort()
        return pts[idx].astype(np.float32, copy=False)

    def _spline_resample_dispatch(self, control_points_batch, num_points=20):
        if (not self.fast_spline_enabled) or self.fast_spline_mode == "original":
            return self._original_spline_fn(control_points_batch, num_points=num_points)

        target_points = int(num_points)
        if self.fast_spline_max_points > 1:
            target_points = min(target_points, int(self.fast_spline_max_points))

        if self.fast_spline_mode == "linear":
            return self._batch_linear_resample(control_points_batch, target_points)
        return self._batch_catmull_rom_resample(control_points_batch, target_points)

    @torch.inference_mode()
    def _batch_linear_resample(self, control_points_batch: torch.Tensor, num_points: int) -> torch.Tensor:
        points = control_points_batch
        batch_size, num_control_points, dims = points.shape
        if num_points <= 0:
            return torch.empty((batch_size, 0, dims), dtype=points.dtype, device=points.device)
        if num_control_points == 1:
            return points.expand(batch_size, num_points, dims)
        if num_points == num_control_points:
            return points

        t = torch.linspace(
            0.0,
            float(num_control_points - 1),
            num_points,
            dtype=points.dtype,
            device=points.device,
        )
        i0 = torch.floor(t).to(torch.long).clamp(0, num_control_points - 1)
        i1 = (i0 + 1).clamp(0, num_control_points - 1)
        w = (t - i0.to(points.dtype)).view(1, num_points, 1)
        return points[:, i0, :] * (1.0 - w) + points[:, i1, :] * w

    @torch.inference_mode()
    def _batch_catmull_rom_resample(self, control_points_batch: torch.Tensor, num_points: int) -> torch.Tensor:
        points = control_points_batch
        batch_size, num_control_points, dims = points.shape
        if num_points <= 0:
            return torch.empty((batch_size, 0, dims), dtype=points.dtype, device=points.device)
        if num_control_points < 4:
            return self._batch_linear_resample(points, num_points)
        if num_points == num_control_points:
            return points

        t = torch.linspace(
            0.0,
            float(num_control_points - 1),
            num_points,
            dtype=points.dtype,
            device=points.device,
        )
        i1 = torch.floor(t).to(torch.long).clamp(0, num_control_points - 1)
        i0 = (i1 - 1).clamp(0, num_control_points - 1)
        i2 = (i1 + 1).clamp(0, num_control_points - 1)
        i3 = (i1 + 2).clamp(0, num_control_points - 1)
        u = (t - i1.to(points.dtype)).view(1, num_points, 1)
        u2 = u * u
        u3 = u2 * u

        p0 = points[:, i0, :]
        p1 = points[:, i1, :]
        p2 = points[:, i2, :]
        p3 = points[:, i3, :]

        out = 0.5 * (
            (2.0 * p1)
            + (-p0 + p2) * u
            + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * u2
            + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * u3
        )
        out[:, 0, :] = points[:, 0, :]
        out[:, -1, :] = points[:, -1, :]
        return out

    def _costmap_generate_dispatch(self, points, ranges=None):
        self._costmap_call_count += 1
        reuse_frames = int(self.fast_costmap_reuse_frames)
        can_reuse = (
            reuse_frames > 1
            and self._cached_costmaps is not None
            and self._cached_costmaps.shape[0] == int(points.shape[0])
            and ((self._costmap_call_count - 1) % reuse_frames != 0)
        )
        if can_reuse:
            self._reused_costmap_this_frame = True
            self.costmap.costmaps = self._cached_costmaps
            return self._cached_costmaps

        self._reused_costmap_this_frame = False
        costmaps = self._original_costmap_generate(points, ranges)
        self._cached_costmaps = costmaps
        self._cached_inflated_costmaps = None
        return costmaps

    def _costmap_inflate_dispatch(self, *args, **kwargs):
        if (
            self._reused_costmap_this_frame
            and self._cached_inflated_costmaps is not None
            and self._cached_inflated_costmaps.shape == self.costmap.costmaps.shape
        ):
            self.costmap.inflated_costmaps = self._cached_inflated_costmaps
            return self._cached_inflated_costmaps

        if self.fast_costmap_mode == "original" or not getattr(self, "use_esdf", True):
            inflated = self._original_costmap_inflate(*args, **kwargs)
        else:
            inflated = self._approximate_esdf_inflate()

        self._cached_inflated_costmaps = inflated
        return inflated

    @torch.inference_mode()
    def _approximate_esdf_inflate(self) -> torch.Tensor:
        if self.costmap.costmaps is None:
            raise RuntimeError("call generate() before inflate()")

        costmaps = self.costmap.costmaps
        batch_size, height, width = costmaps.shape
        device = costmaps.device
        dtype = torch.float32

        clip = float(getattr(self.costmap.params, "esdf_clip", self.cfg.r_max))
        resolution = float(getattr(self.costmap, "resolution", 1.0))
        occ = (costmaps == 255).unsqueeze(1)
        esdf = torch.full((batch_size, 1, height, width), clip, dtype=dtype, device=device)
        esdf = torch.where(occ, torch.zeros_like(esdf), esdf)

        assigned = occ.clone()
        occ_f = occ.to(dtype)
        for radius in self.fast_costmap_radii_cells:
            kernel = 2 * int(radius) + 1
            dilated = F.max_pool2d(occ_f, kernel_size=kernel, stride=1, padding=int(radius)) > 0.0
            ring = dilated & (~assigned)
            value = min(float(radius) * resolution, clip)
            esdf = torch.where(ring, torch.full_like(esdf, value), esdf)
            assigned = assigned | ring

        inflated = esdf.squeeze(1)
        self.costmap.inflated_costmaps = inflated
        return inflated


def main():
    rclpy.init()
    node = FastRLLocalPlannerNodeROS2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
