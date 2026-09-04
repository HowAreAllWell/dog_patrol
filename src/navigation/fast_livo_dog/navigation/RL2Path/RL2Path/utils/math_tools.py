import math
import torch
import numpy as np

from scipy.interpolate import CubicSpline
from typing import Union, Tuple

@torch.no_grad()
def polar_to_cartesian(polar: torch.Tensor) -> torch.Tensor:
    """
    输入: 形状 (..., 2) 的 tensor，其中 polar[..., 0] = r，polar[..., 1] = θ (弧度)。
    输出: 同形状的 tensor，其中 out[..., 0] = x，out[..., 1] = y。
    """
    r, theta = polar.unbind(-1)
    x = r * torch.cos(theta)
    y = r * torch.sin(theta)
    return torch.stack((x, y), dim=-1)

@torch.no_grad()
def cartesian_to_polar(cart: torch.Tensor) -> torch.Tensor:
    """
    输入: 形状 (..., 2) 的 tensor，其中 cart[..., 0] = x，cart[..., 1] = y。
    输出: 同形状的 tensor，其中 out[..., 0] = r，out[..., 1] = θ (弧度, 范围[-π, π))。
    """
    x, y = cart.unbind(-1)
    r = torch.sqrt(x**2 + y**2)          
    theta = torch.atan2(y, x)                  
    return torch.stack((r, theta), dim=-1)

@torch.no_grad()
def normalize_polar_point(
    polar: torch.Tensor,
    r_max: float = 1.0,
    theta_range: Tuple[float, float] = (-math.pi, math.pi)
) -> torch.Tensor:
    r, theta = polar.unbind(-1)
    theta_min, theta_max = theta_range

    r_norm = r / r_max
    theta_norm = (theta - theta_min) / (theta_max - theta_min)

    return torch.stack((r_norm, theta_norm), dim=-1)

@torch.no_grad()
def denormalize_polar_point(
    polar: torch.Tensor,
    r_max: float = 1.0,
    theta_range: Tuple[float, float] = (-math.pi, math.pi)
) -> torch.Tensor:
    r_norm, theta_norm = polar.unbind(-1)
    theta_min, theta_max = theta_range

    r = r_norm * r_max
    theta = theta_norm * (theta_max - theta_min) + theta_min

    return torch.stack((r, theta), dim=-1)

@torch.no_grad()
def denormalize_action_tensor(
    action_norm: torch.Tensor,
    max_distance: Union[float, torch.Tensor],
    theta_range: Tuple[float, float] = (-math.pi / 2, math.pi / 2)
) -> torch.Tensor:
    if action_norm.shape[-1] == 2:
        point_num = action_norm.shape[-2]
        pair = action_norm
    else:
        point_num = action_norm.shape[-1] // 2
        pair = action_norm.view(*action_norm.shape[:-1], point_num, 2)

    theta_min, theta_max = theta_range
    r_min = 0.0

    r_max_seq = torch.linspace(0.0, float(max_distance), point_num + 1,
                               dtype=pair.dtype, device=pair.device)[1:]
    r_max_seq = r_max_seq.view(*([1] * (pair.dim() - 2)), point_num)

    r = pair[..., 0] * (r_max_seq - r_min) + r_min
    theta = pair[..., 1] * (theta_max - theta_min) + theta_min

    return torch.stack((r, theta), dim=-1)

@torch.no_grad()
def normalize_action_tensor(
    control_points: torch.Tensor,
    max_distance: Union[float, torch.Tensor],
    flatten: bool = True,
    theta_range: Tuple[float, float] = (-math.pi / 2, math.pi / 2)
) -> torch.Tensor:
    if control_points.shape[-1] != 2:
        raise ValueError("最后一维必须是 (r, θ)。")

    point_num = control_points.shape[-2]
    r, theta = control_points.unbind(dim=-1)

    theta_min, theta_max = theta_range
    r_min = 0.0

    r_max_seq = torch.linspace(0.0, float(max_distance), point_num + 1,
                               dtype=control_points.dtype, device=control_points.device)[1:]
    r_max_seq = r_max_seq.view(*([1] * (control_points.dim() - 2)), point_num)

    r_norm = (r - r_min) / (r_max_seq - r_min)
    theta_norm = (theta - theta_min) / (theta_max - theta_min)

    pair_norm = torch.stack((r_norm, theta_norm), dim=-1)
    return pair_norm.view(*pair_norm.shape[:-2], point_num*2) if flatten else pair_norm

@torch.no_grad()
def batch_cubic_spline_interpolation(control_points_batch, num_points=20):
    """
    对于输入的控制点批次，进行三次样条插值。
    
    参数:
        control_points_batch (Tensor): shape (batch_size, num_control_points, 2)
        num_points (int): 输出的插值点数量（默认为21，类似 np.arange(0, 1.05, 0.05)）。
    
    返回:
        Tensor: 插值后的路径，shape (batch_size, num_points, 2)
    """
    batch_size, num_control_points, _ = control_points_batch.shape

    # 生成时间点 (0到1的均匀分布)
    time_points = torch.linspace(0, 1, control_points_batch.shape[1], device=control_points_batch.device)
    
    # 构建插值点
    output_points = []
    for i in range(batch_size):
        control_points = control_points_batch[i].cpu().numpy()
        time_points_np = time_points.cpu().numpy()
        
        # 构建x和y方向的CubicSpline插值
        spline_x = CubicSpline(time_points_np, control_points[:, 0])
        spline_y = CubicSpline(time_points_np, control_points[:, 1])
        
        # 在时间点上进行插值，生成路径
        interpolated_x = spline_x(np.linspace(0, 1, num_points))
        interpolated_y = spline_y(np.linspace(0, 1, num_points))
        
        # 将结果合并到一个batch结果中
        output_points.append(np.column_stack((interpolated_x, interpolated_y)))
    
    # 将列表转换为Tensor并返回
    return torch.tensor(np.stack(output_points), dtype=control_points_batch.dtype, device=control_points_batch.device)

@torch.no_grad()
def triangle_curvature(p0: torch.Tensor,
                        p1: torch.Tensor,
                        p2: torch.Tensor,
                        eps: float = 1e-8) -> torch.Tensor:
    """
        离散曲率 κ = 2 * |(p1-p0) × (p2-p1)| / (‖p1-p0‖ · ‖p2-p1‖ · ‖p2-p0‖)
        p0, p1, p2: (..., 2)
        返回: (...,)  (与输入批量维度一致)
    """
    v1 = p1 - p0            # (..., 2)
    v2 = p2 - p1
    v0 = p2 - p0

    # 平行四边形面积 = |v1 × v2|（这里是 2×三角形面积）
    cross = v1[..., 0] * v2[..., 1] - v1[..., 1] * v2[..., 0]
    area2 = torch.abs(cross)

    a = v1.norm(dim=-1)     # ‖p1-p0‖
    b = v2.norm(dim=-1)     # ‖p2-p1‖
    c = v0.norm(dim=-1)     # ‖p2-p0‖
    prod = a * b * c

    kappa = 2.0 * area2 / (prod + eps)   # 加 eps 保证数值稳定
    return kappa

@torch.no_grad()
def rotate_xy_tensor(
    pts: torch.Tensor,           # (..., D)   D ≥ 2
    theta: Union[float, torch.Tensor]  # broadcast-able 到批维的角度（弧度）
) -> torch.Tensor:
    """
    将张量 pts 的前两维 (x, y) 绕原点旋转 theta，其他维度保持不变。
    
    参数
    ----
    pts   : torch.Tensor [..., D]，最后一维 D ≥ 2
    theta : float 或 shape 与 pts 批维可广播的张量，单位：弧度
    
    返回
    ----
    torch.Tensor，与 pts 形状相同，(x, y) 已旋转
    """
    # 计算旋转矩阵，shape: (..., 2, 2)
    c, s = torch.cos(theta), torch.sin(theta)
    rot = torch.stack(
        (torch.stack((c, -s), dim=-1),
         torch.stack((s,  c), dim=-1)),
        dim=-2
    ).to(dtype=pts.dtype, device=pts.device)
    
    rot = rot.unsqueeze(-3)

    # 提取 (x, y)，做矩阵乘法后回填
    xy = pts[..., :2]                               # (..., 2)
    rot_xy = (xy.unsqueeze(-2) @ rot).squeeze(-2)   # (..., 2)

    out = pts.clone()
    out[..., :2] = rot_xy
    return out

# points_cartesian = torch.tensor([
#     [1.0, 0.0],
#     [0.5, 0.5],
#     [0.0, 1.0],
#     [-0.5, 0.5],
#     [-1.0, 0.0]
# ], dtype=torch.float32)

# points_polar = cartesian_to_polar(points_cartesian)
# print(f"Polar coordinates:\n{points_polar}")

# r_max = points_polar[..., 0].max().item()  # 计算 r_max
# points_polar_norm = normalize_polar_point(points_polar, r_max)
# print(f"r_max: {r_max}")
# print(f"Normalized polar coordinates:\n{points_polar_norm}")

# points_polar_denorm = denormalize_polar_point(points_polar_norm, r_max)
# print(f"Denormalized polar coordinates:\n{points_polar_denorm}")
# points_cartesian_reconstructed = polar_to_cartesian(points_polar_denorm)
# print(f"Reconstructed Cartesian coordinates:\n{points_cartesian_reconstructed}")
# print(torch.allclose(points_cartesian, points_cartesian_reconstructed, atol=1e-5))

# control_points_batch = points_cartesian_reconstructed.unsqueeze(0)  # 加 batch 维
# interp_path = batch_cubic_spline_interpolation(control_points_batch, num_points=50)

# p0, p1, p2 = interp_path[0, 10], interp_path[0, 25], interp_path[0, 40]
# curv = triangle_curvature(p0, p1, p2)
# print(f"Estimated curvature: {curv.item():.6f}")

# import matplotlib.pyplot as plt

# # 原始点
# plt.plot(points_cartesian[:, 0], points_cartesian[:, 1], 'ro-', label='Original')

# # 插值路径
# plt.plot(interp_path[0, :, 0].cpu(), interp_path[0, :, 1].cpu(), 'b-', label='Interpolated')

# plt.axis('equal')
# plt.legend()
# plt.title("Path Interpolation")
# plt.savefig("logs/plt/path_interpolation.png")