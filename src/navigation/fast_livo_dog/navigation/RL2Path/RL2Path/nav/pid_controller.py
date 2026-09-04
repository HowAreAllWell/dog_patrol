import torch
from typing import Tuple, Sequence, Union
from dataclasses import dataclass

@dataclass
class PPIDCfg:
    """参数配置 (IsaacLab 样式)."""

    # ───── 跟踪算法参数 ─────
    look_forward_gain: float = 0.0      # k   : 前视距离随速度增益
    look_ahead_base: float = 0.5        # Lf0 : 最小前视距离 [m]

    # ───── 纵向速度 P 控制 ─────
    kp_speed: float = 2.0               # Kp  : 速度 P 增益

    # ───── 车辆几何/目标速度 ─────
    wheel_base: float = 0.66            # L   : 轴距 [m]
    cruise_speed: float = 0.6           # v_c : 目标巡航速度 [m/s]

    # ───── 轮子参数 ─────
    wheel_radius: float = 0.1           # r   : 驱动轮半径 [m]

    # ───── 原地旋转 (pivot‑turn) 参数 ─────
    turn_in_place_omega: float = 1.0    # 期望机器人原地旋转角速度 ω [rad/s]
    turn_in_place_thresh: float = 1.0472  # |α| 超过 ~60° 触发 pivot [rad] (≈60°)


class State:
    """车辆在自身坐标系下的运动学状态"""

    __slots__ = ("x", "y", "yaw", "v")

    def __init__(self, x: float = 0.0, y: float = 0.0,
                 yaw: float = 0.0, v: float = 0.0) -> None:
        self.x = float(x)
        self.y = float(y)
        self.yaw = float(yaw)  # 航向角（弧度）
        self.v = float(v)      # 线速度  [m/s]

    # 仅用于调试打印
    def __repr__(self) -> str:  # pragma: no cover
        return (
            f"State(x={self.x:.2f}, y={self.y:.2f}, "
            f"yaw={torch.rad2deg(self.yaw):.2f}°, v={self.v:.2f})")


# ────────────────────────────── 工具函数 ──────────────────────────────── #

def _to_tensor(arr: Union[torch.Tensor, Sequence, float]) -> torch.Tensor:
    """确保输入转换为 `torch.float32` Tensor（保留原 device）"""
    if isinstance(arr, torch.Tensor):
        return arr.float()
    return torch.as_tensor(arr, dtype=torch.float32)


# ────────────────────────────── 基础函数 ──────────────────────────────── #

def pid_speed(target: float, current: float, kp: float) -> float:
    """简易 P 控制，返回纵向加速度 a = Kp * (v* - v)"""
    return kp * (target - current)


def calc_target_index(state: State,
                      cx: torch.Tensor,
                      cy: torch.Tensor,
                      look_forward_gain: float,
                      look_ahead_base: float) -> int:
    """寻找距离当前车辆前视距离 Lf 最近的路径点索引"""

    # ① 当前位置到所有路径点的欧氏距离
    dx = state.x - cx
    dy = state.y - cy
    dists = torch.sqrt(dx * dx + dy * dy)
    ind = int(torch.argmin(dists).item())

    # ② 依据速度动态增大前视距离 Lf = k·v + Lf0
    Lf = look_forward_gain * state.v + look_ahead_base
    traveled: float = 0.0
    while traveled < Lf and (ind + 1) < len(cx):
        seg_dx = float(cx[ind + 1] - cx[ind])
        seg_dy = float(cy[ind + 1] - cy[ind])
        traveled += torch.hypot(torch.tensor(seg_dx), torch.tensor(seg_dy))
        ind += 1

    return ind


def pure_pursuit_control(state: State,
                         cx: torch.Tensor,
                         cy: torch.Tensor,
                         prev_ind: int,
                         cfg: PPIDCfg,
                         accel_cmd: float = 0.0) -> Tuple[float, int, torch.Tensor]:
    """纯跟踪解算方向盘转角 δ

    返回 (δ, ind, α)，其中 α 为车辆朝向与目标向量夹角。
    """

    ind = calc_target_index(state, cx, cy,
                            cfg.look_forward_gain, cfg.look_ahead_base)
    ind = max(ind, prev_ind)  # 保证索引单调递增

    # 取目标点坐标
    if ind >= len(cx):
        tx, ty = cx[-1], cy[-1]
    else:
        tx, ty = cx[ind], cy[ind]

    # 确保 tx 和 ty 是 Tensor
    tx = torch.as_tensor(tx, dtype=torch.float32).clone().detach()
    ty = torch.as_tensor(ty, dtype=torch.float32).clone().detach()

        # 计算夹角 α，并规范化至 (-π, π]
    alpha = torch.atan2(ty - state.y, tx - state.x) - state.yaw
    alpha = torch.atan2(torch.sin(alpha), torch.cos(alpha))  # wrap angle

    # 倒车场景修正
    if state.v < 0 or (state.v == 0 and accel_cmd < 0):
        alpha = torch.pi - alpha

    # 重新计算 Lf (前视距离) — 倒车时前视方向取负基准
    Lf = cfg.look_forward_gain * state.v + (
        -cfg.look_ahead_base if state.v < 0 else cfg.look_ahead_base)

    # Pure‑Pursuit 几何关系
    delta = torch.atan2(2.0 * cfg.wheel_base * torch.sin(alpha) / Lf, torch.tensor(1.0))

    return delta, ind, alpha


def unicycle_to_diff(v: float, omega: torch.Tensor,
                     B: float, R: float) -> Tuple[torch.Tensor, torch.Tensor]:
    """线速度 v 与角速度 ω → 左/右轮角速度 (rad/s)"""
    v_l = v - omega * B / 2.0
    v_r = v + omega * B / 2.0
    return v_l / R, v_r / R


# ────────────────────────────── 对外接口 ──────────────────────────────── #

def follow_path_speed(
    current_speeds: Union[torch.Tensor, Sequence[float]],
    current_poses: Union[torch.Tensor, Sequence[Sequence[float]]],
    path: Union[torch.Tensor, Sequence],
    cfg: PPIDCfg,
) -> Tuple[torch.Tensor, torch.Tensor]:
    """纯跟踪 + 恒速控制 (支持批量)，并在目标处于后方时先原地旋转。

    兼容两种路径输入尺寸：
      1. **共享路径** → `(N, 2)`
      2. **每机器人独立路径** → `(B, N, 2)`

    参数:
        current_speeds: 标量或 `Tensor[B]`
        current_poses : `(B,3)` 或 `(3,)`
        path          : `(N,2)` *或* `(B,N,2)`
        cfg           : PID 配置

    返回:
        (omega_l, omega_r): `Tensor[B]` (或标量)
    """

    # 将输入规范为 Tensor
    speeds_t = _to_tensor(current_speeds).view(-1)        # [B]
    poses_t = _to_tensor(current_poses).view(-1, 3)       # [B,3]
    path_t = _to_tensor(path)                             # [*,N,2] or [N,2]

    # 判定路径是否带 Batch 维
    path_has_batch = path_t.dim() == 3  # True -> (B,N,2)

    # print(f"path_has_batch: {path_has_batch}")

    batch_size = speeds_t.shape[0]
    omega_l_list, omega_r_list = [], []

    for i in range(batch_size):
        # 选取该机器人的轨迹
        if path_has_batch:
            cx, cy = path_t[i, :, 0], path_t[i, :, 1]  # [N]
        else:
            cx, cy = path_t[:, 0], path_t[:, 1]

        state = State(v=float(speeds_t[i].item()),
                      x=float(poses_t[i, 0].item()),
                      y=float(poses_t[i, 1].item()),
                      yaw=float(poses_t[i, 2].item()))

        # Pure‑Pursuit 计算及朝向判断
        target_ind = calc_target_index(state, cx, cy,
                                       cfg.look_forward_gain, cfg.look_ahead_base)
        delta, _, alpha = pure_pursuit_control(state, cx, cy, target_ind, cfg)

        # ── 后方：原地旋转 ──
        if torch.abs(alpha) > cfg.turn_in_place_thresh:
            # 目标在后方，先旋转
            omega_robot = cfg.turn_in_place_omega * torch.sign(alpha)
            v_l = -omega_robot * cfg.wheel_base / 2.0
            v_r = -v_l
            ol = v_l / cfg.wheel_radius
            or_ = v_r / cfg.wheel_radius
        else:
            # 正常跟踪
            omega = cfg.cruise_speed * torch.tan(delta) / cfg.wheel_base
            ol, or_ = unicycle_to_diff(cfg.cruise_speed, omega, cfg.wheel_base, cfg.wheel_radius)

        omega_l_list.append(torch.as_tensor(ol, dtype=torch.float32, device=speeds_t.device))
        omega_r_list.append(torch.as_tensor(or_, dtype=torch.float32, device=speeds_t.device))

    omega_l = torch.stack(omega_l_list)
    omega_r = torch.stack(omega_r_list)

    # print(omega_l.shape, omega_r.shape)

    # 若输入为标量形式，则返回标量
    if omega_l.numel() == 1:
        return omega_l.squeeze(0), omega_r.squeeze(0)

    return omega_l, omega_r
