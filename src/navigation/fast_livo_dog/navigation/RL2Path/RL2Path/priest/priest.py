from dataclasses import dataclass
from typing import Optional, Union

import numpy as np
import torch

from RL2Path.priest.priest_core import PriestPlannerCore

TensorLike = Union[np.ndarray, torch.Tensor]


def _copy_tensor_like(value):
    if isinstance(value, torch.Tensor):
        return value.clone()
    return value.copy()


def _to_numpy(value):
    if isinstance(value, torch.Tensor):
        return value.detach().cpu().numpy()
    return np.asarray(value)

@dataclass
class OptimizationPacket:
    initial_state: torch.Tensor
    dynamic_obstacle_x_positions: TensorLike
    dynamic_obstacle_y_positions: TensorLike
    dynamic_obstacle_x_velocities: TensorLike
    dynamic_obstacle_y_velocities: TensorLike
    static_obstacle_x_positions: TensorLike
    static_obstacle_y_positions: TensorLike
    x_waypoint: TensorLike
    y_waypoint: TensorLike
    arc_vec: TensorLike
    x_diff: TensorLike
    y_diff: TensorLike
    custom_x_coefficients: Optional[TensorLike] = None
    custom_y_coefficients: Optional[TensorLike] = None

    def __post_init__(self):
        self.initial_state = _copy_tensor_like(self.initial_state)
        self.dynamic_obstacle_x_positions = _copy_tensor_like(self.dynamic_obstacle_x_positions)
        self.dynamic_obstacle_y_positions = _copy_tensor_like(self.dynamic_obstacle_y_positions)
        self.dynamic_obstacle_x_velocities = _copy_tensor_like(self.dynamic_obstacle_x_velocities)
        self.dynamic_obstacle_y_velocities = _copy_tensor_like(self.dynamic_obstacle_y_velocities)
        self.static_obstacle_x_positions = _copy_tensor_like(self.static_obstacle_x_positions)
        self.static_obstacle_y_positions = _copy_tensor_like(self.static_obstacle_y_positions)
        self.x_waypoint = _copy_tensor_like(self.x_waypoint)
        self.y_waypoint = _copy_tensor_like(self.y_waypoint)
        self.arc_vec = _copy_tensor_like(self.arc_vec)
        self.x_diff = _copy_tensor_like(self.x_diff)
        self.y_diff = _copy_tensor_like(self.y_diff)
        if self.custom_x_coefficients is not None:
            self.custom_x_coefficients = _copy_tensor_like(self.custom_x_coefficients)
        if self.custom_y_coefficients is not None:
            self.custom_y_coefficients = _copy_tensor_like(self.custom_y_coefficients)


class PriestPlanner:
    def __init__(
        self,
        num_dynamic_obstacles=10,
        num_static_obstacles=100,
        time_horizon=5,
        trajectory_length=50,
        max_velocity=1.0,
        min_velocity=0.0,
        max_acceleration=1.0,
        max_inner_iterations=2,
        max_outer_iterations=2,
        tracking_weight=0.1,
        smoothness_weight=0.2,
        static_obstacle_semi_minor_axis=0.5,
        static_obstacle_semi_major_axis=0.5,
        dynamic_obstacle_semi_minor_axis=0.68,
        dynamic_obstacle_semi_major_axis=0.68,
        trajectory_batch_size=110,
        desired_velocity=1.0,
        num_waypoints=1000,
        device: Optional[Union[str, torch.device]] = None,
    ):
        self.device = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))
        self.num_dynamic_obstacles = num_dynamic_obstacles
        self.num_static_obstacles = num_static_obstacles
        self.time_horizon, self.trajectory_length, self.trajectory_batch_size = (
            time_horizon,
            trajectory_length,
            trajectory_batch_size,
        )
        self.desired_velocity = desired_velocity
        self.num_waypoints = num_waypoints

        self.priest = PriestPlannerCore(
            static_obstacle_semi_minor_axis,
            static_obstacle_semi_major_axis,
            dynamic_obstacle_semi_minor_axis,
            dynamic_obstacle_semi_major_axis,
            max_velocity,
            min_velocity,
            max_acceleration,
            self.num_static_obstacles,
            self.num_dynamic_obstacles,
            self.time_horizon,
            self.trajectory_length,
            self.trajectory_batch_size,
            max_inner_iterations,
            max_outer_iterations,
            smoothness_weight,
            tracking_weight,
            self.num_waypoints,
            self.desired_velocity,
            self.device,
        )

        self.key = 0
        self.x_waypoint, self.y_waypoint = None, None

    def _as_tensor(self, value) -> torch.Tensor:
        if isinstance(value, torch.Tensor):
            return value.to(device=self.device, dtype=torch.float32)
        return torch.as_tensor(value, device=self.device, dtype=torch.float32)

    def _add_padding(
        self, array: TensorLike, max_size: int, padding_value: float
    ) -> torch.Tensor:
        input_array = _to_numpy(array).reshape(-1)
        output = np.ones(max_size, dtype=np.float32) * np.float32(padding_value)
        output[: min(input_array.shape[0], max_size)] = input_array[:max_size]
        return self._as_tensor(output)

    def optimize(
        self,
        optimization_packet: OptimizationPacket,
    ):
        optimization_packet.initial_state = self._as_tensor(optimization_packet.initial_state)
        optimization_packet.x_waypoint = self._as_tensor(optimization_packet.x_waypoint)
        optimization_packet.y_waypoint = self._as_tensor(optimization_packet.y_waypoint)
        optimization_packet.arc_vec = self._as_tensor(optimization_packet.arc_vec)
        optimization_packet.x_diff = self._as_tensor(optimization_packet.x_diff)
        optimization_packet.y_diff = self._as_tensor(optimization_packet.y_diff)
        if optimization_packet.custom_x_coefficients is not None:
            optimization_packet.custom_x_coefficients = self._as_tensor(
                optimization_packet.custom_x_coefficients
            )
        if optimization_packet.custom_y_coefficients is not None:
            optimization_packet.custom_y_coefficients = self._as_tensor(
                optimization_packet.custom_y_coefficients
            )

        (
            x_obs_trajectory,
            y_obs_trajectory,
            x_obs_trajectory_proj,
            y_obs_trajectory_proj,
            x_obs_trajectory_dy,
            y_obs_trajectory_dy,
        ) = self.priest.compute_obs_traj_prediction(
            self._add_padding(
                optimization_packet.dynamic_obstacle_x_positions,
                self.num_dynamic_obstacles,
                1000,
            ),
            self._add_padding(
                optimization_packet.dynamic_obstacle_y_positions,
                self.num_dynamic_obstacles,
                1000,
            ),
            self._add_padding(
                optimization_packet.dynamic_obstacle_x_velocities,
                self.num_dynamic_obstacles,
                0,
            ),
            self._add_padding(
                optimization_packet.dynamic_obstacle_y_velocities,
                self.num_dynamic_obstacles,
                0,
            ),
            self._add_padding(
                optimization_packet.static_obstacle_x_positions,
                self.num_static_obstacles,
                1000,
            ),
            self._add_padding(
                optimization_packet.static_obstacle_y_positions,
                self.num_static_obstacles,
                1000,
            ),
            torch.zeros(self.num_static_obstacles, device=self.device, dtype=torch.float32),
            torch.zeros(self.num_static_obstacles, device=self.device, dtype=torch.float32),
            optimization_packet.initial_state[0],
            optimization_packet.initial_state[1],
        )

        if (
            optimization_packet.custom_x_coefficients is not None
            and optimization_packet.custom_y_coefficients is not None
        ):
            self.priest.ellite_num_const = (
                optimization_packet.custom_x_coefficients.shape[0]
            )
            if self.priest.ellite_num_const != self.trajectory_batch_size:
                raise ValueError(
                    "custom coefficient batch size must match trajectory_batch_size "
                    f"({self.priest.ellite_num_const} != {self.trajectory_batch_size})"
                )
            # self.priest.initial_up_sampling = 1

            traj_guess = self.priest.compute_traj_guess_from_coefficients(
                # optimization_packet.initial_state,
                # self.desired_velocity,
                optimization_packet.x_waypoint,
                optimization_packet.y_waypoint,
                optimization_packet.custom_x_coefficients,
                optimization_packet.custom_y_coefficients,
                # optimization_packet.arc_vec,
            )
        else:
            x_guess_per, y_guess_per = self.priest.compute_warm_traj(
                optimization_packet.initial_state,
                self.desired_velocity,
                self.x_waypoint,
                self.y_waypoint,
                self.arc_vec,
                self.x_diff,
                self.y_diff,
            )
            traj_guess = self.priest.compute_traj_guess(
                optimization_packet.initial_state,
                x_obs_trajectory,
                y_obs_trajectory,
                x_obs_trajectory_dy,
                y_obs_trajectory_dy,
                self.desired_velocity,
                optimization_packet.x_waypoint,
                optimization_packet.y_waypoint,
                optimization_packet.arc_vec,
                x_guess_per,
                y_guess_per,
                optimization_packet.x_diff,
                optimization_packet.y_diff,
            )

        (
            sol_x_bar,
            sol_y_bar,
            x_guess,
            y_guess,
            xdot_guess,
            ydot_guess,
            xddot_guess,
            yddot_guess,
            c_mean,
            c_cov,
            x_fin,
            y_fin,
        ) = traj_guess

        lamda_x = torch.zeros(
            (self.trajectory_batch_size, self.priest.nvar),
            device=self.device,
            dtype=torch.float32,
        )
        lamda_y = torch.zeros(
            (self.trajectory_batch_size, self.priest.nvar),
            device=self.device,
            dtype=torch.float32,
        )

        x_elite, y_elite, c_x_elite, c_y_elite, idx_min = self.priest.compute_cem(
            self.key,
            optimization_packet.initial_state,
            x_fin,
            y_fin,
            lamda_x,
            lamda_y,
            x_obs_trajectory,
            y_obs_trajectory,
            x_obs_trajectory_proj,
            y_obs_trajectory_proj,
            x_obs_trajectory_dy,
            y_obs_trajectory_dy,
            sol_x_bar,
            sol_y_bar,
            x_guess,
            y_guess,
            xdot_guess,
            ydot_guess,
            xddot_guess,
            yddot_guess,
            optimization_packet.x_waypoint,
            optimization_packet.y_waypoint,
            optimization_packet.arc_vec,
            c_mean,
            c_cov,
        )

        c_x_best = c_x_elite[idx_min]
        c_y_best = c_y_elite[idx_min]
        x_best = x_elite[idx_min]
        y_best = y_elite[idx_min]

        return (
            _to_numpy(c_x_best),
            _to_numpy(c_y_best),
            _to_numpy(x_best),
            _to_numpy(y_best),
            _to_numpy(c_x_elite),
            _to_numpy(c_y_elite),
            _to_numpy(x_elite),
            _to_numpy(y_elite),
            int(idx_min.detach().cpu().item() if isinstance(idx_min, torch.Tensor) else idx_min),
        )

    def update_waypoints(
        self,
        initial_x_position: float,
        initial_y_position: float,
        goal_x_position: float,
        goal_y_position: float,
        custom_x_waypoint: Optional[TensorLike] = None,
        custom_y_waypoint: Optional[TensorLike] = None,
        use_eager_waypoints: bool = True,
    ):
        if custom_x_waypoint is not None and custom_y_waypoint is not None:
            custom_x_waypoint = self._as_tensor(custom_x_waypoint)
            custom_y_waypoint = self._as_tensor(custom_y_waypoint)
            # extrapolation_length = 1
            extrapolation_length = self.time_horizon * self.desired_velocity
            extrapolation_steps = 20

            # Extrapolate waypoints when close to goal in the case of eager waypoints
            if use_eager_waypoints:
                current_arc_length, _, _, _ = self.priest.path_spline(
                    custom_x_waypoint, custom_y_waypoint
                )
                current_arc_length_value = float(current_arc_length.detach().cpu().item())
                num_waypoints_before_threshold = self.num_waypoints
                if current_arc_length_value < self.time_horizon * self.desired_velocity:
                    num_waypoints_before_threshold = int(
                        self.num_waypoints
                        * current_arc_length_value
                        / (self.time_horizon * self.desired_velocity)
                    )
                    extrapolation_length -= current_arc_length_value

                extrapolation_steps = (
                    self.num_waypoints - num_waypoints_before_threshold
                )

            # angle_index = int((custom_x_waypoint.shape[0] - 1) * 0.9)
            angle_index = -2
            extrapolation_angle = torch.atan2(
                custom_y_waypoint[-1] - custom_y_waypoint[angle_index],
                custom_x_waypoint[-1] - custom_x_waypoint[angle_index],
            )
            x_extend_start = float(custom_x_waypoint[-1].detach().cpu().item())
            y_extend_start = float(custom_y_waypoint[-1].detach().cpu().item())
            x_extend_end = float(
                (
                    custom_x_waypoint[-1]
                    + extrapolation_length * torch.cos(extrapolation_angle)
                )
                .detach()
                .cpu()
                .item()
            )
            y_extend_end = float(
                (
                    custom_y_waypoint[-1]
                    + extrapolation_length * torch.sin(extrapolation_angle)
                )
                .detach()
                .cpu()
                .item()
            )

            custom_x_waypoint = torch.cat(
                [
                    custom_x_waypoint[:-1],
                    torch.linspace(
                        x_extend_start,
                        x_extend_end,
                        extrapolation_steps,
                        device=self.device,
                        dtype=torch.float32,
                    ),
                ]
            )
            custom_y_waypoint = torch.cat(
                [
                    custom_y_waypoint[:-1],
                    torch.linspace(
                        y_extend_start,
                        y_extend_end,
                        extrapolation_steps,
                        device=self.device,
                        dtype=torch.float32,
                    ),
                ]
            )

            self.num_waypoints = int(custom_x_waypoint.shape[0])
            self.x_waypoint = custom_x_waypoint
            self.y_waypoint = custom_y_waypoint
        else:
            theta_des = np.arctan2(
                goal_y_position - initial_y_position,
                goal_x_position - initial_x_position,
            )
            theta_des_t = self._as_tensor(theta_des)
            x_waypoint_end = float(
                (
                    self._as_tensor(
                        initial_x_position if use_eager_waypoints else goal_x_position
                    )
                    + (self.time_horizon * self.desired_velocity) * torch.cos(theta_des_t)
                )
                .detach()
                .cpu()
                .item()
            )
            y_waypoint_end = float(
                (
                    self._as_tensor(
                        initial_y_position if use_eager_waypoints else goal_y_position
                    )
                    + (self.time_horizon * self.desired_velocity) * torch.sin(theta_des_t)
                )
                .detach()
                .cpu()
                .item()
            )
            self.x_waypoint = torch.linspace(
                float(initial_x_position),
                x_waypoint_end,
                self.num_waypoints if use_eager_waypoints else 1000,
                device=self.device,
                dtype=torch.float32,
            )
            self.y_waypoint = torch.linspace(
                float(initial_y_position),
                y_waypoint_end,
                self.num_waypoints if use_eager_waypoints else 1000,
                device=self.device,
                dtype=torch.float32,
            )

        self.arc_length, self.arc_vec, self.x_diff, self.y_diff = (
            self.priest.path_spline(self.x_waypoint, self.y_waypoint)
        )

    @torch.inference_mode()
    def run_optimization(
        self,
        initial_x_position: float,
        initial_y_position: float,
        initial_x_velocity: float,
        initial_y_velocity: float,
        initial_x_acceleration: float,
        initial_y_acceleration: float,
        goal_x_position: float,
        goal_y_position: float,
        dynamic_obstacle_x_positions: Optional[np.ndarray],
        dynamic_obstacle_y_positions: Optional[np.ndarray],
        dynamic_obstacle_x_velocities: Optional[np.ndarray],
        dynamic_obstacle_y_velocities: Optional[np.ndarray],
        static_obstacle_x_positions: Optional[np.ndarray],
        static_obstacle_y_positions: Optional[np.ndarray],
        custom_x_waypoint: Optional[np.ndarray] = None,
        custom_y_waypoint: Optional[np.ndarray] = None,
        custom_x_coefficients: Optional[np.ndarray] = None,
        custom_y_coefficients: Optional[np.ndarray] = None,
        update_waypoints: bool = True,
        use_eager_waypoints: bool = True,
    ):
        if dynamic_obstacle_x_positions is None:
            dynamic_obstacle_x_positions = np.ones(self.num_dynamic_obstacles) * 1000
        if dynamic_obstacle_y_positions is None:
            dynamic_obstacle_y_positions = np.ones(self.num_dynamic_obstacles) * 1000
        if dynamic_obstacle_x_velocities is None:
            dynamic_obstacle_x_velocities = np.zeros(self.num_dynamic_obstacles)
        if dynamic_obstacle_y_velocities is None:
            dynamic_obstacle_y_velocities = np.zeros(self.num_dynamic_obstacles)
        if static_obstacle_x_positions is None:
            static_obstacle_x_positions = np.ones(self.num_static_obstacles) * 1000
        if static_obstacle_y_positions is None:
            static_obstacle_y_positions = np.ones(self.num_static_obstacles) * 1000

        initial_state = self._as_tensor(
            [
                initial_x_position,
                initial_y_position,
                initial_x_velocity,
                initial_y_velocity,
                initial_x_acceleration,
                initial_y_acceleration,
            ]
        )

        if update_waypoints:
            self.update_waypoints(
                initial_x_position=initial_x_position,
                initial_y_position=initial_y_position,
                goal_x_position=goal_x_position,
                goal_y_position=goal_y_position,
                custom_x_waypoint=custom_x_waypoint,
                custom_y_waypoint=custom_y_waypoint,
                use_eager_waypoints=use_eager_waypoints,
            )

        assert self.x_waypoint is not None and self.y_waypoint is not None

        return self.optimize(
            OptimizationPacket(
                initial_state=initial_state,
                dynamic_obstacle_x_positions=dynamic_obstacle_x_positions,
                dynamic_obstacle_y_positions=dynamic_obstacle_y_positions,
                dynamic_obstacle_x_velocities=dynamic_obstacle_x_velocities,
                dynamic_obstacle_y_velocities=dynamic_obstacle_y_velocities,
                static_obstacle_x_positions=static_obstacle_x_positions,
                static_obstacle_y_positions=static_obstacle_y_positions,
                x_waypoint=self.x_waypoint,
                y_waypoint=self.y_waypoint,
                arc_vec=self.arc_vec,
                x_diff=self.x_diff,
                y_diff=self.y_diff,
                custom_x_coefficients=custom_x_coefficients,
                custom_y_coefficients=custom_y_coefficients,
            )
        )
        
        

def visualize_paths(x_rl, y_rl, x_optimized, y_optimized):
    import matplotlib.pyplot as plt
    
    plt.figure(figsize=(10, 6))

    # 绘制 RL 路径（优化前）
    plt.plot(x_rl, y_rl, label='RL 原始路径', linestyle='--', linewidth=2, color='blue')

    # 绘制优化后的路径
    plt.plot(x_optimized, y_optimized, label='优化后路径', linestyle='-', linewidth=2.5, color='red')

    # 起点终点
    plt.scatter(x_rl[0], y_rl[0], color='green', marker='o', s=100, label='起点')
    plt.scatter(x_rl[-1], y_rl[-1], color='black', marker='x', s=100, label='终点')

    # 图形美化
    plt.title('RL 路径优化前后对比', fontsize=16)
    plt.xlabel('X 坐标', fontsize=12)
    plt.ylabel('Y 坐标', fontsize=12)
    plt.legend(fontsize=12)
    plt.grid(True)
    plt.axis('equal')  # 保持坐标比例一致
    plt.tight_layout()
    plt.savefig('logs/plt/rl_path_optimization_comparison.png', dpi=300)

if __name__=="__main__":
    # demo.py

    # 1) 初始化 planner（torch 会自动选择 CUDA；无 CUDA 时使用 CPU）
    planner = PriestPlanner(
        num_dynamic_obstacles=10,
        num_static_obstacles=100,
        time_horizon=5.0,
        trajectory_length=50,
        trajectory_batch_size=110,
        desired_velocity=1.0,
        num_waypoints=400,   # 你的 RL 路径点数可以不同，下面会直接传 custom
    )

    # 2) 伪造一条 RL 局部路径（真实环境下换成你的 RL 输出）
    t = np.linspace(0, 1, 300)
    x_rl = 2.0 * t
    y_rl = 1.0 * np.sin(2*np.pi*t)  # 举例：稍微起伏的曲线

    # 3) 没有障碍时可以传 None；有的话传长度匹配的向量
    result = planner.run_optimization(
        initial_x_position=x_rl[0],
        initial_y_position=y_rl[0],
        initial_x_velocity=0.0,
        initial_y_velocity=0.0,
        initial_x_acceleration=0.0,
        initial_y_acceleration=0.0,
        goal_x_position=x_rl[-1],
        goal_y_position=y_rl[-1],
        dynamic_obstacle_x_positions=None,
        dynamic_obstacle_y_positions=None,
        dynamic_obstacle_x_velocities=None,
        dynamic_obstacle_y_velocities=None,
        static_obstacle_x_positions=None,
        static_obstacle_y_positions=None,
        custom_x_waypoint=x_rl,       # 直接塞 RL 路径
        custom_y_waypoint=y_rl,
        custom_x_coefficients=None,   # 本例不走系数路线
        custom_y_coefficients=None,
        update_waypoints=True,
        use_eager_waypoints=True,     # 弧长不够时自动外推
    )

    (c_x_best, c_y_best, x_best, y_best,
    c_x_elite, c_y_elite, x_elite, y_elite, idx_min) = result

    print('best coeff shapes:', c_x_best.shape, c_y_best.shape)  # (11,), (11,)
    print('best traj shapes:', x_best.shape, y_best.shape)        # (trajectory_length,)
    
    visualize_paths(x_rl, y_rl, x_best, y_best)
