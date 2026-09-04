# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import math
import pickle
import torch
from collections.abc import Sequence

import isaaclab.sim as sim_utils
import isaaclab.utils.math as math_utils

from isaaclab.assets import Articulation, RigidObject
from isaaclab.envs import DirectRLEnv
from isaaclab.sim.spawners.from_files import GroundPlaneCfg, spawn_ground_plane
from isaaclab.utils.math import sample_uniform
from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from isaaclab.sim.schemas import CollisionPropertiesCfg, RigidBodyPropertiesCfg, activate_contact_sensors
from isaaclab.sim.spawners.from_files import spawn_from_usd
from isaaclab.sensors import RayCaster, ContactSensor
from isaaclab.utils.math import euler_xyz_from_quat, subtract_frame_transforms

from scipy.interpolate import CubicSpline
from .rl2path_env_cfg import Rl2pathEnvCfg
from RL2Path.nav.cost_map_2d import Costmap_2d
from RL2Path.nav.grid_2d import Grid
from RL2Path.nav.pid_controller import follow_path_speed
from RL2Path.nav.rl_controller import RLControllerInterface
from RL2Path.utils.math_tools import *

def define_markers() -> VisualizationMarkers:
    """只定义目标点的可视化标记 (目标点小球)。"""
    marker_cfg = VisualizationMarkersCfg(
        prim_path="/Visuals/myMarkers",
        markers={
            "sphere": sim_utils.SphereCfg(
                radius=0.05,
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 1.0, 0.0)),
            ),
            "dot": sim_utils.SphereCfg(
                radius=0.02,
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 1.0))  # 粉红色小球
            ),
        },
    )
    return VisualizationMarkers(cfg=marker_cfg)

def generate_sine_paths(
    start: float,
    end: float,
    num_points: int,
    amplitudes: torch.Tensor,
    frequencies: torch.Tensor
) -> torch.Tensor:
    B = amplitudes.shape[0]  # Batch size

    # Generate shared x-values
    x_vals = torch.linspace(start, end, num_points)  # (num_points,)
    x_vals = x_vals.unsqueeze(0).expand(B, -1)       # (B, num_points)

    # Compute y-values for each batch using broadcasting
    y_vals = amplitudes.unsqueeze(1) * torch.sin(frequencies.unsqueeze(1) * x_vals)  # (B, num_points)

    # Stack x and y to shape (B, num_points, 2)
    paths = torch.stack([x_vals, y_vals], dim=2)

    return paths

class Rl2pathEnv(DirectRLEnv):
    cfg: Rl2pathEnvCfg

    def __init__(self, cfg: Rl2pathEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        # self.cfg: Rl2pathEnvCfg = cfg
        if self.cfg.robot_name == "CF2X":
            self._robot_mass = self.robot.root_physx_view.get_masses()[0].sum()
            self._gravity_magnitude = torch.tensor(self.sim.cfg.gravity, device=self.cfg.sim.device).norm()
            self._robot_weight = (self._robot_mass * self._gravity_magnitude).item()
            self._body_id = self.robot.find_bodies("body")[0]
            
            self._thrust = torch.zeros(self.num_envs, 1, 3, device=self.device)
            self._moment = torch.zeros(self.num_envs, 1, 3, device=self.device)
            self._desired_pos_w = torch.zeros(self.num_envs, 3, device=self.device)
        elif self.cfg.robot_name == "Jetbot" or self.cfg.robot_name == "Create3":
            self.dof_idx, _ = self.robot.find_joints(self.cfg.dof_names)

    def _setup_scene(self):
        # add ground plane
        spawn_ground_plane(prim_path=self.cfg.ground_prim_path, cfg=GroundPlaneCfg())
        
        # spawn_from_usd(prim_path=self.cfg.obstacle_prim_path, cfg=self.cfg.obstacle_cfg, translation=(0.0, 0.0, 0.0))
        self.obstacle = RigidObject(self.cfg.obstacle_cfg)
        spawn_from_usd(prim_path=self.cfg.static_obstacle_prim_path, cfg=self.cfg.static_obstacle_cfg, translation=(0.0, 0.0, 0.0))

        self.robot = Articulation(self.cfg.robot_cfg, )
        self.lidar = RayCaster(self.cfg.lidar_cfg)
        self.contact_CH = ContactSensor(self.cfg.contact_CH)
        
        # clone and replicate
        self.scene.clone_environments(copy_from_source=False)
        # add articulation to scene
        self.scene.rigid_objects["obstacle"] = self.obstacle
        self.scene.articulations["robot"] = self.robot
        self.scene.sensors["lidar"] = self.lidar
        self.scene.sensors["contact_CH"] = self.contact_CH
        
        if self.cfg.robot_name == "Jetbot" :
            self.contact_LW = ContactSensor(self.cfg.contact_LW)
            self.contact_RW = ContactSensor(self.cfg.contact_RW)
            self.scene.sensors["contact_LW"] = self.contact_LW
            self.scene.sensors["contact_RW"] = self.contact_RW
        elif self.cfg.robot_name == "CF2X":
            self.rl_controller = RLControllerInterface(
                cfg_path=self.cfg.rc_cfg_path,
                checkpoint_path=self.cfg.rc_ckpt_path,
                obs_space=12,
                act_space=4
            )
        elif self.cfg.robot_name == "Create3":
            pass
            
        # add lights
        light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
        light_cfg.func("/World/Light", light_cfg)

        self.visualization_markers = define_markers()

        # nav utils
        self.costmap = Costmap_2d(self.cfg.costmap_cfg)
        with open(self.cfg.grid_pkl_path, "rb") as f:
            self.grid: Grid = pickle.load(f)

        self.collision = torch.tensor(self.scene.num_envs * [True], device=self.cfg.sim.device)
        self.v_prefer = torch.tensor(self.cfg.v_prefer, device=self.cfg.sim.device, dtype=torch.float32).view(1)  # shape (1,)

        # setting aside useful variables for later
        self.goal_locations = torch.zeros((self.cfg.scene.num_envs, 3), device=self.cfg.sim.device)
        self.goal_orientations = torch.zeros((self.cfg.scene.num_envs, 4), device=self.cfg.sim.device)
        self.goal_orientations[:, 0] = 1.0    

    def _visualize_markers(self):
        """渲染目标点 + 路径点（小球）"""
        # ------------------- 目标点 marker -------------------
        goal_loc = self.goal_locations + torch.tensor([0.0, 0.0, 0.2], device=self.cfg.sim.device)
        goal_rot = self.goal_orientations
        goal_indices = torch.zeros(self.cfg.scene.num_envs, dtype=torch.long, device=self.cfg.sim.device)  # "sphere" 索引 0

        # ------------------- 路径点 marker（dot） -------------------
        # actions: (B, N, 2)
        B, N, _ = self.paths.shape
        dot_loc = self.paths.clone()                        # (B, N, 2)
        z = torch.ones((B, N), device=self.cfg.sim.device) * 0.1           # 给定 z 高度
        dot_loc = torch.cat([dot_loc, z.unsqueeze(-1)], dim=-1)  # (B, N, 3)
        dot_loc = dot_loc.reshape(-1, 3)                      # (B*N, 3)

        dot_rot = torch.zeros((dot_loc.shape[0], 4), device=self.cfg.sim.device)
        dot_rot[:, 0] = 1.0                                   # 单位四元数
        dot_indices = torch.ones(dot_loc.shape[0], dtype=torch.long, device=self.cfg.sim.device)  # 全部是 "dot"，索引 1

        # ------------------- 合并 -------------------
        all_locs = torch.cat([goal_loc, dot_loc], dim=0)
        all_rots = torch.cat([goal_rot, dot_rot], dim=0)
        all_indices = torch.cat([goal_indices, dot_indices], dim=0)

        self.visualization_markers.visualize(all_locs, all_rots, marker_indices=all_indices)

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        self.actions = actions.clone()
        
        self.paths = denormalize_action_tensor(actions.clone(), max_distance=self.cfg.action_range[1], theta_range= self.cfg.action_theta_range)
        self.paths = polar_to_cartesian(self.paths)
        start_points = torch.zeros((self.cfg.scene.num_envs, 1, 2), device=self.cfg.sim.device, dtype=torch.float32)
        if not self.cfg.action_with_goal:
            # print(f"start_points shape: {start_points.shape}")
            # print(f"self.paths shape: {self.paths.shape}")
            # print(f"self.relative_goals shape: {self.relative_goals.shape}")
            self.paths = torch.cat([start_points, self.paths, self.relative_goals.unsqueeze(1)], dim=1)  
            
            # print(f"actions pre step shape: {self.paths.shape}")
            
        else:
            self.paths = torch.cat([start_points, self.paths], dim=1)  
        self.paths = batch_cubic_spline_interpolation(self.paths, num_points=self.cfg.num_points)
        
        _, _, yaw = euler_xyz_from_quat(self.robot.data.root_quat_w)
        
        # self.paths = rotate_xy_tensor(self.paths, -yaw)  # 将动作从局部坐标系转换到全局坐标系
        
        # self.costmap.debug_plot(agent_idx=0, inflated=True, goals=self.relative_goals, actions=self.paths)
        
        # to global coordinates
        robot_pos = self.robot.data.root_pos_w[..., :2]
        self.paths = rotate_xy_tensor(self.paths, -yaw)  # 将动作从局部坐标系转换到全局坐标系
        self.paths = rotate_xy_tensor(self.paths, -self.delta_yaw)
        self.paths = self.paths + robot_pos.unsqueeze(1)  # (B, num_points, 2)

        # print(self.paths.shape)
        # print(robot_pos.shape)

        # test action
        # amplitudes = torch.tensor([1.0, -1.0, 1.0])       # shape: (3,)
        # frequencies = torch.tensor([1.0, 2.0, 10.0])      # shape: (3,)

        # amplitudes = torch.tensor([1.0])       # shape: (3,)
        # frequencies = torch.tensor([1.0])      # shape: (3,)

        # self.paths = generate_sine_paths(start=0.0, end=2 * torch.pi, num_points=100, amplitudes=amplitudes, frequencies=frequencies)

        # print("pre physics step")
        self._visualize_markers()

    def _apply_action(self) -> None:
        # self.robot.set_joint_velocity_target(self.paths, joint_ids=self.dof_idx)
        # print("apply action step")

        robot_state = self.robot.data.root_link_state_w
        robot_pos = robot_state[:, :3]

        # print("robot_pos:", robot_pos)

        robot_quat = robot_state[:, 3:7]
        robot_speed = robot_state[:, 7:10]
        _, _, yaw = euler_xyz_from_quat(robot_quat)
        
        # print(f"yaw: {yaw}, degrees: {torch.rad2deg(yaw)}")

        collision_CH = self.contact_CH.data.force_matrix_w.norm(dim=-1) > 0
        if self.cfg.robot_name == "Jetbot":
            collision_LW = self.contact_LW.data.force_matrix_w.norm(dim=-1) > 0
            collision_RW = self.contact_RW.data.force_matrix_w.norm(dim=-1) > 0

            collision = collision_CH.squeeze(-1) | collision_LW.squeeze(-1) | collision_RW.squeeze(-1)
        elif self.cfg.robot_name == "CF2X" or self.cfg.robot_name == "Create3":
            collision = collision_CH.squeeze(-1)  # 仅使用 CH 的碰撞检测
            
        self.collision = collision.to(device=self.cfg.sim.device).squeeze(-1)
        
        # print(f"collision: {self.collision}")

        # print(self.collision)
        # print(self.collision.shape)

        # print(f"collision: {self.collision}")
        # print(f"collision_CH: {collision_CH}, collision_LW: {collision_LW}, collision_RW: {collision_RW}")

        # 设置机器人关节速度目标
        if self.cfg.robot_name == "CF2X":
            # CF2X 使用 RL 控制器
            z_tensor = torch.full(
                (self.paths.size(0), self.paths.size(1), 1),
                self.cfg.target_height,
                device=self.cfg.sim.device,
                dtype=torch.float32
            )
            # desired_pos_b, _ = subtract_frame_transforms(
            #     self.robot.data.root_pos_w, self.robot.data.root_quat_w, self._desired_pos_w
            # )
            rl_observation_wo_dp = torch.cat(
                [
                    self.robot.data.root_lin_vel_b,
                    self.robot.data.root_ang_vel_b,
                    self.robot.data.projected_gravity_b,
                    #  desired_pos_b,
                ],
                dim=-1
            )
            # rl_actions = self.rl_controller.act(rl_observation).clamp(-1.0, 1.0)
            # print(f"robot_pos: {robot_pos.shape}, robot_speed: {robot_speed.shape}, yaw: {yaw.shape}")
            
            # print(self.paths.shape)
            # print(z_tensor.shape)
            
            # print(f"actions env shape: {self.paths.shape}")
            
            actions_cp = torch.cat(
                [self.paths, z_tensor],
                dim=-1
            )
            
            rl_actions, _ = self.rl_controller.act_pure_pursuit(rl_observation_wo_dp, actions_cp, robot_pos, 0.1)
            
            self._thrust[:, 0, 2] = self.cfg.thrust_to_weight * self._robot_weight * (rl_actions[:, 0] + 1.0) / 2.0
            self._moment[:, 0, :] = self.cfg.moment_scale * rl_actions[:, 1:]
            self.robot.set_external_force_and_torque(self._thrust, self._moment, body_ids=self._body_id)
            
        elif self.cfg.robot_name == "Jetbot" or self.cfg.robot_name == "Create3":
            current_pose = torch.stack((robot_pos[:, 0], robot_pos[:, 1], yaw), dim=1)

            # print("paths dim:", self.paths.dim())

            o_left, o_right = follow_path_speed(
                current_speeds=torch.linalg.norm(robot_speed, dim=1),
                current_poses=current_pose,
                path=self.paths,
                cfg=self.cfg.pid_cfg
            )

            # print(f"o_left: {o_left}, o_right: {o_right}")

            # 合并左/右轮速度
            actions = torch.stack((o_left, o_right), dim=-1)

            # print("actions:", actions)
            self.robot.set_joint_velocity_target(actions, joint_ids=self.dof_idx)        

    def _get_observations(self) -> dict:
        # 采样目标点（仅使用 x, y 坐标）
        robot_pos_xy = self.robot.data.root_pos_w.cpu()[..., :2]
        _, _, yaw = euler_xyz_from_quat(self.robot.data.root_quat_w)
        
        sampled_goals = self.grid.sample_goal_points(robot_pos_xy, yaw.cpu(), distance_range=self.cfg.action_range)
        sampled_goals = torch.from_numpy(sampled_goals).cuda().to(torch.float32)

        # 转换目标点为局部极坐标并归一化
        relative_goals = sampled_goals - self.robot.data.root_pos_w[..., :2]
        relative_goals = rotate_xy_tensor(relative_goals.unsqueeze(1), yaw)
        relative_goals = relative_goals.squeeze(1)  # (B, 2)
        
        delta_yaw = torch.atan2(relative_goals[:, 1], relative_goals[:, 0]) 

        point_cloud = self.lidar.data.ray_hits_w
        
        # print(f"point_cloud.shape: {point_cloud.shape}")
        # print(f"robot_pose: {self.robot.data.root_pos_w}")
        local_point_cloud = point_cloud - self.robot.data.root_pos_w[:, None, :]  # (B, N, 3)
        
        local_point_cloud = rotate_xy_tensor(local_point_cloud, yaw)
        
        local_point_cloud = rotate_xy_tensor(local_point_cloud, delta_yaw) 
        
        relative_goals = rotate_xy_tensor(relative_goals.unsqueeze(1), delta_yaw).squeeze(1)  # (B, 2)

        # 更新 costmap
        self.costmap.generate(local_point_cloud)
        self.costmap.inflate()

        # print(self.lidar.data.ray_hits_w)

        # 构建三维目标点（添加 z=0）
        self.goal_locations[:] = torch.cat(
            [sampled_goals, torch.zeros((sampled_goals.shape[0], 1), device=self.cfg.sim.device)],
            dim=1
        )
        
        # print(f"yaw: {yaw}, degrees: {torch.rad2deg(yaw)}")
        
        # self.costmap.debug_plot(agent_idx=0, inflated=True, goals=relative_goals)
        
        # self.costmap.inflated_costmaps = torch.flip(self.costmap.inflated_costmaps, dims=(-2, -1))
    
        polar_goals = cartesian_to_polar(relative_goals)
        normalized_goals = normalize_polar_point(polar_goals, r_max=self.cfg.action_range[1], theta_range=self.cfg.action_theta_range)

        self.goals = sampled_goals.clone()
        self.relative_goals = relative_goals.clone()
        self.delta_yaw = delta_yaw.clone()
        
        if self.cfg.robot_name == "CF2X":
            z_tensor = torch.full(
                (self.relative_goals.size(0), 1),
                self.cfg.target_height,
                device=self.cfg.sim.device,
                dtype=torch.float32
            )
            self._desired_pos_w = torch.cat(
                [self.goals, z_tensor],
                dim=1
            )

        # 拉平成成本图并归一化
        costmap_flat = self.costmap.inflated_costmaps.view(
            self.cfg.scene.num_envs, -1
        ) / self.costmap.params.lethal_obstacle
        costmap_flat = costmap_flat.to(torch.float32)

        # 合并观测数据
        observation = torch.cat([costmap_flat, costmap_flat, costmap_flat, normalized_goals], dim=1)

        return {"policy": observation}
    
    def _reward_reach_target(self) -> torch.Tensor:
        robot_pos = self.robot.data.root_pos_w[..., :2]
        goals = self.goals
        # 计算与目标点的距离
        distances = torch.linalg.norm(robot_pos - goals, dim=-1)

        # print(distances)
        
        reached_goal = distances <= self.cfg.reach_target_threshold
        
        # print(reached_goal)

        return torch.where(
            reached_goal, 
            torch.tensor(self.cfg.reward_target_value_1, device=distances.device), 
            torch.tensor(self.cfg.reward_target_value_2, device=distances.device)
        )
    
    def _reward_collision(self):
        return torch.where(
            self.collision,
            torch.tensor(self.cfg.reward_collision_value_1, device=self.cfg.sim.device),
            torch.tensor(self.cfg.reward_collision_value_2, device=self.cfg.sim.device)
        )
    
    def _reward_smoothness(self):
        p0 = self.paths[:, :-2, :]   # [B, N-2, 2]
        p1 = self.paths[:, 1:-1, :]
        p2 = self.paths[:, 2:, :]

        kappa = triangle_curvature(p0, p1, p2)  # [B, N-2]
        dkappa = torch.abs(kappa[:, 1:] - kappa[:, :-1])  # [B, N-3]
        total_diff = dkappa.sum(dim=-1)             # [B]

        # print(f"total_diff: {total_diff.shape}, {total_diff}")
        # print(f"smoothness penalty: {total_diff.shape}, {total_diff}")
        reward = -torch.tanh(self.cfg.reward_smoothness_scale_1 * total_diff)    # [B]
        return reward
    
    def _pairwise_dist(self) -> torch.Tensor:
        """d_i = ||p_{i+1} - p_i||, 形状 [B, N-1]"""
        return (self.paths[:, 1:] - self.paths[:, :-1]).norm(dim=-1)
    
    def _reward_length(self) -> torch.Tensor:
        """
        R_short = -tanh(scale · L)
        L = (Σ d_i - ‖p_last - p_first‖) / ‖p_last - p_first‖   (≥0)
        → L = 0 表示直线最短；越绕路 L 越大，奖励越负
        """
        d = self._pairwise_dist()                      # [B, N-1]
        total_len = d.sum(dim=-1)                      # [B]
        straight_len = (self.paths[:, -1] - self.paths[:, 0]).norm(dim=-1) + 1e-8
        L = (total_len - straight_len) / straight_len  # [B]
        # print(f"length penalty: {L.shape}, {L}")
        return -torch.tanh(self.cfg.reward_length_scale_1 * L)                  # [B]
    
    def _reward_velocity(self) -> torch.Tensor:
        """
        R = -tanh(scale * mean(|d_i - d̄| / d̄))
        d̄ = batch 内该轨迹的平均段长（几何尺度）
        """
        d = self._pairwise_dist()                         # [B, N-1]
        d_mean = d.mean(dim=-1, keepdim=True)             # [B, 1]
        dev = torch.abs(d - d_mean) / (d_mean + 1e-8)     # 归一化偏差
        penalty = dev.mean(dim=-1)                        # [B]
        # print(f"velocity penalty: {penalty.shape}, {penalty}")
        return -torch.tanh(self.cfg.reward_vel_scale_1 * penalty)               # [B]

    def _reward_acceleration(self) -> torch.Tensor:
        """
        R = -tanh(scale * mean(|d_{i+1} - d_i| / d̄))
        本质是段长的一阶差分，等价于时间域 |a_i|/v_pref
        """
        d = self._pairwise_dist()                         # [B, N-1]
        d_mean = d.mean(dim=-1, keepdim=True)             # [B, 1]
        acc = torch.abs(d[:, 1:] - d[:, :-1]) / (d_mean + 1e-8)  # [B, N-2]
        penalty = acc.mean(dim=-1)                        # [B]
        # print(f"acc penalty: {penalty.shape}, {penalty}")
        return -torch.tanh(self.cfg.reward_acc_scale_1 * penalty)               # [B]
    
        # ---------- 速度平滑性奖励 ----------
    def _reward_speed(self) -> torch.Tensor:
        """
        R_speed = -tanh(0.05 * Σ|v_i - v_pref|)
        其中 v_i = d_i / Δt, Δt 由「机器人→目标距离 / v_pref / 20」给出（与原实现一致）。
        """
        d_i = self._pairwise_dist()                                  # (B, N-1)
        # Δt (B,1)：用 _get_observations() 里更新的 self.relative_goals
        goal_dist = torch.norm(self.relative_goals, dim=-1, keepdim=True) + 1e-6
        delta_t   = goal_dist / self.cfg.num_points / self.v_prefer                  # (B,1)
        v_i       = d_i / delta_t                                    # (B, N-1)
        deviation = torch.abs(v_i - self.v_prefer)                   # (B, N-1)
        total_dev = deviation.sum(dim=-1)                            # (B,)
        return -torch.tanh(self.cfg.reward_speed_scale_1 * total_dev)

    # ---------- 加速度平滑性奖励 ----------
    def _reward_acceleration_phys(self) -> torch.Tensor:
        """
        R_acc = -tanh(0.01 * Σ|a_i|)
        a_i = (v_i - v_{i-1}) / Δt，同样沿用上面的 Δt。
        """
        d_i = self._pairwise_dist()                                  # (B, N-1)
        goal_dist = torch.norm(self.relative_goals, dim=-1, keepdim=True) + 1e-6
        delta_t   = goal_dist / self.v_prefer / self.cfg.num_points                 # (B,1)

        v_i = d_i / delta_t                                          # (B, N-1)
        a_i = (v_i[:, 1:] - v_i[:, :-1]) / delta_t                   # (B, N-2)
        total_acc = torch.abs(a_i).sum(dim=-1)                       # (B,)
        return -torch.tanh(0.01 * total_acc)


    def _get_rewards(self) -> torch.Tensor:
        # arrive target reward
        reward_target = self._reward_reach_target().view(-1)
        # print(f"reward_target: {reward_target}")

        reward_collision = torch.squeeze(self._reward_collision()).view(-1)
        # print(f"reward_collision: {reward_collision}")

        reward_smoothness = self._reward_smoothness().view(-1)
        # print(f"reward_smoothness: {reward_smoothness}")

        # reward_length = self._reward_length().view(-1)
        # # print(f"reward_length: {reward_length}")
        # reward_velocity_geo = self._reward_velocity().view(-1) # 纯几何速度奖励
        # # print(f"reward_velocity_geo: {reward_velocity_geo}").view(-1)
        # reward_acceleration_geo = self._reward_acceleration() # 纯几何加速度奖励
        # # print(f"reward_acceleration_geo: {reward_acceleration_geo}")
        
        reward_speed_phyx        = self._reward_speed().view(-1) # 原速度奖励
        # print(f"reward_speed_phyx: {reward_speed_phyx}")
        
        reward_acceleration_phyx = self._reward_acceleration_phys().view(-1) # 原加速度奖励
        # print(f"reward_acceleration_phyx: {reward_acceleration_phyx}")
        
        # ==== 新增逻辑处理 reward_target 和 reward_collision ====
        # 逻辑 1: 两者都为 0，设置 collision 为 reward_collision_value_1
        condition_both_zero = (reward_collision == 0) & (reward_target == 0)
        reward_collision[condition_both_zero] = self.cfg.reward_collision_value_1

        # 逻辑 2: 两者都非零，设置 target 为 0
        condition_both_nonzero = (reward_collision != 0) & (reward_target != 0)
        reward_target[condition_both_nonzero] = self.cfg.reward_target_value_2
        # ========================================================

        
        total_reward = (
            self.cfg.reward_target_scale * reward_target +
            self.cfg.reward_collision_scale * reward_collision +
            self.cfg.reward_smoothness_scale_2 * reward_smoothness +
            self.cfg.reward_speed_scale_2 * reward_speed_phyx +
            self.cfg.reward_acc_phyx_scale_2 * reward_acceleration_phyx
        )

        # print("total reward: ", total_reward)

        # total_reward = (
        #     self.cfg.reward_target_scale * reward_target +
        #     self.cfg.reward_collision_scale * reward_collision +
        #     self.cfg.reward_smoothness_scale_2 * reward_smoothness +
        #     self.cfg.reward_length_scale_2 * reward_length +
        #     self.cfg.reward_vel_scale_2 * reward_velocity_geo +
        #     self.cfg.reward_acc_scale_2 * reward_acceleration_geo
        #     # self.cfg.reward_speed_scale_2 * reward_speed_phyx +
        #     # self.cfg.reward_acc_phyx_scale_2 * reward_acceleration_phyx
        # )

        
        return torch.squeeze(total_reward)

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        # ---------- 1) 统计逻辑终止 ----------
        robot_pos = self.robot.data.root_pos_w[..., :2]
        dist_goal = torch.linalg.norm(robot_pos - self.goals, dim=-1)
        reached   = dist_goal <= self.cfg.reach_target_threshold          # 成功
        collided  = self.collision                                        # 碰撞
        resets    = reached | collided                                    # 逻辑 done

        # ---------- 2) 时间终止 (单步 episode) ----------
        time_out  = torch.ones_like(resets)                               # 全 True

        return resets, time_out

    def _reset_idx(self, env_ids: Sequence[int] | None):
        # ---------- 0）父类逻辑 ----------
        if env_ids is None:
            env_ids = self.robot._ALL_INDICES            # 全部环境
        super()._reset_idx(env_ids)

        # ---------- 0.5）转成 tensor ----------
        env_ids = torch.as_tensor(
            env_ids, dtype=torch.long, device=self.cfg.sim.device
        )

        # ---------- 1）先备份 root_state ----------
        root_state = self.robot.data.root_state_w.clone()      # (N_env, 13)

        # ---------- 2）收集需「完全重置」的 env ----------
        # 2-a. 碰撞
        collided_env_ids = env_ids[self.collision[env_ids]]

        # 2-b. 坠机 / 出界（对所有 env 检测一次，防止漏检）
        died_mask = (
            (self.robot.data.root_pos_w[:, 2] < 0.1) |
            (self.robot.data.root_pos_w[:, 2] > 2.0) |
            (self.robot.data.root_pos_w[:, 0] < self.grid.cfg.x_min) |
            (self.robot.data.root_pos_w[:, 0] > self.grid.cfg.x_max) |
            (self.robot.data.root_pos_w[:, 1] < self.grid.cfg.y_min) |
            (self.robot.data.root_pos_w[:, 1] > self.grid.cfg.y_max)
        )
        died_env_ids = torch.arange(root_state.shape[0], device=root_state.device)[died_mask]

        # 2-c. 两者并集（去重 + 排序，方便后续索引）
        if collided_env_ids.numel() or died_env_ids.numel():
            reset_env_ids, _ = torch.sort(
                torch.unique(torch.cat([collided_env_ids, died_env_ids], dim=0))
            )
        else:
            reset_env_ids = torch.empty(0, dtype=torch.long, device=self.cfg.sim.device)

        # ---------- 3）为 reset_env_ids 重新采样位姿 ----------
        if reset_env_ids.numel() > 0:
            n_reset = reset_env_ids.numel()

            # 3-a. 采样平面坐标
            safety_dist = getattr(self.cfg, "reset_safety_distance", 0.20)
            new_xy_np = self.grid.sample_free_points(
                n=n_reset,
                safety_distance=safety_dist,
                max_trials_per_point=2000,
            )
            new_xy = torch.from_numpy(new_xy_np).to(
                device=self.cfg.sim.device, dtype=torch.float32
            )

            # 3-b. 采样 yaw
            new_yaw  = sample_uniform(
                torch.tensor([-math.pi], device=self.cfg.sim.device),
                torch.tensor([ math.pi], device=self.cfg.sim.device),
                (n_reset, 1),
                device=self.cfg.sim.device,
            ).squeeze(-1)
            new_quat = math_utils.quat_from_euler_xyz(
                torch.zeros_like(new_yaw), torch.zeros_like(new_yaw), new_yaw
            )

            # 3-c. 写回 root_state
            root_state[reset_env_ids, 0:2] = new_xy                            # x, y
            root_state[reset_env_ids, 2]   = self.cfg.target_height            # z
            root_state[reset_env_ids, 3:7] = new_quat                          # quat

        # ---------- 4）翻车只矫正姿态 ----------
        roll, pitch, yaw = euler_xyz_from_quat(root_state[:, 3:7])
        crash_thresh = torch.deg2rad(torch.tensor(30.0, device=self.cfg.sim.device))
        crashed_env_ids = torch.arange(root_state.shape[0], device=roll.device)[
            (torch.abs(roll) > crash_thresh) | (torch.abs(pitch) > crash_thresh)
        ]
        if crashed_env_ids.numel() > 0:
            roll[crashed_env_ids]  = 0.0
            pitch[crashed_env_ids] = 0.0
            root_state[:, 3:7] = math_utils.quat_from_euler_xyz(roll, pitch, yaw)

        # ---------- 5）清零速度 ----------
        # 需要清零的集合 = 调用方传入的 env_ids ∪ reset_env_ids
        zero_env_ids, _ = torch.sort(torch.unique(torch.cat([env_ids, reset_env_ids])))
        root_state[zero_env_ids, 7:13] = 0.0
        self.robot.write_root_state_to_sim(root_state[zero_env_ids], zero_env_ids)

        if self.cfg.robot_name == "CF2X":
            self.rl_controller.reset_timestep()

        # ---------- 6）重置碰撞标志 ----------
        self.collision[zero_env_ids] = False
