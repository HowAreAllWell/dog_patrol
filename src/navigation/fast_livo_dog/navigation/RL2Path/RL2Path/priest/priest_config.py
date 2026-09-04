#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from dataclasses import dataclass
from typing import Optional, Dict, Any
import numpy as np
import rclpy
from rclpy.node import Node

@dataclass
class PriestConfig:
    # —— 外部约束（用于自动对齐）——
    rl_max_path_length_m: float = 1.8
    robot_max_speed_mps: float = 0.500
    temporal_resolution_hz: float = 10.0

    # —— PRIEST 直接参数（全部可调）——
    num_dynamic_obstacles: int = 10
    num_static_obstacles: int = 100
    max_velocity: float = 0.500
    min_velocity: float = 0.0
    max_acceleration: float = 0.500
    max_inner_iterations: int = 2
    max_outer_iterations: int = 2
    tracking_weight: float = 0.1
    smoothness_weight: float = 0.3
    static_obstacle_semi_minor_axis: float = 0.500
    static_obstacle_semi_major_axis: float = 0.500
    dynamic_obstacle_semi_minor_axis: float = 0.3
    dynamic_obstacle_semi_major_axis: float = 0.3
    trajectory_batch_size: int = 50
    num_waypoints: int = 1000

    # —— 可由上面自动推导，也可手工覆盖 —— 
    time_horizon: Optional[float] = None
    trajectory_length: Optional[int] = None
    desired_velocity: Optional[float] = None

    # —— 运行开关/杂项 —— 
    enable: bool = True
    use_obstacle_constraints: bool = True
    dynamic_topic: str = "pedestrians/markers"      # 默认接你上面的 MarkerArray 话题
    dynamic_msg_type: str = "marker_array"          # marker_array / tracked_persons / agent_states
    publish_rl_debug: bool = True
    padding_distance: float = 1000.0

    # ===== 逻辑 =====
    def finalize(self) -> "PriestConfig":
        if self.desired_velocity is None:
            self.desired_velocity = min(self.robot_max_speed_mps, self.max_velocity)
        else:
            self.desired_velocity = min(float(self.desired_velocity), self.max_velocity, self.robot_max_speed_mps)

        if self.time_horizon is None:
            v = max(self.desired_velocity, 1e-6)
            self.time_horizon = max(0.500, self.rl_max_path_length_m / v)

        if self.trajectory_length is None:
            self.trajectory_length = max(10, int(round(self.time_horizon * self.temporal_resolution_hz)))

        self.max_velocity = max(self.min_velocity, self.max_velocity)
        self.max_acceleration = max(0.0, self.max_acceleration)
        self.tracking_weight = float(self.tracking_weight)
        self.smoothness_weight = float(self.smoothness_weight)
        return self

    def planner_kwargs(self) -> Dict[str, Any]:
        return dict(
            num_dynamic_obstacles=self.num_dynamic_obstacles,
            num_static_obstacles=self.num_static_obstacles,
            time_horizon=self.time_horizon,
            trajectory_length=self.trajectory_length,
            max_velocity=self.max_velocity,
            min_velocity=self.min_velocity,
            max_acceleration=self.max_acceleration,
            max_inner_iterations=self.max_inner_iterations,
            max_outer_iterations=self.max_outer_iterations,
            tracking_weight=self.tracking_weight,
            smoothness_weight=self.smoothness_weight,
            static_obstacle_semi_minor_axis=self.static_obstacle_semi_minor_axis,
            static_obstacle_semi_major_axis=self.static_obstacle_semi_major_axis,
            dynamic_obstacle_semi_minor_axis=self.dynamic_obstacle_semi_minor_axis,
            dynamic_obstacle_semi_major_axis=self.dynamic_obstacle_semi_major_axis,
            trajectory_batch_size=self.trajectory_batch_size,
            desired_velocity=self.desired_velocity,
            num_waypoints=self.num_waypoints,
        )

    @classmethod
    def from_rosparams(cls, node: Node) -> "PriestConfig":
        gp = node.get_parameter

        def param(name: str, default):
            if not node.has_parameter(name):
                node.declare_parameter(name, default)
            return gp(name).get_parameter_value()

        # 读取参数（ROS2: 使用层级名，无~）
        inst = cls(
            rl_max_path_length_m=float(param("priest.rl_max_path_length_m", 4.0).double_value if hasattr(param("priest.rl_max_path_length_m", 4.0), "double_value") else param("priest.rl_max_path_length_m", 4.0).value),
            robot_max_speed_mps=float(param("priest.robot_max_speed_mps", 1.0).double_value if hasattr(param("priest.robot_max_speed_mps", 1.0), "double_value") else param("priest.robot_max_speed_mps", 1.0).value),
            temporal_resolution_hz=float(param("priest.temporal_resolution_hz", 20.0).double_value if hasattr(param("priest.temporal_resolution_hz", 20.0), "double_value") else param("priest.temporal_resolution_hz", 20.0).value),

            enable=bool(param("priest.enable", True).bool_value if hasattr(param("priest.enable", True), "bool_value") else param("priest.enable", True).value),
            use_obstacle_constraints=bool(param("priest.use_obstacle_constraints", True).bool_value if hasattr(param("priest.use_obstacle_constraints", True), "bool_value") else param("priest.use_obstacle_constraints", True).value),
            dynamic_topic=str(param("priest.dynamic_topic", "pedestrians/markers").string_value if hasattr(param("priest.dynamic_topic", "pedestrians/markers"), "string_value") else param("priest.dynamic_topic", "pedestrians/markers").value),
            dynamic_msg_type=str(param("priest.dynamic_msg_type", "marker_array").string_value if hasattr(param("priest.dynamic_msg_type", "marker_array"), "string_value") else param("priest.dynamic_msg_type", "marker_array").value).lower(),
            publish_rl_debug=bool(param("priest.publish_rl_debug", False).bool_value if hasattr(param("priest.publish_rl_debug", False), "bool_value") else param("priest.publish_rl_debug", False).value),
            padding_distance=float(param("priest.padding_distance", 1000.0).double_value if hasattr(param("priest.padding_distance", 1000.0), "double_value") else param("priest.padding_distance", 1000.0).value),

            num_dynamic_obstacles=int(param("priest.num_dynamic_obstacles", 10).integer_value if hasattr(param("priest.num_dynamic_obstacles", 10), "integer_value") else param("priest.num_dynamic_obstacles", 10).value),
            num_static_obstacles=int(param("priest.num_static_obstacles", 300).integer_value if hasattr(param("priest.num_static_obstacles", 300), "integer_value") else param("priest.num_static_obstacles", 300).value),
            max_velocity=float(param("priest.max_velocity", 1.0).double_value if hasattr(param("priest.max_velocity", 1.0), "double_value") else param("priest.max_velocity", 1.0).value),
            min_velocity=float(param("priest.min_velocity", 0.0).double_value if hasattr(param("priest.min_velocity", 0.0), "double_value") else param("priest.min_velocity", 0.0).value),
            max_acceleration=float(param("priest.max_acceleration", 0.3).double_value if hasattr(param("priest.max_acceleration", 0.3), "double_value") else param("priest.max_acceleration", 0.3).value),
            max_outer_iterations=int(param("priest.max_outer_iterations", 2).integer_value if hasattr(param("priest.max_outer_iterations", 2), "integer_value") else param("priest.max_outer_iterations", 2).value),
            max_inner_iterations=int(param("priest.max_inner_iterations", 7).integer_value if hasattr(param("priest.max_inner_iterations", 7), "integer_value") else param("priest.max_inner_iterations", 7).value),
            tracking_weight=float(param("priest.tracking_weight", 0.4).double_value if hasattr(param("priest.tracking_weight", 0.4), "double_value") else param("priest.tracking_weight", 0.4).value),
            smoothness_weight=float(param("priest.smoothness_weight", 0.8).double_value if hasattr(param("priest.smoothness_weight", 0.8), "double_value") else param("priest.smoothness_weight", 0.8).value),
            # static_obstacle_semi_minor_axis=float(param("priest.static_obstacle_semi_minor_axis", 0.233).double_value if hasattr(param("priest.static_obstacle_semi_minor_axis", 0.233), "double_value") else param("priest.static_obstacle_semi_minor_axis", 0.233).value),
            # static_obstacle_semi_major_axis=float(param("priest.static_obstacle_semi_major_axis", 0.233).double_value if hasattr(param("priest.static_obstacle_semi_major_axis", 0.233), "double_value") else param("priest.static_obstacle_semi_major_axis", 0.233).value),
            static_obstacle_semi_minor_axis=float(param("priest.static_obstacle_semi_minor_axis", 0.6).double_value if hasattr(param("priest.static_obstacle_semi_minor_axis", 0.6), "double_value") else param("priest.static_obstacle_semi_minor_axis", 0.6).value),
            static_obstacle_semi_major_axis=float(param("priest.static_obstacle_semi_major_axis", 0.6).double_value if hasattr(param("priest.static_obstacle_semi_major_axis", 0.6), "double_value") else param("priest.static_obstacle_semi_major_axis", 0.6).value),
            dynamic_obstacle_semi_minor_axis=float(param("priest.dynamic_obstacle_semi_minor_axis", 0.8).double_value if hasattr(param("priest.dynamic_obstacle_semi_minor_axis", 0.8), "double_value") else param("priest.dynamic_obstacle_semi_minor_axis", 0.8).value),
            dynamic_obstacle_semi_major_axis=float(param("priest.dynamic_obstacle_semi_major_axis", 1.2).double_value if hasattr(param("priest.dynamic_obstacle_semi_major_axis", 1.2), "double_value") else param("priest.dynamic_obstacle_semi_major_axis", 1.2).value),
            trajectory_batch_size=int(param("priest.trajectory_batch_size", 50).integer_value if hasattr(param("priest.trajectory_batch_size", 50), "integer_value") else param("priest.trajectory_batch_size", 50).value),
            num_waypoints=int(param("priest.num_waypoints", 400).integer_value if hasattr(param("priest.num_waypoints", 400), "integer_value") else param("priest.num_waypoints", 400).value),

            time_horizon=(float(param("priest.time_horizon", 4.0).double_value) if param("priest.time_horizon", 4.0) is not None else None),
            trajectory_length=(int(param("priest.trajectory_length", 50).integer_value) if param("priest.trajectory_length", 50) is not None else None),
            desired_velocity=(float(param("priest.desired_velocity", 1.0).double_value) if param("priest.desired_velocity", 1.0) is not None else None),
        )
        return inst.finalize()
