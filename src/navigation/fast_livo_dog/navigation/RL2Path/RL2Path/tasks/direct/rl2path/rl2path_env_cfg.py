# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import isaaclab.sim as sim_utils

from isaaclab.assets import ArticulationCfg, RigidObjectCfg
from isaaclab_assets import CRAZYFLIE_CFG 
from isaaclab.envs import DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg, RigidBodyPropertiesCfg, MassPropertiesCfg, CollisionPropertiesCfg, RigidBodyMaterialCfg
from isaaclab.sim.schemas import CollisionPropertiesCfg, RigidBodyPropertiesCfg, activate_contact_sensors
from isaaclab.sim import UsdFileCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from isaaclab.sensors import RayCasterCfg, ContactSensorCfg
from isaaclab.sensors.ray_caster import RayCasterCfg, patterns


import numpy as np
from gymnasium.spaces import Box
from RL2Path.robots.jetbot import JETBOT_CONFIG
from RL2Path.robots.create3 import CREATE3_CONFIG
# from RL2Path.robots.cf2x import CF2X_CONFIG
from RL2Path.nav.cost_map_2d import CostmapCfg  
from RL2Path.nav.grid_2d import GridCfg
from RL2Path.nav.pid_controller import PPIDCfg

@configclass
class Rl2pathEnvCfg(DirectRLEnvCfg):
    # env
    decimation = 300
    episode_length_s = 10.0
    num_envs = 2048
    
    # - spaces definition
    action_dim = 8
    action_space = Box(
        low=0.0,
        high=1.0,
        shape=(action_dim,),
        dtype=np.float32
    )
    action_range = (0.5, 1.0)
    action_theta_range = (-np.pi/2, np.pi/2)
    num_points = 50
    action_with_goal = False
    observation_space = Box(low=0, high=1, shape=(7502,), dtype=np.float32)
    state_space = 0
    
    robot_name = "Create3"  # "Jetbot" "Create3", "CF2X"

    # reward cfg
    reach_target_threshold = 0.25
    reward_target_value_1 = 8.0
    reward_target_value_2 = 0.0
    reward_target_scale = 0.125

    reward_collision_value_1 = -5.0
    reward_collision_value_2 = 0.0
    reward_collision_scale = 0.125

    reward_smoothness_scale_1 = 0.01
    reward_smoothness_scale_2 = 0.125

    reward_length_scale_1 = 1.0
    reward_length_scale_2 = 0.1

    reward_vel_scale_1 = 0.6
    reward_vel_scale_2 = 0.1

    reward_acc_scale_1 = 2.0
    reward_acc_scale_2 = 0.1

    reset_safety_distance = 0.2
    
    # origin reward
    v_prefer = 0.30
    reward_speed_scale_1 = 0.05
    reward_speed_scale_2 = 0.125
    
    reward_acc_phyx_scale_1 = 0.01
    reward_acc_phyx_scale_2 = 0.125
    
    # ground
    ground_prim_path = "/World/ground"

    # obstacle
    scene_name = "scene_4"
    
    obstacle_usd_path = f"/fast_data/zhangziyang/workspace/Assets/{scene_name}/scene_kin_rig.usd"
    obstacle_prim_path = "/World/envs/env_.*/obstacle"

    static_obstacle_path = f"/fast_data/zhangziyang/workspace/Assets/{scene_name}/scene_coll.usd"
    static_obstacle_prim_path = "/World/static_obstacle"

    static_obstacle_cfg = UsdFileCfg(
        usd_path=static_obstacle_path, 
    )

    obstacle_cfg = RigidObjectCfg(
        prim_path=obstacle_prim_path,
        spawn=UsdFileCfg(
            usd_path=obstacle_usd_path,
            rigid_props=RigidBodyPropertiesCfg(
                rigid_body_enabled=True
            ),
            collision_props=CollisionPropertiesCfg(),
            mass_props=MassPropertiesCfg(mass=100.0),
        ),
    )
    
    
    if robot_name == "CF2X":
        thrust_to_weight = 1.01
        moment_scale = 0.001
        target_height = 1.0
        
        sim: SimulationCfg = SimulationCfg(
            dt=1 / 100, 
            render_interval=3,
            physics_material=sim_utils.RigidBodyMaterialCfg(
                friction_combine_mode="multiply",
                restitution_combine_mode="multiply",
                static_friction=1.0,
                dynamic_friction=1.0,
                restitution=0.0,
            ),
        )
        robot_cfg: ArticulationCfg = CRAZYFLIE_CFG.replace(prim_path="/World/envs/env_.*/Robot")
        robot_cfg = robot_cfg.replace(init_state=ArticulationCfg.InitialStateCfg(pos=(0, 0, target_height)))
        robot_cfg = robot_cfg.replace(spawn=sim_utils.UsdFileCfg(usd_path=f"{ISAAC_NUCLEUS_DIR}/Robots/Crazyflie/cf2x.usd", activate_contact_sensors=True,))
        sensor_cage = "/World/envs/env_.*/Robot/body"
        contact_CH = ContactSensorCfg(
            prim_path=sensor_cage,
            update_period=0.0,
            history_length=0,
            debug_vis=False,
            filter_prim_paths_expr=[f"{obstacle_prim_path}/mesh"],
        )
        rc_cfg_path = "checkpoints/drone/skrl_ppo_cfg.yaml"
        rc_ckpt_path = "checkpoints/drone/best_agent.pt"
        
    elif robot_name == "Jetbot":
        target_height = 0.0
        
        # simulation
        sim: SimulationCfg = SimulationCfg(dt=1 / 30, render_interval=3)
        # robot(s)
        robot_cfg: ArticulationCfg = JETBOT_CONFIG.replace(prim_path="/World/envs/env_.*/Robot")
        # robot_cfg: ArticulationCfg = CREATE3_CONFIG.replace(prim_path="/World/envs/env_.*/Robot")
        
        sensor_cage = "/World/envs/env_.*/Robot/chassis"
        
        contact_CH = ContactSensorCfg(
            prim_path=sensor_cage,
            update_period=0.0,
            history_length=0,
            debug_vis=False,
            filter_prim_paths_expr=[f"{obstacle_prim_path}/mesh"],
        )

        contact_LW = ContactSensorCfg(
            prim_path="/World/envs/env_.*/Robot/left_wheel",
            update_period=0.0,
            history_length=0,
            debug_vis=False,
            filter_prim_paths_expr=[f"{obstacle_prim_path}/mesh"],
        )

        contact_RW = ContactSensorCfg(
            prim_path="/World/envs/env_.*/Robot/right_wheel",
            update_period=0.0,
            history_length=6,
            debug_vis=False,
            filter_prim_paths_expr=[f"{obstacle_prim_path}/mesh"],
        )
        
        # controller
        pid_cfg = PPIDCfg(
            look_ahead_base=0.1,
            wheel_base=0.11,
            cruise_speed=v_prefer,
            wheel_radius=0.03,
        )
        
        dof_names = ["left_wheel_joint", "right_wheel_joint"]
        
    elif robot_name == "Create3":
        target_height = 0.0
        
        # simulation
        sim: SimulationCfg = SimulationCfg(dt=1 / 30, render_interval=3)
        # robot(s)
        robot_cfg: ArticulationCfg = CREATE3_CONFIG.replace(prim_path="/World/envs/env_.*/Robot")
        
        sensor_cage = "/World/envs/env_.*/Robot/base_link"
        
        contact_CH = ContactSensorCfg(
            prim_path=sensor_cage,
            update_period=0.0,
            history_length=0,
            debug_vis=False,
            filter_prim_paths_expr=[f"{obstacle_prim_path}/mesh"],
        )
        
        # controller
        pid_cfg = PPIDCfg(
            look_ahead_base=0.1,
            wheel_base=0.233,
            kp_speed=2.5,
            cruise_speed=v_prefer,
            wheel_radius=0.036,
            turn_in_place_omega=1.5,
            turn_in_place_thresh=0.87
        )
        
        dof_names = ["left_wheel_joint", "right_wheel_joint"]

    # sensor
    lidar_cfg = RayCasterCfg(
        prim_path=sensor_cage,
        update_period=1 / 30,
        offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 0.2)),
        mesh_prim_paths=[static_obstacle_prim_path],
        attach_yaw_only=True,
        pattern_cfg=patterns.LidarPatternCfg(
            channels=1, vertical_fov_range=[0, 0], horizontal_fov_range=[-180.0, 180.0], horizontal_res=360.0 / 641.0
        ),
        debug_vis=False
    )

    # costmap
    costmap_cfg = CostmapCfg()

    # grid
    grid_pkl_path = f"./outputs/{scene_name}/grid.pkl"

    # scene
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=num_envs, env_spacing=0.0, replicate_physics=True)