"""Launch the external M20 local navigation chain.

RViz Publish Point drives the waypoint manager. This launch starts:

  /clicked_point -> global_path_seq_publisher.py -> /global_path
  /global_path -> pure_pursuit.py -> /subgoal, /final_goal
  /global_path + /subgoal -> priest_rl_publisher_nav_cmd_fast.py -> /local_path
  /local_path -> priest_mppi_adapter_nav_cmd_dwb_smooth_responsive.py -> /NAV_CMD
"""

import sys
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    python_executable = LaunchConfiguration("python_executable")
    rl_python_executable = LaunchConfiguration("rl_python_executable")

    start_pure_pursuit = LaunchConfiguration("start_pure_pursuit")
    start_rl_local_path = LaunchConfiguration("start_rl_local_path")
    start_adapter = LaunchConfiguration("start_adapter")
    start_rviz_waypoints = LaunchConfiguration("start_rviz_waypoints")

    global_path_topic = LaunchConfiguration("global_path_topic")
    pure_pursuit_plan_topic = LaunchConfiguration("pure_pursuit_plan_topic")
    subgoal_topic = LaunchConfiguration("subgoal_topic")
    final_goal_topic = LaunchConfiguration("final_goal_topic")
    local_path_topic = LaunchConfiguration("local_path_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    nav_cmd_topic = LaunchConfiguration("nav_cmd_topic")
    clicked_point_topic = LaunchConfiguration("clicked_point_topic")
    waypoint_delete_topic = LaunchConfiguration("waypoint_delete_topic")
    waypoint_replace_topic = LaunchConfiguration("waypoint_replace_topic")
    waypoint_status_topic = LaunchConfiguration("waypoint_status_topic")
    waypoint_resume_from_current_topic = LaunchConfiguration("waypoint_resume_from_current_topic")
    waypoints_topic = LaunchConfiguration("waypoints_topic")
    waypoints_pose_topic = LaunchConfiguration("waypoints_pose_topic")
    waypoint_replan_period = LaunchConfiguration("waypoint_replan_period")
    waypoint_goal_tolerance = LaunchConfiguration("waypoint_goal_tolerance")
    waypoint_edit_radius = LaunchConfiguration("waypoint_edit_radius")
    waypoint_enable_interactive_markers = LaunchConfiguration("waypoint_enable_interactive_markers")
    waypoint_interactive_marker_ns = LaunchConfiguration("waypoint_interactive_marker_ns")

    global_frame = LaunchConfiguration("global_frame")
    robot_frame = LaunchConfiguration("robot_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    path_target_frame = LaunchConfiguration("path_target_frame")

    lookahead = LaunchConfiguration("lookahead")
    pure_pursuit_rate = LaunchConfiguration("pure_pursuit_rate")
    rl_hz = LaunchConfiguration("rl_hz")
    device = LaunchConfiguration("device")
    require_localization_confidence = LaunchConfiguration("require_localization_confidence")
    adapter_path_timeout = LaunchConfiguration("adapter_path_timeout")

    common_env = {
        "PYTHONUNBUFFERED": "1",
        "XLA_PYTHON_CLIENT_PREALLOCATE": "false",
        "MPLCONFIGDIR": "/tmp/matplotlib",
        "OPENBLAS_NUM_THREADS": "1",
        "OMP_NUM_THREADS": "1",
    }

    rviz_waypoints = Node(
        condition=IfCondition(start_rviz_waypoints),
        package="move",
        executable="global_path_seq_publisher",
        name="global_path_sequence_publisher",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            {
                "use_static_goals": False,
                "clicked_point_topic": clicked_point_topic,
                "delete_clicked_point_topic": waypoint_delete_topic,
                "replace_clicked_point_topic": waypoint_replace_topic,
                "status_topic": waypoint_status_topic,
                "resume_from_current_topic": waypoint_resume_from_current_topic,
                "path_topic": global_path_topic,
                "publish_pure_pursuit_plan": True,
                "pure_pursuit_plan_topic": pure_pursuit_plan_topic,
                "waypoints_topic": waypoints_topic,
                "waypoints_pose_topic": waypoints_pose_topic,
                "global_frame": global_frame,
                "robot_frame": robot_frame,
                "replan_period": waypoint_replan_period,
                "goal_tolerance": waypoint_goal_tolerance,
                "edit_radius": waypoint_edit_radius,
                "enable_interactive_markers": waypoint_enable_interactive_markers,
                "interactive_marker_namespace": waypoint_interactive_marker_ns,
            }
        ],
    )

    pure_pursuit = ExecuteProcess(
        condition=IfCondition(start_pure_pursuit),
        cmd=[
            python_executable,
            "-m",
            "move.pure_pursuit",
            "--ros-args",
            "-r",
            ["plan:=", pure_pursuit_plan_topic],
            "-r",
            ["subgoal:=", subgoal_topic],
            "-r",
            ["final_goal:=", final_goal_topic],
            "-p",
            ["use_sim_time:=", use_sim_time],
            "-p",
            ["lookahead:=", lookahead],
            "-p",
            ["rate:=", pure_pursuit_rate],
            "-p",
            ["world_frame:=", global_frame],
            "-p",
            ["robot_frame:=", robot_frame],
        ],
        output="screen",
        additional_env=common_env,
    )

    rl_local_path = ExecuteProcess(
        condition=IfCondition(start_rl_local_path),
        cmd=[
            rl_python_executable,
            "-m",
            "move.priest_rl_publisher_nav_cmd_fast",
            "--ros-args",
            "-r",
            ["local_path:=", local_path_topic],
            "-r",
            ["scan:=", "/scan"],
            "-p",
            ["use_scan:=", "true"],
            "-r",
            ["subgoal:=", subgoal_topic],
            "-r",
            ["final_goal:=", final_goal_topic],
            "-p",
            ["use_sim_time:=", use_sim_time],
            "-p",
            ["global_plan_topic:=", global_path_topic],
            "-p",
            ["global_frame:=", global_frame],
            "-p",
            ["base_frame:=", robot_frame],
            "-p",
            ["frame_id:=", robot_frame],
            "-p",
            ["odom_frame:=", odom_frame],
            "-p",
            ["hz:=", rl_hz],
            "-p",
            ["device:=", device],
        ],
        output="screen",
        additional_env=common_env,
    )

    adapter = ExecuteProcess(
        condition=IfCondition(start_adapter),
        cmd=[
            python_executable,
            "-m",
            "move.priest_mppi_adapter_nav_cmd_dwb_smooth_responsive",
            "--ros-args",
            "-p",
            ["use_sim_time:=", use_sim_time],
            "-p",
            ["priest_path_topic:=", local_path_topic],
            "-p",
            ["path_target_frame:=", path_target_frame],
            "-p",
            ["base_frame:=", robot_frame],
            "-p",
            ["cmd_vel_topic:=", cmd_vel_topic],
            "-p",
            ["nav_cmd_topic:=", nav_cmd_topic],
            "-p",
            ["require_localization_confidence:=", require_localization_confidence],
            "-p",
            ["path_timeout:=", adapter_path_timeout],
        ],
        output="screen",
        additional_env=common_env,
    )

    domain_bridge = ExecuteProcess(
        condition=IfCondition(start_adapter),
        cmd=[
            python_executable,
            "-m",
            "move.nav_cmd_domain_bridge"
        ],
        output="screen",
        additional_env=common_env,
    )

    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/cloud_registered'),
            ('scan', '/scan')
        ],
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.05,
            'min_height': -0.3,
            'max_height': 0.5,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.01745,
            'scan_time': 0.1,
            'range_min': 0.15,
            'range_max': 10.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("python_executable", default_value="python3"),
            DeclareLaunchArgument(
                "rl_python_executable",
                default_value="python3",
            ),
            DeclareLaunchArgument("start_pure_pursuit", default_value="true"),
            DeclareLaunchArgument("start_rl_local_path", default_value="true"),
            DeclareLaunchArgument("start_adapter", default_value="true"),
            DeclareLaunchArgument("start_rviz_waypoints", default_value="true"),
            DeclareLaunchArgument("global_path_topic", default_value="global_path"),
            DeclareLaunchArgument("pure_pursuit_plan_topic", default_value="global_path"),
            DeclareLaunchArgument("subgoal_topic", default_value="subgoal"),
            DeclareLaunchArgument("final_goal_topic", default_value="final_goal"),
            DeclareLaunchArgument("local_path_topic", default_value="local_path"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("nav_cmd_topic", default_value="/NAV_CMD"),
            DeclareLaunchArgument("clicked_point_topic", default_value="/clicked_point"),
            DeclareLaunchArgument("waypoint_delete_topic", default_value="/waypoint_sequence/delete_nearest"),
            DeclareLaunchArgument("waypoint_replace_topic", default_value="/waypoint_sequence/replace_nearest"),
            DeclareLaunchArgument("waypoint_status_topic", default_value="/waypoint_sequence/status"),
            DeclareLaunchArgument(
                "waypoint_resume_from_current_topic",
                default_value="/waypoint_sequence/resume_from_current",
            ),
            DeclareLaunchArgument("waypoints_topic", default_value="waypoints"),
            DeclareLaunchArgument("waypoints_pose_topic", default_value="waypoints_pose_array"),
            DeclareLaunchArgument("waypoint_replan_period", default_value="1.0"),
            DeclareLaunchArgument("waypoint_goal_tolerance", default_value="1.0"),
            DeclareLaunchArgument("waypoint_edit_radius", default_value="1.5"),
            DeclareLaunchArgument("waypoint_enable_interactive_markers", default_value="true"),
            DeclareLaunchArgument("waypoint_interactive_marker_ns", default_value="waypoint_editor"),
            DeclareLaunchArgument("global_frame", default_value="map"),
            DeclareLaunchArgument("robot_frame", default_value="base_footprint"),
            DeclareLaunchArgument("odom_frame", default_value="camera_init_footprint"),
            DeclareLaunchArgument("path_target_frame", default_value="camera_init_footprint"),
            DeclareLaunchArgument("lookahead", default_value="1.8"),
            DeclareLaunchArgument("pure_pursuit_rate", default_value="10.0"),
            DeclareLaunchArgument("rl_hz", default_value="10.0"),
            DeclareLaunchArgument("device", default_value="cuda"),
            DeclareLaunchArgument("require_localization_confidence", default_value="false"),
            DeclareLaunchArgument("adapter_path_timeout", default_value="1.2"),
            rviz_waypoints,
            pure_pursuit,
            rl_local_path,
            adapter,
            domain_bridge,
            pointcloud_to_laserscan_node,
        ]
    )
