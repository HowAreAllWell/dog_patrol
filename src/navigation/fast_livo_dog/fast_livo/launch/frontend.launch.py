#!/usr/bin/python3
# -- coding: utf-8 --

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    # 暴力强注 libusb 动态库，根治 Exit Code 127
    import platform
    if platform.machine() == "aarch64":
        os.environ['LD_PRELOAD'] = '/lib/aarch64-linux-gnu/libusb-1.0.so.0'
    else:
        os.environ['LD_PRELOAD'] = '/lib/x86_64-linux-gnu/libusb-1.0.so.0'
    
    pkg_share = get_package_share_directory("fast_livo")
    ws_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(pkg_share))))
    nav_root = os.environ.get(
        'DOG_PATROL_NAV_ROOT', os.path.join(ws_root, 'src', 'navigation', 'fast_livo_dog'))
    config_file_path = os.path.join(nav_root, 'config', 'device_parameters.yaml')
    if not os.path.exists(config_file_path):
        config_file_path = os.path.join(ws_root, 'src', 'navigation', 'fast_livo_dog', 'config', 'device_parameters.yaml')

    mid360_config_arg = DeclareLaunchArgument(
        'params_file',
        default_value=config_file_path,
        description='Full path to the ROS2 central parameters file to use for fast_livo nodes',
    )

    use_respawn_arg = DeclareLaunchArgument(
        'use_respawn', 
        default_value='True',
        description='Whether to respawn if a node crashes.'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (bag) clock if true'
    )

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        mid360_config_arg,
        use_respawn_arg,
        use_sim_time_arg,
        
        # 启动核心算法节点
        Node(
            package="fast_livo",
            executable="fastlivo_mapping",
            name="laserMapping",
            parameters=[
                params_file,
                {"use_sim_time": use_sim_time}
            ],
            output="screen"
        ),
    ])
