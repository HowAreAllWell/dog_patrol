import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('dog_patrol_perception_tracking')
    params = os.path.join(pkg_share, 'config', 'perception_tracking_params.yaml')
    tracker_config = os.path.join(pkg_share, 'config', 'bot_sort.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=params),
        DeclareLaunchArgument('tracker_config', default_value=tracker_config),
        DeclareLaunchArgument('preview', default_value='false'),
        DeclareLaunchArgument('record', default_value='false'),
        Node(
            package='dog_patrol_perception_tracking',
            executable='dog_patrol_perception_tracking_node',
            name='dog_patrol_perception_tracking_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'tracker.config_path': LaunchConfiguration('tracker_config'),
                    'runtime.mode': 'standalone',
                    'visualization.enable': ParameterValue(
                        LaunchConfiguration('preview'), value_type=bool),
                    'recording.enable': ParameterValue(
                        LaunchConfiguration('record'), value_type=bool),
                },
            ],
        ),
    ])
