from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('dog_patrol_perception_tracking')
    params = os.path.join(pkg_share, 'config', 'perception_tracking_params.yaml')
    tracker_config = os.path.join(pkg_share, 'config', 'bot_sort.yaml')

    return LaunchDescription([
        Node(
            package='dog_patrol_perception_tracking',
            executable='dog_patrol_perception_tracking_node',
            name='dog_patrol_perception_tracking_node',
            output='screen',
            parameters=[params, {'tracker.config_path': tracker_config}],
        )
    ])
