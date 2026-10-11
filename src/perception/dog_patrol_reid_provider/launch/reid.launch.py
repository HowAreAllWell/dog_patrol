import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('dog_patrol_reid_provider'), 'config',
        'reid.yaml')
    return LaunchDescription([
        Node(package='dog_patrol_reid_provider',
             executable='reid_node',
             name='perception_reid',
             output='screen',
             parameters=[default_config]),
    ])
