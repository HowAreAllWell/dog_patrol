from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('vision_demo_host')
    params = os.path.join(pkg_share, 'config', 'demo_params.yaml')

    return LaunchDescription([
        Node(
            package='vision_demo_host',
            executable='vision_demo_node',
            name='vision_demo_host_node',
            output='screen',
            parameters=[params],
        )
    ])
