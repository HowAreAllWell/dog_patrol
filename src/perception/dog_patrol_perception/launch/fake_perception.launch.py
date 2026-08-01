from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="dog_patrol_perception",
                executable="fake_perception",
                name="fake_perception",
                output="screen",
            )
        ]
    )
