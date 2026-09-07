import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("dog_patrol_navigation")
    ws_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(pkg_share))))
    nav_root = os.environ.get(
        "DOG_PATROL_NAV_ROOT",
        os.path.join(ws_root, "src", "navigation", "fast_livo_dog"),
    )
    default_params = PathJoinSubstitution(
        [FindPackageShare("dog_patrol_navigation"), "config", "m20_patrol_navigation.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Navigation integration parameter file",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("mission_path_topic", default_value="/mission_global_path"),
            DeclareLaunchArgument("resume_topic", default_value="/waypoint_sequence/resume"),
            DeclareLaunchArgument(
                "device_parameters_file",
                default_value=PathJoinSubstitution(
                    [EnvironmentVariable("DOG_PATROL_NAV_ROOT", default_value=nav_root),
                     "config", "device_parameters.yaml"]
                ),
            ),
            Node(
                package="dog_patrol_navigation",
                executable="navigation_mission_coordinator",
                name="navigation_mission_coordinator",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "calibration.device_parameters_file": LaunchConfiguration(
                            "device_parameters_file"
                        ),
                        "topics.global_path": LaunchConfiguration("mission_path_topic"),
                        "topics.resume_patrol": LaunchConfiguration("resume_topic"),
                    },
                ],
            ),
        ]
    )
