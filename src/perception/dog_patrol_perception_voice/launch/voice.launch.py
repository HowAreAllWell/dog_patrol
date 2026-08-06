from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_dir = LaunchConfiguration("model_dir")
    config_file = LaunchConfiguration("config_file")
    helper_path = LaunchConfiguration("helper_path")
    state_topic = LaunchConfiguration("mission_state_topic")
    capability_status_topic = LaunchConfiguration("capability_status_topic")
    evidence_topic = LaunchConfiguration("authorization_evidence_topic")
    provider = LaunchConfiguration("provider")

    readiness = Node(
        package="dog_patrol_perception_voice",
        executable="perception_voice_readiness",
        name="perception_voice_readiness",
        output="screen",
        parameters=[
            {
                "mission_state_topic": state_topic,
                "capability_status_topic": capability_status_topic,
                "capability": provider,
                "model_dir": model_dir,
                "config_file": config_file,
                "helper_path": helper_path,
            }
        ],
    )
    evidence = Node(
        package="dog_patrol_perception_voice",
        executable="perception_voice_provider",
        name="perception_voice_provider",
        output="screen",
        parameters=[
            {
                "mission_state_topic": state_topic,
                "authorization_evidence_topic": evidence_topic,
                "provider": provider,
                "model_dir": model_dir,
                "config_file": config_file,
                "helper_path": helper_path,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("model_dir", default_value=""),
            DeclareLaunchArgument("config_file", default_value=""),
            DeclareLaunchArgument("helper_path", default_value=""),
            DeclareLaunchArgument("mission_state_topic", default_value="/mission/state"),
            DeclareLaunchArgument(
                "capability_status_topic",
                default_value="/perception/capability_status",
            ),
            DeclareLaunchArgument(
                "authorization_evidence_topic",
                default_value="/perception/authorization_evidence",
            ),
            DeclareLaunchArgument("provider", default_value="voice"),
            readiness,
            evidence,
        ]
    )
