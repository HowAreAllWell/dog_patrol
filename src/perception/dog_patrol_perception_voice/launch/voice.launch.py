from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    model_dir = LaunchConfiguration("model_dir")
    config_file = LaunchConfiguration("config_file")
    helper_path = LaunchConfiguration("helper_path")
    state_topic = LaunchConfiguration("mission_state_topic")
    capability_status_topic = LaunchConfiguration("capability_status_topic")
    evidence_topic = LaunchConfiguration("authorization_evidence_topic")
    command_topic = LaunchConfiguration("authorization_command_topic")
    provider = LaunchConfiguration("provider")
    state_timeout = LaunchConfiguration("state_timeout")
    python_prefix = [
        EnvironmentVariable("DOG_PATROL_PYTHON", default_value="python3"),
        " ",
    ]

    readiness = Node(
        package="dog_patrol_perception_voice",
        executable="perception_voice_readiness",
        name="perception_voice_readiness",
        output="screen",
        prefix=python_prefix,
        on_exit=Shutdown(reason="voice readiness exited"),
        parameters=[
            {
                "mission_state_topic": state_topic,
                "capability_status_topic": capability_status_topic,
                "capability": provider,
                "model_dir": model_dir,
                "config_file": config_file,
                "helper_path": helper_path,
                "state_timeout": ParameterValue(state_timeout, value_type=float),
            }
        ],
    )
    evidence = Node(
        package="dog_patrol_perception_voice",
        executable="perception_voice_provider",
        name="perception_voice_provider",
        output="screen",
        prefix=python_prefix,
        on_exit=Shutdown(reason="voice provider exited"),
        parameters=[
            {
                "mission_state_topic": state_topic,
                "authorization_evidence_topic": evidence_topic,
                "authorization_command_topic": command_topic,
                "provider": provider,
                "model_dir": model_dir,
                "config_file": config_file,
                "helper_path": helper_path,
                "state_timeout": ParameterValue(state_timeout, value_type=float),
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
            DeclareLaunchArgument(
                "authorization_command_topic",
                default_value="/perception/authorization_command",
            ),
            DeclareLaunchArgument("provider", default_value="voice"),
            DeclareLaunchArgument("state_timeout", default_value="0.75"),
            readiness,
            evidence,
        ]
    )
