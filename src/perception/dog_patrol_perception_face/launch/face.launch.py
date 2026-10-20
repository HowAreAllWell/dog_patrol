from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    detector_engine = LaunchConfiguration("detector_engine")
    recognition_engine = LaunchConfiguration("recognition_engine")
    whitelist_dir = LaunchConfiguration("whitelist_dir")
    config_file = LaunchConfiguration("config_file")
    state_topic = LaunchConfiguration("mission_state_topic")
    capability_status_topic = LaunchConfiguration("capability_status_topic")
    crop_topic = LaunchConfiguration("tracked_target_image_topic")
    evidence_topic = LaunchConfiguration("authorization_evidence_topic")
    command_topic = LaunchConfiguration("authorization_command_topic")
    provider = LaunchConfiguration("provider")
    state_timeout = LaunchConfiguration("state_timeout")
    photos_root = LaunchConfiguration("photos_root")
    photos_enabled = LaunchConfiguration("photos_enabled")
    python_prefix = [
        EnvironmentVariable("DOG_PATROL_PYTHON", default_value="python3"),
        " ",
    ]

    readiness = Node(
        package="dog_patrol_perception_face",
        executable="perception_face_readiness",
        name="perception_face_readiness",
        output="screen",
        prefix=python_prefix,
        on_exit=Shutdown(reason="face readiness exited"),
        parameters=[
            {
                "mission_state_topic": state_topic,
                "capability_status_topic": capability_status_topic,
                "capability": provider,
                "config_file": config_file,
                "detector_engine": detector_engine,
                "recognition_engine": recognition_engine,
                "whitelist_dir": whitelist_dir,
                "state_timeout": ParameterValue(state_timeout, value_type=float),
                "photos_root": ParameterValue(photos_root, value_type=str),
                "photos_enabled": ParameterValue(photos_enabled, value_type=bool),
            }
        ],
    )
    evidence = Node(
        package="dog_patrol_perception_face",
        executable="perception_face_provider",
        name="perception_face_provider",
        output="screen",
        prefix=python_prefix,
        on_exit=Shutdown(reason="face provider exited"),
        parameters=[
            {
                "mission_state_topic": state_topic,
                "tracked_target_image_topic": crop_topic,
                "authorization_evidence_topic": evidence_topic,
                "authorization_command_topic": command_topic,
                "provider": provider,
                "config_file": config_file,
                "detector_engine": detector_engine,
                "recognition_engine": recognition_engine,
                "whitelist_dir": whitelist_dir,
                "state_timeout": ParameterValue(state_timeout, value_type=float),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("detector_engine", default_value=""),
            DeclareLaunchArgument("recognition_engine", default_value=""),
            DeclareLaunchArgument("whitelist_dir", default_value=""),
            DeclareLaunchArgument("config_file", default_value=""),
            DeclareLaunchArgument("mission_state_topic", default_value="/mission/state"),
            DeclareLaunchArgument(
                "capability_status_topic",
                default_value="/perception/capability_status",
            ),
            DeclareLaunchArgument(
                "tracked_target_image_topic",
                default_value="/perception/tracked_target_image",
            ),
            DeclareLaunchArgument(
                "authorization_evidence_topic",
                default_value="/perception/authorization_evidence",
            ),
            DeclareLaunchArgument(
                "authorization_command_topic",
                default_value="/perception/authorization_command",
            ),
            DeclareLaunchArgument("provider", default_value="face"),
            DeclareLaunchArgument("state_timeout", default_value="0.75"),
            DeclareLaunchArgument("photos_root", default_value=""),
            DeclareLaunchArgument("photos_enabled", default_value="true"),
            readiness,
            evidence,
        ]
    )
