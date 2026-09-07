from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


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
        parameters=[
            {
                "mission_state_topic": state_topic,
                "capability_status_topic": capability_status_topic,
                "capability": provider,
                "config_file": config_file,
                "detector_engine": detector_engine,
                "recognition_engine": recognition_engine,
                "whitelist_dir": whitelist_dir,
            }
        ],
    )
    evidence = Node(
        package="dog_patrol_perception_face",
        executable="perception_face_provider",
        name="perception_face_provider",
        output="screen",
        prefix=python_prefix,
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
            readiness,
            evidence,
        ]
    )
