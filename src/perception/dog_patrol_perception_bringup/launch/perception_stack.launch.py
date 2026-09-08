"""Start the complete perception mission stack using the navigation camera."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def include(package, launch_file, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), "launch", launch_file])
        ),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    assets_root = LaunchConfiguration("assets_root")
    image_topic = LaunchConfiguration("image_topic")
    camera_frame = LaunchConfiguration("camera_frame")
    preview = LaunchConfiguration("preview")
    preview_window = LaunchConfiguration("preview_window")
    visualization_topic = LaunchConfiguration("visualization_topic")
    record = LaunchConfiguration("record")
    voice_helper = LaunchConfiguration("voice_helper")
    mission_state_topic = LaunchConfiguration("mission_state_topic")
    mission_event_topic = LaunchConfiguration("mission_event_topic")
    capability_status_topic = LaunchConfiguration("capability_status_topic")
    target_bbox_topic = LaunchConfiguration("target_bbox_topic")
    tracked_target_image_topic = LaunchConfiguration("tracked_target_image_topic")
    authorization_evidence_topic = LaunchConfiguration("authorization_evidence_topic")
    authorization_command_topic = LaunchConfiguration("authorization_command_topic")
    python_prefix = [
        EnvironmentVariable("DOG_PATROL_PYTHON", default_value="python3"),
        " ",
    ]

    tracking_params = PathJoinSubstitution([assets_root, "runtime", "perception_tracking.yaml"])
    tracker_config = PathJoinSubstitution([assets_root, "runtime", "bot_sort.yaml"])
    tracking_engine = PathJoinSubstitution(
        [assets_root, "tracking", "yolo26n_fp16_640.engine"]
    )
    face_config = PathJoinSubstitution([assets_root, "runtime", "face.yaml"])
    face_detector_engine = PathJoinSubstitution([assets_root, "face", "detector.engine"])
    face_recognition_engine = PathJoinSubstitution(
        [assets_root, "face", "recognition.engine"]
    )
    face_whitelist = PathJoinSubstitution([assets_root, "face", "whitelist"])
    voice_config = PathJoinSubstitution([assets_root, "runtime", "voice.yaml"])
    voice_model = PathJoinSubstitution(
        [assets_root, "voice", "vosk-model-small-en-us-0.15"]
    )

    tracking = Node(
        package="dog_patrol_perception_tracking",
        executable="dog_patrol_perception_tracking_node",
        name="dog_patrol_perception_tracking_node",
        output="screen",
        parameters=[
            tracking_params,
            {
                "use_sim_time": use_sim_time,
                "runtime.mode": "mission",
                "camera.input_mode": "ros_image",
                "camera.image_topic": image_topic,
                "perception.camera_optical_frame_id": camera_frame,
                "detector.runtime_path": tracking_engine,
                "tracker.config_path": tracker_config,
                "mission.state_topic": mission_state_topic,
                "mission.event_topic": mission_event_topic,
                "mission.selected_target_bbox_topic": target_bbox_topic,
                "perception.capability_status_topic": capability_status_topic,
                "target_image.topic": tracked_target_image_topic,
                "visualization.enable": ParameterValue(preview, value_type=bool),
                "visualization.publish_image": True,
                "visualization.image_topic": visualization_topic,
                "visualization.window": ParameterValue(preview_window, value_type=bool),
                "recording.enable": ParameterValue(record, value_type=bool),
            },
        ],
    )
    face = include(
        "dog_patrol_perception_face",
        "face.launch.py",
        {
            "config_file": face_config,
            "detector_engine": face_detector_engine,
            "recognition_engine": face_recognition_engine,
            "whitelist_dir": face_whitelist,
            "mission_state_topic": mission_state_topic,
            "capability_status_topic": capability_status_topic,
            "tracked_target_image_topic": tracked_target_image_topic,
            "authorization_evidence_topic": authorization_evidence_topic,
            "authorization_command_topic": authorization_command_topic,
            "provider": "face",
        },
    )
    voice = include(
        "dog_patrol_perception_voice",
        "voice.launch.py",
        {
            "model_dir": voice_model,
            "config_file": voice_config,
            "helper_path": voice_helper,
            "mission_state_topic": mission_state_topic,
            "capability_status_topic": capability_status_topic,
            "authorization_evidence_topic": authorization_evidence_topic,
            "authorization_command_topic": authorization_command_topic,
            "provider": "voice",
        },
    )
    readiness = Node(
        package="dog_patrol_perception_orchestrator",
        executable="perception_readiness",
        name="perception_readiness",
        output="screen",
        prefix=python_prefix,
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "mission_state_topic": mission_state_topic,
                "mission_event_topic": mission_event_topic,
                "capability_status_topic": capability_status_topic,
            }
        ],
    )
    authorization = Node(
        package="dog_patrol_perception_orchestrator",
        executable="perception_authorization",
        name="perception_authorization",
        output="screen",
        prefix=python_prefix,
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "mission_state_topic": mission_state_topic,
                "mission_event_topic": mission_event_topic,
                "authorization_evidence_topic": authorization_evidence_topic,
                "authorization_command_topic": authorization_command_topic,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "assets_root",
                default_value=EnvironmentVariable(
                    "DOG_PATROL_ASSETS_ROOT",
                    default_value=(
                        "/home/orin/workspace/dog_patrol/src/perception/"
                        "dog_patrol_perception_assets_20260813"
                    ),
                ),
            ),
            DeclareLaunchArgument("image_topic", default_value="/left_camera/image_raw"),
            DeclareLaunchArgument("camera_frame", default_value="camera_link"),
            DeclareLaunchArgument("preview", default_value="true"),
            DeclareLaunchArgument("preview_window", default_value="false"),
            DeclareLaunchArgument(
                "visualization_topic",
                default_value="/perception/tracking_overlay",
            ),
            DeclareLaunchArgument("record", default_value="false"),
            DeclareLaunchArgument(
                "voice_helper",
                default_value=PathJoinSubstitution(
                    [
                        assets_root,
                        "..",
                        "dog_patrol_perception_voice",
                        "dog_patrol_perception_voice",
                        "assets",
                        "r818_pcm_base64_aarch64",
                    ]
                ),
            ),
            DeclareLaunchArgument("mission_state_topic", default_value="/mission/state"),
            DeclareLaunchArgument("mission_event_topic", default_value="/mission/event"),
            DeclareLaunchArgument(
                "capability_status_topic",
                default_value="/perception/capability_status",
            ),
            DeclareLaunchArgument(
                "target_bbox_topic",
                default_value="/perception/selected_target_bbox",
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
            tracking,
            face,
            voice,
            readiness,
            authorization,
        ]
    )
