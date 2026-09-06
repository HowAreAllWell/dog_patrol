"""Real-robot tracking-only perception test launch.

Starts real person tracking, synthetic face/voice capability readiness, and
the perception readiness aggregator. It intentionally does not start face,
voice, or authorization providers.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import EnvironmentVariable, FindExecutable, LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    assets_root = LaunchConfiguration("assets_root")
    image_topic = LaunchConfiguration("image_topic")
    camera_frame = LaunchConfiguration("camera_frame")
    preview = LaunchConfiguration("preview")
    record = LaunchConfiguration("record")
    fake_nodes_script = LaunchConfiguration("fake_nodes_script")
    mission_state_topic = LaunchConfiguration("mission_state_topic")
    mission_event_topic = LaunchConfiguration("mission_event_topic")
    capability_status_topic = LaunchConfiguration("capability_status_topic")
    target_bbox_topic = LaunchConfiguration("target_bbox_topic")
    tracked_target_image_topic = LaunchConfiguration("tracked_target_image_topic")

    tracking_params = PathJoinSubstitution(
        [assets_root, "runtime", "perception_tracking.yaml"]
    )
    tracker_config = PathJoinSubstitution([assets_root, "runtime", "bot_sort.yaml"])
    tracking_engine = PathJoinSubstitution(
        [assets_root, "tracking", "yolo26n_fp16_640.engine"]
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
                "recording.enable": ParameterValue(record, value_type=bool),
            },
        ],
    )

    fake_capabilities = ExecuteProcess(
        cmd=[
            FindExecutable(name="python3"),
            fake_nodes_script,
            "--role",
            "capabilities",
        ],
        output="screen",
        emulate_tty=True,
    )

    readiness = Node(
        package="dog_patrol_perception_orchestrator",
        executable="perception_readiness",
        name="perception_readiness",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "mission_state_topic": mission_state_topic,
                "mission_event_topic": mission_event_topic,
                "capability_status_topic": capability_status_topic,
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
                        "/mnt/nvme/workspace/dog_patrol/src/perception/"
                        "dog_patrol_perception_assets_20260813"
                    ),
                ),
            ),
            DeclareLaunchArgument(
                "fake_nodes_script",
                default_value=(
                    "/mnt/nvme/workspace/dog_patrol/tools/fake_integration/fake_nodes.py"
                ),
            ),
            DeclareLaunchArgument("image_topic", default_value="/left_camera/image_raw"),
            DeclareLaunchArgument("camera_frame", default_value="camera_link"),
            DeclareLaunchArgument("preview", default_value="true"),
            DeclareLaunchArgument("record", default_value="false"),
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
            tracking,
            fake_capabilities,
            readiness,
        ]
    )
