from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    state_topic = LaunchConfiguration("state_topic")
    event_topic = LaunchConfiguration("event_topic")
    task_state_topic = LaunchConfiguration("task_state_topic")
    waypoint_state_topic = LaunchConfiguration("waypoint_state_topic")
    task_command_service = LaunchConfiguration("task_command_service")
    state_publish_rate = LaunchConfiguration("state_publish_rate")
    confirm_target_timeout = LaunchConfiguration("confirm_target_timeout")
    intruder_tracking_timeout = LaunchConfiguration("intruder_tracking_timeout")
    patrol_recovery_timeout = LaunchConfiguration("patrol_recovery_timeout")

    supervisor = Node(
        package="dog_patrol_manager",
        executable="mission_supervisor",
        name="mission_supervisor",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "state_topic": state_topic,
                "event_topic": event_topic,
                "task_state_topic": task_state_topic,
                "waypoint_state_topic": waypoint_state_topic,
                "task_command_service": task_command_service,
                "state_publish_rate": state_publish_rate,
                "confirm_target_timeout": confirm_target_timeout,
                "intruder_tracking_timeout": intruder_tracking_timeout,
                "patrol_recovery_timeout": patrol_recovery_timeout,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("state_topic", default_value="/mission/state"),
            DeclareLaunchArgument("event_topic", default_value="/mission/event"),
            DeclareLaunchArgument("task_state_topic", default_value="/patrol_task/state"),
            DeclareLaunchArgument("waypoint_state_topic", default_value="/waypoint_sequence/state"),
            DeclareLaunchArgument("task_command_service", default_value="/mission/task_command"),
            # Publish the authoritative mission snapshot frequently enough for
            # perception, navigation, and the UI to observe state changes.
            DeclareLaunchArgument("state_publish_rate", default_value="10.0"),
            DeclareLaunchArgument("confirm_target_timeout", default_value="5.0"),
            DeclareLaunchArgument(
                "intruder_tracking_timeout",
                default_value="60.0",
                description="Maximum seconds in TRACK_INTRUDER before restoring patrol",
            ),
            DeclareLaunchArgument("patrol_recovery_timeout", default_value="10.0"),
            supervisor,
        ]
    )
