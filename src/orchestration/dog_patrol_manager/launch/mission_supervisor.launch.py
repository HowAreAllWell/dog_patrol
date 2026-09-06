from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    state_topic = LaunchConfiguration("state_topic")
    event_topic = LaunchConfiguration("event_topic")
    state_publish_rate = LaunchConfiguration("state_publish_rate")
    confirm_target_timeout = LaunchConfiguration("confirm_target_timeout")
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
                "state_publish_rate": state_publish_rate,
                "confirm_target_timeout": confirm_target_timeout,
                "patrol_recovery_timeout": patrol_recovery_timeout,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("state_topic", default_value="/mission/state"),
            DeclareLaunchArgument("event_topic", default_value="/mission/event"),
            # Publish the authoritative mission snapshot frequently enough for
            # perception, navigation, and the UI to observe state changes.
            DeclareLaunchArgument("state_publish_rate", default_value="10.0"),
            DeclareLaunchArgument("confirm_target_timeout", default_value="5.0"),
            DeclareLaunchArgument("patrol_recovery_timeout", default_value="10.0"),
            supervisor,
        ]
    )
