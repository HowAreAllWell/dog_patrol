from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from dog_patrol_navigation.patrol_recovery_controller import (
    PatrolRecoveryController,
    RecoveryConfig,
)


def pose(x, y):
    result = PoseStamped()
    result.pose.position.x = x
    result.pose.position.y = y
    return result


def make_recovery(now):
    return PatrolRecoveryController(
        RecoveryConfig(
            position_tolerance=0.25,
            linear_speed=0.05,
            angular_speed=0.1,
            odom_timeout=0.5,
            hold_time=0.5,
            republish_period=0.2,
            request_retry_period=0.5,
        ),
        lambda: now[0],
    )


def test_recovery_requests_path_and_republishes_cached_path():
    now = [0.0]
    recovery = make_recovery(now)
    recovery.begin(pose(5.0, 2.0))
    command = recovery.tick(pose(0.0, 0.0), 0.0, 0.0, 0.1, False)
    assert command.request_path

    path = Path()
    path.poses.append(pose(1.0, 0.0))
    recovery.set_path_result(path)
    now[0] = 0.1
    command = recovery.tick(pose(0.0, 0.0), 0.2, 0.0, 0.1, False)
    assert command.publish_path is path
    now[0] = 0.21
    assert recovery.tick(pose(0.0, 0.0), 0.2, 0.0, 0.1, False).publish_path is None
    now[0] = 0.31
    assert recovery.tick(pose(0.0, 0.0), 0.2, 0.0, 0.1, False).publish_path is path


def test_recovery_completes_after_stopped_hold_and_is_idempotent():
    now = [0.0]
    recovery = make_recovery(now)
    recovery.begin(pose(1.0, 1.0))
    first = recovery.tick(pose(1.1, 1.1), 0.01, 0.01, 0.1, False)
    assert not first.complete

    now[0] = 0.51
    second = recovery.tick(pose(1.1, 1.1), 0.01, 0.01, 0.1, False)
    assert second.complete
    assert recovery.mark_complete() is True
    assert recovery.mark_complete() is False
    assert recovery.return_arrived


def test_recovery_without_interruption_pose_completes_without_planner():
    now = [0.0]
    recovery = make_recovery(now)
    recovery.begin(None)
    command = recovery.tick(None, 0.0, 0.0, 0.1, False)
    assert command.complete
    assert command.request_path is False
    assert command.should_stop is False


def test_recovery_hold_stops_without_republishing_the_motion_path():
    now = [0.0]
    recovery = make_recovery(now)
    recovery.begin(pose(1.0, 1.0))

    path = Path()
    path.poses.append(pose(1.0, 1.0))
    recovery.set_path_result(path)

    command = recovery.tick(pose(1.1, 1.1), 0.01, 0.01, 0.1, False)
    assert command.complete is False
    assert command.should_stop is True
    assert command.publish_path is None


def test_recovery_throttles_path_retries_after_a_failed_request():
    now = [0.0]
    recovery = make_recovery(now)
    recovery.begin(pose(5.0, 2.0))

    first = recovery.tick(pose(0.0, 0.0), 0.0, 0.0, 0.1, False)
    assert first.request_path
    recovery.set_path_requested(True)
    recovery.set_path_requested(False)

    now[0] = 0.2
    assert not recovery.tick(pose(0.0, 0.0), 0.0, 0.0, 0.1, False).request_path

    now[0] = 0.5
    assert recovery.tick(pose(0.0, 0.0), 0.0, 0.0, 0.1, False).request_path


def test_recovery_does_not_republish_cached_motion_path_without_robot_pose():
    now = [0.0]
    recovery = make_recovery(now)
    recovery.begin(pose(5.0, 2.0))
    path = Path()
    path.poses.append(pose(1.0, 0.0))
    recovery.set_path_result(path)

    command = recovery.tick(None, 0.0, 0.0, 0.1, False)

    assert command.should_stop
    assert command.publish_path is None
    assert not command.request_path


def test_recovery_stops_inside_tolerance_until_robot_is_stable():
    now = [0.0]
    recovery = make_recovery(now)
    recovery.begin(pose(1.0, 1.0))
    path = Path()
    path.poses.append(pose(1.0, 1.0))
    recovery.set_path_result(path)

    command = recovery.tick(pose(1.1, 1.1), 0.2, 0.0, 0.1, False)

    assert command.should_stop
    assert command.publish_path is None
    assert not command.complete
