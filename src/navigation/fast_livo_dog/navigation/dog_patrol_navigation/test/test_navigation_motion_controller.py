import pytest

from dog_patrol_navigation.navigation_motion_controller import (
    MotionConfig,
    TargetMotionController,
)


def make_motion(now):
    return TargetMotionController(
        MotionConfig(
            approach_distance=3.0,
            tracking_distance=3.0,
            planning_goal_distance=1.0,
            arrival_tolerance=0.1,
            arrival_linear_speed=0.05,
            arrival_angular_speed=0.1,
            arrival_hold_time=0.5,
            approach_replan_period=0.5,
            tracking_replan_period=0.5,
            target_replan_distance=0.25,
        ),
        lambda: now[0],
    )


def test_motion_requests_a_standoff_goal_and_respects_replan_period():
    now = [0.0]
    motion = make_motion(now)
    first = motion.tick(
        3, 3, 5, [8.0, 0.0, 0.0], [0.0, 0.0, 0.0],
        8.0, 0.0, 0.0, 0.0, False,
    )

    assert first.goal.tolist() == [7.0, 0.0, 0.0]
    motion.mark_plan_requested([8.0, 0.0, 0.0])
    now[0] = 0.2
    held = motion.tick(
        3, 3, 5, [8.0, 0.0, 0.0], [0.0, 0.0, 0.0],
        8.0, 0.0, 0.0, 0.0, False,
    )
    assert held.goal is None

    now[0] = 0.3
    moved = motion.tick(
        3, 3, 5, [8.3, 0.0, 0.0], [0.0, 0.0, 0.0],
        8.3, 0.0, 0.0, 0.0, False,
    )
    assert moved.goal is not None
    assert moved.goal[0] == pytest.approx(7.3)


def test_motion_reports_arrival_only_after_fresh_stopped_hold():
    now = [0.0]
    motion = make_motion(now)
    first = motion.tick(
        3, 3, 5, [3.08, 0.0, 0.0], [0.0, 0.0, 0.0],
        3.08, 0.01, 0.01, 0.1, False,
    )
    assert first.should_stop
    assert not first.arrived

    now[0] = 0.49
    second = motion.tick(
        3, 3, 5, [3.08, 0.0, 0.0], [0.0, 0.0, 0.0],
        3.08, 0.01, 0.01, 0.1, False,
    )
    assert not second.arrived

    now[0] = 0.51
    third = motion.tick(
        3, 3, 5, [3.08, 0.0, 0.0], [0.0, 0.0, 0.0],
        3.08, 0.01, 0.01, 0.1, False,
    )
    assert third.arrived
    assert motion.arrival_reported


def test_motion_clears_arrival_hold_when_odom_is_stale():
    now = [0.0]
    motion = make_motion(now)
    motion.tick(
        3, 3, 5, [3.08, 0.0, 0.0], [0.0, 0.0, 0.0],
        3.08, 0.01, 0.01, 0.1, False,
    )
    now[0] = 0.6
    stale = motion.tick(
        3, 3, 5, [3.08, 0.0, 0.0], [0.0, 0.0, 0.0],
        3.08, 0.01, 0.01, 0.6, False,
    )
    assert stale.should_stop
    assert not stale.arrived


def test_motion_invalidates_only_the_last_plan():
    now = [0.0]
    motion = make_motion(now)
    motion.mark_plan_requested([8.0, 0.0, 0.0])
    motion.invalidate_last_plan()
    command = motion.tick(
        3, 3, 5, [8.0, 0.0, 0.0], [0.0, 0.0, 0.0],
        8.0, 0.0, 0.0, 0.0, False,
    )
    assert command.goal is not None


def test_arrival_reset_preserves_existing_planner_throttle():
    now = [0.0]
    motion = make_motion(now)
    motion.mark_plan_requested([8.0, 0.0, 0.0])

    motion.reset_arrival()
    now[0] = 0.2
    command = motion.tick(
        5, 3, 5, [8.0, 0.0, 0.0], [0.0, 0.0, 0.0],
        8.0, 0.0, 0.0, 0.0, False,
    )

    assert command.goal is None
    assert motion.last_planned_target.tolist() == [8.0, 0.0, 0.0]
