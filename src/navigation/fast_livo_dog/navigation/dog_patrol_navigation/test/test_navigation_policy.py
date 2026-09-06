from dog_patrol_navigation.navigation_policy import (
    compute_standoff_decision,
    navigation_policy,
)


def test_patrol_resumes_and_clears_target():
    policy = navigation_policy(1)
    assert policy.resume_patrol
    assert not policy.pause_patrol
    assert not policy.clear_navigation_path
    assert policy.reset_target


def test_confirm_keeps_patrol_running_while_fusing_target():
    policy = navigation_policy(2)
    assert not policy.pause_patrol
    assert not policy.clear_navigation_path
    assert policy.allow_target_fusion
    assert not policy.allow_target_motion


def test_approach_and_tracking_allow_motion():
    approach = navigation_policy(3)
    tracking = navigation_policy(5)
    assert approach.pause_patrol
    assert approach.clear_navigation_path
    assert approach.allow_target_fusion
    assert approach.allow_target_motion
    assert tracking.pause_patrol
    assert not tracking.clear_navigation_path
    assert tracking.allow_target_fusion
    assert tracking.allow_target_motion


def test_verify_stops_motion_but_keeps_target_fusion():
    policy = navigation_policy(4)
    assert policy.pause_patrol
    assert policy.clear_navigation_path
    assert policy.allow_target_fusion
    assert not policy.allow_target_motion


def test_recovery_returns_before_resuming_waypoint():
    recovery = navigation_policy(6)
    assert not recovery.resume_patrol
    assert recovery.pause_patrol
    assert recovery.clear_navigation_path
    assert recovery.reset_target
    assert not recovery.allow_target_fusion
    assert not recovery.allow_target_motion

    patrol = navigation_policy(1, previous_mission_state=6)
    assert patrol.resume_patrol
    assert not patrol.clear_navigation_path
    assert patrol.reset_target


def test_unknown_state_forces_safe_stop_actions():
    policy = navigation_policy(99)
    assert policy.safe_stop
    assert policy.pause_patrol
    assert policy.clear_navigation_path
    assert not policy.resume_patrol
    assert not policy.allow_target_fusion
    assert not policy.allow_target_motion


def test_moving_target_updates_three_meter_standoff_goal():
    first = compute_standoff_decision(0.0, 0.0, 8.0, 0.0, 3.0, 0.25)
    moved = compute_standoff_decision(0.0, 0.0, 9.0, 1.0, 3.0, 0.25)

    assert not first.should_stop
    assert first.goal_x == 5.0
    assert first.goal_y == 0.0
    assert not moved.should_stop
    assert moved.goal_x > first.goal_x
    assert moved.goal_y > first.goal_y


def test_standoff_stops_inside_three_meter_tolerance_band():
    decision = compute_standoff_decision(1.0, 2.0, 4.08, 2.0, 3.0, 0.10)

    assert decision.should_stop
    assert decision.distance == 3.08
    assert decision.goal_x == 1.0
    assert decision.goal_y == 2.0


def test_planning_goal_can_be_near_target_while_stop_range_remains_three_metres():
    decision = compute_standoff_decision(
        0.0,
        0.0,
        8.0,
        0.0,
        3.0,
        0.10,
        planning_goal_distance=1.0,
        measured_distance=7.8,
    )

    assert not decision.should_stop
    assert decision.goal_x == 7.0
    assert decision.goal_y == 0.0
    assert decision.distance == 7.8
    assert decision.map_distance == 8.0


def test_sensor_distance_controls_stop_when_map_distance_disagrees():
    decision = compute_standoff_decision(
        0.0,
        0.0,
        4.0,
        0.0,
        3.0,
        0.10,
        planning_goal_distance=1.0,
        measured_distance=3.05,
    )

    assert decision.should_stop
    assert decision.distance == 3.05
    assert decision.map_distance == 4.0
