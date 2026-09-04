from dog_patrol_navigation.navigation_fsm import (
    NavigationMode,
    NavigationStateMachine,
    compute_standoff_decision,
)


def test_patrol_resumes_and_clears_target():
    machine = NavigationStateMachine()
    actions = machine.enter(1, False, 2, 0)
    assert actions.mode == NavigationMode.PATROLLING
    assert actions.resume_patrol
    assert not actions.pause_patrol
    assert actions.clear_navigation_path
    assert actions.reset_target


def test_confirm_stops_before_fusion():
    machine = NavigationStateMachine()
    actions = machine.enter(2, False, 3, 41)
    assert actions.mode == NavigationMode.ACQUIRING_TARGET
    assert actions.pause_patrol
    assert actions.clear_navigation_path
    assert actions.allow_target_fusion
    assert not actions.allow_target_motion


def test_approach_and_tracking_allow_motion():
    machine = NavigationStateMachine()
    actions = machine.enter(3, False, 4, 41)
    assert actions.mode == NavigationMode.APPROACHING_TARGET
    assert actions.allow_target_motion
    assert not actions.clear_navigation_path

    actions = machine.enter(5, False, 6, 41)
    assert actions.mode == NavigationMode.TRACKING_TARGET
    assert actions.allow_target_motion


def test_blocked_always_forces_safe_stop():
    machine = NavigationStateMachine()
    actions = machine.enter(5, True, 7, 41)
    assert actions.mode == NavigationMode.SAFE_STOP
    assert actions.pause_patrol
    assert actions.clear_navigation_path
    assert not actions.allow_target_motion


def test_target_lost_recovery_keeps_business_mode_but_requires_new_measurement():
    machine = NavigationStateMachine()
    machine.enter(2, False, 10, 41)
    actions = machine.enter(2, True, 11, 41)
    assert actions.mode == NavigationMode.SAFE_STOP
    assert actions.clear_navigation_path
    assert not actions.allow_target_motion

    actions = machine.enter(2, False, 12, 41)
    assert actions.mode == NavigationMode.ACQUIRING_TARGET
    assert actions.allow_target_fusion
    assert not actions.allow_target_motion


def test_execution_error_does_not_become_tracking_mode():
    machine = NavigationStateMachine()
    actions = machine.enter(5, True, 20, 41)
    assert actions.mode == NavigationMode.SAFE_STOP
    assert not actions.allow_target_fusion
    assert not actions.allow_target_motion


def test_unknown_global_state_is_safe_stop():
    machine = NavigationStateMachine()
    actions = machine.enter(99, False, 30, 41)
    assert actions.mode == NavigationMode.SAFE_STOP
    assert actions.clear_navigation_path
    assert not actions.allow_target_motion


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
