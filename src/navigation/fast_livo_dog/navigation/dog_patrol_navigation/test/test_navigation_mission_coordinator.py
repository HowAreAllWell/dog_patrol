from types import SimpleNamespace

import numpy as np
from dog_patrol_interfaces.msg import MissionEvent, MissionState, PatrolTaskState

from dog_patrol_navigation import navigation_mission_coordinator as coordinator_module
from dog_patrol_navigation.navigation_planner_client import PlannerRequest


NavigationMissionCoordinator = coordinator_module.NavigationMissionCoordinator
execution_mode = getattr(coordinator_module, "execution_mode", lambda mission, task: "missing")


def mission(seq, state, target=0):
    return SimpleNamespace(state_seq=seq, state=state, target_id=target)


def task(seq, state, mode, home_valid=False, outcome=0):
    return SimpleNamespace(
        state_seq=seq,
        task_state=state,
        navigation_mode=mode,
        home_pose_valid=home_valid,
        last_task_outcome=outcome,
    )


def test_return_execution_requires_matching_authoritative_snapshots():
    returning = task(
        12,
        PatrolTaskState.TASK_RETURNING,
        PatrolTaskState.NAVIGATION_MISSION,
        home_valid=True,
    )
    ready = mission(12, MissionState.READY)
    assert execution_mode(ready, returning) == "return"
    assert execution_mode(mission(11, MissionState.READY), returning) == "stop"
    assert execution_mode(mission(12, MissionState.PATROL), returning) == "stop"
    assert execution_mode(
        ready,
        task(12, PatrolTaskState.TASK_RETURNING, PatrolTaskState.NAVIGATION_STOP, home_valid=True),
    ) == "stop"
    assert execution_mode(
        ready,
        task(12, PatrolTaskState.TASK_RETURNING, PatrolTaskState.NAVIGATION_MISSION, home_valid=False),
    ) == "stop"


def test_running_mission_modes_require_matching_parent_task_state():
    running_waypoint = task(
        4,
        PatrolTaskState.TASK_RUNNING,
        PatrolTaskState.NAVIGATION_WAYPOINT,
        home_valid=True,
    )
    running_mission = task(
        5,
        PatrolTaskState.TASK_RUNNING,
        PatrolTaskState.NAVIGATION_MISSION,
        home_valid=True,
    )
    assert execution_mode(mission(4, MissionState.PATROL), running_waypoint) == "mission"
    assert execution_mode(mission(5, MissionState.RECOVER_PATROL), running_mission) == "mission"
    assert execution_mode(mission(5, MissionState.RECOVER_PATROL), running_waypoint) == "stop"
    assert execution_mode(mission(4, MissionState.PATROL), running_mission) == "stop"


def test_target_position_ready_captures_interruption_pose_before_event():
    actions = []
    node = SimpleNamespace(
        _capture_patrol_interruption_pose=lambda: actions.append("capture") or True,
        _publish_event=lambda event, detail: actions.append((event, detail)),
    )
    NavigationMissionCoordinator._on_target_position_ready(node)
    assert actions[0] == "capture"
    assert actions[1][0] == MissionEvent.TARGET_POSITION_READY


def test_target_position_ready_is_not_published_when_pose_capture_fails():
    events = []
    node = SimpleNamespace(
        _capture_patrol_interruption_pose=lambda: False,
        _publish_event=lambda event, detail: events.append((event, detail)),
    )
    NavigationMissionCoordinator._on_target_position_ready(node)
    assert events == []


def planner_request(seq, target, kind):
    return PlannerRequest(3, seq, target, np.zeros(3), kind)


def test_return_planner_result_requires_matching_returning_snapshot():
    node = SimpleNamespace(
        _mission_state=mission(12, MissionState.READY),
        _task_state=task(
            12,
            PatrolTaskState.TASK_RETURNING,
            PatrolTaskState.NAVIGATION_MISSION,
            home_valid=True,
        ),
    )
    assert NavigationMissionCoordinator._planner_request_matches_current_mission(
        node, planner_request(12, 0, "return")
    )
    assert not NavigationMissionCoordinator._planner_request_matches_current_mission(
        node, planner_request(11, 0, "return")
    )
    node._task_state = task(
        12,
        PatrolTaskState.TASK_NO_TASK,
        PatrolTaskState.NAVIGATION_STOP,
    )
    assert not NavigationMissionCoordinator._planner_request_matches_current_mission(
        node, planner_request(12, 0, "return")
    )


def test_pause_recovery_planner_allows_zero_target_with_running_parent():
    node = SimpleNamespace(
        _mission_state=mission(20, MissionState.RECOVER_PATROL, 0),
        _task_state=task(
            20,
            PatrolTaskState.TASK_RUNNING,
            PatrolTaskState.NAVIGATION_MISSION,
            home_valid=True,
        ),
    )
    assert NavigationMissionCoordinator._planner_request_matches_current_mission(
        node, planner_request(20, 0, "recovery")
    )


def test_recovery_completion_only_reports_event_and_does_not_command_waypoint():
    events = []
    node = SimpleNamespace(
        _recovery=SimpleNamespace(return_arrived=False, mark_complete=lambda: True),
        _publish_empty_path=lambda: None,
        _publish_event=lambda event, detail: events.append((event, detail)),
        get_logger=lambda: SimpleNamespace(info=lambda message: None),
    )
    NavigationMissionCoordinator._finish_recovery_return(node)
    assert events == [
        (
            MissionEvent.PATROL_RECOVERY_COMPLETE,
            "patrol interruption pose reached",
        )
    ]


def test_failed_return_replan_clears_the_cached_motion_path(monkeypatch):
    actions = []
    monkeypatch.setattr(
        coordinator_module,
        "now_time",
        lambda _node: coordinator_module.Time(nanoseconds=1_000_000_000),
    )
    node = SimpleNamespace(
        _planner_request_matches_current_mission=lambda _request: True,
        _task_return=SimpleNamespace(
            set_path_requested=lambda requested: actions.append(
                ("requested", requested)
            ),
            clear_path=lambda: actions.append("clear"),
        ),
        _recovery=SimpleNamespace(set_path_requested=lambda _requested: None),
        _publish_empty_path=lambda: actions.append("empty"),
        _planner_failure_start=None,
        _planner_failure_reason="",
        technical_error_timeout=3.0,
        _publish_event=lambda *_args, **_kwargs: actions.append("event"),
    )

    NavigationMissionCoordinator._record_planner_failure(
        node, "no current route", planner_request(12, 0, "return")
    )

    assert "clear" in actions
    assert "empty" in actions
    assert "event" not in actions
