from types import SimpleNamespace

import numpy as np

from move.global_path_seq_publisher import GlobalPathSequencePublisher


def test_resume_restores_cached_path_and_requests_fresh_plan_immediately():
    calls = []
    node = SimpleNamespace(
        waypoints=[object()],
        paused=True,
        sequence_done=False,
        sequence_version=3,
        _invalidate_planning_request=lambda reason: calls.append(("invalidate", reason)),
        _publish_cached_path=lambda: 12,
        _publish_empty_paths=lambda: calls.append(("empty", None)),
        _publish_waypoints=lambda: calls.append(("waypoints", None)),
        _publish_status=lambda detail: calls.append(("status", detail)),
        _on_timer=lambda: calls.append(("timer", None)),
    )

    GlobalPathSequencePublisher._on_resume(node, None)

    assert not node.paused
    assert not node.sequence_done
    assert node.sequence_version == 4
    assert calls[-1] == ("timer", None)
    assert ("status", "resumed; restored 12 cached path poses") in calls


def test_resume_does_not_reopen_completed_waypoint_sequence():
    calls = []
    node = SimpleNamespace(
        waypoints=[object()],
        paused=True,
        sequence_done=True,
        sequence_version=3,
        _last_path=object(),
        _invalidate_planning_request=lambda reason: calls.append(("invalidate", reason)),
        _publish_empty_paths=lambda: calls.append(("empty", None)),
        _publish_waypoints=lambda: calls.append(("waypoints", None)),
        _publish_status=lambda detail: calls.append(("status", detail)),
        _on_timer=lambda: calls.append(("timer", None)),
    )

    GlobalPathSequencePublisher._on_resume(node, None)

    assert not node.paused
    assert node.sequence_done
    assert node.sequence_version == 4
    assert node._last_path is None
    assert ("status", "resumed; waypoint sequence already complete") in calls
    assert not any(kind == "timer" for kind, _ in calls)


def test_resume_from_current_discards_cached_path_and_replans_immediately():
    calls = []
    node = SimpleNamespace(
        waypoints=[object()],
        paused=True,
        sequence_done=False,
        sequence_version=8,
        _last_path=object(),
        _invalidate_planning_request=lambda reason: calls.append(("invalidate", reason)),
        _publish_empty_paths=lambda: calls.append(("empty", None)),
        _publish_waypoints=lambda: calls.append(("waypoints", None)),
        _publish_status=lambda detail: calls.append(("status", detail)),
        _on_timer=lambda: calls.append(("timer", None)),
    )

    GlobalPathSequencePublisher._on_resume_from_current(node, None)

    assert not node.paused
    assert not node.sequence_done
    assert node.sequence_version == 9
    assert node._last_path is None
    assert ("empty", None) in calls
    assert calls[-1] == ("timer", None)


def test_advance_if_reached_moves_to_next_waypoint():
    calls = []
    node = SimpleNamespace(
        auto_advance=True,
        paused=False,
        sequence_done=False,
        waypoints=[
            SimpleNamespace(x=1.0, y=2.0),
            SimpleNamespace(x=4.0, y=5.0),
        ],
        current_index=0,
        goal_tolerance=0.5,
        sequence_version=7,
        _interactive_markers_dirty=False,
        _last_path=object(),
        _lookup_robot_xy=lambda: np.asarray([1.2, 2.0]),
        _invalidate_planning_request=lambda reason: calls.append(("invalidate", reason)),
        _publish_empty_paths=lambda: calls.append(("empty", None)),
        _publish_waypoints=lambda: calls.append(("waypoints", None)),
        get_logger=lambda: SimpleNamespace(debug=lambda message: None),
    )

    advanced = GlobalPathSequencePublisher._advance_if_reached(node)

    assert advanced
    assert node.current_index == 1
    assert node.sequence_version == 8
    assert node._last_path is None
    assert node._interactive_markers_dirty
    assert ("invalidate", "advanced to next waypoint") in calls


def test_advance_if_reached_does_not_advance_outside_tolerance():
    calls = []
    node = SimpleNamespace(
        auto_advance=True,
        paused=False,
        sequence_done=False,
        waypoints=[SimpleNamespace(x=1.0, y=2.0)],
        current_index=0,
        goal_tolerance=0.5,
        _lookup_robot_xy=lambda: np.asarray([1.6, 2.0]),
    )

    advanced = GlobalPathSequencePublisher._advance_if_reached(node)

    assert not advanced
    assert node.current_index == 0
    assert calls == []


def test_removing_completed_waypoint_preserves_completion_progress():
    current_index, sequence_done, plan_affected = (
        GlobalPathSequencePublisher._progress_after_waypoint_removal(
            current_index=2,
            sequence_done=True,
            removed_index=2,
            remaining_count=2,
        )
    )

    assert current_index == 1
    assert sequence_done
    assert not plan_affected


def test_removing_middle_waypoint_keeps_previous_and_next_progress():
    current_index, sequence_done, plan_affected = (
        GlobalPathSequencePublisher._progress_after_waypoint_removal(
            current_index=1,
            sequence_done=False,
            removed_index=1,
            remaining_count=2,
        )
    )

    assert current_index == 1
    assert not sequence_done
    assert plan_affected


def test_removing_last_pending_waypoint_does_not_reopen_previous_waypoint():
    current_index, sequence_done, plan_affected = (
        GlobalPathSequencePublisher._progress_after_waypoint_removal(
            current_index=1,
            sequence_done=False,
            removed_index=1,
            remaining_count=1,
        )
    )

    assert current_index == 0
    assert sequence_done
    assert plan_affected


def test_editing_completed_or_future_waypoint_does_not_affect_active_goal():
    assert not GlobalPathSequencePublisher._waypoint_edit_affects_active_goal(
        current_index=2,
        sequence_done=True,
        edited_index=0,
    )
    assert not GlobalPathSequencePublisher._waypoint_edit_affects_active_goal(
        current_index=2,
        sequence_done=False,
        edited_index=0,
    )
    assert not GlobalPathSequencePublisher._waypoint_edit_affects_active_goal(
        current_index=2,
        sequence_done=False,
        edited_index=3,
    )
    assert GlobalPathSequencePublisher._waypoint_edit_affects_active_goal(
        current_index=2,
        sequence_done=False,
        edited_index=2,
    )
