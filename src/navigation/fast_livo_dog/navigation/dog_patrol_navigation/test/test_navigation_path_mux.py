from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time
from dog_patrol_interfaces.msg import PatrolTaskState
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from dog_patrol_navigation import navigation_path_mux as mux_module


MISSION_SOURCE = mux_module.MISSION_SOURCE
NavigationPathMux = mux_module.NavigationPathMux
NO_SOURCE = mux_module.NO_SOURCE
WAYPOINT_SOURCE = mux_module.WAYPOINT_SOURCE
path_source_for_task_state = getattr(
    mux_module, "path_source_for_task_state", lambda message: "missing"
)


def task_state(seq, task, mode, stamp=10, outcome=0):
    return SimpleNamespace(
        state_seq=seq,
        task_state=task,
        last_task_outcome=outcome,
        navigation_mode=mode,
        header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp, nanosec=0)),
    )


def path(stamp, *, nonempty=True):
    message = Path()
    message.header.stamp.sec = stamp
    if nonempty:
        message.poses.append(PoseStamped())
    return message


def node(source=NO_SOURCE, seen=False, seq=0):
    published = []
    instance = SimpleNamespace(
        _active_source=source,
        _task_state_seen=seen,
        _last_state_seq=seq,
        _state_signature=None,
        _state_conflicted=False,
        _barrier_stamp_ns=0,
        _state_timed_out=False,
        _publish=lambda message: published.append(message),
        _publish_empty=lambda: published.append(Path()),
        _refresh_heartbeat=lambda: None,
        get_logger=lambda: SimpleNamespace(
            info=lambda message: None,
            warning=lambda message: None,
            error=lambda message: None,
        ),
    )
    instance._path_is_after_barrier = lambda message: NavigationPathMux._path_is_after_barrier(instance, message)
    instance._stamp_ns = NavigationPathMux._stamp_ns
    instance._on_source_path = lambda source_name, message: NavigationPathMux._on_source_path(
        instance, source_name, message
    )
    return instance, published


def test_navigation_mode_is_the_only_path_source_selector():
    running = PatrolTaskState.TASK_RUNNING
    none = PatrolTaskState.TASK_OUTCOME_NONE
    assert path_source_for_task_state(
        task_state(1, running, PatrolTaskState.NAVIGATION_WAYPOINT, outcome=none)
    ) == WAYPOINT_SOURCE
    assert path_source_for_task_state(
        task_state(1, running, PatrolTaskState.NAVIGATION_MISSION, outcome=none)
    ) == MISSION_SOURCE
    assert path_source_for_task_state(
        task_state(1, PatrolTaskState.TASK_PAUSED, PatrolTaskState.NAVIGATION_STOP, outcome=none)
    ) == NO_SOURCE


def test_illegal_parent_task_combinations_fail_closed():
    none = PatrolTaskState.TASK_OUTCOME_NONE
    completed = PatrolTaskState.TASK_OUTCOME_COMPLETED
    cases = [
        task_state(0, PatrolTaskState.TASK_RUNNING, PatrolTaskState.NAVIGATION_WAYPOINT),
        task_state(1, 255, PatrolTaskState.NAVIGATION_WAYPOINT),
        task_state(1, PatrolTaskState.TASK_NO_TASK, PatrolTaskState.NAVIGATION_WAYPOINT),
        task_state(1, PatrolTaskState.TASK_PAUSED, PatrolTaskState.NAVIGATION_MISSION),
        task_state(1, PatrolTaskState.TASK_RETURNING, PatrolTaskState.NAVIGATION_STOP),
        task_state(1, PatrolTaskState.TASK_RUNNING, 255),
        task_state(1, PatrolTaskState.TASK_RUNNING, PatrolTaskState.NAVIGATION_WAYPOINT, outcome=completed),
        task_state(1, PatrolTaskState.TASK_NO_TASK, PatrolTaskState.NAVIGATION_STOP, outcome=255),
        task_state(1, PatrolTaskState.TASK_RETURNING, PatrolTaskState.NAVIGATION_MISSION, outcome=completed),
    ]
    for message in cases:
        assert path_source_for_task_state(message) == NO_SOURCE
    assert path_source_for_task_state(
        task_state(1, PatrolTaskState.TASK_NO_TASK, PatrolTaskState.NAVIGATION_STOP, outcome=none)
    ) == NO_SOURCE


def test_mux_drops_both_paths_until_first_parent_task_state():
    instance, published = node()
    NavigationPathMux._on_waypoint_path(instance, path(12))
    NavigationPathMux._on_mission_path(instance, path(12))
    assert published == []


def test_each_new_state_sequence_publishes_empty_even_when_source_is_unchanged():
    instance, published = node(WAYPOINT_SOURCE, True, 20)
    NavigationPathMux._on_task_state(
        instance,
        task_state(21, PatrolTaskState.TASK_RUNNING, PatrolTaskState.NAVIGATION_WAYPOINT, stamp=30),
    )
    assert instance._active_source == WAYPOINT_SOURCE
    assert instance._last_state_seq == 21
    assert instance._barrier_stamp_ns == 30_000_000_000
    assert len(published) == 1
    assert not published[0].poses


def test_waypoint_and_mission_paths_must_be_newer_than_state_barrier():
    instance, published = node()
    NavigationPathMux._on_task_state(
        instance,
        task_state(3, PatrolTaskState.TASK_RUNNING, PatrolTaskState.NAVIGATION_WAYPOINT, stamp=10),
    )
    NavigationPathMux._on_waypoint_path(instance, path(0))
    NavigationPathMux._on_waypoint_path(instance, path(10))
    NavigationPathMux._on_waypoint_path(instance, path(11))
    assert len(published) == 2
    assert published[-1].header.stamp.sec == 11

    NavigationPathMux._on_task_state(
        instance,
        task_state(4, PatrolTaskState.TASK_RUNNING, PatrolTaskState.NAVIGATION_MISSION, stamp=20),
    )
    NavigationPathMux._on_mission_path(instance, path(0))
    NavigationPathMux._on_mission_path(instance, path(20))
    NavigationPathMux._on_mission_path(instance, path(21))
    assert len(published) == 4
    assert published[-1].header.stamp.sec == 21


def test_empty_path_from_active_source_is_forwarded_as_stop():
    instance, published = node(WAYPOINT_SOURCE, True, 5)
    NavigationPathMux._on_waypoint_path(instance, path(0, nonempty=False))
    assert len(published) == 1
    assert not published[0].poses


def test_stop_mode_drops_nonempty_paths_from_both_sources():
    instance, published = node()
    NavigationPathMux._on_task_state(
        instance,
        task_state(2, PatrolTaskState.TASK_PAUSED, PatrolTaskState.NAVIGATION_STOP),
    )
    NavigationPathMux._on_waypoint_path(instance, path(12))
    NavigationPathMux._on_mission_path(instance, path(12))
    assert len(published) == 1
    assert not published[0].poses


def test_state_sequence_wrap_from_max_to_one_establishes_new_barrier():
    instance, published = node(MISSION_SOURCE, True, 0xFFFFFFFF)
    NavigationPathMux._on_task_state(
        instance,
        task_state(1, PatrolTaskState.TASK_RETURNING, PatrolTaskState.NAVIGATION_MISSION, stamp=44),
    )
    assert instance._last_state_seq == 1
    assert instance._barrier_stamp_ns == 44_000_000_000
    assert len(published) == 1


def test_older_state_is_ignored_without_changing_source_or_barrier():
    instance, published = node(MISSION_SOURCE, True, 100)
    instance._barrier_stamp_ns = 90
    heartbeats = []
    instance._refresh_heartbeat = lambda: heartbeats.append(True)
    NavigationPathMux._on_task_state(
        instance,
        task_state(99, PatrolTaskState.TASK_RUNNING, PatrolTaskState.NAVIGATION_WAYPOINT, stamp=50),
    )
    assert instance._active_source == MISSION_SOURCE
    assert instance._last_state_seq == 100
    assert instance._barrier_stamp_ns == 90
    assert published == []
    assert heartbeats == []


def test_same_sequence_heartbeat_has_no_repeated_side_effect():
    instance, published = node(WAYPOINT_SOURCE, True, 7)
    instance._barrier_stamp_ns = 6_000_000_000
    NavigationPathMux._on_task_state(
        instance,
        task_state(7, PatrolTaskState.TASK_RUNNING, PatrolTaskState.NAVIGATION_WAYPOINT, stamp=8),
    )
    assert instance._barrier_stamp_ns == 6_000_000_000
    assert published == []


def test_zero_sequence_is_ignored_before_initialization():
    instance, published = node()
    NavigationPathMux._on_task_state(
        instance,
        task_state(0, PatrolTaskState.TASK_RUNNING, PatrolTaskState.NAVIGATION_WAYPOINT),
    )
    assert not instance._task_state_seen
    assert published == []


def test_same_sequence_conflict_stops_and_waits_for_newer_sequence():
    instance, published = node()
    NavigationPathMux._on_task_state(
        instance,
        task_state(8, PatrolTaskState.TASK_RUNNING, PatrolTaskState.NAVIGATION_WAYPOINT),
    )
    published.clear()
    NavigationPathMux._on_task_state(
        instance,
        task_state(8, PatrolTaskState.TASK_PAUSED, PatrolTaskState.NAVIGATION_STOP),
    )
    assert instance._state_conflicted
    assert instance._active_source == NO_SOURCE
    assert len(published) == 1
    NavigationPathMux._on_task_state(
        instance,
        task_state(8, PatrolTaskState.TASK_RUNNING, PatrolTaskState.NAVIGATION_WAYPOINT),
    )
    assert instance._state_conflicted
    assert len(published) == 1
    NavigationPathMux._on_task_state(
        instance,
        task_state(9, PatrolTaskState.TASK_RUNNING, PatrolTaskState.NAVIGATION_WAYPOINT),
    )
    assert not instance._state_conflicted
    assert instance._active_source == WAYPOINT_SOURCE


@pytest.mark.parametrize(
    "initial_task,initial_mode",
    [
        (PatrolTaskState.TASK_RUNNING, PatrolTaskState.NAVIGATION_WAYPOINT),
        (PatrolTaskState.TASK_RUNNING, PatrolTaskState.NAVIGATION_MISSION),
        (PatrolTaskState.TASK_PAUSED, PatrolTaskState.NAVIGATION_STOP),
        (PatrolTaskState.TASK_RETURNING, PatrolTaskState.NAVIGATION_MISSION),
    ],
)
def test_terminate_publishes_transformable_empty_path_and_rejects_late_paths(
    initial_task, initial_mode
):
    instance, published = node()
    instance.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=30))
    )
    instance._publish_empty = lambda: NavigationPathMux._publish_empty(instance)
    NavigationPathMux._on_task_state(
        instance, task_state(10, initial_task, initial_mode, stamp=10)
    )
    NavigationPathMux._on_waypoint_path(instance, path(11))
    NavigationPathMux._on_mission_path(instance, path(11))
    published.clear()

    terminated = task_state(
        11,
        PatrolTaskState.TASK_NO_TASK,
        PatrolTaskState.NAVIGATION_STOP,
        stamp=20,
        outcome=PatrolTaskState.TASK_OUTCOME_TERMINATED,
    )
    NavigationPathMux._on_task_state(instance, terminated)
    assert instance._active_source == NO_SOURCE
    assert len(published) == 1
    assert published[0].header.frame_id == "map"
    assert published[0].header.stamp.sec == 30
    assert not published[0].poses

    NavigationPathMux._on_waypoint_path(instance, path(21))
    NavigationPathMux._on_mission_path(instance, path(21))
    NavigationPathMux._on_task_state(instance, terminated)
    assert len(published) == 1
