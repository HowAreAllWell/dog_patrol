from types import SimpleNamespace

from dog_patrol_interfaces.msg import MissionState
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from move.navigation_path_mux import (
    MISSION_SOURCE,
    NavigationPathMux,
    NO_SOURCE,
    WAYPOINT_SOURCE,
    path_source_for_mission_state,
)


def test_patrol_and_confirmation_use_waypoint_path():
    assert path_source_for_mission_state(MissionState.PATROL) == WAYPOINT_SOURCE
    assert path_source_for_mission_state(MissionState.CONFIRM_TARGET) == WAYPOINT_SOURCE


def test_target_and_recovery_use_mission_path():
    assert path_source_for_mission_state(MissionState.APPROACH_TARGET) == MISSION_SOURCE
    assert path_source_for_mission_state(MissionState.VERIFY_IDENTITY) == MISSION_SOURCE
    assert path_source_for_mission_state(MissionState.TRACK_INTRUDER) == MISSION_SOURCE
    assert path_source_for_mission_state(MissionState.RECOVER_PATROL) == MISSION_SOURCE


def test_startup_and_unknown_state_have_no_motion_path_source():
    assert path_source_for_mission_state(MissionState.STARTUP) == NO_SOURCE
    assert path_source_for_mission_state(255) == NO_SOURCE


def test_mux_forwards_only_the_path_owned_by_current_mission_state():
    published = []
    node = SimpleNamespace(
        _active_source=WAYPOINT_SOURCE,
        _mission_state_seen=False,
        _last_state_seq=-1,
        _wait_for_fresh_waypoint_path=False,
        _publish=lambda message: published.append(message),
        _publish_empty=lambda: published.append(Path()),
        get_logger=lambda: SimpleNamespace(info=lambda message: None),
    )
    waypoint_path = Path()
    waypoint_path.poses.append(PoseStamped())
    mission_path = Path()
    mission_path.poses.append(PoseStamped())

    NavigationPathMux._on_waypoint_path(node, waypoint_path)
    NavigationPathMux._on_mission_path(node, mission_path)
    assert published[-1] is waypoint_path

    state = SimpleNamespace(state_seq=10, state=MissionState.APPROACH_TARGET)
    NavigationPathMux._on_mission_state(node, state)
    assert len(published[-1].poses) == 0
    NavigationPathMux._on_mission_path(node, mission_path)
    assert published[-1] is mission_path


def test_return_to_patrol_waits_for_a_post_switch_waypoint_path():
    published = []
    node = SimpleNamespace(
        _active_source=MISSION_SOURCE,
        _mission_state_seen=True,
        _last_state_seq=20,
        _wait_for_fresh_waypoint_path=False,
        _waypoint_min_stamp_ns=0,
        _publish=lambda message: published.append(message),
        _publish_empty=lambda: published.append(Path()),
        get_logger=lambda: SimpleNamespace(info=lambda message: None),
    )

    state = SimpleNamespace(
        state_seq=21,
        state=MissionState.PATROL,
        header=SimpleNamespace(stamp=SimpleNamespace(sec=11, nanosec=0)),
    )
    NavigationPathMux._on_mission_state(node, state)

    assert node._active_source == WAYPOINT_SOURCE
    assert node._wait_for_fresh_waypoint_path
    assert len(published) == 1
    assert not published[-1].poses

    fresh_waypoint_path = Path()
    fresh_waypoint_path.poses.append(PoseStamped())
    NavigationPathMux._on_waypoint_path(node, fresh_waypoint_path)

    assert published[-1] is fresh_waypoint_path
    assert not node._wait_for_fresh_waypoint_path


def test_return_to_patrol_rejects_a_delayed_pre_switch_waypoint_path():
    published = []
    node = SimpleNamespace(
        _active_source=MISSION_SOURCE,
        _mission_state_seen=True,
        _last_state_seq=20,
        _wait_for_fresh_waypoint_path=False,
        _waypoint_min_stamp_ns=0,
        _publish=lambda message: published.append(message),
        _publish_empty=lambda: published.append(Path()),
        get_logger=lambda: SimpleNamespace(info=lambda message: None),
    )

    state = SimpleNamespace(
        state_seq=21,
        state=MissionState.PATROL,
        header=SimpleNamespace(stamp=SimpleNamespace(sec=11, nanosec=0)),
    )
    NavigationPathMux._on_mission_state(node, state)

    delayed = Path()
    delayed.header.stamp.sec = 10
    delayed.poses.append(PoseStamped())
    NavigationPathMux._on_waypoint_path(node, delayed)
    assert len(published) == 1
    assert not published[-1].poses

    fresh = Path()
    fresh.header.stamp.sec = 12
    fresh.poses.append(PoseStamped())
    NavigationPathMux._on_waypoint_path(node, fresh)
    assert published[-1] is fresh
    assert not node._wait_for_fresh_waypoint_path
