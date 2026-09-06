from types import SimpleNamespace

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import numpy as np

from move.global_path_seq_publisher import GlobalPathSequencePublisher


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Clock:
    def now(self):
        return SimpleNamespace(to_msg=lambda: Time(sec=42, nanosec=7))


def test_publish_cached_path_refreshes_copy_without_mutating_cache():
    cached = Path()
    cached.header.frame_id = "map"
    cached.header.stamp.sec = 1
    cached.poses.append(PoseStamped())
    cached.poses[0].header.frame_id = "map"
    cached.poses[0].header.stamp.sec = 1

    node = SimpleNamespace(
        _last_path=cached,
        global_frame="map",
        path_pub=_Publisher(),
        pure_pursuit_plan_pub=None,
        get_clock=lambda: _Clock(),
    )

    count = GlobalPathSequencePublisher._publish_cached_path(node)

    assert count == 1
    assert len(node.path_pub.messages) == 1
    published = node.path_pub.messages[0]
    assert published.header.stamp.sec == 42
    assert published.poses[0].header.stamp.sec == 42
    assert cached.header.stamp.sec == 1
    assert cached.poses[0].header.stamp.sec == 1


def test_resume_restores_cache_and_requests_fresh_plan_immediately():
    calls = []
    node = SimpleNamespace(
        waypoints=[object()],
        paused=True,
        sequence_done=True,
        sequence_version=3,
        _resume_bridge_active=False,
        _invalidate_planning_request=lambda reason: calls.append(("invalidate", reason)),
        _publish_cached_path=lambda: 12,
        _publish_waypoints=lambda: calls.append(("waypoints", None)),
        _publish_status=lambda detail: calls.append(("status", detail)),
        _on_timer=lambda: calls.append(("timer", None)),
    )

    GlobalPathSequencePublisher._on_resume(node, None)

    assert not node.paused
    assert not node.sequence_done
    assert node.sequence_version == 4
    assert node._resume_bridge_active
    assert calls[-1] == ("timer", None)
    assert ("status", "resumed; restored 12 cached path poses") in calls


def test_resume_from_current_discards_cached_path_and_replans_immediately():
    calls = []
    node = SimpleNamespace(
        waypoints=[object()],
        paused=True,
        sequence_done=True,
        sequence_version=8,
        _last_path=object(),
        _resume_bridge_active=True,
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
    assert not node._resume_bridge_active
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
