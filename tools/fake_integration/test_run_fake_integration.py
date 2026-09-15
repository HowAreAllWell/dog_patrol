from types import SimpleNamespace
import time

from builtin_interfaces.msg import Time
from dog_patrol_interfaces.msg import MissionState, PatrolTaskState

from tools.fake_integration import fake_nodes
from tools.fake_integration import run_fake_integration


def test_parent_start_requires_same_ready_no_task_snapshot() -> None:
    mission = {"state_seq": 12, "state": MissionState.READY, "target_id": 0}
    task = {
        "state_seq": 12,
        "task_state": PatrolTaskState.TASK_NO_TASK,
        "navigation_mode": PatrolTaskState.NAVIGATION_STOP,
    }

    assert run_fake_integration.parent_ready_for_start(mission, task)
    assert not run_fake_integration.parent_ready_for_start(
        mission, {**task, "state_seq": 11}
    )
    assert not run_fake_integration.parent_ready_for_start(
        {**mission, "state": MissionState.PATROL}, task
    )
    assert not run_fake_integration.parent_ready_for_start(
        mission, {**task, "task_state": PatrolTaskState.TASK_RUNNING}
    )
    assert not run_fake_integration.parent_ready_for_start(
        mission,
        {**task, "navigation_mode": PatrolTaskState.NAVIGATION_WAYPOINT},
    )
    assert not run_fake_integration.parent_ready_for_start(
        {**mission, "target_id": 5}, task
    )
    assert not run_fake_integration.parent_ready_for_start(None, task)
    assert not run_fake_integration.parent_ready_for_start(mission, None)


def test_observer_rejects_old_and_conflicting_state_snapshots() -> None:
    observer = SimpleNamespace(
        states=[],
        _state_seq=None,
        _state_signature=None,
        _state_conflicted=False,
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=0)
        ),
        saw_verify=False,
        final_state=None,
    )
    first = SimpleNamespace(
        state_seq=10,
        state=MissionState.READY,
        target_id=0,
        handled_target_id=0,
        header=SimpleNamespace(),
    )
    old = SimpleNamespace(
        state_seq=9,
        state=MissionState.STARTUP,
        target_id=0,
        handled_target_id=0,
        header=SimpleNamespace(),
    )
    conflict = SimpleNamespace(
        state_seq=10,
        state=MissionState.PATROL,
        target_id=0,
        handled_target_id=0,
        header=SimpleNamespace(),
    )

    run_fake_integration.Observer.on_state(observer, first)
    run_fake_integration.Observer.on_state(observer, old)
    run_fake_integration.Observer.on_state(observer, conflict)

    assert [item["state_seq"] for item in observer.states] == [10]
    assert observer._state_conflicted is True


def test_fake_navigation_rejects_stale_state_after_sequence_wrap() -> None:
    node = SimpleNamespace(
        _state=SimpleNamespace(state_seq=1),
        _state_seq=1,
        _state_signature=(MissionState.STARTUP, 0, 0),
        _state_conflicted=False,
    )
    stale = SimpleNamespace(
        state_seq=0xFFFFFFFF,
        state=MissionState.STARTUP,
        target_id=0,
        handled_target_id=0,
    )

    fake_nodes.FakeNavigation._on_state(node, stale)

    assert node._state.state_seq == 1


def test_fake_readiness_nodes_reject_late_old_startup_snapshot() -> None:
    late_startup = SimpleNamespace(
        state_seq=3,
        state=MissionState.STARTUP,
        target_id=0,
        handled_target_id=0,
    )
    face_publishes = []
    face = SimpleNamespace(
        _state=SimpleNamespace(
            state_seq=4,
            state=MissionState.READY,
            target_id=0,
            handled_target_id=0,
        ),
        _state_seq=4,
        _state_signature=(MissionState.READY, 0, 0),
        _state_conflicted=False,
        _publish_ready=face_publishes.append,
    )
    capability_messages = []
    capabilities = SimpleNamespace(
        _state_seq=4,
        _state_signature=MissionState.READY,
        _state_conflicted=False,
        _last_startup=2,
        _pub=SimpleNamespace(publish=capability_messages.append),
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=Time)
        ),
    )

    fake_nodes.FakeFace._on_state(face, late_startup)
    fake_nodes.FakeCapabilities._on_state(capabilities, late_startup)

    assert face._state.state_seq == 4
    assert face_publishes == []
    assert capability_messages == []


def test_fake_nodes_fail_closed_on_same_sequence_conflict() -> None:
    first = SimpleNamespace(state_seq=7, state=MissionState.STARTUP)
    conflict = SimpleNamespace(
        state_seq=7,
        state=MissionState.READY,
        target_id=0,
        handled_target_id=0,
    )
    first.target_id = 0
    first.handled_target_id = 0
    navigation = SimpleNamespace(
        _state=first,
        _state_seq=7,
        _state_signature=(MissionState.STARTUP, 0, 0),
        _state_conflicted=False,
    )
    face = SimpleNamespace(
        _state=first,
        _state_seq=7,
        _state_signature=(MissionState.STARTUP, 0, 0),
        _state_conflicted=False,
        _publish_ready=lambda _: None,
    )

    fake_nodes.FakeNavigation._on_state(navigation, conflict)
    fake_nodes.FakeFace._on_state(face, conflict)

    assert navigation._state is None
    assert navigation._state_conflicted is True
    assert face._state is None
    assert face._state_conflicted is True


def test_same_sequence_conflict_recovers_only_at_a_newer_sequence() -> None:
    first = SimpleNamespace(
        state_seq=7,
        state=MissionState.STARTUP,
        target_id=0,
        handled_target_id=0,
    )
    conflict = SimpleNamespace(
        state_seq=7,
        state=MissionState.READY,
        target_id=0,
        handled_target_id=0,
    )
    newer = SimpleNamespace(
        state_seq=8,
        state=MissionState.READY,
        target_id=0,
        handled_target_id=0,
    )
    navigation = SimpleNamespace(
        _state=first,
        _state_seq=7,
        _state_signature=(MissionState.STARTUP, 0, 0),
        _state_conflicted=False,
    )
    fake_nodes.FakeNavigation._on_state(navigation, conflict)
    fake_nodes.FakeNavigation._on_state(navigation, newer)
    assert navigation._state is newer
    assert navigation._state_conflicted is False


def test_start_rejection_allows_a_later_retry() -> None:
    future = SimpleNamespace(
        result=lambda: SimpleNamespace(
            accepted=False, error_code="home_pose_unavailable", message="retry"
        )
    )
    observer = SimpleNamespace(
        _start_requested=True,
        _start_retry_after=0.0,
        start_accepted=False,
        start_error="",
    )

    run_fake_integration.Observer._on_start_result(observer, future)

    assert observer.start_accepted is False
    assert "home_pose_unavailable" in observer.start_error
    assert observer._start_requested is False
    assert observer._start_retry_after > time.monotonic()
