"""Exercise supervisor deadlines with real mission transitions and a controlled clock."""

from concurrent.futures import ThreadPoolExecutor
import threading
from types import SimpleNamespace

import pytest
from rclpy.time import Time

from dog_patrol_manager.mission_supervisor import MissionSupervisor
from dog_patrol_manager.state_machine import (
    EventSource,
    EventType,
    GlobalState,
    HomePoseData,
    MissionEventData,
    MissionStateMachine,
    NavigationMode,
    TaskCommand,
    TaskCommandData,
    WaypointStateData,
)


class SupervisorHarness:
    """Run the production callbacks without ROS timers or wall-clock sleeps."""

    _on_event = MissionSupervisor._on_event
    _on_waypoint_state = MissionSupervisor._on_waypoint_state
    _publish_state = MissionSupervisor._publish_state
    _check_state_timeout = MissionSupervisor._check_state_timeout
    _enum_name = staticmethod(MissionSupervisor._enum_name)

    def __init__(self, **parameters):
        self.parameters = {}
        MissionSupervisor._declare_parameters(self)
        self.parameters.update(parameters)
        MissionSupervisor._read_parameters(self)
        self._lock = threading.RLock()
        self._machine = MissionStateMachine()
        self.now_ns = 100_000_000_000
        self._state_entered_at = self.now()
        self.published = []

    def declare_parameter(self, name, default):
        self.parameters[name] = default

    def get_parameter(self, name):
        return SimpleNamespace(value=self.parameters[name])

    def get_clock(self):
        return self

    def now(self):
        return Time(nanoseconds=self.now_ns)

    def advance(self, seconds):
        self.now_ns += round(seconds * 1_000_000_000)

    def get_logger(self):
        return SimpleNamespace(info=lambda _: None, warning=lambda _: None, debug=lambda _: None)

    def _publish_state_locked(self):
        self.published.append(self._machine.snapshot)

    def event(self, source, event_type, target=0):
        message = MissionEventData(
            observed_state_seq=self._machine.snapshot.state_seq,
            target_id=target,
            source=int(source),
            event=int(event_type),
        )
        self._on_event(message)
        return message

    def enter_verification(self):
        self.event(EventSource.PERCEPTION, EventType.READY)
        self.event(EventSource.NAVIGATION, EventType.READY)
        result = self._machine.handle_task_command(
            TaskCommandData(
                command=int(TaskCommand.START),
                route=WaypointStateData(1, 3, 0, 0, False, True),
                home_pose=HomePoseData(pose=object()),
            )
        )
        assert result.changed
        self.event(EventSource.PERCEPTION, EventType.TARGET_CONFIRMED, 87)
        self.event(EventSource.NAVIGATION, EventType.TARGET_POSITION_READY, 87)
        self.event(EventSource.NAVIGATION, EventType.ARRIVED_AND_STOPPED, 87)
        assert self._machine.snapshot.state == GlobalState.VERIFY_IDENTITY

    def enter_tracking(self):
        self.enter_verification()
        return self.event(EventSource.PERCEPTION, EventType.UNAUTHORIZED, 87)


@pytest.mark.parametrize("timeout", [0.0, -1.0, float("nan"), float("inf"), -float("inf")])
def test_tracking_timeout_rejects_invalid_values(timeout):
    with pytest.raises(ValueError, match="intruder_tracking_timeout must be finite and positive"):
        SupervisorHarness(intruder_tracking_timeout=timeout)


def test_default_tracking_deadline_starts_after_verification_finishes():
    node = SupervisorHarness()
    node.enter_verification()
    node.advance(1200)
    node._check_state_timeout()
    assert node._machine.snapshot.state == GlobalState.VERIFY_IDENTITY

    node.event(EventSource.PERCEPTION, EventType.UNAUTHORIZED, 87)
    node.advance(59.999)
    node._check_state_timeout()
    assert node._machine.snapshot.state == GlobalState.TRACK_INTRUDER

    node.advance(0.001)
    node._check_state_timeout()
    snapshot = node._machine.snapshot
    assert snapshot.state == GlobalState.RECOVER_PATROL
    assert snapshot.navigation_mode == NavigationMode.NAVIGATION_MISSION
    assert snapshot.target_id == 87
    assert snapshot.handled_target_id == 0
    assert "intruder tracking timed out after 60.0s" in snapshot.detail


def test_tracking_timeout_can_be_configured_independently():
    node = SupervisorHarness(intruder_tracking_timeout=0.5)
    node.enter_tracking()
    node.advance(0.499)
    node._check_state_timeout()
    assert node._machine.snapshot.state == GlobalState.TRACK_INTRUDER
    node.advance(0.001)
    node._check_state_timeout()
    assert node._machine.snapshot.state == GlobalState.RECOVER_PATROL


def test_heartbeats_and_unchanged_events_do_not_extend_tracking_deadline():
    node = SupervisorHarness()
    previous_event = node.enter_tracking()
    entered_at = node._state_entered_at
    node.advance(40)
    node._publish_state()
    node._on_waypoint_state(WaypointStateData(1, 3, 1, 1, False, True))
    node._on_event(previous_event)
    node.event(EventSource.NAVIGATION, EventType.TARGET_POSITION_READY, 87)
    assert node._state_entered_at == entered_at

    # A brief loss and reacquisition below the final-loss threshold do not
    # produce a supervisor state change; time without an event still counts.
    node.advance(20)
    node._check_state_timeout()
    assert node._machine.snapshot.state == GlobalState.RECOVER_PATROL


@pytest.mark.parametrize("loss_first", [False, True])
def test_final_loss_and_tracking_deadline_only_end_target_once(loss_first):
    node = SupervisorHarness()
    node.enter_tracking()
    node.advance(60)
    before = node._machine.snapshot
    loss = MissionEventData(before.state_seq, 87, int(EventSource.PERCEPTION), int(EventType.TARGET_LOST))
    published_before = len(node.published)
    actions = [node._check_state_timeout, lambda: node._on_event(loss)]
    if loss_first:
        actions.reverse()
    for action in actions:
        action()
    node._check_state_timeout()

    after = node._machine.snapshot
    assert after.state == GlobalState.RECOVER_PATROL
    assert after.state_seq == before.state_seq + 1
    assert after.handled_target_id == 0
    assert len(node.published) == published_before + 1


def test_concurrent_loss_and_timeout_share_the_supervisor_lock():
    node = SupervisorHarness()
    node.enter_tracking()
    node.advance(60)
    before = node._machine.snapshot
    loss = MissionEventData(before.state_seq, 87, int(EventSource.PERCEPTION), int(EventType.TARGET_LOST))
    published_before = len(node.published)
    barrier = threading.Barrier(2)

    def run(action):
        barrier.wait(timeout=5)
        action()

    with ThreadPoolExecutor(max_workers=2) as pool:
        futures = [pool.submit(run, action) for action in (node._check_state_timeout, lambda: node._on_event(loss))]
        for future in futures:
            future.result(timeout=5)

    assert node._machine.snapshot.state == GlobalState.RECOVER_PATROL
    assert node._machine.snapshot.state_seq == before.state_seq + 1
    assert node._machine.snapshot.handled_target_id == 0
    assert len(node.published) == published_before + 1


def test_timeout_restores_patrol_and_next_target_has_a_new_deadline():
    node = SupervisorHarness()
    node.enter_tracking()
    node.advance(60)
    node._check_state_timeout()
    node.event(EventSource.NAVIGATION, EventType.PATROL_RECOVERY_COMPLETE, 87)
    snapshot = node._machine.snapshot
    assert snapshot.state == GlobalState.PATROL
    assert snapshot.target_id == 0
    assert snapshot.handled_target_id == 0

    node.advance(100)
    node.event(EventSource.PERCEPTION, EventType.TARGET_CONFIRMED, 87)
    node.event(EventSource.NAVIGATION, EventType.TARGET_POSITION_READY, 87)
    node.event(EventSource.NAVIGATION, EventType.ARRIVED_AND_STOPPED, 87)
    node.event(EventSource.PERCEPTION, EventType.UNAUTHORIZED, 87)
    node._check_state_timeout()
    assert node._machine.snapshot.state == GlobalState.TRACK_INTRUDER
    node.advance(60)
    node._check_state_timeout()
    assert node._machine.snapshot.state == GlobalState.RECOVER_PATROL
