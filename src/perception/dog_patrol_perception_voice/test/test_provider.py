from __future__ import annotations

from collections import deque
from collections.abc import Callable
import time
import threading

import pytest
import rclpy
from dog_patrol_interfaces.msg import MissionEvent, MissionState
from dog_patrol_perception_interfaces.msg import AuthorizationEvidence
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from dog_patrol_perception_orchestrator.authorization_node import (
    PerceptionAuthorizationNode,
)
from dog_patrol_perception_voice.adapter import VoiceTaskCleanupError
from dog_patrol_perception_voice.provider import VoiceEvidenceProviderNode
from dog_patrol_perception_voice.result import VoiceWindowResult


def _qos(*, transient: bool, depth: int) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=(
            DurabilityPolicy.TRANSIENT_LOCAL
            if transient
            else DurabilityPolicy.VOLATILE
        ),
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def _spin_until(executor, predicate: Callable[[], bool], timeout: float = 3.0) -> bool:
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
    return predicate()


class FakeTask:
    def __init__(
        self,
        results: list[VoiceWindowResult],
        *,
        error: Exception | None = None,
        wait_for_release: bool = False,
        close_error: Exception | None = None,
        wait_for_close: bool = False,
    ) -> None:
        self._results = deque(results)
        self._error = error
        self._wait_for_release = wait_for_release
        self._close_error = close_error
        self._wait_for_close = wait_for_close
        self._on_enter: Callable[[], None] | None = None
        self._on_exit: Callable[[], None] | None = None
        self.calls: list[int] = []
        self.entered = threading.Event()
        self.release = threading.Event()
        self.closed = threading.Event()
        self.close_started = threading.Event()
        self.release_close = threading.Event()
        self.cancel_called = threading.Event()

    def __enter__(self) -> FakeTask:
        if self._on_enter is not None:
            self._on_enter()
        self.entered.set()
        return self

    def __exit__(self, _exc_type, _exc, _traceback) -> None:
        self.close_started.set()
        if self._wait_for_close:
            assert self.release_close.wait(timeout=3.0)
        if self._on_exit is not None:
            self._on_exit()
        self.closed.set()
        if self._close_error is not None:
            raise VoiceTaskCleanupError(str(self._close_error)) from self._close_error

    def respond(self, *args, **kwargs) -> VoiceWindowResult:
        del args, kwargs
        self.calls.append(len(self.calls) + 1)
        if self._wait_for_release:
            assert self.release.wait(timeout=3.0)
        if self._error is not None:
            raise self._error
        return self._results.popleft()

    def cancel(self) -> None:
        # Deliberately do not release the response: the controller must gate the
        # late result and wait for the old hardware context to close.
        self.cancel_called.set()


class FakeAdapter:
    def __init__(self, task: FakeTask) -> None:
        self.task_instance = task

    def task(self) -> FakeTask:
        return self.task_instance


class FakeAdapterFactory:
    def __init__(self, tasks: list[FakeTask]) -> None:
        self._tasks = deque(tasks)
        self.created: list[FakeTask] = []
        self.active_sessions = 0
        self.max_active_sessions = 0

    def __call__(self) -> FakeAdapter:
        task = self._tasks.popleft()
        task._on_enter = self._enter
        task._on_exit = self._exit
        self.created.append(task)
        return FakeAdapter(task)

    def _enter(self) -> None:
        self.active_sessions += 1
        self.max_active_sessions = max(self.max_active_sessions, self.active_sessions)

    def _exit(self) -> None:
        self.active_sessions -= 1


class ProviderHarness:
    def __init__(self, adapter_factory) -> None:
        rclpy.init()
        suffix = f"t{time.monotonic_ns()}"
        state_topic = f"/issue34/{suffix}/mission/state"
        event_topic = f"/issue34/{suffix}/mission/event"
        evidence_topic = f"/issue34/{suffix}/perception/authorization_evidence"
        self.source = Node(f"issue34_provider_source_{suffix}")
        self.probe = Node(f"issue34_provider_probe_{suffix}")
        self.provider = VoiceEvidenceProviderNode(
            parameter_overrides=[
                Parameter("mission_state_topic", value=state_topic),
                Parameter("authorization_evidence_topic", value=evidence_topic),
            ],
            adapter_factory=adapter_factory,
        )
        self.authorization = PerceptionAuthorizationNode(
            parameter_overrides=[
                Parameter("mission_state_topic", value=state_topic),
                Parameter("mission_event_topic", value=event_topic),
                Parameter("authorization_evidence_topic", value=evidence_topic),
            ]
        )
        self.state_pub = self.source.create_publisher(
            MissionState, state_topic, _qos(transient=True, depth=1)
        )
        self.events: list[MissionEvent] = []
        self.evidence: list[AuthorizationEvidence] = []
        self.event_sub = self.probe.create_subscription(
            MissionEvent,
            event_topic,
            self.events.append,
            _qos(transient=False, depth=10),
        )
        self.evidence_sub = self.probe.create_subscription(
            AuthorizationEvidence,
            evidence_topic,
            self.evidence.append,
            _qos(transient=False, depth=10),
        )
        self.executor = SingleThreadedExecutor()
        for node in (self.source, self.probe, self.provider, self.authorization):
            self.executor.add_node(node)

    def publish_state(
        self,
        state_seq: int,
        target_id: int,
        *,
        state: int = MissionState.VERIFY_IDENTITY,
        blocked: bool = False,
    ) -> None:
        msg = MissionState()
        msg.state_seq = state_seq
        msg.state = state
        msg.target_id = target_id
        msg.blocked = blocked
        self.state_pub.publish(msg)
        for _ in range(5):
            self.executor.spin_once(timeout_sec=0.02)

    def wait(self, predicate: Callable[[], bool], timeout: float = 3.0) -> bool:
        return _spin_until(self.executor, predicate, timeout)

    def close(self) -> None:
        self.provider.destroy_node()
        self.authorization.destroy_node()
        for node in (self.source, self.probe):
            node.destroy_node()
        self.executor.shutdown()
        del self.event_sub, self.evidence_sub
        if rclpy.ok():
            rclpy.shutdown()


def test_first_window_pass_reaches_real_authorization_adapter() -> None:
    task = FakeTask([VoiceWindowResult(accepted=True, decision_time_seconds=0.2)])
    harness = ProviderHarness(FakeAdapterFactory([task]))
    try:
        harness.publish_state(17, 42)

        assert harness.wait(task.closed.is_set)
        assert harness.wait(lambda: len(harness.events) == 1 and len(harness.evidence) == 1)
        assert task.calls == [1]
        assert [message.result for message in harness.evidence] == [
            AuthorizationEvidence.PASSED
        ]
        assert [event.event for event in harness.events] == [MissionEvent.AUTHORIZED]
        assert (harness.events[0].observed_state_seq, harness.events[0].target_id) == (
            17,
            42,
        )
    finally:
        harness.close()


def test_first_window_failure_continues_to_second_window_and_second_pass_authorizes() -> None:
    task = FakeTask(
        [
            VoiceWindowResult(accepted=False, decision_time_seconds=20.0),
            VoiceWindowResult(accepted=True, decision_time_seconds=0.4),
        ]
    )
    harness = ProviderHarness(FakeAdapterFactory([task]))
    try:
        harness.publish_state(18, 43)

        assert harness.wait(task.closed.is_set)
        assert harness.wait(lambda: len(harness.events) == 1 and len(harness.evidence) == 2)
        assert task.calls == [1, 2]
        assert [message.result for message in harness.evidence] == [
            AuthorizationEvidence.NOT_PASSED,
            AuthorizationEvidence.PASSED,
        ]
        assert harness.events[0].event == MissionEvent.AUTHORIZED
    finally:
        harness.close()


def test_two_failed_windows_publish_two_evidence_messages_and_unauthorize() -> None:
    task = FakeTask(
        [
            VoiceWindowResult(accepted=False, decision_time_seconds=20.0),
            VoiceWindowResult(accepted=False, decision_time_seconds=20.0),
        ]
    )
    harness = ProviderHarness(FakeAdapterFactory([task]))
    try:
        harness.publish_state(19, 44)

        assert harness.wait(task.closed.is_set)
        assert harness.wait(lambda: len(harness.events) == 1 and len(harness.evidence) == 2)
        assert task.calls == [1, 2]
        assert [message.result for message in harness.evidence] == [
            AuthorizationEvidence.NOT_PASSED,
            AuthorizationEvidence.NOT_PASSED,
        ]
        assert harness.events[0].event == MissionEvent.UNAUTHORIZED
    finally:
        harness.close()


@pytest.mark.parametrize(
    "task",
    [
        FakeTask([], error=RuntimeError("stream failed")),
        FakeTask(
            [VoiceWindowResult(accepted=True, decision_time_seconds=0.1)],
            close_error=RuntimeError("R818 restore failed"),
        ),
    ],
)
def test_hardware_or_restore_failure_publishes_error(task: FakeTask) -> None:
    harness = ProviderHarness(FakeAdapterFactory([task]))
    try:
        harness.publish_state(20, 45)

        assert harness.wait(task.closed.is_set)
        assert harness.wait(lambda: len(harness.events) == 1)
        assert [message.result for message in harness.evidence] == [
            AuthorizationEvidence.ERROR
        ]
        assert harness.events[0].event == MissionEvent.EXECUTION_ERROR
    finally:
        harness.close()


def test_duplicate_mission_state_does_not_create_a_second_task() -> None:
    task = FakeTask([VoiceWindowResult(accepted=True, decision_time_seconds=0.1)])
    factory = FakeAdapterFactory([task])
    harness = ProviderHarness(factory)
    try:
        harness.publish_state(21, 46)
        assert harness.wait(task.closed.is_set)
        harness.publish_state(21, 46)
        harness.executor.spin_once(timeout_sec=0.1)

        assert len(factory.created) == 1
        assert task.calls == [1]
    finally:
        harness.close()


def test_invalid_mission_state_does_not_create_a_voice_task() -> None:
    task = FakeTask([VoiceWindowResult(accepted=True, decision_time_seconds=0.1)])
    factory = FakeAdapterFactory([task])
    harness = ProviderHarness(factory)
    try:
        harness.publish_state(21, 0, state=MissionState.VERIFY_IDENTITY)
        harness.publish_state(22, 46, state=MissionState.PATROL)
        harness.publish_state(23, 46, blocked=True)
        harness.executor.spin_once(timeout_sec=0.1)

        assert factory.created == []
    finally:
        harness.close()


def test_session_replacement_cancels_late_result_and_keeps_one_hardware_session() -> None:
    old_task = FakeTask(
        [VoiceWindowResult(accepted=True, decision_time_seconds=0.1)],
        wait_for_release=True,
    )
    new_task = FakeTask([VoiceWindowResult(accepted=True, decision_time_seconds=0.1)])
    factory = FakeAdapterFactory([old_task, new_task])
    harness = ProviderHarness(factory)
    try:
        harness.publish_state(22, 47)
        assert old_task.entered.wait(timeout=1.0)

        harness.publish_state(23, 48)
        assert old_task.cancel_called.is_set()
        old_task.release.set()

        assert harness.wait(new_task.closed.is_set)
        assert harness.wait(lambda: len(harness.events) == 1)
        assert [
            (message.observed_state_seq, message.target_id, message.result)
            for message in harness.evidence
        ] == [
            (22, 47, AuthorizationEvidence.CANCELLED),
            (23, 48, AuthorizationEvidence.PASSED),
        ]
        assert (harness.events[0].observed_state_seq, harness.events[0].target_id) == (
            23,
            48,
        )
        assert factory.max_active_sessions == 1
    finally:
        old_task.release.set()
        harness.close()


def test_restore_failure_during_replacement_reports_error_and_blocks_new_session() -> None:
    old_task = FakeTask(
        [VoiceWindowResult(accepted=True, decision_time_seconds=0.1)],
        close_error=RuntimeError("R818 restore failed"),
        wait_for_close=True,
    )
    new_task = FakeTask([VoiceWindowResult(accepted=True, decision_time_seconds=0.1)])
    factory = FakeAdapterFactory([old_task, new_task])
    harness = ProviderHarness(factory)
    try:
        harness.publish_state(23, 48)
        assert old_task.close_started.wait(timeout=1.0)

        harness.publish_state(24, 49)
        old_task.release_close.set()

        assert harness.wait(old_task.closed.is_set)
        assert harness.wait(lambda: len(harness.evidence) == 1)
        assert [message.result for message in harness.evidence] == [
            AuthorizationEvidence.ERROR
        ]
        assert len(factory.created) == 1
    finally:
        old_task.release_close.set()
        harness.close()


@pytest.mark.parametrize(
    "state,blocked",
    [
        (MissionState.VERIFY_IDENTITY, True),
        (MissionState.PATROL, False),
    ],
)
def test_blocking_or_leaving_verify_cancels_current_task(
    state: int, blocked: bool
) -> None:
    task = FakeTask(
        [VoiceWindowResult(accepted=True, decision_time_seconds=0.1)],
        wait_for_release=True,
    )
    harness = ProviderHarness(FakeAdapterFactory([task]))
    try:
        harness.publish_state(24, 49)
        assert task.entered.wait(timeout=1.0)

        harness.publish_state(24, 49, state=state, blocked=blocked)
        assert task.cancel_called.is_set()
        task.release.set()

        assert harness.wait(task.closed.is_set)
        assert harness.wait(
            lambda: len(harness.evidence) == 1 and not harness.events
        )
        assert harness.evidence[0].result == AuthorizationEvidence.CANCELLED
    finally:
        task.release.set()
        harness.close()
