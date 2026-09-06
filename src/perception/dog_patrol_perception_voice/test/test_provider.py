from __future__ import annotations

from collections import deque
from collections.abc import Callable
import threading
import time

import pytest
import rclpy
from dog_patrol_interfaces.msg import MissionState
from dog_patrol_perception_interfaces.msg import (
    AuthorizationCommand,
    AuthorizationEvidence,
)
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

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


class FakeTask:
    def __init__(
        self,
        results: list[VoiceWindowResult],
        *,
        error: Exception | None = None,
        wait_for_release: bool = False,
        close_error: Exception | None = None,
    ) -> None:
        self._results = deque(results)
        self._error = error
        self._wait_for_release = wait_for_release
        self._close_error = close_error
        self.calls: list[bool] = []
        self.entered = threading.Event()
        self.release = threading.Event()
        self.closed = threading.Event()
        self.cancel_called = threading.Event()

    def __enter__(self):
        self.entered.set()
        return self

    def __exit__(self, _exc_type, _exc, _traceback):
        self.closed.set()
        if self._close_error is not None:
            raise VoiceTaskCleanupError(str(self._close_error))

    def respond(self, *args, retry=False, **kwargs):
        del args, kwargs
        self.calls.append(bool(retry))
        if self._wait_for_release:
            assert self.release.wait(timeout=3.0)
        if self._error is not None:
            raise self._error
        return self._results.popleft()

    def cancel(self):
        self.cancel_called.set()


class FakeAdapter:
    def __init__(self, task: FakeTask) -> None:
        self._task = task

    def task(self):
        return self._task


class FakeAdapterFactory:
    def __init__(self, tasks: list[FakeTask]) -> None:
        self._tasks = deque(tasks)
        self.created: list[FakeTask] = []

    def __call__(self):
        task = self._tasks.popleft()
        self.created.append(task)
        return FakeAdapter(task)


class ProviderHarness:
    def __init__(self, adapter_factory) -> None:
        suffix = f"t{time.monotonic_ns()}"
        self.state_topic = f"/voice/{suffix}/mission/state"
        self.command_topic = f"/voice/{suffix}/authorization/command"
        self.evidence_topic = f"/voice/{suffix}/authorization/evidence"
        self.source = Node(f"voice_source_{suffix}")
        self.probe = Node(f"voice_probe_{suffix}")
        self.provider = VoiceEvidenceProviderNode(
            parameter_overrides=[
                Parameter("mission_state_topic", value=self.state_topic),
                Parameter("authorization_command_topic", value=self.command_topic),
                Parameter("authorization_evidence_topic", value=self.evidence_topic),
            ],
            adapter_factory=adapter_factory,
        )
        self.state_pub = self.source.create_publisher(
            MissionState, self.state_topic, _qos(transient=True, depth=1)
        )
        self.command_pub = self.source.create_publisher(
            AuthorizationCommand,
            self.command_topic,
            _qos(transient=False, depth=10),
        )
        self.evidence: list[AuthorizationEvidence] = []
        self.evidence_sub = self.probe.create_subscription(
            AuthorizationEvidence,
            self.evidence_topic,
            self.evidence.append,
            _qos(transient=False, depth=10),
        )
        self.executor = SingleThreadedExecutor()
        for node in (self.source, self.probe, self.provider):
            self.executor.add_node(node)

    def close(self):
        self.provider.destroy_node()
        for node in (self.probe, self.source):
            node.destroy_node()
        self.executor.shutdown()

    def wait(self, predicate: Callable[[], bool], timeout=3.0):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.05)
        return predicate()

    def publish_state(
        self,
        seq,
        target,
        *,
        state=MissionState.VERIFY_IDENTITY,
    ):
        msg = MissionState()
        msg.state_seq = seq
        msg.state = state
        msg.target_id = target
        self.state_pub.publish(msg)
        for _ in range(5):
            self.executor.spin_once(timeout_sec=0.03)

    def publish_command(self, seq, target, stage):
        msg = AuthorizationCommand()
        msg.observed_state_seq = seq
        msg.target_id = target
        msg.stage = stage
        self.command_pub.publish(msg)
        for _ in range(5):
            self.executor.spin_once(timeout_sec=0.03)


@pytest.fixture(autouse=True)
def ros_context():
    rclpy.init()
    try:
        yield
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def evidence_keys(messages):
    return [
        (message.stage, message.result)
        for message in messages
    ]


def test_voice_does_not_start_during_initial_face_stage():
    task = FakeTask([VoiceWindowResult(accepted=True, decision_time_seconds=0.1)])
    factory = FakeAdapterFactory([task])
    harness = ProviderHarness(factory)
    try:
        harness.publish_state(17, 42)
        harness.publish_command(17, 42, AuthorizationCommand.INITIAL_FACE)

        assert factory.created == []
        assert harness.evidence == []
    finally:
        harness.close()


def test_each_dual_stage_runs_exactly_one_response_window():
    first = FakeTask([VoiceWindowResult(accepted=False, decision_time_seconds=20.0)])
    second = FakeTask([VoiceWindowResult(accepted=True, decision_time_seconds=0.2)])
    harness = ProviderHarness(FakeAdapterFactory([first, second]))
    try:
        harness.publish_state(18, 43)
        harness.publish_command(18, 43, AuthorizationCommand.DUAL_FIRST)
        assert harness.wait(first.closed.is_set)
        assert harness.wait(lambda: len(harness.evidence) == 1)

        harness.publish_command(18, 43, AuthorizationCommand.DUAL_SECOND)
        assert harness.wait(second.closed.is_set)
        assert harness.wait(lambda: len(harness.evidence) == 2)

        assert first.calls == [False]
        assert second.calls == [True]
        assert evidence_keys(harness.evidence) == [
            (AuthorizationEvidence.DUAL_FIRST, AuthorizationEvidence.NOT_PASSED),
            (AuthorizationEvidence.DUAL_SECOND, AuthorizationEvidence.PASSED),
        ]
    finally:
        harness.close()


def test_duplicate_stage_command_does_not_run_twice():
    task = FakeTask([VoiceWindowResult(accepted=False, decision_time_seconds=20.0)])
    factory = FakeAdapterFactory([task])
    harness = ProviderHarness(factory)
    try:
        harness.publish_state(19, 44)
        harness.publish_command(19, 44, AuthorizationCommand.DUAL_FIRST)
        assert harness.wait(task.closed.is_set)
        harness.publish_command(19, 44, AuthorizationCommand.DUAL_FIRST)

        assert len(factory.created) == 1
        assert len(harness.evidence) == 1
    finally:
        harness.close()


@pytest.mark.parametrize(
    "task",
    [
        FakeTask([], error=RuntimeError("stream failed")),
        FakeTask(
            [VoiceWindowResult(accepted=True, decision_time_seconds=0.1)],
            close_error=RuntimeError("restore failed"),
        ),
    ],
)
def test_stage_failure_publishes_error_for_that_stage(task):
    harness = ProviderHarness(FakeAdapterFactory([task]))
    try:
        harness.publish_state(20, 45)
        harness.publish_command(20, 45, AuthorizationCommand.DUAL_FIRST)

        assert harness.wait(task.closed.is_set)
        assert harness.wait(lambda: len(harness.evidence) == 1)
        assert evidence_keys(harness.evidence) == [
            (AuthorizationEvidence.DUAL_FIRST, AuthorizationEvidence.ERROR)
        ]
    finally:
        harness.close()


def test_state_replacement_cancels_late_result_then_accepts_new_command():
    old = FakeTask(
        [VoiceWindowResult(accepted=True, decision_time_seconds=0.1)],
        wait_for_release=True,
    )
    new = FakeTask([VoiceWindowResult(accepted=True, decision_time_seconds=0.1)])
    harness = ProviderHarness(FakeAdapterFactory([old, new]))
    try:
        harness.publish_state(21, 46)
        harness.publish_command(21, 46, AuthorizationCommand.DUAL_FIRST)
        assert old.entered.wait(timeout=1.0)

        harness.publish_state(22, 47)
        assert old.cancel_called.is_set()
        old.release.set()
        assert harness.wait(old.closed.is_set)

        harness.publish_command(22, 47, AuthorizationCommand.DUAL_FIRST)
        assert harness.wait(new.closed.is_set)
        assert harness.wait(lambda: len(harness.evidence) == 2)
        assert [message.result for message in harness.evidence] == [
            AuthorizationEvidence.CANCELLED,
            AuthorizationEvidence.PASSED,
        ]
        assert [message.observed_state_seq for message in harness.evidence] == [21, 22]
    finally:
        old.release.set()
        harness.close()


def test_cancel_command_stops_active_stage():
    task = FakeTask(
        [VoiceWindowResult(accepted=True, decision_time_seconds=0.1)],
        wait_for_release=True,
    )
    harness = ProviderHarness(FakeAdapterFactory([task]))
    try:
        harness.publish_state(23, 48)
        harness.publish_command(23, 48, AuthorizationCommand.DUAL_FIRST)
        assert task.entered.wait(timeout=1.0)

        harness.publish_command(23, 48, AuthorizationCommand.CANCEL)
        assert task.cancel_called.is_set()
        task.release.set()

        assert harness.wait(lambda: len(harness.evidence) == 1)
        assert evidence_keys(harness.evidence) == [
            (AuthorizationEvidence.DUAL_FIRST, AuthorizationEvidence.CANCELLED)
        ]
    finally:
        task.release.set()
        harness.close()
