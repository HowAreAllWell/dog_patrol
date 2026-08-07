"""
Unattended voice deployment acceptance for an installed Orin workspace.

The command in this module deliberately keeps recognition outcomes separate from
audio transport.  A deployment fixture can therefore drive the ROS contract
without recording speech, while hardware mode still takes ownership of the real
AC107/R818 stream and exercises its cleanup path.
"""

from __future__ import annotations

import argparse
from collections import deque
from collections.abc import Callable, Iterator, Sequence
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import platform
import shlex
import sys
import threading
import time
from typing import Any

try:
    import rclpy
    from dog_patrol_interfaces.msg import MissionEvent, MissionState
    from dog_patrol_perception_interfaces.msg import AuthorizationEvidence, CapabilityStatus
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
except ImportError:  # pragma: no cover - only used on non-ROS development hosts.
    rclpy = None  # type: ignore[assignment]

from .adapter import R818VoiceAdapter
from .adb import SubprocessAdbFileTransfer
from .config import VoiceConfig, load_voice_config
from .preflight import (
    ERROR,
    NOT_READY,
    READY,
    VoicePreflight,
    VoicePreflightOutcome,
    default_helper_path,
)
from .prompt import FfmpegAlsaPromptPlayer
from .result import VoiceWindowResult
from .r818_stream import SubprocessAdbEncodedPcmStream


_SCHEMA_VERSION = 1
_FAILURE_SCENARIOS = (
    "prompt_cancel",
    "first_window_cancel",
    "second_window_cancel",
    "state_seq_replace",
    "target_id_replace",
    "stream_failure",
    "adb_failure",
    "playback_failure",
    "restore_failure",
)


@dataclass(frozen=True)
class FixtureWindow:
    """One recognition result injected without storing microphone audio."""

    accepted: bool
    decision_time_seconds: float = 0.0

    def __post_init__(self) -> None:
        if not isinstance(self.accepted, bool):
            raise ValueError("accepted must be a boolean")
        if (
            not math.isfinite(self.decision_time_seconds)
            or self.decision_time_seconds < 0
        ):
            raise ValueError("decision_time_seconds must be finite and non-negative")


@dataclass(frozen=True)
class FixtureTask:
    """The one or two response-window results for one task-level cycle."""

    windows: tuple[FixtureWindow, ...]

    def __post_init__(self) -> None:
        if not 1 <= len(self.windows) <= 2:
            raise ValueError("a fixture task must contain one or two windows")
        if len(self.windows) == 2 and self.windows[0].accepted:
            raise ValueError("a task cannot contain a second window after acceptance")


@dataclass(frozen=True)
class AcceptanceFixture:
    """Validated task outcomes loaded from a deployment-local JSON file."""

    tasks: tuple[FixtureTask, ...]

    @classmethod
    def from_tasks(cls, tasks: Sequence[Sequence[dict[str, Any]]]) -> AcceptanceFixture:
        return cls(
            tuple(
                FixtureTask(
                    tuple(
                        FixtureWindow(
                            accepted=window["accepted"],
                            decision_time_seconds=float(
                                window.get("decision_time_seconds", 0.0)
                            ),
                        )
                        for window in task
                    )
                )
                for task in tasks
            )
        )


def load_acceptance_fixture(
    path: str | Path,
    *,
    expected_cycles: int | None = None,
) -> AcceptanceFixture:
    """Load strict, audio-free task outcomes from a deployment-local JSON file."""
    fixture_path = Path(path)
    try:
        payload = json.loads(fixture_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot read acceptance fixture {fixture_path}: {exc}") from exc
    if not isinstance(payload, dict):
        raise ValueError("acceptance fixture root must be an object")
    if payload.get("schema_version") != _SCHEMA_VERSION:
        raise ValueError(f"acceptance fixture schema_version must be {_SCHEMA_VERSION}")
    tasks = payload.get("tasks")
    if not isinstance(tasks, list):
        raise ValueError("acceptance fixture tasks must be an array")
    if expected_cycles is not None and len(tasks) != expected_cycles:
        raise ValueError(
            f"acceptance fixture contains {len(tasks)} tasks; expected {expected_cycles}"
        )

    parsed_tasks: list[list[dict[str, Any]]] = []
    for task_index, task in enumerate(tasks, 1):
        if not isinstance(task, dict):
            raise ValueError(f"task {task_index} must be an object")
        unknown_task_fields = set(task) - {"windows"}
        if unknown_task_fields:
            names = ", ".join(sorted(unknown_task_fields))
            raise ValueError(f"task {task_index} has unsupported field(s): {names}")
        windows = task.get("windows")
        if not isinstance(windows, list):
            raise ValueError(f"task {task_index}.windows must be an array")
        parsed_windows: list[dict[str, Any]] = []
        for window_index, window in enumerate(windows, 1):
            if not isinstance(window, dict):
                raise ValueError(
                    f"task {task_index} window {window_index} must be an object"
                )
            unknown_window_fields = set(window) - {"accepted", "decision_time_seconds"}
            if unknown_window_fields:
                names = ", ".join(sorted(unknown_window_fields))
                raise ValueError(
                    f"task {task_index} window {window_index} has unsupported field(s): {names}"
                )
            if "accepted" not in window or not isinstance(window["accepted"], bool):
                raise ValueError(
                    f"task {task_index} window {window_index}.accepted must be a boolean"
                )
            decision_time = window.get("decision_time_seconds", 0.0)
            if isinstance(decision_time, bool) or not isinstance(
                decision_time, (int, float)
            ):
                raise ValueError(
                    f"task {task_index} window {window_index}.decision_time_seconds "
                    "must be a number"
                )
            parsed_windows.append(
                {"accepted": window["accepted"], "decision_time_seconds": decision_time}
            )
        try:
            parsed_tasks.append(parsed_windows)
        except ValueError as exc:
            raise ValueError(f"task {task_index}: {exc}") from exc
    try:
        return AcceptanceFixture.from_tasks(parsed_tasks)
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"invalid acceptance fixture: {exc}") from exc


@dataclass
class _LifecycleRecord:
    behavior: str
    started: threading.Event
    closed: threading.Event
    close_finished: threading.Event
    prompt_started: threading.Event
    recognition_started: dict[int, threading.Event]
    stop_requested: threading.Event
    start_called: bool = False
    close_called: bool = False
    close_error: str | None = None

    @classmethod
    def create(cls, behavior: str) -> _LifecycleRecord:
        return cls(
            behavior=behavior,
            started=threading.Event(),
            closed=threading.Event(),
            close_finished=threading.Event(),
            prompt_started=threading.Event(),
            recognition_started={1: threading.Event(), 2: threading.Event()},
            stop_requested=threading.Event(),
        )


@dataclass(frozen=True)
class _TaskBehavior:
    windows: tuple[FixtureWindow, ...] = ()
    cancel_at: str | None = None
    failure: str | None = None
    close_failure: bool = False


class _FixtureStream:
    def __init__(
        self,
        behavior: _TaskBehavior,
        record: _LifecycleRecord,
        active_changed: Callable[[int], None],
    ) -> None:
        self._behavior = behavior
        self._record = record
        self._active_changed = active_changed
        self._active = False

    def start(self) -> None:
        self._record.start_called = True
        if self._behavior.failure == "adb":
            raise RuntimeError("ADB transport failure")
        self._active = True
        self._active_changed(1)
        self._record.started.set()

    def window_chunks(self, _timeout_seconds: float) -> Iterator[bytes]:
        if self._behavior.failure == "stream":
            raise RuntimeError("R818 stream failure")
        if self._record.stop_requested.is_set():
            return iter(())
        return iter((b"\x00" * 16,))

    def close(self) -> None:
        self._record.close_called = True
        if self._active:
            self._active = False
            self._active_changed(-1)
        self._record.closed.set()
        self._record.close_finished.set()
        if self._behavior.close_failure:
            self._record.close_error = "R818 restore failure"
            raise RuntimeError(self._record.close_error)

    def request_stop(self) -> None:
        self._record.stop_requested.set()


class _FixturePromptPlayer:
    def __init__(self, behavior: _TaskBehavior, record: _LifecycleRecord) -> None:
        self._behavior = behavior
        self._record = record

    def reset_stop(self) -> None:
        self._record.stop_requested.clear()

    def request_stop(self) -> None:
        self._record.stop_requested.set()

    def play(self, _prompt: str) -> None:
        self._record.prompt_started.set()
        if self._behavior.failure == "playback":
            raise RuntimeError("Prompt playback failure")
        if self._behavior.cancel_at == "prompt":
            if not self._record.stop_requested.wait(timeout=3.0):
                raise TimeoutError("Prompt cancellation was not delivered")
            raise RuntimeError("Prompt was cancelled")


class _FixtureRecognizer:
    def __init__(
        self,
        behavior: _TaskBehavior,
        record: _LifecycleRecord,
        stream: _FixtureStream | Any,
        *,
        consume_stream: bool,
    ) -> None:
        self._behavior = behavior
        self._record = record
        self._stream = stream
        self._consume_stream = consume_stream

    def recognize(self, attempt_number: int, timeout_seconds: float) -> VoiceWindowResult:
        self._record.recognition_started[attempt_number].set()
        if self._behavior.cancel_at == f"window-{attempt_number}":
            if not self._record.stop_requested.wait(timeout=3.0):
                raise TimeoutError("response cancellation was not delivered")
            raise RuntimeError("response window was cancelled")
        if self._consume_stream:
            chunks = iter(self._stream.window_chunks(timeout_seconds))
            try:
                next(chunks, None)
            finally:
                close = getattr(chunks, "close", None)
                if callable(close):
                    close()
        window = self._behavior.windows[attempt_number - 1]
        return VoiceWindowResult(
            accepted=window.accepted,
            decision_time_seconds=window.decision_time_seconds,
            attempt_number=attempt_number,
        )


class _FixtureAdapterFactory:
    def __init__(self, behaviors: Sequence[_TaskBehavior]) -> None:
        self._behaviors = deque(behaviors)
        self.records: list[_LifecycleRecord] = []
        self.active_sessions = 0
        self.max_active_sessions = 0
        self._active_lock = threading.Lock()

    def __call__(self) -> R818VoiceAdapter:
        if not self._behaviors:
            raise RuntimeError("acceptance created more tasks than the fixture provides")
        behavior = self._behaviors.popleft()
        record = _LifecycleRecord.create(behavior.failure or behavior.cancel_at or "normal")
        self.records.append(record)
        stream = _FixtureStream(behavior, record, self._active_changed)
        recognizer = _FixtureRecognizer(
            behavior, record, stream, consume_stream=True
        )
        return R818VoiceAdapter(
            config=VoiceConfig(response_timeout_seconds=0.05),
            stream=stream,
            recognizer=recognizer,
            prompt_player=_FixturePromptPlayer(behavior, record),
        )

    def _active_changed(self, delta: int) -> None:
        with self._active_lock:
            self.active_sessions += delta
            self.max_active_sessions = max(self.max_active_sessions, self.active_sessions)


class _HardwareAdapterFactory:
    def __init__(
        self,
        config: VoiceConfig,
        helper_path: Path,
        adb: SubprocessAdbFileTransfer,
        fixture: AcceptanceFixture,
    ) -> None:
        self._config = config
        self._helper_path = helper_path
        self._adb = adb
        self._tasks = deque(fixture.tasks)
        self.created = 0

        self._prompt_player = FfmpegAlsaPromptPlayer(
            device=config.prompt_device,
            mixer_card=config.prompt_mixer_card,
            mixer_control=config.prompt_mixer_control,
            volume_percent=config.prompt_volume_percent,
        )

    def __call__(self) -> R818VoiceAdapter:
        if not self._tasks:
            raise RuntimeError("acceptance created more tasks than the fixture provides")
        task = self._tasks.popleft()
        self.created += 1

        def stream_factory() -> SubprocessAdbEncodedPcmStream:
            return SubprocessAdbEncodedPcmStream(
                self._adb,
                helper_path=self._helper_path,
                start_timeout_seconds=self._config.start_timeout_seconds,
                restore_timeout_seconds=self._config.restore_timeout_seconds,
                max_buffer_seconds=self._config.max_buffer_seconds,
            )

        def recognizer_factory(stream: SubprocessAdbEncodedPcmStream) -> _FixtureRecognizer:
            behavior = _TaskBehavior(windows=task.windows)
            return _FixtureRecognizer(
                behavior,
                _LifecycleRecord.create("hardware"),
                stream,
                consume_stream=True,
            )

        return R818VoiceAdapter(
            config=self._config,
            stream_factory=stream_factory,
            recognizer_factory=recognizer_factory,
            prompt_player=self._prompt_player,
        )


def _qos(*, transient: bool, depth: int) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=(
            DurabilityPolicy.TRANSIENT_LOCAL if transient else DurabilityPolicy.VOLATILE
        ),
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


class _RosAcceptanceHarness:
    def __init__(
        self,
        adapter_factory: Callable[[], R818VoiceAdapter],
        preflight: Callable[[], VoicePreflightOutcome],
    ) -> None:
        if rclpy is None:
            raise RuntimeError("ROS 2 Python bindings are unavailable on this host")
        from .provider import VoiceEvidenceProviderNode
        from .readiness_node import VoiceReadinessNode

        self._owns_rclpy = not rclpy.ok()
        if self._owns_rclpy:
            rclpy.init()
        suffix = f"t{time.monotonic_ns()}"
        self.state_topic = f"/issue37/{suffix}/mission/state"
        self.event_topic = f"/issue37/{suffix}/mission/event"
        self.evidence_topic = f"/issue37/{suffix}/perception/authorization_evidence"
        self.status_topic = f"/issue37/{suffix}/perception/capability_status"
        self.source = Node(f"issue37_acceptance_source_{suffix}")
        self.probe = Node(f"issue37_acceptance_probe_{suffix}")
        self.provider = VoiceEvidenceProviderNode(
            parameter_overrides=[
                Parameter("mission_state_topic", value=self.state_topic),
                Parameter("authorization_evidence_topic", value=self.evidence_topic),
            ],
            adapter_factory=adapter_factory,
        )
        self.readiness = VoiceReadinessNode(
            parameter_overrides=[
                Parameter("mission_state_topic", value=self.state_topic),
                Parameter("capability_status_topic", value=self.status_topic),
            ],
            preflight=preflight,
        )
        from dog_patrol_perception_orchestrator.authorization_node import (
            PerceptionAuthorizationNode,
        )

        self.authorization = PerceptionAuthorizationNode(
            parameter_overrides=[
                Parameter("mission_state_topic", value=self.state_topic),
                Parameter("mission_event_topic", value=self.event_topic),
                Parameter("authorization_evidence_topic", value=self.evidence_topic),
            ]
        )
        self.state_pub = self.source.create_publisher(
            MissionState, self.state_topic, _qos(transient=True, depth=1)
        )
        self.events: list[MissionEvent] = []
        self.evidence: list[AuthorizationEvidence] = []
        self.statuses: list[CapabilityStatus] = []
        self.event_sub = self.probe.create_subscription(
            MissionEvent, self.event_topic, self.events.append, _qos(transient=False, depth=10)
        )
        self.evidence_sub = self.probe.create_subscription(
            AuthorizationEvidence,
            self.evidence_topic,
            self.evidence.append,
            _qos(transient=False, depth=20),
        )
        self.status_sub = self.probe.create_subscription(
            CapabilityStatus,
            self.status_topic,
            self.statuses.append,
            _qos(transient=True, depth=16),
        )
        from rclpy.executors import SingleThreadedExecutor

        self.executor = SingleThreadedExecutor()
        for node in (
            self.source,
            self.probe,
            self.provider,
            self.readiness,
            self.authorization,
        ):
            self.executor.add_node(node)

    def publish_state(
        self,
        state_seq: int,
        target_id: int,
        *,
        state: int | None = None,
        blocked: bool = False,
    ) -> None:
        if state is None:
            state = MissionState.VERIFY_IDENTITY
        message = MissionState()
        message.header.stamp = self.source.get_clock().now().to_msg()
        message.state_seq = state_seq
        message.state = state
        message.target_id = target_id
        message.blocked = blocked
        self.state_pub.publish(message)
        for _ in range(5):
            self.executor.spin_once(timeout_sec=0.02)

    def wait(self, predicate: Callable[[], bool], timeout: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not predicate():
            self.executor.spin_once(timeout_sec=0.05)
        return predicate()

    def close(self) -> None:
        for node in (
            self.provider,
            self.readiness,
            self.authorization,
            self.source,
            self.probe,
        ):
            node.destroy_node()
        self.executor.shutdown()
        if self._owns_rclpy and rclpy.ok():
            rclpy.shutdown()


def _event_for_windows(windows: Sequence[FixtureWindow]) -> int:
    return MissionEvent.AUTHORIZED if windows[-1].accepted else MissionEvent.UNAUTHORIZED


def _result_name(result: int) -> str:
    return {
        AuthorizationEvidence.PASSED: "PASSED",
        AuthorizationEvidence.NOT_PASSED: "NOT_PASSED",
        AuthorizationEvidence.ERROR: "ERROR",
        AuthorizationEvidence.CANCELLED: "CANCELLED",
    }.get(result, f"UNKNOWN_{result}")


def _event_name(event: int) -> str:
    return {
        MissionEvent.AUTHORIZED: "AUTHORIZED",
        MissionEvent.UNAUTHORIZED: "UNAUTHORIZED",
        MissionEvent.EXECUTION_ERROR: "EXECUTION_ERROR",
    }.get(event, f"UNKNOWN_{event}")


def _cycle_behaviors(fixture: AcceptanceFixture) -> list[_TaskBehavior]:
    return [_TaskBehavior(windows=task.windows) for task in fixture.tasks]


def _run_cycles(
    fixture: AcceptanceFixture,
    *,
    cycles: int,
    adapter_factory: Callable[[], R818VoiceAdapter],
    cleanup_checker: Callable[[], list[str]] | None = None,
    preflight: Callable[[], VoicePreflightOutcome],
) -> dict[str, Any]:
    if len(fixture.tasks) != cycles:
        raise ValueError(f"fixture contains {len(fixture.tasks)} tasks; expected {cycles}")
    harness = _RosAcceptanceHarness(adapter_factory, preflight)
    cycle_reports: list[dict[str, Any]] = []
    try:
        harness.publish_state(1, 0, state=MissionState.STARTUP)
        readiness_ok = harness.wait(
            lambda: (
                len(harness.statuses) == 1
                and harness.statuses[0].observed_startup_state_seq == 1
            )
        )
        readiness_report = {
            "status": (
                "READY"
                if readiness_ok and harness.statuses[0].status == CapabilityStatus.READY
                else "FAILED"
            ),
            "observed_startup_state_seq": (
                int(harness.statuses[0].observed_startup_state_seq)
                if harness.statuses
                else None
            ),
            "diagnostic": harness.statuses[0].diagnostic if harness.statuses else "missing",
        }
        for index, task in enumerate(fixture.tasks):
            state_seq = 1000 + index
            target_id = 2000 + index
            evidence_start = len(harness.evidence)
            events_start = len(harness.events)
            harness.publish_state(state_seq, target_id)
            task_started = harness.wait(
                lambda: len(harness.evidence) > evidence_start
                or len(harness.events) > events_start,
                timeout=0.5,
            )
            evidence_ok = harness.wait(
                lambda: len(harness.evidence) >= evidence_start + len(task.windows)
            )
            event_ok = harness.wait(
                lambda: len(harness.events) > events_start
            )
            evidence = harness.evidence[evidence_start:]
            events = harness.events[events_start:]
            expected_evidence = [
                AuthorizationEvidence.PASSED
                if window.accepted
                else AuthorizationEvidence.NOT_PASSED
                for window in task.windows
            ]
            if task.windows[0].accepted:
                expected_evidence = expected_evidence[:1]
            expected_event = _event_for_windows(task.windows)
            residuals = cleanup_checker() if cleanup_checker is not None else []
            cycle_reports.append(
                {
                    "index": index + 1,
                    "state_seq": state_seq,
                    "target_id": target_id,
                    "task_started": task_started,
                    "evidence": [_result_name(message.result) for message in evidence],
                    "events": [_event_name(message.event) for message in events],
                    "expected_evidence": [_result_name(result) for result in expected_evidence],
                    "expected_event": _event_name(expected_event),
                    "cleanup": {
                        "complete": evidence_ok and event_ok and not residuals,
                        "remote_residuals": residuals,
                    },
                    "passed": (
                        task_started
                        and evidence_ok
                        and event_ok
                        and [message.result for message in evidence] == expected_evidence
                        and len(events) == 1
                        and events[0].event == expected_event
                        and events[0].observed_state_seq == state_seq
                        and events[0].target_id == target_id
                        and not residuals
                    ),
                }
            )
        return {
            "readiness": readiness_report,
            "cycles": cycle_reports,
            "passed": readiness_report["status"] == "READY"
            and all(cycle["passed"] for cycle in cycle_reports),
        }
    finally:
        harness.close()


def _scenario_behaviors(name: str) -> tuple[list[_TaskBehavior], str]:
    accepted = FixtureWindow(True, 0.2)
    rejected = FixtureWindow(False, 20.0)
    if name == "prompt_cancel":
        return [_TaskBehavior(windows=(accepted,), cancel_at="prompt")], "cancel"
    if name == "first_window_cancel":
        return [_TaskBehavior(windows=(accepted,), cancel_at="window-1")], "cancel"
    if name == "second_window_cancel":
        return [
            _TaskBehavior(windows=(rejected, accepted), cancel_at="window-2")
        ], "cancel"
    if name in {"state_seq_replace", "target_id_replace"}:
        return [
            _TaskBehavior(windows=(accepted,), cancel_at="window-1"),
            _TaskBehavior(windows=(accepted,)),
        ], "replace"
    if name == "stream_failure":
        return [_TaskBehavior(windows=(accepted,), failure="stream")], "failure"
    if name == "adb_failure":
        return [_TaskBehavior(windows=(accepted,), failure="adb")], "failure"
    if name == "playback_failure":
        return [_TaskBehavior(windows=(accepted,), failure="playback")], "failure"
    if name == "restore_failure":
        return [_TaskBehavior(windows=(accepted,), close_failure=True)], "failure"
    raise ValueError(f"unknown acceptance scenario: {name}")


def _run_failure_scenario(name: str) -> dict[str, Any]:
    behaviors, kind = _scenario_behaviors(name)
    factory = _FixtureAdapterFactory(behaviors)
    harness = _RosAcceptanceHarness(
        factory,
        lambda: VoicePreflightOutcome(READY, "fixture readiness ready"),
    )
    expected_evidence: list[int]
    expected_events: list[int]
    try:
        harness.publish_state(10, 1)
        if not harness.wait(lambda: len(factory.records) >= 1):
            raise RuntimeError("provider did not create the first task")
        first = factory.records[0]
        if kind == "cancel":
            if name == "prompt_cancel":
                stage_ready = first.prompt_started
            elif name == "first_window_cancel":
                stage_ready = first.recognition_started[1]
            else:
                stage_ready = first.recognition_started[2]
            if not stage_ready.wait(timeout=2.0):
                raise RuntimeError(f"scenario did not reach {name} stage")
            harness.publish_state(10, 1, state=MissionState.PATROL)
            expected_evidence = [AuthorizationEvidence.CANCELLED]
            if name == "second_window_cancel":
                expected_evidence.insert(0, AuthorizationEvidence.NOT_PASSED)
            expected_events = []
        elif kind == "replace":
            if not first.recognition_started[1].wait(timeout=2.0):
                raise RuntimeError(f"scenario did not reach {name} replacement stage")
            if name == "state_seq_replace":
                replacement = (11, 2)
            else:
                replacement = (10, 2)
            harness.publish_state(*replacement)
            expected_evidence = [
                AuthorizationEvidence.CANCELLED,
                AuthorizationEvidence.PASSED,
            ]
            expected_events = [MissionEvent.AUTHORIZED]
        else:
            expected_evidence = [AuthorizationEvidence.ERROR]
            expected_events = [MissionEvent.EXECUTION_ERROR]

        if not harness.wait(lambda: first.closed.is_set(), timeout=4.0):
            raise RuntimeError(f"scenario {name} did not complete cleanup")
        if kind == "replace" and not harness.wait(
            lambda: len(factory.records) == 2 and factory.records[1].closed.is_set(),
            timeout=4.0,
        ):
            raise RuntimeError(f"scenario {name} did not complete replacement cleanup")
        if not harness.wait(
            lambda: len(harness.evidence) >= len(expected_evidence), timeout=4.0
        ):
            raise RuntimeError(f"scenario {name} did not publish expected evidence")
        if expected_events and not harness.wait(
            lambda: len(harness.events) >= len(expected_events), timeout=4.0
        ):
            raise RuntimeError(f"scenario {name} did not publish expected event")
        evidence = [message.result for message in harness.evidence]
        events = [message.event for message in harness.events]
        cleanup_complete = all(record.close_called for record in factory.records)
        passed = (
            evidence == expected_evidence
            and events == expected_events
            and cleanup_complete
            and factory.max_active_sessions <= 1
        )
        return {
            "name": name,
            "passed": passed,
            "evidence": [_result_name(result) for result in evidence],
            "expected_evidence": [_result_name(result) for result in expected_evidence],
            "events": [_event_name(event) for event in events],
            "expected_events": [_event_name(event) for event in expected_events],
            "cleanup": {
                "complete": cleanup_complete,
                "tasks": len(factory.records),
                "close_errors": [
                    record.close_error
                    for record in factory.records
                    if record.close_error
                ],
            },
            "max_active_sessions": factory.max_active_sessions,
            "late_evidence": evidence != expected_evidence,
        }
    except Exception as exc:
        return {
            "name": name,
            "passed": False,
            "error": f"{type(exc).__name__}: {exc}",
            "cleanup": {
                "complete": all(record.close_called for record in factory.records),
                "tasks": len(factory.records),
            },
            "max_active_sessions": factory.max_active_sessions,
            "late_evidence": False,
        }
    finally:
        for record in factory.records:
            record.stop_requested.set()
        harness.close()


def run_failure_matrix() -> list[dict[str, Any]]:
    """Run deterministic lifecycle and ROS fault scenarios without hardware."""
    return [_run_failure_scenario(name) for name in _FAILURE_SCENARIOS]


def run_fixture_acceptance(
    fixture: AcceptanceFixture,
    *,
    cycles: int,
) -> dict[str, Any]:
    """Run the full ROS acceptance using fixture outcomes and fake hardware seams."""
    factory = _FixtureAdapterFactory(_cycle_behaviors(fixture))
    cycle_report = _run_cycles(
        fixture,
        cycles=cycles,
        adapter_factory=factory,
        preflight=lambda: VoicePreflightOutcome(READY, "fixture readiness ready"),
    )
    matrix = run_failure_matrix()
    cycle_report["failure_matrix"] = matrix
    cycle_report["mode"] = "fixture"
    cycle_report["passed"] = cycle_report["passed"] and all(
        scenario["passed"] for scenario in matrix
    )
    return cycle_report


def check_remote_residue(adb: SubprocessAdbFileTransfer) -> list[str]:
    """Return remote R818 temp paths or recorder processes left after cleanup."""
    probe = (
        "for path in /tmp/dog-patrol-r818-stream.pcm "
        "/tmp/dog-patrol-r818-stream.pid /tmp/dog-patrol-r818-stream.err "
        "/tmp/dog-patrol-r818-base64; do "
        "test ! -e \"$path\" || echo \"path:$path\"; done; "
        "ps | grep '[a]record -q -D hw:1,0' && echo 'process:arecord' || true"
    )
    try:
        output = adb.shell_script(probe).stdout
    except Exception as exc:
        return [f"residue probe failed: {type(exc).__name__}: {exc}"]
    return [line.strip() for line in str(output).splitlines() if line.strip()]


def run_hardware_acceptance(
    fixture: AcceptanceFixture,
    *,
    cycles: int,
    model_dir: str | Path,
    config_file: str | Path,
    helper_path: str | Path | None = None,
) -> dict[str, Any]:
    """Run clean-install preflight, real R818 cycles, and the fault matrix."""
    config = load_voice_config(config_file)
    selected_helper = Path(helper_path) if helper_path else default_helper_path()
    preflight = VoicePreflight(
        model_dir=model_dir,
        config_file=config_file,
        helper_path=selected_helper,
    )
    outcome = preflight.run()
    readiness = {
        "status": {READY: "READY", NOT_READY: "NOT_READY", ERROR: "ERROR"}.get(
            outcome.status, "UNKNOWN"
        ),
        "diagnostic": outcome.diagnostic,
    }
    report: dict[str, Any] = {
        "mode": "hardware",
        "readiness": readiness,
        "host_pcm_capture": False,
    }
    if outcome.status != READY:
        report["cycles"] = []
        report["failure_matrix"] = run_failure_matrix()
        report["passed"] = False
        return report

    adb = SubprocessAdbFileTransfer(
        config.adb_executable,
        device_serial=config.device_serial,
        timeout_seconds=config.adb_timeout_seconds,
    )
    factory = _HardwareAdapterFactory(config, selected_helper, adb, fixture)
    cycle_report = _run_cycles(
        fixture,
        cycles=cycles,
        adapter_factory=factory,
        cleanup_checker=lambda: check_remote_residue(adb),
        preflight=preflight.run,
    )
    matrix = run_failure_matrix()
    report["preflight"] = readiness
    report.update(cycle_report)
    report["failure_matrix"] = matrix
    report["passed"] = cycle_report["passed"] and all(
        scenario["passed"] for scenario in matrix
    )
    return report


def _default_config_path() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        return (
            Path(get_package_share_directory("dog_patrol_perception_voice"))
            / "config"
            / "voice.yaml"
        )
    except Exception:
        return Path("/__dog_patrol_voice_install_missing__/config/voice.yaml")


def _write_report(path: Path, report: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run unattended dog_patrol voice deployment acceptance."
    )
    parser.add_argument("--mode", choices=("fixture", "hardware"), default="fixture")
    parser.add_argument("--fixture", required=True, help="deployment-local task outcome JSON")
    parser.add_argument("--cycles", type=int, default=33)
    parser.add_argument("--model-dir", help="Vosk model directory (required in hardware mode)")
    parser.add_argument("--config-file", default=str(_default_config_path()))
    parser.add_argument("--helper-path")
    parser.add_argument("--report", type=Path, help="write a JSON report at this path")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.cycles <= 0:
        print("acceptance cycles must be positive")
        return 2
    try:
        fixture = load_acceptance_fixture(args.fixture, expected_cycles=args.cycles)
        if args.mode == "hardware":
            if not args.model_dir:
                raise ValueError("--model-dir is required in hardware mode")
            report = run_hardware_acceptance(
                fixture,
                cycles=args.cycles,
                model_dir=args.model_dir,
                config_file=args.config_file,
                helper_path=args.helper_path,
            )
        else:
            report = run_fixture_acceptance(fixture, cycles=args.cycles)
        report["issue"] = 37
        report["started_at_utc"] = datetime.now(timezone.utc).isoformat()
        report["environment"] = {
            "architecture": platform.machine(),
            "python": sys.version.split()[0],
            "fixture": str(args.fixture),
            "cycles_requested": args.cycles,
            "host_pcm_capture": False,
        }
        report["command"] = shlex.join(["perception_voice_acceptance", *sys.argv[1:]])
        if args.report:
            _write_report(args.report, report)
        print(json.dumps(report, ensure_ascii=False, sort_keys=True))
        return 0 if report["passed"] else 1
    except (OSError, ValueError, RuntimeError) as exc:
        print(f"voice acceptance failed before completion: {type(exc).__name__}: {exc}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
