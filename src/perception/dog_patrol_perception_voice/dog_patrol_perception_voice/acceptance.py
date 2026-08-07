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
import hashlib
import json
import math
from pathlib import Path
import platform
import re
import shlex
import subprocess
import sys
import threading
import time
from typing import Any

try:
    import rclpy
except ImportError as exc:  # pragma: no cover - only used on non-ROS development hosts.
    rclpy = None  # type: ignore[assignment]
    ROS_IMPORT_ERROR = exc
else:
    from dog_patrol_interfaces.msg import MissionEvent, MissionState
    from dog_patrol_perception_interfaces.msg import AuthorizationEvidence, CapabilityStatus
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    ROS_IMPORT_ERROR = None

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
from .r818_stream import R818StreamingVoskSession, SubprocessAdbEncodedPcmStream
from .vosk import load_vosk_model


_SCHEMA_VERSION = 1
_ENVIRONMENT_CHECK_TIMEOUT_SECONDS = 300.0
_DEFAULT_ACCEPTANCE_CYCLES = 3
_MIN_FAILURE_STAGE_TIMEOUT_SECONDS = 30.0
_REQUIRED_HARDWARE_PROVENANCE = {
    "source_commit",
    "source_matrix",
    "task_manifest_sha256",
}
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
    provenance: dict[str, str] | None = None

    @classmethod
    def from_tasks(
        cls,
        tasks: Sequence[Sequence[dict[str, Any]]],
        provenance: dict[str, str] | None = None,
    ) -> AcceptanceFixture:
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
            ),
            provenance=provenance,
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
    unknown_root_fields = set(payload) - {"schema_version", "tasks", "provenance"}
    if unknown_root_fields:
        names = ", ".join(sorted(unknown_root_fields))
        raise ValueError(f"acceptance fixture has unsupported field(s): {names}")
    provenance = payload.get("provenance")
    if provenance is not None:
        if not isinstance(provenance, dict) or any(
            not isinstance(key, str) or not isinstance(value, str)
            for key, value in provenance.items()
        ):
            raise ValueError("acceptance fixture provenance must be a string map")
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
        return AcceptanceFixture.from_tasks(parsed_tasks, provenance=provenance)
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
        self.records: list[_LifecycleRecord] = []
        self.active_sessions = 0
        self.max_active_sessions = 0
        self._active_lock = threading.Lock()

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
        behavior = _TaskBehavior(windows=task.windows)
        record = _LifecycleRecord.create("hardware")
        self.records.append(record)
        stream = _HardwareTrackedStream(
            SubprocessAdbEncodedPcmStream(
                self._adb,
                helper_path=self._helper_path,
                start_timeout_seconds=self._config.start_timeout_seconds,
                restore_timeout_seconds=self._config.restore_timeout_seconds,
                max_buffer_seconds=self._config.max_buffer_seconds,
            ),
            behavior,
            record,
            self._active_changed,
        )
        recognizer = _FixtureRecognizer(
            behavior,
            record,
            stream,
            consume_stream=True,
        )

        return R818VoiceAdapter(
            config=self._config,
            stream=stream,
            recognizer=recognizer,
            prompt_player=self._prompt_player,
        )

    def _active_changed(self, delta: int) -> None:
        with self._active_lock:
            self.active_sessions += delta
            self.max_active_sessions = max(
                self.max_active_sessions, self.active_sessions
            )

    def task_started(self, index: int) -> bool:
        return index < len(self.records) and self.records[index].started.is_set()


class _HardwareTrackedStream:
    """Track real R818 ownership and optionally inject one controlled failure."""

    def __init__(
        self,
        stream: SubprocessAdbEncodedPcmStream,
        behavior: _TaskBehavior | None,
        record: _LifecycleRecord,
        active_changed: Callable[[int], None],
    ) -> None:
        self._stream = stream
        self._behavior = behavior
        self._record = record
        self._active_changed = active_changed
        self._active = False

    def start(self) -> None:
        self._record.start_called = True
        if self._behavior is not None and self._behavior.failure == "adb":
            raise RuntimeError("injected ADB transport failure")
        self._stream.start()
        self._active = True
        self._active_changed(1)
        self._record.started.set()

    def window_chunks(self, timeout_seconds: float) -> Iterator[bytes]:
        if self._behavior is not None and self._behavior.failure == "stream":
            raise RuntimeError("injected R818 stream failure")
        return self._stream.window_chunks(timeout_seconds)

    def close(self) -> None:
        self._record.close_called = True
        try:
            self._stream.close()
        finally:
            if self._active:
                self._active = False
                self._active_changed(-1)
            self._record.closed.set()
            self._record.close_finished.set()
        if self._behavior is not None and self._behavior.close_failure:
            self._record.close_error = "injected R818 restore failure"
            raise RuntimeError(self._record.close_error)

    def request_stop(self) -> None:
        self._record.stop_requested.set()
        self._stream.request_stop()


class _HardwareFailurePromptPlayer:
    def __init__(
        self,
        player: FfmpegAlsaPromptPlayer,
        behavior: _TaskBehavior,
        record: _LifecycleRecord,
    ) -> None:
        self._player = player
        self._behavior = behavior
        self._record = record

    def reset_stop(self) -> None:
        self._player.reset_stop()

    def request_stop(self) -> None:
        self._record.stop_requested.set()
        self._player.request_stop()

    def play(self, prompt: str) -> None:
        self._record.prompt_started.set()
        if self._behavior.failure == "playback":
            raise RuntimeError("injected Prompt playback failure")
        self._player.play(prompt)


class _HardwareFailureAdapterFactory:
    """Build real hardware adapters for the automatic failure matrix."""

    def __init__(
        self,
        config: VoiceConfig,
        helper_path: Path,
        adb: SubprocessAdbFileTransfer,
        behaviors: Sequence[_TaskBehavior],
    ) -> None:
        self._config = config
        self._helper_path = helper_path
        self._adb = adb
        self._behaviors = deque(behaviors)
        self.records: list[_LifecycleRecord] = []
        self.active_sessions = 0
        self.max_active_sessions = 0
        self._active_lock = threading.Lock()

    def __call__(self) -> R818VoiceAdapter:
        if not self._behaviors:
            raise RuntimeError("hardware failure matrix created too many tasks")
        behavior = self._behaviors.popleft()
        record = _LifecycleRecord.create(
            behavior.failure or behavior.cancel_at or "hardware"
        )
        self.records.append(record)
        stream = _HardwareTrackedStream(
            SubprocessAdbEncodedPcmStream(
                self._adb,
                helper_path=self._helper_path,
                start_timeout_seconds=self._config.start_timeout_seconds,
                restore_timeout_seconds=self._config.restore_timeout_seconds,
                max_buffer_seconds=self._config.max_buffer_seconds,
            ),
            behavior,
            record,
            self._active_changed,
        )
        recognizer = _FixtureRecognizer(
            behavior,
            record,
            stream,
            consume_stream=True,
        )
        player = _HardwareFailurePromptPlayer(
            FfmpegAlsaPromptPlayer(
                device=self._config.prompt_device,
                mixer_card=self._config.prompt_mixer_card,
                mixer_control=self._config.prompt_mixer_control,
                volume_percent=self._config.prompt_volume_percent,
            ),
            behavior,
            record,
        )
        return R818VoiceAdapter(
            config=self._config,
            stream=stream,
            recognizer=recognizer,
            prompt_player=player,
        )

    def _active_changed(self, delta: int) -> None:
        with self._active_lock:
            self.active_sessions += delta
            self.max_active_sessions = max(
                self.max_active_sessions, self.active_sessions
            )


class _LiveHardwareAdapterFactory:
    """Build production adapters that derive results from the live R818 stream."""

    def __init__(
        self,
        config: VoiceConfig,
        helper_path: Path,
        adb: SubprocessAdbFileTransfer,
        model_dir: str | Path,
    ) -> None:
        self._config = config
        self._helper_path = helper_path
        self._adb = adb
        self._model, self._recognizer_factory = load_vosk_model(model_dir)
        self._prompt_player = FfmpegAlsaPromptPlayer(
            device=config.prompt_device,
            mixer_card=config.prompt_mixer_card,
            mixer_control=config.prompt_mixer_control,
            volume_percent=config.prompt_volume_percent,
        )
        self.records: list[_LifecycleRecord] = []
        self.active_sessions = 0
        self.max_active_sessions = 0
        self._active_lock = threading.Lock()

    def __call__(self) -> R818VoiceAdapter:
        record = _LifecycleRecord.create("field")
        self.records.append(record)
        stream = _HardwareTrackedStream(
            SubprocessAdbEncodedPcmStream(
                self._adb,
                helper_path=self._helper_path,
                start_timeout_seconds=self._config.start_timeout_seconds,
                restore_timeout_seconds=self._config.restore_timeout_seconds,
                max_buffer_seconds=self._config.max_buffer_seconds,
            ),
            None,
            record,
            self._active_changed,
        )
        recognizer = R818StreamingVoskSession(
            stream,
            self._config,
            model=self._model,
            recognizer_factory=self._recognizer_factory,
        )
        return R818VoiceAdapter(
            config=self._config,
            stream=stream,
            recognizer=recognizer,
            prompt_player=self._prompt_player,
        )

    def _active_changed(self, delta: int) -> None:
        with self._active_lock:
            self.active_sessions += delta
            self.max_active_sessions = max(
                self.max_active_sessions, self.active_sessions
            )

    def task_started(self, index: int) -> bool:
        return index < len(self.records) and self.records[index].started.is_set()


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
            detail = f": {ROS_IMPORT_ERROR}" if ROS_IMPORT_ERROR else ""
            raise RuntimeError(
                f"ROS 2 Python bindings are unavailable on this host{detail}"
            )
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

    def wait_quiet(
        self,
        evidence_count: int,
        event_count: int,
        duration: float = 0.25,
    ) -> bool:
        """Spin a bounded grace period and reject any late ROS output."""
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.05)
            if len(self.evidence) != evidence_count or len(self.events) != event_count:
                return False
        return len(self.evidence) == evidence_count and len(self.events) == event_count

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


def minimal_field_matrix() -> AcceptanceFixture:
    """Return the three live-speech outcome shapes required by issue #38."""
    return AcceptanceFixture.from_tasks(
        [
            [{"accepted": True}],
            [{"accepted": False}, {"accepted": True}],
            [{"accepted": False}, {"accepted": False}],
        ]
    )


def _run_cycles(
    fixture: AcceptanceFixture,
    *,
    cycles: int,
    adapter_factory: Callable[[], R818VoiceAdapter],
    cleanup_checker: Callable[[], dict[str, Any]] | None = None,
    preflight: Callable[[], VoicePreflightOutcome],
    task_started_checker: Callable[[int], bool] | None = None,
    wait_for_idle: Callable[[], bool] | None = None,
    session_count: Callable[[], int] | None = None,
    task_timeout_seconds: float = 5.0,
    cancel_timeout_seconds: float = 5.0,
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
        aborted = False
        for index, task in enumerate(fixture.tasks):
            state_seq = 1000 + index
            target_id = 2000 + index
            sessions_before = session_count() if session_count is not None else None
            evidence_start = len(harness.evidence)
            events_start = len(harness.events)
            harness.publish_state(state_seq, target_id)
            task_started = harness.wait(
                lambda: (
                    len(harness.evidence) > evidence_start
                    or len(harness.events) > events_start
                    or (
                        task_started_checker is not None
                        and task_started_checker(index)
                    )
                ),
                timeout=min(5.0, task_timeout_seconds),
            )
            evidence_ok = harness.wait(
                lambda: len(harness.evidence) >= evidence_start + len(task.windows),
                timeout=task_timeout_seconds,
            )
            event_ok = harness.wait(
                lambda: len(harness.events) > events_start,
                timeout=task_timeout_seconds,
            )
            if task_started_checker is not None:
                task_started = task_started or task_started_checker(index)
            idle_ok = True
            if wait_for_idle is not None and not (evidence_ok and event_ok):
                harness.publish_state(state_seq, target_id, state=MissionState.PATROL)
                idle_ok = harness.wait(
                    wait_for_idle,
                    timeout=cancel_timeout_seconds,
                )
            quiet_ok = harness.wait_quiet(
                len(harness.evidence), len(harness.events)
            )
            sessions_started = (
                session_count() - sessions_before
                if session_count is not None and sessions_before is not None
                else None
            )
            sessions_ok = sessions_started is None or sessions_started == 1
            evidence = harness.evidence[evidence_start:]
            events = harness.events[events_start:]
            evidence_matches_session = all(
                message.observed_state_seq == state_seq
                and message.target_id == target_id
                for message in evidence
            )
            expected_evidence = [
                AuthorizationEvidence.PASSED
                if window.accepted
                else AuthorizationEvidence.NOT_PASSED
                for window in task.windows
            ]
            if task.windows[0].accepted:
                expected_evidence = expected_evidence[:1]
            expected_event = _event_for_windows(task.windows)
            cleanup = cleanup_checker() if cleanup_checker is not None else {}
            residuals = cleanup.get("remote_residuals", [])
            vendor_owner = cleanup.get("vendor_owner")
            owner_ok = (
                vendor_owner is None or vendor_owner.get("status") == "READY"
            )
            cycle_reports.append(
                {
                    "index": index + 1,
                    "state_seq": state_seq,
                    "target_id": target_id,
                    "task_started": task_started,
                    "hardware_sessions_started": sessions_started,
                    "evidence": [_result_name(message.result) for message in evidence],
                    "evidence_details": [message.detail for message in evidence],
                    "evidence_sessions": [
                        {
                            "state_seq": int(message.observed_state_seq),
                            "target_id": int(message.target_id),
                        }
                        for message in evidence
                    ],
                    "events": [_event_name(message.event) for message in events],
                    "event_details": [message.detail for message in events],
                    "expected_evidence": [_result_name(result) for result in expected_evidence],
                    "expected_event": _event_name(expected_event),
                    "cleanup": {
                        "complete": evidence_ok
                        and event_ok
                        and quiet_ok
                        and idle_ok
                        and sessions_ok
                        and evidence_matches_session
                        and not residuals
                        and owner_ok,
                        "remote_residuals": residuals,
                        "vendor_owner": vendor_owner,
                        "late_evidence": not quiet_ok,
                    },
                    "passed": (
                        task_started
                        and evidence_ok
                        and event_ok
                        and idle_ok
                        and sessions_ok
                        and evidence_matches_session
                        and [message.result for message in evidence] == expected_evidence
                        and len(events) == 1
                        and events[0].event == expected_event
                        and events[0].observed_state_seq == state_seq
                        and events[0].target_id == target_id
                        and not residuals
                        and owner_ok
                        and quiet_ok
                    ),
                }
            )
            if wait_for_idle is not None and not (evidence_ok and event_ok and idle_ok):
                aborted = True
                break
        return {
            "readiness": readiness_report,
            "cycles": cycle_reports,
            "cycles_completed": len(cycle_reports),
            "cycles_aborted": aborted,
            "passed": readiness_report["status"] == "READY"
            and len(cycle_reports) == cycles
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


def _run_failure_scenario(
    name: str,
    *,
    factory_builder: Callable[[Sequence[_TaskBehavior]], Any] | None = None,
    preflight: Callable[[], VoicePreflightOutcome] | None = None,
    cleanup_checker: Callable[[], dict[str, Any]] | None = None,
    stage_timeout_seconds: float = 2.0,
    completion_timeout_seconds: float = 4.0,
) -> dict[str, Any]:
    behaviors, kind = _scenario_behaviors(name)
    factory = (
        factory_builder(behaviors)
        if factory_builder is not None
        else _FixtureAdapterFactory(behaviors)
    )
    harness = _RosAcceptanceHarness(
        factory,
        preflight or (lambda: VoicePreflightOutcome(READY, "fixture readiness ready")),
    )
    expected_evidence: list[int]
    expected_events: list[int]
    result: dict[str, Any] | None = None
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
            if not stage_ready.wait(timeout=stage_timeout_seconds):
                raise RuntimeError(f"scenario did not reach {name} stage")
            harness.publish_state(10, 1, state=MissionState.PATROL)
            expected_evidence = [AuthorizationEvidence.CANCELLED]
            if name == "second_window_cancel":
                expected_evidence.insert(0, AuthorizationEvidence.NOT_PASSED)
            expected_events = []
        elif kind == "replace":
            if not first.recognition_started[1].wait(timeout=stage_timeout_seconds):
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

        if not harness.wait(lambda: first.closed.is_set(), timeout=completion_timeout_seconds):
            raise RuntimeError(f"scenario {name} did not complete cleanup")
        if kind == "replace" and not harness.wait(
            lambda: len(factory.records) == 2 and factory.records[1].closed.is_set(),
            timeout=completion_timeout_seconds,
        ):
            raise RuntimeError(f"scenario {name} did not complete replacement cleanup")
        if not harness.wait(
            lambda: len(harness.evidence) >= len(expected_evidence),
            timeout=completion_timeout_seconds,
        ):
            raise RuntimeError(f"scenario {name} did not publish expected evidence")
        if expected_events and not harness.wait(
            lambda: len(harness.events) >= len(expected_events),
            timeout=completion_timeout_seconds,
        ):
            raise RuntimeError(f"scenario {name} did not publish expected event")
        quiet_ok = harness.wait_quiet(len(harness.evidence), len(harness.events))
        evidence = [message.result for message in harness.evidence]
        events = [message.event for message in harness.events]
        cleanup = cleanup_checker() if cleanup_checker is not None else {}
        remote_residuals = cleanup.get("remote_residuals", [])
        vendor_owner = cleanup.get("vendor_owner")
        cleanup_complete = (
            all(record.close_called for record in factory.records)
            and not remote_residuals
            and quiet_ok
            and (
                vendor_owner is None
                or vendor_owner.get("status") == "READY"
            )
        )
        passed = (
            evidence == expected_evidence
            and events == expected_events
            and cleanup_complete
            and factory.max_active_sessions <= 1
        )
        result = {
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
                "remote_residuals": remote_residuals,
                "vendor_owner": vendor_owner,
            },
            "max_active_sessions": factory.max_active_sessions,
            "late_evidence": not quiet_ok or evidence != expected_evidence,
        }
    except Exception as exc:
        result = {
            "name": name,
            "passed": False,
            "error": f"{type(exc).__name__}: {exc}",
            "cleanup": {
                "complete": False,
                "tasks": len(factory.records),
                "remote_residuals": [],
            },
            "max_active_sessions": factory.max_active_sessions,
            "late_evidence": not harness.wait_quiet(
                len(harness.evidence), len(harness.events)
            ),
        }
    finally:
        for record in factory.records:
            record.stop_requested.set()
        harness.close()
    assert result is not None
    if cleanup_checker is not None:
        try:
            post_cleanup = cleanup_checker()
        except Exception as exc:
            post_cleanup = {
                "remote_residuals": [
                    f"post-cleanup probe failed: {type(exc).__name__}: {exc}"
                ],
                "vendor_owner": None,
            }
        cleanup = result["cleanup"]
        cleanup["remote_residuals"] = post_cleanup.get("remote_residuals", [])
        cleanup["vendor_owner"] = post_cleanup.get("vendor_owner")
        owner = cleanup.get("vendor_owner")
        cleanup["complete"] = (
            all(record.close_called for record in factory.records)
            and not cleanup["remote_residuals"]
            and (owner is None or owner.get("status") == "READY")
        )
        if not cleanup["complete"]:
            result["passed"] = False
    return result


def run_failure_matrix() -> list[dict[str, Any]]:
    """Run deterministic lifecycle and ROS fault scenarios without hardware."""
    return [_run_failure_scenario(name) for name in _FAILURE_SCENARIOS]


def run_hardware_failure_matrix(
    config: VoiceConfig,
    *,
    helper_path: Path,
    adb: SubprocessAdbFileTransfer,
    timeout_seconds: float = 30.0,
) -> list[dict[str, Any]]:
    """Run the same fault matrix with real R818/ALSA resources and injected faults."""
    # A matrix stage is expected to become observable after startup, not after a
    # full recognition window.  Keep a hard upper bound here so a missing stream
    # cannot turn one matrix item into an unattended multi-minute wait.
    stage_timeout_seconds = min(
        timeout_seconds,
        max(_MIN_FAILURE_STAGE_TIMEOUT_SECONDS, config.start_timeout_seconds + 5.0),
    )
    completion_timeout_seconds = min(
        timeout_seconds,
        max(
            60.0,
            3.0 * config.adb_timeout_seconds
            + config.restore_timeout_seconds
            + 10.0,
        ),
    )
    return [
        _run_failure_scenario(
            name,
            factory_builder=lambda behaviors: _HardwareFailureAdapterFactory(
                config, helper_path, adb, behaviors
            ),
            cleanup_checker=lambda: check_remote_state(adb),
            stage_timeout_seconds=stage_timeout_seconds,
            completion_timeout_seconds=completion_timeout_seconds,
        )
        for name in _FAILURE_SCENARIOS
    ]


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
        "ps_output=$(ps 2>&1); ps_status=$?; "
        "test $ps_status -eq 0 || echo probe-error:ps:$ps_status; "
        "recorder_name=arecord; recorder_args='-q -D hw:1,0'; "
        "helper_prefix=dog-patrol-r818; helper_suffix=base64; "
        "if printf '%s\\n' \"$ps_output\" | grep -F \"$recorder_name $recorder_args\" >/dev/null; then "
        "echo 'process:arecord'; fi; "
        "if printf '%s\\n' \"$ps_output\" | grep -F \"$helper_prefix-$helper_suffix\" >/dev/null; then "
        "echo 'process:helper'; fi; exit 0"
    )
    try:
        output = adb.shell_script(probe).stdout
    except Exception as exc:
        return [f"residue probe failed: {type(exc).__name__}: {exc}"]
    return [line.strip() for line in str(output).splitlines() if line.strip()]


def check_vendor_owner(adb: SubprocessAdbFileTransfer) -> dict[str, Any]:
    """Confirm that the vendor ``demo`` process owns AC107 after cleanup."""
    try:
        status = str(
            adb.shell(("cat", "/proc/asound/card1/pcm0c/sub0/status"), timeout_seconds=2.0).stdout
        )
        demo_output = str(adb.shell(("pidof", "demo"), allow_failure=True).stdout)
    except Exception as exc:
        return {"status": "ERROR", "diagnostic": f"owner probe failed: {exc}"}
    owner_match = re.search(r"^owner_pid\s*:\s*(\d+)\s*$", status, re.MULTILINE)
    owner_pid = int(owner_match.group(1)) if owner_match else None
    demo_pids = sorted(
        int(value) for value in re.findall(r"\b\d+\b", demo_output)
    )
    owner_is_demo = owner_pid is not None and owner_pid in demo_pids
    return {
        "status": "READY" if owner_is_demo and "state: RUNNING" in status else "FAILED",
        "state": "RUNNING" if "state: RUNNING" in status else "UNKNOWN",
        "owner_pid": owner_pid,
        "demo_pids": demo_pids,
        "owner_is_demo": owner_is_demo,
    }


def check_remote_state(adb: SubprocessAdbFileTransfer) -> dict[str, Any]:
    return {
        "remote_residuals": check_remote_residue(adb),
        "vendor_owner": check_vendor_owner(adb),
    }


def run_unified_environment_check(command: Sequence[str] | None) -> dict[str, Any]:
    """Run the deployment's complete perception environment gate."""
    if not command:
        return {
            "status": "NOT_RUN",
            "passed": False,
            "diagnostic": "hardware mode requires --environment-check-command",
        }
    try:
        completed = subprocess.run(
            list(command),
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=_ENVIRONMENT_CHECK_TIMEOUT_SECONDS,
        )
    except subprocess.TimeoutExpired as exc:
        return {
            "status": "ERROR",
            "passed": False,
            "command": list(command),
            "diagnostic": (
                "environment check timed out after "
                f"{_ENVIRONMENT_CHECK_TIMEOUT_SECONDS:g} seconds: {exc}"
            ),
        }
    except OSError as exc:
        return {
            "status": "ERROR",
            "passed": False,
            "command": list(command),
            "diagnostic": f"environment check could not start: {exc}",
        }
    output = completed.stdout.strip()
    passed = completed.returncode == 0 and "PERCEPTION ENVIRONMENT: PASS" in output
    return {
        "status": "PASS" if passed else "FAIL",
        "passed": passed,
        "command": list(command),
        "returncode": completed.returncode,
        "output": output[-4000:],
    }


def _host_pcm_snapshot() -> dict[str, list[str]]:
    paths: set[str] = set()
    for directory in (Path.cwd(), Path("/tmp")):
        if directory.is_dir():
            for suffix in ("*.pcm", "*.wav", "*.raw"):
                paths.update(str(path) for path in directory.glob(suffix))
    try:
        process = subprocess.run(
            ["pgrep", "-af", "[a]record"],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
        )
    except OSError as exc:
        return {
            "paths": sorted(paths),
            "arecord_processes": [],
            "errors": [f"host process probe failed: {exc}"],
        }
    errors = [] if process.returncode in (0, 1) else [
        f"host process probe exited with status {process.returncode}"
    ]
    return {
        "paths": sorted(paths),
        "arecord_processes": sorted(
            line.strip() for line in process.stdout.splitlines() if line.strip()
        ),
        "errors": errors,
    }


def _host_pcm_check(before: dict[str, list[str]], after: dict[str, list[str]]) -> dict[str, Any]:
    new_paths = sorted(set(after["paths"]) - set(before["paths"]))
    new_processes = sorted(
        set(after["arecord_processes"]) - set(before["arecord_processes"])
    )
    return {
        "passed": (
            not new_paths
            and not new_processes
            and not before.get("errors")
            and not after.get("errors")
        ),
        "before": before,
        "after": after,
        "new_paths": new_paths,
        "new_arecord_processes": new_processes,
        "errors": before.get("errors", []) + after.get("errors", []),
    }


def _path_fingerprint(path: str | Path) -> dict[str, Any]:
    selected = Path(path)
    if not selected.exists():
        return {"path": str(selected), "exists": False}
    digest = hashlib.sha256()
    files = [selected] if selected.is_file() else sorted(
        candidate for candidate in selected.rglob("*") if candidate.is_file()
    )
    for candidate in files:
        relative = candidate.relative_to(selected) if selected.is_dir() else candidate.name
        digest.update(str(relative).encode())
        digest.update(b"\0")
        with candidate.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
    return {
        "path": str(selected),
        "exists": True,
        "files": len(files),
        "sha256": digest.hexdigest(),
    }


def _deployment_assets(
    *,
    model_dir: str | Path,
    config_file: str | Path,
    helper_path: Path,
) -> dict[str, dict[str, Any]]:
    return {
        "model": _path_fingerprint(model_dir),
        "config": _path_fingerprint(config_file),
        "helper": _path_fingerprint(helper_path),
        "acceptance": _path_fingerprint(Path(__file__)),
    }


def _issue37_matrix_diagnostic(payload: dict[str, Any]) -> str | None:
    expected_tasks = minimal_field_matrix().tasks
    cycles = payload.get("cycles")
    if not isinstance(cycles, list) or len(cycles) != len(expected_tasks):
        return "issue37 normal task matrix is incomplete"
    if payload.get("cycles_completed") != len(expected_tasks) or payload.get(
        "cycles_aborted"
    ) is not False:
        return "issue37 normal task matrix did not complete cleanly"
    for index, (cycle, task) in enumerate(zip(cycles, expected_tasks), start=1):
        expected_evidence = [
            "PASSED" if window.accepted else "NOT_PASSED"
            for window in task.windows
        ]
        expected_event = "AUTHORIZED" if task.windows[-1].accepted else "UNAUTHORIZED"
        cleanup = cycle.get("cleanup") if isinstance(cycle, dict) else None
        evidence = cycle.get("evidence") if isinstance(cycle, dict) else None
        events = cycle.get("events") if isinstance(cycle, dict) else None
        if (
            not isinstance(cycle, dict)
            or cycle.get("passed") is not True
            or cycle.get("hardware_sessions_started") != 1
            or cycle.get("expected_evidence") != expected_evidence
            or cycle.get("expected_event") != expected_event
            or not isinstance(cleanup, dict)
            or cleanup.get("complete") is not True
            or cleanup.get("late_evidence") is not False
            or cleanup.get("remote_residuals") not in ([], None)
            or (
                cleanup.get("vendor_owner") is not None
                and (
                    not isinstance(cleanup.get("vendor_owner"), dict)
                    or cleanup["vendor_owner"].get("status") != "READY"
                )
            )
            or evidence != expected_evidence
            or events != [expected_event]
        ):
            return f"issue37 normal task {index} did not pass the required shape"
    matrix = payload.get("failure_matrix")
    if not isinstance(matrix, list):
        return "issue37 failure matrix is missing"
    scenarios = {
        scenario.get("name"): scenario
        for scenario in matrix
        if isinstance(scenario, dict) and isinstance(scenario.get("name"), str)
    }
    if len(scenarios) != len(matrix) or set(scenarios) != set(_FAILURE_SCENARIOS):
        return "issue37 failure matrix does not cover every required scenario"
    for name in _FAILURE_SCENARIOS:
        scenario = scenarios[name]
        cleanup = scenario.get("cleanup")
        max_active_sessions = scenario.get("max_active_sessions")
        if (
            scenario.get("passed") is not True
            or scenario.get("late_evidence") is not False
            or not isinstance(max_active_sessions, int)
            or max_active_sessions < 0
            or max_active_sessions > 1
            or not isinstance(cleanup, dict)
            or cleanup.get("complete") is not True
        ):
            return f"issue37 failure scenario {name} did not pass cleanly"
    return None


def _redact_field_report(report: dict[str, Any]) -> None:
    """Keep field reports to outcomes, task linkage, cleanup, and fingerprints."""
    automatic_gate = report.get("automatic_gate", {})
    if isinstance(automatic_gate, dict):
        automatic_gate = {
            key: automatic_gate[key]
            for key in ("report", "passed", "assets")
            if key in automatic_gate
        }
    deployment_assets = report.get("deployment_assets", {})
    passed = report.get("passed") is True
    cycles_aborted = report.get("cycles_aborted", False)
    cycles = []
    for cycle in report.get("cycles", []):
        cleanup = cycle.get("cleanup", {})
        cycles.append(
            {
                "index": cycle.get("index"),
                "state_seq": cycle.get("state_seq"),
                "target_id": cycle.get("target_id"),
                "evidence": cycle.get("evidence", []),
                "expected_evidence": cycle.get("expected_evidence", []),
                "events": cycle.get("events", []),
                "expected_event": cycle.get("expected_event"),
                "cleanup": {
                    "complete": cleanup.get("complete") is True,
                    "late_evidence": cleanup.get("late_evidence") is True,
                },
                "passed": cycle.get("passed") is True,
            }
        )
    readiness = report.get("readiness", {})
    report.clear()
    report.update(
        {
            "mode": "field",
            "automatic_gate": automatic_gate,
            "field_matrix": [
                {
                    "name": "first_window_pass",
                    "expected_evidence": ["PASSED"],
                    "expected_event": "AUTHORIZED",
                },
                {
                    "name": "second_window_pass",
                    "expected_evidence": ["NOT_PASSED", "PASSED"],
                    "expected_event": "AUTHORIZED",
                },
                {
                    "name": "two_windows_not_passed",
                    "expected_evidence": ["NOT_PASSED", "NOT_PASSED"],
                    "expected_event": "UNAUTHORIZED",
                },
            ],
            "failure_matrix_scope": "verified separately by issue37 hardware acceptance",
            "readiness": {"status": readiness.get("status", "UNKNOWN")},
            "cycles": cycles,
            "cycles_completed": len(cycles),
            "cycles_aborted": cycles_aborted,
            "passed": passed,
            "deployment_assets": deployment_assets,
        }
    )


def _field_automatic_gate(
    report_path: str | Path,
    *,
    model_dir: str | Path,
    config_file: str | Path,
    helper_path: Path,
) -> dict[str, Any]:
    """Validate that the same deployment already passed issue #37."""
    path = Path(report_path)
    result: dict[str, Any] = {"report": str(path), "passed": False}
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        result["diagnostic"] = f"cannot read issue37 automated report: {exc}"
        return result
    if not isinstance(payload, dict):
        result["diagnostic"] = "issue37 automated report root must be an object"
        return result
    if payload.get("issue") != 37 or payload.get("mode") != "hardware":
        result["diagnostic"] = "automated report must be a hardware report for issue 37"
        return result
    if payload.get("passed") is not True:
        result["diagnostic"] = "issue37 automated report did not pass"
        return result
    expected_assets = payload.get("deployment_assets")
    if not isinstance(expected_assets, dict):
        result["diagnostic"] = "issue37 automated report lacks deployment assets"
        return result
    current_assets = _deployment_assets(
        model_dir=model_dir,
        config_file=config_file,
        helper_path=helper_path,
    )
    mismatches = []
    for name, current in current_assets.items():
        expected = expected_assets.get(name)
        if not isinstance(expected, dict) or any(
            expected.get(field) != current.get(field)
            for field in ("exists", "files", "sha256")
        ):
            mismatches.append(name)
    if mismatches:
        result["diagnostic"] = (
            "issue37 automated report assets do not match this field deployment: "
            + ", ".join(mismatches)
        )
        return result
    matrix_diagnostic = _issue37_matrix_diagnostic(payload)
    if matrix_diagnostic is not None:
        result["diagnostic"] = matrix_diagnostic
        return result
    result["passed"] = True
    result["diagnostic"] = "matching issue37 hardware acceptance passed"
    result["assets"] = current_assets
    return result


def _runtime_install_check() -> dict[str, Any]:
    """Reject hardware acceptance when this module was imported from source."""
    module_path = Path(__file__).resolve()
    marker = ("src", "perception", "dog_patrol_perception_voice")
    parts = module_path.parts
    source_import = any(
        parts[index : index + len(marker)] == marker
        for index in range(len(parts) - len(marker) + 1)
    )
    return {
        "passed": not source_import,
        "module": str(module_path),
        "diagnostic": (
            "installed package runtime"
            if not source_import
            else "voice acceptance was imported from the source tree"
        ),
    }


@dataclass(frozen=True)
class _PreparedHardwareAcceptance:
    config: VoiceConfig
    helper_path: Path
    adb: SubprocessAdbFileTransfer
    preflight: VoicePreflight
    readiness: dict[str, str]
    host_before: dict[str, list[str]]
    task_timeout_seconds: float


def _hardware_fixture_provenance_diagnostic(
    provenance: dict[str, str] | None,
) -> str | None:
    values = provenance or {}
    missing = _REQUIRED_HARDWARE_PROVENANCE - set(values)
    invalid = []
    if "source_commit" in values and not re.fullmatch(
        r"[0-9a-fA-F]{40}", values["source_commit"]
    ):
        invalid.append("source_commit must be a 40-digit hexadecimal commit")
    if "source_matrix" in values and not values["source_matrix"].strip():
        invalid.append("source_matrix must not be empty")
    if "task_manifest_sha256" in values and not re.fullmatch(
        r"[0-9a-fA-F]{64}", values["task_manifest_sha256"]
    ):
        invalid.append("task_manifest_sha256 must be a 64-digit hexadecimal SHA-256")
    diagnostics = []
    if missing:
        diagnostics.append(
            "hardware fixture provenance is incomplete: " + ", ".join(sorted(missing))
        )
    diagnostics.extend(invalid)
    return "; ".join(diagnostics) if diagnostics else None


def _block_hardware_acceptance(
    report: dict[str, Any],
    host_before: dict[str, list[str]],
    diagnostic: str,
) -> None:
    report["readiness"] = {"status": "BLOCKED", "diagnostic": diagnostic}
    report["cycles"] = []
    report["failure_matrix"] = []
    report["passed"] = False
    report["host_pcm_capture"] = _host_pcm_check(host_before, _host_pcm_snapshot())


def _prepare_hardware_acceptance(
    *,
    report: dict[str, Any],
    model_dir: str | Path,
    config_file: str | Path,
    helper_path: str | Path | None,
    environment_check_command: Sequence[str] | None,
    fixture_provenance: dict[str, str] | None = None,
) -> _PreparedHardwareAcceptance | None:
    runtime_check = _runtime_install_check()
    config = load_voice_config(config_file)
    selected_helper = Path(helper_path) if helper_path else default_helper_path()
    host_before = _host_pcm_snapshot()
    environment_check = run_unified_environment_check(environment_check_command)
    report.update(
        {
            "runtime_check": runtime_check,
            "environment_check": environment_check,
            "host_pcm_capture": False,
        }
    )
    if not runtime_check["passed"]:
        _block_hardware_acceptance(report, host_before, runtime_check["diagnostic"])
        return None
    if fixture_provenance is not None:
        diagnostic = _hardware_fixture_provenance_diagnostic(fixture_provenance)
        if diagnostic is not None:
            _block_hardware_acceptance(report, host_before, diagnostic)
            return None
    if not environment_check["passed"]:
        _block_hardware_acceptance(
            report,
            host_before,
            "unified perception environment check did not pass",
        )
        return None
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
    report["readiness"] = readiness
    if outcome.status != READY:
        report["cycles"] = []
        report["failure_matrix"] = []
        report["passed"] = False
        report["host_pcm_capture"] = _host_pcm_check(host_before, _host_pcm_snapshot())
        return None
    adb = SubprocessAdbFileTransfer(
        config.adb_executable,
        device_serial=config.device_serial,
        timeout_seconds=config.adb_timeout_seconds,
    )
    timeout_seconds = max(
        30.0,
        2.0 * config.response_timeout_seconds
        + config.start_timeout_seconds
        + config.restore_timeout_seconds
        + 5.0,
        3.0 * config.adb_timeout_seconds
        + config.restore_timeout_seconds
        + 10.0,
    )
    return _PreparedHardwareAcceptance(
        config=config,
        helper_path=selected_helper,
        adb=adb,
        preflight=preflight,
        readiness=readiness,
        host_before=host_before,
        task_timeout_seconds=timeout_seconds,
    )


def _finalize_hardware_report(
    report: dict[str, Any],
    prepared: _PreparedHardwareAcceptance,
    *,
    model_dir: str | Path,
    config_file: str | Path,
) -> None:
    report["host_pcm_capture"] = _host_pcm_check(
        prepared.host_before, _host_pcm_snapshot()
    )
    report["passed"] = report["passed"] and report["host_pcm_capture"]["passed"]
    report["deployment_assets"] = _deployment_assets(
        model_dir=model_dir,
        config_file=config_file,
        helper_path=prepared.helper_path,
    )


def run_hardware_acceptance(
    fixture: AcceptanceFixture,
    *,
    cycles: int,
    model_dir: str | Path,
    config_file: str | Path,
    helper_path: str | Path | None = None,
    environment_check_command: Sequence[str] | None = None,
) -> dict[str, Any]:
    """Run the complete clean-install gate, real R818 cycles, and fault matrix."""
    report: dict[str, Any] = {
        "mode": "hardware",
        "fixture_provenance": fixture.provenance,
        "failure_matrix_scope": "real_hardware_with_injected_faults",
    }
    prepared = _prepare_hardware_acceptance(
        report=report,
        model_dir=model_dir,
        config_file=config_file,
        helper_path=helper_path,
        environment_check_command=environment_check_command,
        fixture_provenance=fixture.provenance,
    )
    if prepared is None:
        return report
    factory = _HardwareAdapterFactory(
        prepared.config, prepared.helper_path, prepared.adb, fixture
    )
    cycle_report = _run_cycles(
        fixture,
        cycles=cycles,
        adapter_factory=factory,
        cleanup_checker=lambda: check_remote_state(prepared.adb),
        preflight=prepared.preflight.run,
        task_started_checker=factory.task_started,
        wait_for_idle=lambda: factory.active_sessions == 0,
        session_count=lambda: len(factory.records),
        task_timeout_seconds=prepared.task_timeout_seconds,
        cancel_timeout_seconds=prepared.config.restore_timeout_seconds + 5.0,
    )
    matrix = run_hardware_failure_matrix(
        prepared.config,
        helper_path=prepared.helper_path,
        adb=prepared.adb,
        timeout_seconds=prepared.task_timeout_seconds,
    )
    report["preflight"] = prepared.readiness
    report.update(cycle_report)
    report["failure_matrix"] = matrix
    report["passed"] = cycle_report["passed"] and all(
        scenario["passed"] for scenario in matrix
    )
    _finalize_hardware_report(
        report,
        prepared,
        model_dir=model_dir,
        config_file=config_file,
    )
    return report


def run_field_acceptance(
    *,
    model_dir: str | Path,
    config_file: str | Path,
    automated_report: str | Path,
    helper_path: str | Path | None = None,
    environment_check_command: Sequence[str] | None = None,
) -> dict[str, Any]:
    """Run the final live-speech matrix without storing speech or passphrases."""
    matrix = minimal_field_matrix()
    selected_helper = Path(helper_path) if helper_path else default_helper_path()
    automatic_gate = _field_automatic_gate(
        automated_report,
        model_dir=model_dir,
        config_file=config_file,
        helper_path=selected_helper,
    )
    report: dict[str, Any] = {
        "mode": "field",
        "automatic_gate": automatic_gate,
        "field_matrix": [
            {
                "name": "first_window_pass",
                "expected_evidence": ["PASSED"],
                "expected_event": "AUTHORIZED",
            },
            {
                "name": "second_window_pass",
                "expected_evidence": ["NOT_PASSED", "PASSED"],
                "expected_event": "AUTHORIZED",
            },
            {
                "name": "two_windows_not_passed",
                "expected_evidence": ["NOT_PASSED", "NOT_PASSED"],
                "expected_event": "UNAUTHORIZED",
            },
        ],
        "failure_matrix_scope": "verified separately by issue37 hardware acceptance",
    }
    if not automatic_gate["passed"]:
        host_before = _host_pcm_snapshot()
        _block_hardware_acceptance(
            report, host_before, str(automatic_gate["diagnostic"])
        )
        _redact_field_report(report)
        return report
    prepared = _prepare_hardware_acceptance(
        report=report,
        model_dir=model_dir,
        config_file=config_file,
        helper_path=selected_helper,
        environment_check_command=environment_check_command,
    )
    if prepared is None:
        _redact_field_report(report)
        return report
    factory = _LiveHardwareAdapterFactory(
        prepared.config,
        prepared.helper_path,
        prepared.adb,
        model_dir,
    )
    cycle_report = _run_cycles(
        matrix,
        cycles=len(matrix.tasks),
        adapter_factory=factory,
        cleanup_checker=lambda: check_remote_state(prepared.adb),
        preflight=prepared.preflight.run,
        task_started_checker=factory.task_started,
        wait_for_idle=lambda: factory.active_sessions == 0,
        session_count=lambda: len(factory.records),
        task_timeout_seconds=prepared.task_timeout_seconds,
        cancel_timeout_seconds=prepared.config.restore_timeout_seconds + 5.0,
    )
    report["preflight"] = prepared.readiness
    report.update(cycle_report)
    report["passed"] = cycle_report["passed"]
    _finalize_hardware_report(
        report,
        prepared,
        model_dir=model_dir,
        config_file=config_file,
    )
    _redact_field_report(report)
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
        description="Run dog_patrol voice deployment or final field acceptance."
    )
    parser.add_argument(
        "--mode", choices=("fixture", "hardware", "field"), default="fixture"
    )
    parser.add_argument(
        "--fixture",
        help="deployment-local task outcome JSON (required in fixture and hardware modes)",
    )
    parser.add_argument(
        "--cycles",
        type=int,
        default=_DEFAULT_ACCEPTANCE_CYCLES,
        help=(
            "representative normal task cycles; the fixture task count must match "
            f"(default: {_DEFAULT_ACCEPTANCE_CYCLES})"
        ),
    )
    parser.add_argument(
        "--model-dir", help="Vosk model directory (required in hardware and field modes)"
    )
    parser.add_argument("--config-file", default=str(_default_config_path()))
    parser.add_argument("--helper-path")
    parser.add_argument(
        "--environment-check-command",
        help="quoted command for the complete perception environment gate",
    )
    parser.add_argument(
        "--automated-report",
        help="passed issue37 hardware report required in field mode",
    )
    parser.add_argument("--report", type=Path, help="write a JSON report at this path")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    command_args = list(sys.argv[1:] if argv is None else argv)
    args = build_parser().parse_args(command_args)
    if args.cycles <= 0:
        print("acceptance cycles must be positive")
        return 2
    try:
        environment_check_command = (
            shlex.split(args.environment_check_command)
            if args.environment_check_command
            else None
        )
        if args.mode == "hardware":
            if args.automated_report:
                raise ValueError("--automated-report is accepted only in field mode")
            if not args.fixture:
                raise ValueError("--fixture is required in hardware mode")
            if not args.model_dir:
                raise ValueError("--model-dir is required in hardware mode")
            fixture = load_acceptance_fixture(args.fixture, expected_cycles=args.cycles)
            report = run_hardware_acceptance(
                fixture,
                cycles=args.cycles,
                model_dir=args.model_dir,
                config_file=args.config_file,
                helper_path=args.helper_path,
                environment_check_command=environment_check_command,
            )
        elif args.mode == "fixture":
            if args.automated_report:
                raise ValueError("--automated-report is accepted only in field mode")
            if not args.fixture:
                raise ValueError("--fixture is required in fixture mode")
            fixture = load_acceptance_fixture(args.fixture, expected_cycles=args.cycles)
            report = run_fixture_acceptance(fixture, cycles=args.cycles)
        else:
            if args.fixture:
                raise ValueError("--fixture is not accepted in field mode")
            if not args.model_dir:
                raise ValueError("--model-dir is required in field mode")
            if not args.automated_report:
                raise ValueError("--automated-report is required in field mode")
            if args.cycles != _DEFAULT_ACCEPTANCE_CYCLES:
                raise ValueError(
                    "field mode uses the fixed three-scenario minimal matrix"
                )
            report = run_field_acceptance(
                model_dir=args.model_dir,
                config_file=args.config_file,
                automated_report=args.automated_report,
                helper_path=args.helper_path,
                environment_check_command=environment_check_command,
            )
        report["issue"] = 38 if args.mode == "field" else 37
        report["started_at_utc"] = datetime.now(timezone.utc).isoformat()
        report["environment"] = {
            "architecture": platform.machine(),
            "python": sys.version.split()[0],
            "cycles_requested": args.cycles,
        }
        if args.fixture:
            report["environment"]["fixture"] = str(args.fixture)
            report["environment"]["fixture_fingerprint"] = _path_fingerprint(
                args.fixture
            )
        report["command"] = shlex.join(["perception_voice_acceptance", *command_args])
        if args.report:
            _write_report(args.report, report)
        print(json.dumps(report, ensure_ascii=False, sort_keys=True))
        return 0 if report["passed"] else 1
    except (OSError, ValueError, RuntimeError) as exc:
        print(f"voice acceptance failed before completion: {type(exc).__name__}: {exc}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
