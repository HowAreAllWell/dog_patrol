"""Asynchronous MissionState-driven voice authorization evidence provider."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, replace
from enum import Enum
import threading
import time
from pathlib import Path
from typing import Protocol

import rclpy
from ament_index_python.packages import get_package_share_directory
from dog_patrol_interfaces.msg import MissionState
from dog_patrol_perception_interfaces.msg import (
    AuthorizationCommand,
    AuthorizationEvidence,
    VoiceVerificationEvent,
)
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from .adapter import R818VoiceAdapter, VoiceTaskCancelled, VoiceTaskCleanupError
from .config import VoiceConfig, load_voice_config
from .result import VoiceWindowEvent, VoiceWindowResult

UINT32_MASK = (1 << 32) - 1
UINT32_HALF_RANGE = 1 << 31
VOICE_POLICY_PARAMETERS = (
    "response_timeout_seconds", "first_prompt", "retry_prompt", "passphrase"
)


def _is_newer_state_seq(candidate: int, current: int) -> bool:
    delta = (int(candidate) - int(current)) & UINT32_MASK
    return 0 < delta < UINT32_HALF_RANGE


class VoiceEvidenceResult(Enum):
    PASSED = "passed"
    NOT_PASSED = "not_passed"
    ERROR = "error"
    CANCELLED = "cancelled"


@dataclass(frozen=True)
class VoiceVerificationSession:
    observed_state_seq: int
    target_id: int


class VoiceVerificationStage(Enum):
    DUAL_FIRST = "dual_first"
    DUAL_SECOND = "dual_second"


@dataclass(frozen=True)
class VoiceVerificationRequest:
    session: VoiceVerificationSession
    stage: VoiceVerificationStage
    round_number: int = 0

    def __post_init__(self) -> None:
        if self.round_number == 0:
            object.__setattr__(
                self, "round_number", 1 if self.stage is VoiceVerificationStage.DUAL_FIRST else 2
            )


@dataclass(frozen=True)
class VoiceEvidence:
    session: VoiceVerificationSession
    stage: VoiceVerificationStage
    result: VoiceEvidenceResult
    detail: str = ""
    round_number: int = 0


class VoiceTask(Protocol):
    def __enter__(self) -> VoiceTask: ...

    def __exit__(
        self,
        exc_type: object,
        exc: BaseException | None,
        traceback: object,
    ) -> None: ...

    def respond(
        self,
        *,
        retry: bool = False,
        on_event: Callable[[VoiceWindowEvent], None] | None = None,
    ) -> VoiceWindowResult: ...

    def cancel(self) -> None: ...


class VoiceAdapter(Protocol):
    def task(self) -> VoiceTask: ...


@dataclass(frozen=True)
class _PendingEvidence:
    result: VoiceEvidenceResult
    detail: str


class _MissionCancelled(RuntimeError):
    pass


class VoiceEvidenceController:
    """Serialize hardware sessions while keeping MissionState callbacks short."""

    def __init__(
        self,
        adapter_factory: Callable[[], VoiceAdapter],
        publish: Callable[[VoiceEvidence], None],
        publish_window_event: Callable[[VoiceVerificationRequest, VoiceWindowEvent], None] | None = None,
    ) -> None:
        self._adapter_factory = adapter_factory
        self._publish = publish
        self._publish_window_event = publish_window_event
        self._condition = threading.Condition()
        self._mission_session: VoiceVerificationSession | None = None
        self._desired_request: VoiceVerificationRequest | None = None
        self._highest_round_requested = 0
        self._generation = 0
        self._completed_generation = 0
        self._active_task: VoiceTask | None = None
        self._active_cancel: threading.Event | None = None
        self._hardware_faulted = False
        self._stopping = False
        self._worker = threading.Thread(
            target=self._run,
            name="dog-patrol-voice-evidence",
            daemon=True,
        )
        self._worker.start()

    def observe_session(self, session: VoiceVerificationSession | None) -> None:
        """Track mission validity without starting voice work."""
        task_to_cancel: VoiceTask | None = None
        with self._condition:
            if session == self._mission_session:
                return
            self._mission_session = session
            self._desired_request = None
            self._highest_round_requested = 0
            self._generation += 1
            if self._active_cancel is not None:
                self._active_cancel.set()
            task_to_cancel = self._active_task
            self._condition.notify_all()
        self._request_task_cancel(task_to_cancel)

    def request(self, request: VoiceVerificationRequest | None) -> None:
        """Run one voice window for the latest commanded dual stage."""
        task_to_cancel: VoiceTask | None = None
        with self._condition:
            if request is not None and request.session != self._mission_session:
                return
            if (
                request is not None
                and request.round_number <= self._highest_round_requested
            ):
                return
            if request == self._desired_request:
                return
            if request is not None:
                self._highest_round_requested = request.round_number
            self._desired_request = request
            self._generation += 1
            if self._active_cancel is not None:
                self._active_cancel.set()
            task_to_cancel = self._active_task
            self._condition.notify_all()
        self._request_task_cancel(task_to_cancel)

    def stop(self) -> None:
        task_to_cancel: VoiceTask | None = None
        with self._condition:
            if self._stopping:
                return
            self._stopping = True
            if self._active_cancel is not None:
                self._active_cancel.set()
            task_to_cancel = self._active_task
            self._condition.notify_all()
        self._request_task_cancel(task_to_cancel)
        self._worker.join()

    def _run(self) -> None:
        while True:
            with self._condition:
                while (
                    not self._stopping
                    and not self._hardware_faulted
                    and (
                        self._desired_request is None
                        or self._completed_generation == self._generation
                    )
                ):
                    self._condition.wait()
                if self._stopping or self._hardware_faulted:
                    return
                request = self._desired_request
                generation = self._generation
                assert request is not None
                cancel_event = threading.Event()
                self._active_cancel = cancel_event

            healthy = self._execute(request, generation, cancel_event)

            with self._condition:
                if self._generation == generation:
                    self._completed_generation = generation
                if self._active_cancel is cancel_event:
                    self._active_cancel = None
                if not healthy:
                    self._hardware_faulted = True
                self._condition.notify_all()
                if self._hardware_faulted:
                    return

    def _execute(
        self,
        request: VoiceVerificationRequest,
        generation: int,
        cancel_event: threading.Event,
    ) -> bool:
        try:
            adapter = self._adapter_factory()
            task = adapter.task()
            self._register_task(task, request, generation, cancel_event)
            pending: _PendingEvidence | None
            with task as entered_task:
                active_task = entered_task if entered_task is not None else task
                self._register_task(active_task, request, generation, cancel_event)
                pending = self._run_window(request, generation, cancel_event, active_task)

            if self._request_cancelled(generation, request, cancel_event):
                self._publish_cancelled(request)
            elif pending is not None:
                self._publish_current(
                    request, generation, pending.result, pending.detail
                )
            return True
        except VoiceTaskCleanupError as exc:
            self._publish_error(request, exc)
            return False
        except VoiceTaskCancelled:
            if self._request_cancelled(generation, request, cancel_event):
                self._publish_cancelled(request)
            else:
                self._publish_current(
                    request,
                    generation,
                    VoiceEvidenceResult.CANCELLED,
                    "voice task was cancelled",
                )
            return True
        except _MissionCancelled:
            self._publish_cancelled(request)
            return True
        except Exception as exc:
            self._publish_error(request, exc)
            return True
        finally:
            with self._condition:
                self._active_task = None

    def _run_window(
        self,
        request: VoiceVerificationRequest,
        generation: int,
        cancel_event: threading.Event,
        task: VoiceTask,
    ) -> _PendingEvidence | None:
        self._raise_if_request_cancelled(generation, request, cancel_event)
        retry = request.round_number > 1
        if self._publish_window_event is None:
            result = task.respond(retry=retry)
        else:
            result = task.respond(
                retry=retry,
                on_event=lambda event: self._on_window_event(
                    request, generation, cancel_event, event
                ),
            )
        self._raise_if_request_cancelled(generation, request, cancel_event)
        round_number = request.round_number
        evidence_result = (
            VoiceEvidenceResult.PASSED
            if result.accepted
            else VoiceEvidenceResult.NOT_PASSED
        )
        return _PendingEvidence(
            evidence_result,
            f"voice response round {round_number} "
            f"{'passed' if result.accepted else 'not passed'}; "
            f"decision_seconds={result.decision_time_seconds:.3f}",
        )

    def _on_window_event(
        self,
        request: VoiceVerificationRequest,
        generation: int,
        cancel_event: threading.Event,
        event: VoiceWindowEvent,
    ) -> None:
        with self._condition:
            if self._stopping or self._request_cancelled_locked(
                request, generation, self._generation, self._desired_request, cancel_event
            ):
                return
            if self._publish_window_event is not None:
                self._publish_window_event(request, event)

    def _register_task(
        self,
        task: VoiceTask,
        request: VoiceVerificationRequest,
        generation: int,
        cancel_event: threading.Event,
    ) -> None:
        with self._condition:
            self._active_task = task
            should_cancel = self._stopping or self._request_cancelled_locked(
                request,
                generation,
                self._generation,
                self._desired_request,
                cancel_event,
            )
        if should_cancel:
            self._request_task_cancel(task)

    def _raise_if_request_cancelled(
        self,
        generation: int,
        request: VoiceVerificationRequest,
        cancel_event: threading.Event,
    ) -> None:
        if self._request_cancelled(generation, request, cancel_event):
            raise _MissionCancelled

    def _request_cancelled(
        self,
        generation: int,
        request: VoiceVerificationRequest,
        cancel_event: threading.Event,
    ) -> bool:
        with self._condition:
            if self._stopping:
                return False
            return self._request_cancelled_locked(
                request,
                generation,
                self._generation,
                self._desired_request,
                cancel_event,
            )

    @staticmethod
    def _request_cancelled_locked(
        request: VoiceVerificationRequest,
        generation: int,
        current_generation: int,
        desired_request: VoiceVerificationRequest | None,
        cancel_event: threading.Event,
    ) -> bool:
        return (
            cancel_event.is_set()
            or desired_request != request
            or current_generation != generation
        )

    def _publish_current(
        self,
        request: VoiceVerificationRequest,
        generation: int,
        result: VoiceEvidenceResult,
        detail: str,
    ) -> bool:
        with self._condition:
            if self._stopping or self._generation != generation:
                return False
            if self._desired_request != request:
                return False
            self._publish(VoiceEvidence(
                request.session, request.stage, result, detail, request.round_number
            ))
            return True

    def _publish_cancelled(self, request: VoiceVerificationRequest) -> None:
        with self._condition:
            if self._stopping:
                return
            self._publish(
                VoiceEvidence(
                    request.session,
                    request.stage,
                    VoiceEvidenceResult.CANCELLED,
                    "voice task cancelled",
                    request.round_number,
                )
            )

    def _publish_error(
        self,
        request: VoiceVerificationRequest,
        exc: Exception,
    ) -> None:
        with self._condition:
            if self._stopping:
                return
            self._publish(
                VoiceEvidence(
                    request.session,
                    request.stage,
                    VoiceEvidenceResult.ERROR,
                    _exception_detail(exc),
                    request.round_number,
                )
            )

    @staticmethod
    def _request_task_cancel(task: VoiceTask | None) -> None:
        if task is None:
            return
        try:
            task.cancel()
        except Exception:
            # The worker still owns cleanup; the generation gate suppresses its result.
            pass


class VoiceEvidenceProviderNode(Node):
    """ROS adapter for the asynchronous voice evidence controller."""

    _ROS_RESULT_CODES = {
        VoiceEvidenceResult.PASSED: AuthorizationEvidence.PASSED,
        VoiceEvidenceResult.NOT_PASSED: AuthorizationEvidence.NOT_PASSED,
        VoiceEvidenceResult.ERROR: AuthorizationEvidence.ERROR,
        VoiceEvidenceResult.CANCELLED: AuthorizationEvidence.CANCELLED,
    }

    def __init__(
        self,
        *,
        parameter_overrides: list[Parameter] | None = None,
        adapter_factory: Callable[[], VoiceAdapter] | None = None,
    ) -> None:
        super().__init__("perception_voice_provider", parameter_overrides=parameter_overrides)
        self.declare_parameter("mission_state_topic", "/mission/state")
        self.declare_parameter(
            "authorization_evidence_topic", "/perception/authorization_evidence"
        )
        self.declare_parameter(
            "authorization_command_topic", "/perception/authorization_command"
        )
        self.declare_parameter("voice_verification_event_topic", "/perception/voice_verification_event")
        self.declare_parameter("provider", "voice")
        self.declare_parameter("model_dir", "")
        self.declare_parameter("config_file", _default_config_file())
        self.declare_parameter("helper_path", "")
        self.declare_parameter("max_detail_length", 256)
        self.declare_parameter("state_timeout", 0.75)

        config_file = str(self.get_parameter("config_file").value).strip()
        base_config = load_voice_config(config_file) if config_file else VoiceConfig()
        for name in VOICE_POLICY_PARAMETERS:
            self.declare_parameter(name, getattr(base_config, name))
        self._pending_config = replace(base_config, **{
            name: self.get_parameter(name).value for name in VOICE_POLICY_PARAMETERS
        })
        self._session_config = self._pending_config
        self._config_session: VoiceVerificationSession | None = None

        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        evidence_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._provider_name = str(self.get_parameter("provider").value).strip()
        if not self._provider_name:
            raise ValueError("provider must not be empty")
        self._max_detail_length = max(
            32, int(self.get_parameter("max_detail_length").value)
        )
        self._adapter_lock = threading.Lock()
        self._policy_lock = threading.Lock()
        self._production_adapter: VoiceAdapter | None = None
        self._adapter_config: VoiceConfig | None = None
        self.add_on_set_parameters_callback(self._on_policy_parameters)
        self._mission_state_seq: int | None = None
        self._mission_session: VoiceVerificationSession | None = None
        self._mission_state_signature = None
        self._mission_state_conflicted = False
        self._state_timeout = max(
            0.1, float(self.get_parameter("state_timeout").value)
        )
        self._last_state_monotonic = None
        self._state_timed_out = False
        self._evidence_pub = self.create_publisher(
            AuthorizationEvidence,
            str(self.get_parameter("authorization_evidence_topic").value),
            evidence_qos,
        )
        self._window_event_pub = self.create_publisher(
            VoiceVerificationEvent,
            str(self.get_parameter("voice_verification_event_topic").value),
            evidence_qos,
        )
        self._state_sub = self.create_subscription(
            MissionState,
            str(self.get_parameter("mission_state_topic").value),
            self._on_mission,
            state_qos,
        )
        self._command_sub = self.create_subscription(
            AuthorizationCommand,
            str(self.get_parameter("authorization_command_topic").value),
            self._on_command,
            evidence_qos,
        )
        self._controller = VoiceEvidenceController(
            adapter_factory or self._build_adapter,
            self._publish_evidence,
            self._publish_window_event,
        )
        self._state_watchdog = self.create_timer(
            min(0.1, self._state_timeout / 2.0), self._check_state_timeout
        )

    def destroy_node(self):
        self._controller.stop()
        return super().destroy_node()

    def _on_policy_parameters(self, parameters: list[Parameter]) -> SetParametersResult:
        updates = {
            parameter.name: parameter.value for parameter in parameters
            if parameter.name in VOICE_POLICY_PARAMETERS
        }
        try:
            with self._policy_lock:
                self._pending_config = replace(self._pending_config, **updates)
        except (TypeError, ValueError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        return SetParametersResult(successful=True)

    def _observe_session(self, session: VoiceVerificationSession | None) -> None:
        # Snapshot business settings once; a platform update cannot change an answer mid-session.
        if session is not None and session != self._config_session:
            with self._policy_lock:
                self._session_config = self._pending_config
                self._config_session = session
        self._controller.observe_session(session)

    def _on_mission(self, msg: MissionState) -> None:
        state_seq = int(msg.state_seq) & UINT32_MASK
        if state_seq == 0:
            self._state_timed_out = True
            self._observe_session(None)
            return
        session = _session_from_mission(msg)
        signature = (int(msg.state), int(msg.target_id), int(msg.handled_target_id))
        self._last_state_monotonic = time.monotonic()
        if self._mission_state_seq is not None:
            if state_seq == self._mission_state_seq:
                if self._mission_state_conflicted:
                    return
                if signature != self._mission_state_signature:
                    self._mission_state_conflicted = True
                    self._observe_session(None)
                    self._state_timed_out = False
                    return
                if self._state_timed_out:
                    self._state_timed_out = False
                    self._observe_session(session)
                return
            if not _is_newer_state_seq(state_seq, self._mission_state_seq):
                return
        self._mission_state_seq = state_seq
        self._mission_session = session
        self._mission_state_signature = signature
        self._mission_state_conflicted = False
        self._state_timed_out = False
        self._observe_session(session)

    def _check_state_timeout(self) -> None:
        if (
            self._last_state_monotonic is None
            or self._state_timed_out
            or time.monotonic() - self._last_state_monotonic <= self._state_timeout
        ):
            return
        self._state_timed_out = True
        self._observe_session(None)
        self.get_logger().warning(
            "mission state heartbeat timed out; voice authorization session cancelled"
        )

    def _on_command(self, msg: AuthorizationCommand) -> None:
        session = VoiceVerificationSession(
            int(msg.observed_state_seq), int(msg.target_id)
        )
        if int(msg.stage) == AuthorizationCommand.CANCEL:
            if session == self._mission_session:
                self._controller.request(None)
            return
        stage = {
            AuthorizationCommand.DUAL_FIRST: VoiceVerificationStage.DUAL_FIRST,
            AuthorizationCommand.DUAL_SECOND: VoiceVerificationStage.DUAL_SECOND,
        }.get(int(msg.stage))
        if stage is not None:
            round_number = int(msg.round_number)
            if round_number and (
                (stage is VoiceVerificationStage.DUAL_FIRST and round_number != 1)
                or (stage is VoiceVerificationStage.DUAL_SECOND and round_number < 2)
            ):
                return
            self._controller.request(VoiceVerificationRequest(session, stage, round_number))

    def _publish_evidence(self, evidence: VoiceEvidence) -> None:
        msg = AuthorizationEvidence()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.observed_state_seq = evidence.session.observed_state_seq
        msg.target_id = evidence.session.target_id
        msg.round_number = evidence.round_number
        msg.stage = {
            VoiceVerificationStage.DUAL_FIRST: AuthorizationEvidence.DUAL_FIRST,
            VoiceVerificationStage.DUAL_SECOND: AuthorizationEvidence.DUAL_SECOND,
        }[evidence.stage]
        msg.result = self._ROS_RESULT_CODES[evidence.result]
        msg.provider = self._provider_name
        msg.detail = evidence.detail[: self._max_detail_length]
        self._evidence_pub.publish(msg)

    def _publish_window_event(
        self, request: VoiceVerificationRequest, event: VoiceWindowEvent
    ) -> None:
        msg = VoiceVerificationEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.observed_state_seq = request.session.observed_state_seq
        msg.target_id = request.session.target_id
        msg.round_number = request.round_number
        msg.event = {
            VoiceWindowEvent.WAIT_STARTED: VoiceVerificationEvent.WAIT_STARTED,
            VoiceWindowEvent.PASSED: VoiceVerificationEvent.PASSED,
            VoiceWindowEvent.NOT_PASSED: VoiceVerificationEvent.NOT_PASSED,
        }[event]
        self._window_event_pub.publish(msg)

    def _build_adapter(self) -> VoiceAdapter:
        with self._policy_lock:
            config = self._session_config
        with self._adapter_lock:
            if self._production_adapter is not None and self._adapter_config == config:
                return self._production_adapter
            model_dir = str(self.get_parameter("model_dir").value).strip()
            if not model_dir:
                raise ValueError("model_dir parameter must be set")
            helper_path = str(self.get_parameter("helper_path").value).strip()
            self._production_adapter = R818VoiceAdapter.from_model_dir(
                model_dir,
                config=config,
                helper_path=helper_path or None,
            )
            self._adapter_config = config
            return self._production_adapter


def _session_from_mission(msg: MissionState) -> VoiceVerificationSession | None:
    if (
        int(msg.state_seq) <= 0
        or int(msg.state_seq) > UINT32_MASK
        or int(msg.state) != MissionState.VERIFY_IDENTITY
        or int(msg.target_id) <= 0
    ):
        return None
    return VoiceVerificationSession(int(msg.state_seq) & UINT32_MASK, int(msg.target_id))


def _default_config_file() -> str:
    try:
        share_dir = Path(get_package_share_directory("dog_patrol_perception_voice"))
    except Exception:
        return "/__dog_patrol_voice_install_missing__/config/voice_verification.yaml"
    return str(share_dir / "config" / "voice_verification.yaml")


def _exception_detail(exc: Exception) -> str:
    detail = str(exc).strip()
    if detail:
        return f"{type(exc).__name__}: {detail}"
    return type(exc).__name__


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VoiceEvidenceProviderNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
