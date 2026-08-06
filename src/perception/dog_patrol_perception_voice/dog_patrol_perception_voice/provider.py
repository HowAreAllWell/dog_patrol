"""Asynchronous MissionState-driven voice authorization evidence provider."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum
import threading
from pathlib import Path
from typing import Protocol

import rclpy
from ament_index_python.packages import get_package_share_directory
from dog_patrol_interfaces.msg import MissionState
from dog_patrol_perception_interfaces.msg import AuthorizationEvidence
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from .adapter import R818VoiceAdapter, VoiceTaskCancelled, VoiceTaskCleanupError
from .config import VoiceConfig, load_voice_config
from .result import VoiceWindowResult


class VoiceEvidenceResult(Enum):
    PASSED = "passed"
    NOT_PASSED = "not_passed"
    ERROR = "error"
    CANCELLED = "cancelled"


@dataclass(frozen=True)
class VoiceVerificationSession:
    observed_state_seq: int
    target_id: int


@dataclass(frozen=True)
class VoiceEvidence:
    session: VoiceVerificationSession
    result: VoiceEvidenceResult
    detail: str = ""


class VoiceTask(Protocol):
    def __enter__(self) -> VoiceTask: ...

    def __exit__(
        self,
        exc_type: object,
        exc: BaseException | None,
        traceback: object,
    ) -> None: ...

    def respond(self) -> VoiceWindowResult: ...

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
    ) -> None:
        self._adapter_factory = adapter_factory
        self._publish = publish
        self._condition = threading.Condition()
        self._desired_session: VoiceVerificationSession | None = None
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

    def observe(self, session: VoiceVerificationSession | None) -> None:
        """Select the latest mission session without waiting for recognition."""
        task_to_cancel: VoiceTask | None = None
        with self._condition:
            if session == self._desired_session:
                return
            self._desired_session = session
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
                        self._desired_session is None
                        or self._completed_generation == self._generation
                    )
                ):
                    self._condition.wait()
                if self._stopping or self._hardware_faulted:
                    return
                session = self._desired_session
                generation = self._generation
                assert session is not None
                cancel_event = threading.Event()
                self._active_cancel = cancel_event

            healthy = self._execute(session, generation, cancel_event)

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
        session: VoiceVerificationSession,
        generation: int,
        cancel_event: threading.Event,
    ) -> bool:
        try:
            adapter = self._adapter_factory()
            task = adapter.task()
            self._register_task(task, session, generation, cancel_event)
            pending: _PendingEvidence | None
            with task as entered_task:
                active_task = entered_task if entered_task is not None else task
                self._register_task(active_task, session, generation, cancel_event)
                pending = self._run_windows(
                    session, generation, cancel_event, active_task
                )

            if self._mission_cancelled(generation, session, cancel_event):
                self._publish_cancelled(session)
            elif pending is not None:
                self._publish_current(
                    session, generation, pending.result, pending.detail
                )
            return True
        except VoiceTaskCleanupError as exc:
            self._publish_error(session, exc)
            return False
        except VoiceTaskCancelled:
            if self._mission_cancelled(generation, session, cancel_event):
                self._publish_cancelled(session)
            else:
                self._publish_current(
                    session,
                    generation,
                    VoiceEvidenceResult.CANCELLED,
                    "voice task was cancelled",
                )
            return True
        except _MissionCancelled:
            self._publish_cancelled(session)
            return True
        except Exception as exc:
            self._publish_error(session, exc)
            return True
        finally:
            with self._condition:
                self._active_task = None

    def _run_windows(
        self,
        session: VoiceVerificationSession,
        generation: int,
        cancel_event: threading.Event,
        task: VoiceTask,
    ) -> _PendingEvidence | None:
        for attempt_number in (1, 2):
            self._raise_if_mission_cancelled(generation, session, cancel_event)
            result = task.respond()
            self._raise_if_mission_cancelled(generation, session, cancel_event)
            if result.accepted:
                return _PendingEvidence(
                    VoiceEvidenceResult.PASSED,
                    f"voice response window {attempt_number} passed",
                )

            detail = f"voice response window {attempt_number} not passed"
            if attempt_number == 1:
                if not self._publish_current(
                    session, generation, VoiceEvidenceResult.NOT_PASSED, detail
                ):
                    raise _MissionCancelled
                continue
            return _PendingEvidence(VoiceEvidenceResult.NOT_PASSED, detail)
        raise AssertionError("voice provider must execute at most two windows")

    def _register_task(
        self,
        task: VoiceTask,
        session: VoiceVerificationSession,
        generation: int,
        cancel_event: threading.Event,
    ) -> None:
        with self._condition:
            self._active_task = task
            should_cancel = self._stopping or self._mission_cancelled_locked(
                session,
                generation,
                self._generation,
                self._desired_session,
                cancel_event,
            )
        if should_cancel:
            self._request_task_cancel(task)

    def _raise_if_mission_cancelled(
        self,
        generation: int,
        session: VoiceVerificationSession,
        cancel_event: threading.Event,
    ) -> None:
        if self._mission_cancelled(generation, session, cancel_event):
            raise _MissionCancelled

    def _mission_cancelled(
        self,
        generation: int,
        session: VoiceVerificationSession,
        cancel_event: threading.Event,
    ) -> bool:
        with self._condition:
            if self._stopping:
                return False
            return self._mission_cancelled_locked(
                session,
                generation,
                self._generation,
                self._desired_session,
                cancel_event,
            )

    @staticmethod
    def _mission_cancelled_locked(
        session: VoiceVerificationSession,
        generation: int,
        current_generation: int,
        desired_session: VoiceVerificationSession | None,
        cancel_event: threading.Event,
    ) -> bool:
        return (
            cancel_event.is_set()
            or desired_session != session
            or current_generation != generation
        )

    def _publish_current(
        self,
        session: VoiceVerificationSession,
        generation: int,
        result: VoiceEvidenceResult,
        detail: str,
    ) -> bool:
        with self._condition:
            if self._stopping or self._generation != generation:
                return False
            if self._desired_session != session:
                return False
            self._publish(VoiceEvidence(session, result, detail))
            return True

    def _publish_cancelled(self, session: VoiceVerificationSession) -> None:
        with self._condition:
            if self._stopping:
                return
            self._publish(
                VoiceEvidence(session, VoiceEvidenceResult.CANCELLED, "voice task cancelled")
            )

    def _publish_error(
        self,
        session: VoiceVerificationSession,
        exc: Exception,
    ) -> None:
        with self._condition:
            if self._stopping:
                return
            self._publish(
                VoiceEvidence(session, VoiceEvidenceResult.ERROR, _exception_detail(exc))
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
        self.declare_parameter("provider", "voice")
        self.declare_parameter("model_dir", "")
        self.declare_parameter("config_file", _default_config_file())
        self.declare_parameter("helper_path", "")
        self.declare_parameter("max_detail_length", 256)

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
        self._production_adapter: VoiceAdapter | None = None
        self._evidence_pub = self.create_publisher(
            AuthorizationEvidence,
            str(self.get_parameter("authorization_evidence_topic").value),
            evidence_qos,
        )
        self._state_sub = self.create_subscription(
            MissionState,
            str(self.get_parameter("mission_state_topic").value),
            self._on_mission,
            state_qos,
        )
        self._controller = VoiceEvidenceController(
            adapter_factory or self._build_adapter,
            self._publish_evidence,
        )

    def destroy_node(self):
        self._controller.stop()
        return super().destroy_node()

    def _on_mission(self, msg: MissionState) -> None:
        self._controller.observe(_session_from_mission(msg))

    def _publish_evidence(self, evidence: VoiceEvidence) -> None:
        msg = AuthorizationEvidence()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.observed_state_seq = evidence.session.observed_state_seq
        msg.target_id = evidence.session.target_id
        msg.result = self._ROS_RESULT_CODES[evidence.result]
        msg.provider = self._provider_name
        msg.detail = evidence.detail[: self._max_detail_length]
        self._evidence_pub.publish(msg)

    def _build_adapter(self) -> VoiceAdapter:
        with self._adapter_lock:
            if self._production_adapter is not None:
                return self._production_adapter
            model_dir = str(self.get_parameter("model_dir").value).strip()
            if not model_dir:
                raise ValueError("model_dir parameter must be set")
            config_file = str(self.get_parameter("config_file").value).strip()
            config = (
                load_voice_config(config_file) if config_file else VoiceConfig()
            )
            helper_path = str(self.get_parameter("helper_path").value).strip()
            self._production_adapter = R818VoiceAdapter.from_model_dir(
                model_dir,
                config=config,
                helper_path=helper_path or None,
            )
            return self._production_adapter


def _session_from_mission(msg: MissionState) -> VoiceVerificationSession | None:
    if (
        int(msg.state) != MissionState.VERIFY_IDENTITY
        or bool(msg.blocked)
        or int(msg.target_id) <= 0
    ):
        return None
    return VoiceVerificationSession(int(msg.state_seq), int(msg.target_id))


def _default_config_file() -> str:
    try:
        share_dir = Path(get_package_share_directory("dog_patrol_perception_voice"))
    except Exception:
        return "/__dog_patrol_voice_install_missing__/config/voice.yaml"
    return str(share_dir / "config" / "voice.yaml")


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
