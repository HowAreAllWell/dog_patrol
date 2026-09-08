"""MissionState-driven face authorization evidence provider.

The provider consumes ``TrackedTargetImage`` crops published by tracking and
publishes ``AuthorizationEvidence``. It follows the same session/generation
gating as the voice provider: only an active ``VERIFY_IDENTITY`` mission
whose ``target_id`` matches the crop is processed, and results are bound to
``state_seq + target_id``. Late or stale worker results are suppressed.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum
import threading
import time

from dog_patrol_interfaces.msg import MissionState
from dog_patrol_perception_interfaces.msg import (
    AuthorizationCommand,
    AuthorizationEvidence,
    FaceOverlay,
)
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from .config import FaceConfig, load_face_config
from .crop import DecodedCrop, decode_crop
from .detector import TRTInference, ensure_cuda_context
from .recognition import RecModel
from .verifier import FaceVerifier, ProductionFaceVerifier
from .whitelist import load_whitelist_npy


class FaceEvidenceResult(Enum):
    PASSED = "passed"
    NOT_PASSED = "not_passed"
    ERROR = "error"
    CANCELLED = "cancelled"


@dataclass(frozen=True)
class FaceVerificationSession:
    observed_state_seq: int
    target_id: int


class FaceVerificationStage(Enum):
    INITIAL_FACE = "initial_face"
    DUAL_FIRST = "dual_first"
    DUAL_SECOND = "dual_second"


@dataclass(frozen=True)
class FaceVerificationRequest:
    session: FaceVerificationSession
    stage: FaceVerificationStage


@dataclass(frozen=True)
class FaceEvidence:
    session: FaceVerificationSession
    stage: FaceVerificationStage
    result: FaceEvidenceResult
    detail: str = ""
    # Face bounding box in source-image coordinates: (x1, y1, x2, y2).
    # None when no face was detected in the crop.
    box: tuple[int, int, int, int] | None = None


class _MissionCancelled(RuntimeError):
    pass


class _SessionExpired(RuntimeError):
    pass


def _to_source_box(
    crop_box: tuple[int, int, int, int] | None, crop: DecodedCrop
) -> tuple[int, int, int, int] | None:
    """Map a crop-image face box into the source (full frame) coordinates."""
    if crop_box is None:
        return None
    x1, y1, x2, y2 = crop_box
    return (
        crop.bbox_x + x1,
        crop.bbox_y + y1,
        crop.bbox_x + x2,
        crop.bbox_y + y2,
    )


class FaceEvidenceController:
    """Run one latest-crop face window for each orchestrator command."""

    def __init__(
        self,
        verifier_factory: Callable[[], FaceVerifier],
        publish: Callable[[FaceEvidence], None],
        config: FaceConfig = FaceConfig(),
        overlay_callback: Callable[
            [FaceVerificationRequest, DecodedCrop, tuple[int, int, int, int] | None, int],
            None,
        ]
        | None = None,
    ) -> None:
        self._verifier_factory = verifier_factory
        self._publish = publish
        self._overlay_callback = overlay_callback
        self._window_timeout = max(0.1, float(config.window_timeout_seconds))
        self._freshness_timeout = max(
            0.1, float(config.crop_freshness_timeout_seconds)
        )
        self._overlay_hz = max(0.5, float(config.overlay_inference_hz))
        self._stop_after_matched = bool(config.stop_after_matched)
        self._matched_hold_seconds = max(0.0, float(config.matched_hold_seconds))

        self._condition = threading.Condition()
        self._mission_session: FaceVerificationSession | None = None
        self._desired_request: FaceVerificationRequest | None = None
        self._generation = 0
        self._completed_generation = 0
        self._active_cancel: threading.Event | None = None
        self._hardware_faulted = False
        self._stopping = False
        self._queue: deque[tuple[DecodedCrop, int]] = deque()
        self._last_crop_wall_ns: int | None = None
        self._worker_cuda_ctx = None
        self._worker_verifier: FaceVerifier | None = None

        self.crops_submitted = 0
        self.crops_dropped_target = 0
        self.crops_dropped_queue = 0
        self.crops_dropped_stale = 0
        self.crops_invalid = 0
        self.windows_started = 0
        self.inference_count = 0
        self.inference_total_ns = 0

        self._worker = threading.Thread(
            target=self._run,
            name="dog-patrol-face-evidence",
            daemon=True,
        )
        self._worker.start()

    def observe_session(self, session: FaceVerificationSession | None) -> None:
        """Track mission validity without starting face inference."""
        with self._condition:
            if session == self._mission_session:
                return
            self._mission_session = session
            self._desired_request = None
            self._generation += 1
            if self._active_cancel is not None:
                self._active_cancel.set()
            self._queue.clear()
            self._last_crop_wall_ns = None
            self._condition.notify_all()

    def request(self, request: FaceVerificationRequest | None) -> None:
        """Select one stage; duplicate and stale commands are ignored."""
        with self._condition:
            if request is not None and request.session != self._mission_session:
                return
            if request == self._desired_request:
                return
            self._desired_request = request
            self._generation += 1
            if self._active_cancel is not None:
                self._active_cancel.set()
            self._queue.clear()
            self._last_crop_wall_ns = None
            self._condition.notify_all()

    def observe_crop(self, crop: DecodedCrop) -> None:
        """Replace the pending crop so inference never drains an old backlog."""
        with self._condition:
            if self._stopping or self._desired_request is None:
                return
            if crop.target_id != self._desired_request.session.target_id:
                self.crops_dropped_target += 1
                return
            self.crops_dropped_queue += len(self._queue)
            self._queue.clear()
            arrival_ns = time.monotonic_ns()
            self._queue.append((crop, arrival_ns))
            self.crops_submitted += 1
            self._last_crop_wall_ns = arrival_ns
            self._condition.notify_all()

    def note_invalid_crop(self) -> None:
        with self._condition:
            self.crops_invalid += 1

    def metrics(self) -> dict[str, int]:
        with self._condition:
            return {
                "crops_submitted": self.crops_submitted,
                "crops_dropped_target": self.crops_dropped_target,
                "crops_dropped_queue": self.crops_dropped_queue,
                "crops_dropped_stale": self.crops_dropped_stale,
                "crops_invalid": self.crops_invalid,
                "windows_started": self.windows_started,
                "inference_count": self.inference_count,
                "inference_total_ns": self.inference_total_ns,
            }

    def stop(self) -> None:
        with self._condition:
            if self._stopping:
                return
            self._stopping = True
            if self._active_cancel is not None:
                self._active_cancel.set()
            self._condition.notify_all()
        self._worker.join()

    def _run(self) -> None:
        try:
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
                if healthy:
                    self._run_overlay_stream(request, generation, cancel_event)

                with self._condition:
                    if self._generation == generation:
                        self._completed_generation = generation
                    if self._active_cancel is cancel_event:
                        self._active_cancel = None
                    if not healthy:
                        self._hardware_faulted = True
                    self._condition.notify_all()
        finally:
            close = getattr(self._worker_verifier, "close", None)
            if callable(close):
                close()
            self._worker_verifier = None
            if self._worker_cuda_ctx is not None:
                self._worker_cuda_ctx.pop()
                self._worker_cuda_ctx = None

    def _execute(
        self,
        request: FaceVerificationRequest,
        generation: int,
        cancel_event: threading.Event,
    ) -> bool:
        try:
            if self._worker_cuda_ctx is None:
                self._worker_cuda_ctx = ensure_cuda_context()
            with self._condition:
                self.windows_started += 1
            verifier = self._verifier_factory()
            self._worker_verifier = verifier
            verifier.reset()
            result, detail, box = self._run_window(
                request, generation, cancel_event, verifier
            )
            if result is FaceEvidenceResult.ERROR:
                self._publish_error_detail(request, detail)
                return False
            self._publish_current(request, generation, result, detail, box)
            return True
        except _MissionCancelled:
            self._publish_cancelled(request)
            return True
        except Exception as exc:
            self._publish_error(request, exc)
            return True

    def _run_overlay_stream(
        self,
        request: FaceVerificationRequest,
        generation: int,
        cancel_event: threading.Event,
    ) -> None:
        if self._overlay_callback is None or self._worker_verifier is None:
            return
        try:
            period_ns = 1e9 / self._overlay_hz
            last_infer_ns = 0.0
            while True:
                crop_and_arrival = self._pop_latest_crop(
                    request, generation, cancel_event, None
                )
                if crop_and_arrival is None:
                    continue
                crop, _arrival_ns = crop_and_arrival
                now_ns = time.monotonic_ns()
                if now_ns - last_infer_ns < period_ns:
                    continue
                last_infer_ns = now_ns
                verdict = self._verify(self._worker_verifier, crop)
                if self._request_cancelled(generation, request, cancel_event):
                    raise _MissionCancelled
                box = _to_source_box(verdict.box, crop)
                result_code = (
                    AuthorizationEvidence.PASSED
                    if verdict.accepted
                    else AuthorizationEvidence.NOT_PASSED
                )
                self._overlay_callback(request, crop, box, result_code)
                if self._stop_after_matched and verdict.accepted:
                    deadline = time.monotonic() + self._matched_hold_seconds
                    while time.monotonic() < deadline:
                        with self._condition:
                            if self._request_cancelled_locked(
                                request,
                                generation,
                                self._generation,
                                self._desired_request,
                                cancel_event,
                            ):
                                raise _MissionCancelled
                            self.crops_dropped_queue += len(self._queue)
                            self._queue.clear()
                            self._condition.wait(
                                timeout=min(0.25, deadline - time.monotonic())
                            )
        except _MissionCancelled:
            return
        except Exception:
            return

    def _run_window(
        self,
        request: FaceVerificationRequest,
        generation: int,
        cancel_event: threading.Event,
        verifier: FaceVerifier,
    ) -> tuple[FaceEvidenceResult, str, tuple[int, int, int, int] | None]:
        deadline = time.monotonic() + self._window_timeout
        last_detail = ""
        last_source_box = None
        while True:
            crop_and_arrival = self._pop_latest_crop(
                request, generation, cancel_event, deadline
            )
            if crop_and_arrival is None:
                return (
                    FaceEvidenceResult.NOT_PASSED,
                    "verification window elapsed without a match",
                    None,
                )
            crop, arrival_ns = crop_and_arrival
            verdict = self._verify(verifier, crop)
            if verdict.detail:
                last_detail = verdict.detail
            if verdict.box is not None:
                last_source_box = _to_source_box(verdict.box, crop)
            with self._condition:
                newer_crop_waiting = bool(self._queue)
            if (
                self._request_cancelled(generation, request, cancel_event)
                or time.monotonic_ns() - arrival_ns
                > int(self._freshness_timeout * 1e9)
                or newer_crop_waiting
            ):
                with self._condition:
                    self.crops_dropped_stale += 1
                if self._request_cancelled(generation, request, cancel_event):
                    raise _MissionCancelled
                continue
            box = _to_source_box(verdict.box, crop)
            if self._overlay_callback is not None:
                result_code = (
                    AuthorizationEvidence.PASSED
                    if verdict.accepted
                    else AuthorizationEvidence.NOT_PASSED
                )
                self._overlay_callback(request, crop, box, result_code)
            if verdict.error:
                return FaceEvidenceResult.ERROR, verdict.error, box
            if verdict.accepted:
                return FaceEvidenceResult.PASSED, verdict.detail, box
            if time.monotonic() >= deadline:
                detail = "verification window elapsed without a match"
                if last_detail:
                    detail += f"; last_verdict={last_detail}"
                return (
                    FaceEvidenceResult.NOT_PASSED,
                    detail,
                    last_source_box,
                )

    def _verify(self, verifier: FaceVerifier, crop: DecodedCrop):
        started_ns = time.monotonic_ns()
        try:
            return verifier.verify(crop)
        finally:
            elapsed_ns = time.monotonic_ns() - started_ns
            with self._condition:
                self.inference_count += 1
                self.inference_total_ns += elapsed_ns

    def _pop_latest_crop(
        self,
        request: FaceVerificationRequest,
        generation: int,
        cancel_event: threading.Event,
        deadline: float | None,
    ) -> tuple[DecodedCrop, int] | None:
        with self._condition:
            while True:
                if self._stopping or self._request_cancelled_locked(
                    request,
                    generation,
                    self._generation,
                    self._desired_request,
                    cancel_event,
                ):
                    raise _MissionCancelled
                if self._queue:
                    crop_and_arrival = self._queue.pop()
                    self.crops_dropped_queue += len(self._queue)
                    self._queue.clear()
                    return crop_and_arrival
                now = time.monotonic()
                if deadline is not None and now >= deadline:
                    return None
                wait_seconds = 0.25
                if deadline is not None:
                    wait_seconds = min(wait_seconds, deadline - now)
                self._condition.wait(timeout=max(0.0, wait_seconds))

    def _request_cancelled(
        self,
        generation: int,
        request: FaceVerificationRequest,
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
        request: FaceVerificationRequest,
        generation: int,
        current_generation: int,
        desired_request: FaceVerificationRequest | None,
        cancel_event: threading.Event,
    ) -> bool:
        return (
            cancel_event.is_set()
            or desired_request != request
            or current_generation != generation
        )

    def _publish_current(
        self,
        request: FaceVerificationRequest,
        generation: int,
        result: FaceEvidenceResult,
        detail: str,
        box: tuple[int, int, int, int] | None,
    ) -> bool:
        with self._condition:
            if self._stopping or self._generation != generation:
                return False
            if self._desired_request != request:
                return False
            self._publish(
                FaceEvidence(request.session, request.stage, result, detail, box)
            )
            return True

    def _publish_cancelled(self, request: FaceVerificationRequest) -> None:
        with self._condition:
            if self._stopping:
                return
            self._publish(
                FaceEvidence(
                    request.session,
                    request.stage,
                    FaceEvidenceResult.CANCELLED,
                    "face task cancelled",
                )
            )

    def _publish_error(
        self, request: FaceVerificationRequest, exc: Exception
    ) -> None:
        self._publish_error_detail(request, _exception_detail(exc))

    def _publish_error_detail(
        self, request: FaceVerificationRequest, detail: str
    ) -> None:
        with self._condition:
            if self._stopping:
                return
            self._publish(
                FaceEvidence(
                    request.session,
                    request.stage,
                    FaceEvidenceResult.ERROR,
                    detail,
                )
            )


class FaceEvidenceProviderNode(Node):
    """ROS adapter for the asynchronous face evidence controller."""

    _ROS_RESULT_CODES = {
        FaceEvidenceResult.PASSED: AuthorizationEvidence.PASSED,
        FaceEvidenceResult.NOT_PASSED: AuthorizationEvidence.NOT_PASSED,
        FaceEvidenceResult.ERROR: AuthorizationEvidence.ERROR,
        FaceEvidenceResult.CANCELLED: AuthorizationEvidence.CANCELLED,
    }

    def __init__(
        self,
        *,
        parameter_overrides: list[Parameter] | None = None,
        verifier_factory: Callable[[], FaceVerifier] | None = None,
        config: FaceConfig | None = None,
    ) -> None:
        super().__init__(
            "perception_face_provider", parameter_overrides=parameter_overrides
        )
        self.declare_parameter("mission_state_topic", "/mission/state")
        self.declare_parameter(
            "tracked_target_image_topic", "/perception/tracked_target_image"
        )
        self.declare_parameter(
            "authorization_evidence_topic", "/perception/authorization_evidence"
        )
        self.declare_parameter(
            "authorization_command_topic", "/perception/authorization_command"
        )
        self.declare_parameter("face_overlay_topic", "/perception/face_overlay")
        self.declare_parameter("provider", "face")
        self.declare_parameter("config_file", "")
        self.declare_parameter("detector_engine", "")
        self.declare_parameter("recognition_engine", "")
        self.declare_parameter("whitelist_dir", "")
        self.declare_parameter("max_detail_length", 256)

        crop_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
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
        overlay_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._provider_name = str(self.get_parameter("provider").value).strip()
        if not self._provider_name:
            raise ValueError("provider must not be empty")
        self._max_detail_length = max(
            32, int(self.get_parameter("max_detail_length").value)
        )
        self._last_drop_log_ns = 0.0
        self._config = config or self._load_config()
        self._adapter_lock = threading.Lock()
        self._production_verifier: FaceVerifier | None = None
        self._evidence_pub = self.create_publisher(
            AuthorizationEvidence,
            str(self.get_parameter("authorization_evidence_topic").value),
            evidence_qos,
        )
        self._overlay_pub = self.create_publisher(
            FaceOverlay,
            str(self.get_parameter("face_overlay_topic").value),
            overlay_qos,
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
        self._crop_sub = self.create_subscription(
            self._crop_message_type(),
            str(self.get_parameter("tracked_target_image_topic").value),
            self._on_crop,
            crop_qos,
        )
        self._controller = FaceEvidenceController(
            verifier_factory or self._build_verifier,
            self._publish_evidence,
            config=self._config,
            overlay_callback=self._publish_face_overlay,
        )
        self._metrics_timer = self.create_timer(5.0, self._log_metrics)

    @staticmethod
    def _crop_message_type():
        from dog_patrol_perception_interfaces.msg import TrackedTargetImage

        return TrackedTargetImage

    def destroy_node(self):
        self._controller.stop()
        self._log_metrics()
        return super().destroy_node()

    def get_metrics(self) -> dict[str, int]:
        return self._controller.metrics()

    def _log_metrics(self) -> None:
        metrics = self.get_metrics()
        count = metrics["inference_count"]
        average_ms = (
            metrics["inference_total_ns"] / count / 1_000_000.0
            if count
            else 0.0
        )
        self.get_logger().info(
            "face_metrics "
            f"windows={metrics['windows_started']} "
            f"crops={metrics['crops_submitted']} "
            f"queue_drops={metrics['crops_dropped_queue']} "
            f"stale_drops={metrics['crops_dropped_stale']} "
            f"target_drops={metrics['crops_dropped_target']} "
            f"invalid={metrics['crops_invalid']} "
            f"inferences={count} average_inference_ms={average_ms:.3f}"
        )

    def _on_mission(self, msg: MissionState) -> None:
        self._controller.observe_session(_session_from_mission(msg))

    def _on_command(self, msg: AuthorizationCommand) -> None:
        session = FaceVerificationSession(
            int(msg.observed_state_seq), int(msg.target_id)
        )
        if int(msg.stage) == AuthorizationCommand.CANCEL:
            self._controller.request(None)
            return
        stage = {
            AuthorizationCommand.INITIAL_FACE: FaceVerificationStage.INITIAL_FACE,
            AuthorizationCommand.DUAL_FIRST: FaceVerificationStage.DUAL_FIRST,
            AuthorizationCommand.DUAL_SECOND: FaceVerificationStage.DUAL_SECOND,
        }.get(int(msg.stage))
        if stage is not None:
            self._controller.request(FaceVerificationRequest(session, stage))

    def _on_crop(self, msg) -> None:
        crop, reason = decode_crop(msg)
        if crop is None:
            self._controller.note_invalid_crop()
            self._log_drop(reason)
            return
        self._controller.observe_crop(crop)

    def _log_drop(self, reason: str) -> None:
        now = time.monotonic()
        if now - self._last_drop_log_ns < 1.0:
            return
        self._last_drop_log_ns = now
        self.get_logger().warning(f"dropped invalid tracked target crop: {reason}")

    def _publish_evidence(self, evidence: FaceEvidence) -> None:
        msg = AuthorizationEvidence()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.observed_state_seq = evidence.session.observed_state_seq
        msg.target_id = evidence.session.target_id
        msg.stage = {
            FaceVerificationStage.INITIAL_FACE: AuthorizationEvidence.INITIAL_FACE,
            FaceVerificationStage.DUAL_FIRST: AuthorizationEvidence.DUAL_FIRST,
            FaceVerificationStage.DUAL_SECOND: AuthorizationEvidence.DUAL_SECOND,
        }[evidence.stage]
        msg.result = self._ROS_RESULT_CODES[evidence.result]
        msg.provider = self._provider_name
        msg.detail = evidence.detail[: self._max_detail_length]
        self._evidence_pub.publish(msg)

    def _publish_face_overlay(
        self,
        request: FaceVerificationRequest,
        crop: DecodedCrop,
        box: tuple[int, int, int, int] | None,
        result_code: int,
    ) -> None:
        """Publish a lightweight per-frame face box for the tracking overlay.

        Called on the evidence worker thread for every processed crop, so the
        tracking visualizer can draw a live face box on the shared canvas. The
        message is best-effort/last-writer-wins; consumers apply staleness and
        target-match suppression per FaceOverlay.msg.
        """
        msg = FaceOverlay()
        msg.header.stamp.sec = crop.source_stamp_ns // 1_000_000_000
        msg.header.stamp.nanosec = crop.source_stamp_ns % 1_000_000_000
        msg.header.frame_id = crop.source_frame_id
        msg.observed_state_seq = request.session.observed_state_seq
        msg.target_id = request.session.target_id
        msg.stage = {
            FaceVerificationStage.INITIAL_FACE: AuthorizationEvidence.INITIAL_FACE,
            FaceVerificationStage.DUAL_FIRST: AuthorizationEvidence.DUAL_FIRST,
            FaceVerificationStage.DUAL_SECOND: AuthorizationEvidence.DUAL_SECOND,
        }[request.stage]
        msg.source_frame_number = crop.source_frame_number
        msg.source_frame_number_available = crop.source_frame_number_available
        msg.result = result_code
        if box is None:
            msg.bbox_x = -1
            msg.bbox_y = -1
            msg.bbox_width = -1
            msg.bbox_height = -1
        else:
            msg.bbox_x = box[0]
            msg.bbox_y = box[1]
            msg.bbox_width = box[2] - box[0]
            msg.bbox_height = box[3] - box[1]
        self._overlay_pub.publish(msg)

    def _load_config(self) -> FaceConfig:
        config_file = str(self.get_parameter("config_file").value).strip()
        if not config_file:
            return FaceConfig()
        return load_face_config(config_file)

    def _build_verifier(self) -> FaceVerifier:
        with self._adapter_lock:
            if self._production_verifier is not None:
                return self._production_verifier
            config = self._config
            detector_engine = (
                str(self.get_parameter("detector_engine").value).strip()
                or config.detector_engine
            )
            recognition_engine = (
                str(self.get_parameter("recognition_engine").value).strip()
                or config.recognition_engine
            )
            whitelist_dir = (
                str(self.get_parameter("whitelist_dir").value).strip()
                or config.whitelist_dir
            )
            missing = [
                name
                for name, value in (
                    ("detector_engine", detector_engine),
                    ("recognition_engine", recognition_engine),
                    ("whitelist_dir", whitelist_dir),
                )
                if not value
            ]
            if missing:
                raise ValueError(
                    "face deployment inputs are unset: " + ", ".join(missing)
                )

            detector = TRTInference(
                detector_engine,
                conf_thres=config.detector_conf_thres,
                iou_thres=config.detector_iou_thres,
            )
            detector._ensure_gpu_preproc()
            recognition = RecModel(recognition_engine)
            database = load_whitelist_npy(whitelist_dir)
            verifier = ProductionFaceVerifier(
                detector=detector,
                recognition=recognition,
                database=database,
                similarity_threshold=config.similarity_threshold,
                size_sigma=config.size_sigma,
                temporal_window=config.temporal_window,
                min_frames_for_decision=config.min_frames_for_decision,
                min_score=config.face_min_score,
                min_box_ratio=config.face_min_box_ratio,
            )
            self._production_verifier = verifier
            return verifier


def _session_from_mission(msg: MissionState) -> FaceVerificationSession | None:
    if (
        int(msg.state) != MissionState.VERIFY_IDENTITY
        or int(msg.target_id) <= 0
    ):
        return None
    return FaceVerificationSession(int(msg.state_seq), int(msg.target_id))


def _exception_detail(exc: Exception) -> str:
    detail = str(exc).strip()
    if detail:
        return f"{type(exc).__name__}: {detail}"
    return type(exc).__name__


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FaceEvidenceProviderNode()
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
