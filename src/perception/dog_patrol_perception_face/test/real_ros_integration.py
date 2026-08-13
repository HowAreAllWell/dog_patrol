"""Explicit Orin integration check: real models, crop -> evidence binding.

Launches the real ``perception_face_provider`` with the real TensorRT engines
and whitelist, publishes a synthetic ``VERIFY_IDENTITY`` mission and
``TrackedTargetImage`` crops replayed from a recorded video, then asserts the
published ``AuthorizationEvidence`` is bound to ``state_seq + target_id`` and
only emitted while the mission is in an unblocked ``VERIFY_IDENTITY``.

Skips automatically when the model assets or pycuda are unavailable (CI keeps
green). Run on the Orin with the face_rec venv's pycuda available:

    source /opt/ros/humble/setup.bash
    source install/setup.bash
    PYTHONPATH=/home/user/face_rec/YOLO\\ BACKBONE/yolo-backbone/lib/python3.10/site-packages \\
      python3 -m pytest test/test_real_ros_integration.py -v
"""

from __future__ import annotations

from collections.abc import Callable
import os
from pathlib import Path
import time

from dog_patrol_interfaces.msg import MissionState
from dog_patrol_perception_interfaces.msg import (
    AuthorizationCommand,
    AuthorizationEvidence,
    TrackedTargetImage,
)
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

import cv2
import numpy as np
import pytest
import rclpy

from dog_patrol_perception_face.config import FaceConfig
from dog_patrol_perception_face.provider import FaceEvidenceProviderNode


def _cuda_available() -> bool:
    try:
        import pycuda.driver as cuda

        cuda.init()
        cuda.Device(0).compute_capability()
        return True
    except Exception:
        return False


_BASE = Path(os.environ.get("FACE_REC_DIR", "/home/user/face_rec/YOLO BACKBONE"))
_DETECTOR_ENGINE = Path(
    os.environ.get(
        "FACE_DETECTOR_ENGINE",
        _BASE / "engines/yolo26n-e2e-v10-050426-fp32.engine",
    )
)
_RECOGNITION_ENGINE = Path(
    os.environ.get(
        "FACE_RECOGNITION_ENGINE",
        _BASE / "face_models/engines/sface_2021dec_fp16.engine",
    )
)
_WHITELIST_DIR = Path(os.environ.get("FACE_WHITELIST_DIR", _BASE / "whitelist_npy"))
_VIDEO = Path(os.environ.get("FACE_VIDEO", _BASE / "extracted_20260720_183143_raw.avi"))

_ASSETS_PRESENT = all(
    p.exists() for p in (_DETECTOR_ENGINE, _RECOGNITION_ENGINE, _WHITELIST_DIR, _VIDEO)
)

pytestmark = pytest.mark.skipif(
    not (_ASSETS_PRESENT and _cuda_available()),
    reason="real face model assets or pycuda not available on this machine",
)


def _reliable_qos(*, transient: bool, depth: int) -> QoSProfile:
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


def _best_effort_qos(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def _frame_crop_message(
    frame: np.ndarray,
    target_id: int,
    frame_number: int,
) -> TrackedTargetImage:
    h, w = frame.shape[:2]
    msg = TrackedTargetImage()
    msg.encoding = "bgr8"
    msg.crop_width = w
    msg.crop_height = h
    msg.crop_step = w * 3
    msg.crop_data = np.ascontiguousarray(frame).tobytes()
    msg.target_id = target_id
    msg.source_image_width = w
    msg.source_image_height = h
    msg.bbox_x = 0
    msg.bbox_y = 0
    msg.bbox_width = w
    msg.bbox_height = h
    msg.source_stamp.sec = frame_number
    msg.source_stamp.nanosec = 0
    msg.source_frame_id = "integration_smoke"
    msg.source_frame_number = frame_number
    msg.source_frame_number_available = True
    return msg


def _fast_config() -> FaceConfig:
    return FaceConfig(
        detector_conf_thres=0.25,
        detector_iou_thres=0.5,
        similarity_threshold=0.55,
        size_sigma=20.0,
        temporal_window=10,
        min_frames_for_decision=5,
        window_timeout_seconds=5.0,
        crop_freshness_timeout_seconds=3.0,
    )


class RealProviderHarness:
    """ROS harness with the real provider and real engines."""

    def __init__(self) -> None:
        rclpy.init()
        suffix = f"real{time.monotonic_ns()}"
        state_topic = f"/face/{suffix}/mission/state"
        evidence_topic = f"/face/{suffix}/perception/authorization_evidence"
        command_topic = f"/face/{suffix}/perception/authorization_command"
        crop_topic = f"/face/{suffix}/perception/tracked_target_image"
        self.source = Node(f"face_real_source_{suffix}")
        self.probe = Node(f"face_real_probe_{suffix}")
        self.provider = FaceEvidenceProviderNode(
            parameter_overrides=[
                Parameter("mission_state_topic", value=state_topic),
                Parameter("tracked_target_image_topic", value=crop_topic),
                Parameter("authorization_evidence_topic", value=evidence_topic),
                Parameter("authorization_command_topic", value=command_topic),
                Parameter("detector_engine", value=str(_DETECTOR_ENGINE)),
                Parameter("recognition_engine", value=str(_RECOGNITION_ENGINE)),
                Parameter("whitelist_dir", value=str(_WHITELIST_DIR)),
            ],
            config=_fast_config(),
        )
        self.state_pub = self.source.create_publisher(
            MissionState, state_topic, _reliable_qos(transient=True, depth=1)
        )
        self.crop_pub = self.source.create_publisher(
            TrackedTargetImage, crop_topic, _best_effort_qos(depth=1)
        )
        self.command_pub = self.source.create_publisher(
            AuthorizationCommand,
            command_topic,
            _reliable_qos(transient=False, depth=10),
        )
        self.evidence: list[AuthorizationEvidence] = []
        self.evidence_sub = self.probe.create_subscription(
            AuthorizationEvidence,
            evidence_topic,
            self.evidence.append,
            _reliable_qos(transient=False, depth=10),
        )
        self.executor = SingleThreadedExecutor()
        for node in (self.source, self.probe, self.provider):
            self.executor.add_node(node)

    def publish_state(
        self,
        state_seq,
        target_id,
        *,
        state=MissionState.VERIFY_IDENTITY,
        blocked=False,
    ) -> None:
        msg = MissionState()
        msg.state_seq = state_seq
        msg.state = state
        msg.target_id = target_id
        msg.blocked = blocked
        self.state_pub.publish(msg)
        for _ in range(5):
            self.executor.spin_once(timeout_sec=0.02)

    def publish_crop(self, frame: np.ndarray, target_id: int, frame_number: int) -> None:
        self.crop_pub.publish(_frame_crop_message(frame, target_id, frame_number))
        self.executor.spin_once(timeout_sec=0.02)

    def publish_command(self, state_seq: int, target_id: int, stage: int) -> None:
        msg = AuthorizationCommand()
        msg.observed_state_seq = state_seq
        msg.target_id = target_id
        msg.stage = stage
        self.command_pub.publish(msg)
        for _ in range(5):
            self.executor.spin_once(timeout_sec=0.02)

    def spin(self, seconds: float = 0.05) -> None:
        self.executor.spin_once(timeout_sec=seconds)

    def wait(self, predicate: Callable[[], bool], timeout: float = 20.0) -> bool:
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.05)
        return predicate()

    def close(self) -> None:
        self.provider.destroy_node()
        for node in (self.source, self.probe):
            node.destroy_node()
        self.executor.shutdown()
        del self.evidence_sub
        if rclpy.ok():
            rclpy.shutdown()


def _person_frames(start: int = 70, end: int = 97) -> list[np.ndarray]:
    cap = cv2.VideoCapture(str(_VIDEO))
    frames: list[np.ndarray] = []
    index = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if start <= index < end:
            frames.append(frame)
        index += 1
    cap.release()
    return frames


def test_real_models_crop_to_evidence_binding() -> None:
    harness = RealProviderHarness()
    try:
        frames = _person_frames()
        assert frames, "no frames read from video"

        state_seq = 77
        target_id = 9
        harness.publish_state(state_seq, target_id)
        harness.publish_command(
            state_seq, target_id, AuthorizationCommand.INITIAL_FACE
        )

        for frame_number, frame in enumerate(frames):
            harness.publish_crop(frame, target_id, frame_number)
            harness.spin(0.01)

        assert harness.wait(lambda: len(harness.evidence) >= 1), "no face evidence published"
        assert all(
            m.provider == "face" for m in harness.evidence
        ), "evidence provider must be 'face'"
        assert all(
            (m.observed_state_seq, m.target_id) == (state_seq, target_id)
            for m in harness.evidence
        ), "evidence must be bound to the VERIFY_IDENTITY state_seq + target_id"
        assert all(
            m.stage == AuthorizationEvidence.INITIAL_FACE
            for m in harness.evidence
        )
        assert all(
            m.result
            in (AuthorizationEvidence.PASSED, AuthorizationEvidence.NOT_PASSED)
            for m in harness.evidence
        ), "real verification must reach a decided result"

        evidence_before_leave = len(harness.evidence)
        harness.publish_state(state_seq, target_id, state=MissionState.PATROL)
        harness.spin(0.3)
        assert len(harness.evidence) == evidence_before_leave, (
            "a session that already decided must not emit more evidence after "
            "leaving VERIFY_IDENTITY"
        )
    finally:
        harness.close()


def test_real_models_leave_verify_cancels_in_flight_window() -> None:
    harness = RealProviderHarness()
    try:
        frames = _person_frames()
        assert frames

        state_seq = 77
        target_id = 9
        harness.publish_state(state_seq, target_id)
        harness.publish_command(
            state_seq, target_id, AuthorizationCommand.INITIAL_FACE
        )
        harness.publish_crop(frames[0], target_id, frame_number=0)
        assert harness.wait(
            lambda: harness.provider.get_metrics()["windows_started"] >= 1
        ), "face provider never entered a verification window"
        harness.publish_state(state_seq, target_id, state=MissionState.PATROL)

        assert harness.wait(
            lambda: any(m.result == AuthorizationEvidence.CANCELLED for m in harness.evidence)
        ), "leaving VERIFY_IDENTITY mid-window must cancel the pending session"
        assert all(
            (m.observed_state_seq, m.target_id) == (state_seq, target_id)
            and m.provider == "face"
            for m in harness.evidence
        ), "cancelled evidence must stay bound to the interrupted session"
    finally:
        harness.close()


def test_real_models_rejects_wrong_target_and_out_of_session() -> None:
    harness = RealProviderHarness()
    try:
        frames = _person_frames()
        assert frames

        harness.publish_crop(frames[0], target_id=3, frame_number=0)
        harness.spin(0.1)
        assert harness.evidence == [], "crops without a VERIFY_IDENTITY session must be ignored"

        state_seq = 88
        target_id = 12
        harness.publish_state(state_seq, target_id)
        harness.publish_command(
            state_seq, target_id, AuthorizationCommand.INITIAL_FACE
        )
        harness.publish_crop(frames[0], target_id=999, frame_number=0)
        harness.spin(0.1)
        assert harness.evidence == [], "crop with the wrong target_id must be ignored"
    finally:
        harness.close()
