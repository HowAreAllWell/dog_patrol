from __future__ import annotations

from collections import deque
from collections.abc import Callable
import threading
import time

import numpy as np
import pytest
import rclpy
from dog_patrol_interfaces.msg import MissionState
from dog_patrol_perception_interfaces.msg import (
    AuthorizationCommand,
    AuthorizationEvidence,
    FaceOverlay,
    TrackedTargetImage,
)
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from dog_patrol_perception_face.config import FaceConfig
from dog_patrol_perception_face.crop import DecodedCrop
from dog_patrol_perception_face.provider import (
    FaceEvidenceResult,
    FaceEvidenceController,
    FaceEvidenceProviderNode,
    FaceVerificationRequest,
    FaceVerificationSession,
    FaceVerificationStage,
)
from dog_patrol_perception_face.verifier import FaceVerification


def _qos(*, reliable=True, transient=False, depth=10):
    return QoSProfile(
        reliability=(
            ReliabilityPolicy.RELIABLE
            if reliable
            else ReliabilityPolicy.BEST_EFFORT
        ),
        durability=(
            DurabilityPolicy.TRANSIENT_LOCAL
            if transient
            else DurabilityPolicy.VOLATILE
        ),
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def _fast_config(**overrides):
    values = dict(
        window_timeout_seconds=0.2,
        crop_freshness_timeout_seconds=1.0,
        overlay_inference_hz=30.0,
        stop_after_matched=False,
    )
    values.update(overrides)
    return FaceConfig(**values)


class ScriptedFaceVerifier:
    def __init__(self, results):
        self._results = deque(results)
        self.calls = 0

    def reset(self):
        pass

    def verify(self, crop):
        del crop
        self.calls += 1
        return self._results.popleft()


def test_only_initial_face_window_has_a_deadline_with_legacy_fallback():
    controller = FaceEvidenceController(
        lambda: ScriptedFaceVerifier([]),
        lambda _evidence: None,
        FaceConfig(
            window_timeout_seconds=4.0,
            initial_face_window_seconds=3.0,
            dual_verification_window_seconds=10.0,
        ),
    )
    try:
        assert controller._window_timeout_for(FaceVerificationStage.INITIAL_FACE) == 3.0
        assert controller._window_timeout_for(FaceVerificationStage.DUAL_FIRST) is None
        assert controller._window_timeout_for(FaceVerificationStage.DUAL_SECOND) is None
    finally:
        controller.stop()

    legacy = FaceEvidenceController(
        lambda: ScriptedFaceVerifier([]),
        lambda _evidence: None,
        FaceConfig(window_timeout_seconds=4.0),
    )
    try:
        assert legacy._window_timeout_for(FaceVerificationStage.INITIAL_FACE) == 4.0
        assert legacy._window_timeout_for(FaceVerificationStage.DUAL_FIRST) is None
    finally:
        legacy.stop()


def _crop_message(target_id, frame=1):
    msg = TrackedTargetImage()
    msg.encoding = "bgr8"
    msg.crop_width = 32
    msg.crop_height = 24
    msg.crop_step = 32 * 3
    msg.crop_data = bytes(32 * 3 * 24)
    msg.target_id = target_id
    msg.source_image_width = 640
    msg.source_image_height = 480
    msg.bbox_x = 10
    msg.bbox_y = 20
    msg.bbox_width = 32
    msg.bbox_height = 24
    msg.source_stamp.sec = frame
    msg.source_frame_number = frame
    msg.source_frame_number_available = True
    return msg


class ProviderHarness:
    def __init__(self, verifier_factory, config=None):
        suffix = f"t{time.monotonic_ns()}"
        self.state_topic = f"/face/{suffix}/mission/state"
        self.command_topic = f"/face/{suffix}/authorization/command"
        self.evidence_topic = f"/face/{suffix}/authorization/evidence"
        self.crop_topic = f"/face/{suffix}/crop"
        self.overlay_topic = f"/face/{suffix}/overlay"
        self.source = Node(f"face_source_{suffix}")
        self.probe = Node(f"face_probe_{suffix}")
        self.provider = FaceEvidenceProviderNode(
            parameter_overrides=[
                Parameter("mission_state_topic", value=self.state_topic),
                Parameter("authorization_command_topic", value=self.command_topic),
                Parameter("authorization_evidence_topic", value=self.evidence_topic),
                Parameter("tracked_target_image_topic", value=self.crop_topic),
                Parameter("face_overlay_topic", value=self.overlay_topic),
                Parameter("state_timeout", value=10.0),
            ],
            verifier_factory=verifier_factory,
            config=config or _fast_config(),
        )
        self.state_pub = self.source.create_publisher(
            MissionState, self.state_topic, _qos(transient=True, depth=1)
        )
        self.command_pub = self.source.create_publisher(
            AuthorizationCommand, self.command_topic, _qos()
        )
        self.crop_pub = self.source.create_publisher(
            TrackedTargetImage, self.crop_topic, _qos(reliable=False, depth=1)
        )
        self.evidence = []
        self.overlays = []
        self.evidence_sub = self.probe.create_subscription(
            AuthorizationEvidence,
            self.evidence_topic,
            self.evidence.append,
            _qos(),
        )
        self.overlay_sub = self.probe.create_subscription(
            FaceOverlay,
            self.overlay_topic,
            self.overlays.append,
            _qos(reliable=False, depth=1),
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
        handled=0,
    ):
        msg = MissionState()
        msg.state_seq = seq
        msg.state = state
        msg.target_id = target
        msg.handled_target_id = handled
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

    def publish_crop(self, target, frame=1):
        self.crop_pub.publish(_crop_message(target, frame))
        for _ in range(2):
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
    return [(message.stage, message.result) for message in messages]


def test_mission_state_alone_does_not_start_face_inference():
    verifier = ScriptedFaceVerifier(
        [FaceVerification(accepted=True, decided=True, detail="matched")]
    )
    harness = ProviderHarness(lambda: verifier)
    try:
        harness.publish_state(17, 42)
        harness.publish_crop(42)

        assert verifier.calls == 0
        assert harness.evidence == []
    finally:
        harness.close()


@pytest.mark.parametrize(
    ("command_stage", "evidence_stage"),
    [
        (AuthorizationCommand.INITIAL_FACE, AuthorizationEvidence.INITIAL_FACE),
        (AuthorizationCommand.DUAL_FIRST, AuthorizationEvidence.DUAL_FIRST),
        (AuthorizationCommand.DUAL_SECOND, AuthorizationEvidence.DUAL_SECOND),
    ],
)
def test_each_face_stage_runs_one_window_and_labels_evidence(
    command_stage, evidence_stage
):
    verifier = ScriptedFaceVerifier(
        [FaceVerification(accepted=True, decided=True, detail="matched")]
    )
    harness = ProviderHarness(lambda: verifier)
    try:
        harness.publish_state(18, 43)
        harness.publish_command(18, 43, command_stage)
        harness.publish_crop(43)

        assert harness.wait(lambda: len(harness.evidence) == 1)
        assert verifier.calls == 1
        assert evidence_keys(harness.evidence) == [
            (evidence_stage, AuthorizationEvidence.PASSED)
        ]
        assert harness.wait(lambda: len(harness.overlays) == 1)
        overlay = harness.overlays[0]
        assert overlay.header.stamp.sec == 1
        assert overlay.source_frame_number == 1
        assert overlay.source_frame_number_available is True
        assert overlay.observed_state_seq == 18
        assert overlay.target_id == 43
        assert overlay.stage == evidence_stage
    finally:
        harness.close()


def test_duplicate_stage_command_does_not_run_twice():
    verifier = ScriptedFaceVerifier(
        [FaceVerification(accepted=True, decided=True, detail="matched")]
    )
    harness = ProviderHarness(lambda: verifier)
    try:
        harness.publish_state(19, 44)
        harness.publish_command(19, 44, AuthorizationCommand.INITIAL_FACE)
        harness.publish_crop(44)
        assert harness.wait(lambda: len(harness.evidence) == 1)

        harness.publish_command(19, 44, AuthorizationCommand.INITIAL_FACE)
        harness.publish_crop(44, 2)

        assert len(harness.evidence) == 1
        assert verifier.calls >= 1
    finally:
        harness.close()


def test_wrong_target_and_invalid_crop_are_dropped():
    verifier = ScriptedFaceVerifier(
        [FaceVerification(accepted=True, decided=True, detail="matched")]
    )
    harness = ProviderHarness(lambda: verifier)
    try:
        harness.publish_state(20, 45)
        harness.publish_command(20, 45, AuthorizationCommand.INITIAL_FACE)
        harness.publish_crop(99)
        bad = _crop_message(45)
        bad.encoding = "rgb8"
        harness.crop_pub.publish(bad)
        harness.executor.spin_once(timeout_sec=0.1)

        metrics = harness.provider.get_metrics()
        assert verifier.calls == 0
        assert metrics["crops_dropped_target"] == 1
        assert metrics["crops_invalid"] == 1
    finally:
        harness.close()


def test_cancel_command_stops_active_stage_and_labels_cancelled_evidence():
    class BlockingVerifier:
        def __init__(self):
            self.entered = threading.Event()
            self.release = threading.Event()

        def reset(self):
            pass

        def verify(self, crop):
            del crop
            self.entered.set()
            self.release.wait(timeout=3.0)
            return FaceVerification(False, False, "late")

    verifier = BlockingVerifier()
    harness = ProviderHarness(lambda: verifier)
    try:
        harness.publish_state(21, 46)
        harness.publish_command(21, 46, AuthorizationCommand.DUAL_FIRST)
        harness.publish_crop(46)
        assert verifier.entered.wait(timeout=1.0)

        harness.publish_command(21, 46, AuthorizationCommand.CANCEL)
        verifier.release.set()

        assert harness.wait(lambda: len(harness.evidence) == 1)
        assert evidence_keys(harness.evidence) == [
            (AuthorizationEvidence.DUAL_FIRST, AuthorizationEvidence.CANCELLED)
        ]
    finally:
        verifier.release.set()
        harness.close()


def test_late_old_verify_snapshot_cannot_resurrect_ready_session():
    verifier = ScriptedFaceVerifier(
        [FaceVerification(accepted=True, decided=True, detail="matched")]
    )
    harness = ProviderHarness(lambda: verifier)
    try:
        harness.publish_state(31, 56)
        harness.publish_state(32, 0, state=MissionState.READY)

        harness.publish_state(31, 56)
        harness.publish_command(31, 56, AuthorizationCommand.INITIAL_FACE)
        harness.publish_crop(56)

        assert not harness.wait(lambda: verifier.calls > 0, timeout=0.25)
        assert harness.evidence == []
    finally:
        harness.close()


def _decoded_crop(frame):
    return DecodedCrop(
        target_id=1,
        source_stamp_ns=frame * 1_000_000_000,
        source_frame_id="cam",
        source_frame_number=frame,
        source_frame_number_available=True,
        source_image_width=640,
        source_image_height=480,
        bbox_x=10,
        bbox_y=20,
        bbox_width=32,
        bbox_height=24,
        image=np.zeros((24, 32, 3), dtype=np.uint8),
    )


def test_joint_face_match_survives_old_deadline_and_voice_round_transition():
    class BlockingMatch:
        def __init__(self):
            self.resets = 0
            self.missed = threading.Event()
            self.entered = threading.Event()
            self.release = threading.Event()

        def reset(self):
            self.resets += 1

        def verify(self, crop):
            if crop.source_frame_number == 1:
                self.missed.set()
                return FaceVerification(False, True, "no match")
            self.entered.set()
            assert self.release.wait(2.0)
            return FaceVerification(True, True, "matched", person_id="employee-1", similarity=0.91)

    verifier = BlockingMatch()
    evidence = []
    saved_frames = []
    matched = threading.Event()

    def publish(item):
        evidence.append(item)
        matched.set()

    controller = FaceEvidenceController(
        lambda: verifier, publish,
        config=_fast_config(dual_verification_window_seconds=0.1),
        evidence_photo_callback=lambda crop, _request, _result: (
            saved_frames.append(crop.source_frame_number) or "file:///joint-pass.jpg"
        ),
    )
    session = FaceVerificationSession(1, 1)
    try:
        controller.observe_session(session)
        controller.request(FaceVerificationRequest(session, FaceVerificationStage.DUAL_FIRST))
        controller.observe_crop(_decoded_crop(1))
        assert verifier.missed.wait(1.0)
        # The old independent cutoff must not emit NOT_PASSED or stop listening.
        assert not matched.wait(0.25)
        controller.observe_crop(_decoded_crop(2))
        assert verifier.entered.wait(1.0)
        controller.request(FaceVerificationRequest(session, FaceVerificationStage.DUAL_SECOND))
        controller.request(FaceVerificationRequest(session, FaceVerificationStage.DUAL_SECOND))
        verifier.release.set()
        assert matched.wait(1.0)
        assert [item.result for item in evidence] == [FaceEvidenceResult.PASSED]
        assert evidence[0].stage is FaceVerificationStage.DUAL_FIRST
        assert evidence[0].person_id == "employee-1"
        assert evidence[0].image_url == "file:///joint-pass.jpg"
        assert saved_frames == [2]
        assert verifier.resets == controller.windows_started == 1
    finally:
        verifier.release.set()
        controller.stop()


def test_passed_evidence_selects_actual_pass_crop():
    verifier = ScriptedFaceVerifier([
        FaceVerification(False, True, "no match", similarity=0.4),
        FaceVerification(True, True, "matched", person_id="employee-1", similarity=0.91),
    ])
    evidence = []
    saved_frames = []
    controller = FaceEvidenceController(
        lambda: verifier,
        evidence.append,
        config=_fast_config(window_timeout_seconds=1.0),
        evidence_photo_callback=lambda crop, _request, _result: (
            saved_frames.append(crop.source_frame_number) or "file:///pass.jpg"
        ),
    )
    session = FaceVerificationSession(1, 1)
    try:
        controller.observe_session(session)
        controller.request(FaceVerificationRequest(session, FaceVerificationStage.INITIAL_FACE))
        controller.observe_crop(_decoded_crop(1))
        deadline = time.monotonic() + 1.0
        while verifier.calls < 1 and time.monotonic() < deadline:
            time.sleep(0.01)
        controller.observe_crop(_decoded_crop(2))
        while not evidence and time.monotonic() < deadline:
            time.sleep(0.01)
        assert saved_frames == [2]
        assert evidence[0].person_id == "employee-1"
        assert evidence[0].image_url == "file:///pass.jpg"
    finally:
        controller.stop()


def test_not_passed_evidence_selects_first_detected_face():
    class AlwaysMiss:
        def reset(self):
            pass

        def verify(self, _crop):
            return FaceVerification(
                False, True, "no match", box=(1, 1, 8, 8), similarity=0.42,
                similarity_valid=True,
            )

    evidence = []
    saved_frames = []
    controller = FaceEvidenceController(
        AlwaysMiss,
        evidence.append,
        config=_fast_config(window_timeout_seconds=0.12, initial_face_window_seconds=0.12),
        evidence_photo_callback=lambda crop, _request, _result: (
            saved_frames.append(crop.source_frame_number) or "file:///start.jpg"
        ),
    )
    session = FaceVerificationSession(1, 1)
    try:
        controller.observe_session(session)
        controller.request(FaceVerificationRequest(session, FaceVerificationStage.INITIAL_FACE))
        controller.observe_crop(_decoded_crop(10))
        deadline = time.monotonic() + 1.0
        while not evidence and time.monotonic() < deadline:
            time.sleep(0.01)
        assert saved_frames == [10]
        assert evidence[0].result is FaceEvidenceResult.NOT_PASSED
        assert evidence[0].image_url == "file:///start.jpg"
        assert evidence[0].similarity == 0.42
        assert evidence[0].similarity_status == "measured"
    finally:
        controller.stop()


def test_not_passed_keeps_first_recognized_crop_and_its_measured_similarity():
    verifier = ScriptedFaceVerifier([
        FaceVerification(
            False, False, "collecting frames", box=(1, 1, 8, 8),
            similarity=0.27, similarity_valid=True,
        ),
        FaceVerification(
            False, True, "no match", box=(1, 1, 8, 8),
            similarity=0.73, similarity_valid=True,
        ),
    ])
    evidence = []
    saved_frames = []
    controller = FaceEvidenceController(
        lambda: verifier,
        evidence.append,
        config=_fast_config(
            window_timeout_seconds=0.12,
            initial_face_window_seconds=0.12,
        ),
        evidence_photo_callback=lambda crop, _request, _result: (
            saved_frames.append(crop.source_frame_number) or "file:///first-face.jpg"
        ),
    )
    session = FaceVerificationSession(1, 1)
    try:
        controller.observe_session(session)
        controller.request(FaceVerificationRequest(session, FaceVerificationStage.INITIAL_FACE))
        controller.observe_crop(_decoded_crop(20))
        deadline = time.monotonic() + 1.0
        while verifier.calls < 1 and time.monotonic() < deadline:
            time.sleep(0.01)
        controller.observe_crop(_decoded_crop(21))
        while not evidence and time.monotonic() < deadline:
            time.sleep(0.01)
        assert saved_frames == [20]
        assert evidence[0].similarity == 0.27
        assert evidence[0].similarity_status == "measured"
    finally:
        controller.stop()


def test_not_passed_without_detected_face_has_no_photo_or_similarity():
    class NoFace:
        def reset(self):
            pass

        def verify(self, _crop):
            return FaceVerification(False, False, "no face detected")

    evidence = []
    saved_frames = []
    controller = FaceEvidenceController(
        NoFace,
        evidence.append,
        config=_fast_config(
            window_timeout_seconds=0.08,
            initial_face_window_seconds=0.08,
        ),
        evidence_photo_callback=lambda crop, _request, _result: (
            saved_frames.append(crop.source_frame_number) or "file:///unexpected.jpg"
        ),
    )
    session = FaceVerificationSession(1, 1)
    try:
        controller.observe_session(session)
        controller.request(FaceVerificationRequest(session, FaceVerificationStage.INITIAL_FACE))
        controller.observe_crop(_decoded_crop(30))
        deadline = time.monotonic() + 1.0
        while not evidence and time.monotonic() < deadline:
            time.sleep(0.01)
        assert saved_frames == []
        assert evidence[0].image_url == ""
        assert evidence[0].similarity == 0.0
        assert evidence[0].similarity_status == "no_face_detected"
    finally:
        controller.stop()


def test_detected_face_without_score_reports_measurement_error():
    class UnmeasuredFace:
        def reset(self):
            pass

        def verify(self, _crop):
            return FaceVerification(
                False, False, "low quality face", box=(1, 1, 8, 8)
            )

    evidence = []
    controller = FaceEvidenceController(
        UnmeasuredFace,
        evidence.append,
        config=_fast_config(
            window_timeout_seconds=0.08,
            initial_face_window_seconds=0.08,
        ),
    )
    session = FaceVerificationSession(1, 1)
    try:
        controller.observe_session(session)
        controller.request(FaceVerificationRequest(session, FaceVerificationStage.INITIAL_FACE))
        controller.observe_crop(_decoded_crop(31))
        deadline = time.monotonic() + 1.0
        while not evidence and time.monotonic() < deadline:
            time.sleep(0.01)
        assert evidence[0].similarity == 0.0
        assert evidence[0].similarity_status == "measurement_error"
    finally:
        controller.stop()


def test_failure_uses_first_face_processed_after_factory_delay():
    release_factory = threading.Event()

    class AlwaysMiss:
        def reset(self):
            pass

        def verify(self, _crop):
            return FaceVerification(
                False, True, "no match", box=(1, 1, 8, 8), similarity=0.31,
                similarity_valid=True,
            )

    def delayed_factory():
        release_factory.wait(timeout=2.0)
        return AlwaysMiss()

    evidence = []
    saved_frames = []
    controller = FaceEvidenceController(
        delayed_factory,
        evidence.append,
        config=_fast_config(window_timeout_seconds=0.12, initial_face_window_seconds=0.12),
        evidence_photo_callback=lambda crop, _request, _result: (
            saved_frames.append(crop.source_frame_number) or "file:///start.jpg"
        ),
    )
    session = FaceVerificationSession(1, 1)
    try:
        controller.observe_session(session)
        controller.request(FaceVerificationRequest(session, FaceVerificationStage.INITIAL_FACE))
        controller.observe_crop(_decoded_crop(10))
        controller.observe_crop(_decoded_crop(11))
        release_factory.set()
        deadline = time.monotonic() + 1.0
        while not evidence and time.monotonic() < deadline:
            time.sleep(0.01)
        assert saved_frames == [11]
        assert evidence[0].result is FaceEvidenceResult.NOT_PASSED
        assert evidence[0].similarity == 0.31
    finally:
        release_factory.set()
        controller.stop()


def test_failure_uses_first_face_processed_after_window_starts():
    window_waiting = threading.Event()
    release_queue = threading.Event()

    class AlwaysMiss:
        def reset(self):
            pass

        def verify(self, _crop):
            return FaceVerification(
                False, True, "no match", box=(1, 1, 8, 8), similarity=0.31,
                similarity_valid=True,
            )

    evidence = []
    saved_frames = []
    controller = FaceEvidenceController(
        AlwaysMiss,
        evidence.append,
        config=_fast_config(window_timeout_seconds=0.12, initial_face_window_seconds=0.12),
        evidence_photo_callback=lambda crop, _request, _result: (
            saved_frames.append(crop.source_frame_number) or "file:///start.jpg"
        ),
    )
    original_pop = controller._pop_latest_crop
    first_call = True

    def gated_pop(*args, **kwargs):
        nonlocal first_call
        if first_call:
            first_call = False
            window_waiting.set()
            release_queue.wait(timeout=2.0)
        return original_pop(*args, **kwargs)

    controller._pop_latest_crop = gated_pop
    session = FaceVerificationSession(1, 1)
    try:
        controller.observe_session(session)
        controller.request(FaceVerificationRequest(session, FaceVerificationStage.INITIAL_FACE))
        assert window_waiting.wait(timeout=1.0)
        controller.observe_crop(_decoded_crop(10))
        controller.observe_crop(_decoded_crop(11))
        release_queue.set()
        deadline = time.monotonic() + 1.0
        while not evidence and time.monotonic() < deadline:
            time.sleep(0.01)
        assert saved_frames == [11]
        assert evidence[0].result is FaceEvidenceResult.NOT_PASSED
        assert evidence[0].similarity == 0.31
    finally:
        release_queue.set()
        controller.stop()


def test_controller_discards_inflight_result_when_a_newer_crop_arrives():
    class LatestVerifier:
        def __init__(self):
            self.frames = []
            self.first_started = threading.Event()
            self.release_first = threading.Event()

        def reset(self):
            pass

        def verify(self, crop):
            self.frames.append(crop.source_frame_number)
            if len(self.frames) == 1:
                self.first_started.set()
                self.release_first.wait(timeout=3.0)
            return FaceVerification(True, True, f"frame {crop.source_frame_number}")

    verifier = LatestVerifier()
    evidence = []
    controller = FaceEvidenceController(
        lambda: verifier,
        evidence.append,
        config=_fast_config(window_timeout_seconds=2.0),
    )
    session = FaceVerificationSession(1, 1)
    request = FaceVerificationRequest(session, FaceVerificationStage.INITIAL_FACE)
    try:
        controller.observe_session(session)
        controller.request(request)
        controller.observe_crop(_decoded_crop(1))
        assert verifier.first_started.wait(timeout=1.0)
        controller.observe_crop(_decoded_crop(2))
        controller.observe_crop(_decoded_crop(3))
        verifier.release_first.set()

        deadline = time.monotonic() + 3.0
        while not evidence and time.monotonic() < deadline:
            time.sleep(0.01)
        assert verifier.frames == [1, 3]
        assert evidence[0].detail == "frame 3"
        metrics = controller.metrics()
        assert metrics["crops_dropped_queue"] >= 1
        assert metrics["crops_dropped_stale"] >= 1
    finally:
        verifier.release_first.set()
        controller.stop()


def test_mission_state_heartbeat_timeout_invalidates_face_session():
    verifier = ScriptedFaceVerifier([])
    harness = ProviderHarness(lambda: verifier)
    try:
        harness.publish_state(60, 88)
        harness.provider._state_timeout = 0.01
        harness.provider._last_state_monotonic = time.monotonic() - 1.0

        harness.provider._check_state_timeout()

        assert harness.provider._state_timed_out
        assert harness.provider._controller._mission_session is None
        assert harness.evidence == []
    finally:
        harness.close()


def test_same_sequence_non_session_field_conflict_locks_face_provider():
    verifier = ScriptedFaceVerifier(
        [FaceVerification(accepted=True, decided=True, detail="matched")]
    )
    harness = ProviderHarness(lambda: verifier)
    try:
        harness.publish_state(70, 90)
        harness.publish_state(70, 90, handled=90)
        harness.publish_state(70, 90)
        harness.publish_command(70, 90, AuthorizationCommand.INITIAL_FACE)
        harness.publish_crop(90)

        assert not harness.wait(lambda: verifier.calls > 0, timeout=0.25)
        assert harness.evidence == []
    finally:
        harness.close()
