"""Real ROS face/voice/orchestrator transport with simulated recognition hardware."""

import threading
import time
from urllib.parse import unquote, urlparse
from pathlib import Path

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from dog_patrol_interfaces.msg import MissionEvent, MissionState
from dog_patrol_perception_interfaces.msg import (
    AuthorizationEvidence, TrackedTargetImage, VoiceVerificationEvent,
)
from dog_patrol_perception_face.config import FaceConfig
from dog_patrol_perception_face import provider as face_provider
from dog_patrol_perception_face.verifier import FaceVerification
from dog_patrol_perception_orchestrator.authorization_node import PerceptionAuthorizationNode
from dog_patrol_perception_voice.adapter import VoiceTaskCancelled
from dog_patrol_perception_voice.provider import VoiceEvidenceProviderNode
from dog_patrol_perception_voice.result import VoiceWindowEvent, VoiceWindowObservation, VoiceWindowResult


class SimulatedVoiceTask:
    def __init__(self):
        self.playing = threading.Event()
        self.prompt_done = threading.Event()
        self.listening = threading.Event()
        self.answer_ready = threading.Event()
        self.cancelled = threading.Event()
        self.accepted = False

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        pass

    def respond(self, *, retry=False, on_event):
        self.playing.set()
        assert self.prompt_done.wait(5.0), "test did not finish simulated playback"
        if self.cancelled.is_set():
            raise VoiceTaskCancelled("cancelled during playback")
        on_event(VoiceWindowObservation(VoiceWindowEvent.WAIT_STARTED))
        self.listening.set()
        assert self.answer_ready.wait(5.0), "test did not finish simulated response"
        if self.cancelled.is_set():
            raise VoiceTaskCancelled("cancelled while listening")
        on_event(VoiceWindowObservation(
            VoiceWindowEvent.PASSED if self.accepted else VoiceWindowEvent.NOT_PASSED,
            (0,) if self.accepted else (),
        ))
        return VoiceWindowResult(self.accepted, 0.1)

    def cancel(self):
        self.cancelled.set()
        self.prompt_done.set()
        self.answer_ready.set()


class SimulatedHardware:
    def __init__(self):
        self.voice_tasks = []
        self.face_resets = 0

    def task(self):
        task = SimulatedVoiceTask()
        self.voice_tasks.append(task)
        return task

    def reset(self):
        self.face_resets += 1

    def verify(self, crop):
        accepted = crop.source_frame_number == 2
        return FaceVerification(
            accepted, True, "simulated face", person_id="employee-1" if accepted else "",
            similarity=0.95 if accepted else 0.1,
        )


@pytest.mark.parametrize("ending", ["playback_face", "late_face", "third_round_face", "voice", "exhausted", "cancel"])
def test_joint_verification_lifecycle(tmp_path, monkeypatch, ending):
    monkeypatch.setattr(face_provider, "ensure_cuda_context", lambda: None)
    rclpy.init()
    executor = SingleThreadedExecutor()
    nodes = []
    hardware = SimulatedHardware()
    try:
        suffix = f"/joint/t{time.monotonic_ns()}"
        source = Node("joint_verification_test")
        nodes.append(source)
        common = [
            Parameter("mission_state_topic", value=suffix + "/state"),
            Parameter("authorization_command_topic", value=suffix + "/command"),
            Parameter("authorization_evidence_topic", value=suffix + "/evidence"),
            Parameter("state_timeout", value=2.0),
        ]
        face = face_provider.FaceEvidenceProviderNode(
            parameter_overrides=common + [
                Parameter("tracked_target_image_topic", value=suffix + "/crop"),
                Parameter("face_overlay_topic", value=suffix + "/overlay"),
            ], verifier_factory=lambda: hardware,
            config=FaceConfig(
                initial_face_window_seconds=0.1,
                dual_verification_window_seconds=0.1,  # Legacy value must be ignored.
                evidence_photo_enabled=True, evidence_photo_root=str(tmp_path / "face"),
            ),
        )
        voice = VoiceEvidenceProviderNode(
            parameter_overrides=common + [
                Parameter("voice_verification_event_topic", value=suffix + "/voice_event"),
                Parameter("config_file", value=""),
            ], adapter_factory=lambda: hardware,
        )
        nodes.extend([face, voice])
        for node in nodes:
            executor.add_node(node)
        state = MissionState(state_seq=17, state=MissionState.VERIFY_IDENTITY, target_id=42)
        state_pub = source.create_publisher(MissionState, suffix + "/state", QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
        ))
        crop_pub = source.create_publisher(TrackedTargetImage, suffix + "/crop", QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
        ))
        frame = [1]

        def publish_crop():
            msg = TrackedTargetImage(
                target_id=42, encoding="bgr8", crop_width=32, crop_height=24,
                crop_step=96, crop_data=bytes(32 * 24 * 3),
                source_image_width=640, source_image_height=480,
                bbox_width=32, bbox_height=24,
                source_frame_number=frame[0], source_frame_number_available=True,
            )
            msg.source_stamp = source.get_clock().now().to_msg()
            crop_pub.publish(msg)

        source.create_timer(0.05, lambda: state_pub.publish(state))
        source.create_timer(0.03, publish_crop)

        def wait(predicate, timeout=3.0):
            deadline = time.monotonic() + timeout
            while not predicate() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.01)
            assert predicate(), "joint verification did not reach expected state"

        # Deliver the mission identity before allowing the orchestrator to issue commands.
        wait(lambda: face._mission_session is not None and voice._mission_session is not None)
        events, evidence, voice_events = [], [], []
        for message_type, topic, callback in (
            (MissionEvent, "/event", events.append),
            (AuthorizationEvidence, "/evidence", evidence.append),
            (VoiceVerificationEvent, "/voice_event", voice_events.append),
        ):
            source.create_subscription(message_type, suffix + topic, callback, 10)
        authorization = PerceptionAuthorizationNode(parameter_overrides=common + [
            Parameter("mission_event_topic", value=suffix + "/event"),
            Parameter("voice_prompt_count", value=3),
        ])
        nodes.append(authorization)
        executor.add_node(authorization)
        wait(lambda: len(hardware.voice_tasks) == 1 and hardware.voice_tasks[0].playing.is_set())
        assert any(item.provider == "face" and item.result == AuthorizationEvidence.NOT_PASSED for item in evidence)

        rounds = 3 if ending in ("third_round_face", "exhausted") else 1
        for number in range(rounds):
            wait(lambda: len(hardware.voice_tasks) == number + 1 and hardware.voice_tasks[-1].playing.is_set())
            task = hardware.voice_tasks[-1]
            if ending == "playback_face":
                break
            task.prompt_done.set()
            wait(task.listening.is_set)
            if number < rounds - 1 or ending in ("voice", "exhausted"):
                task.accepted = ending == "voice"
                task.answer_ready.set()

        if ending in ("playback_face", "late_face", "third_round_face"):
            # Keep spinning beyond the old independent face deadline before matching.
            start = time.monotonic()
            wait(lambda: time.monotonic() - start > 0.25)
            assert not events
            frame[0] = 2
        elif ending == "cancel":
            state.state_seq += 1
            state.state = MissionState.READY

        if ending == "cancel":
            wait(task.cancelled.is_set)
            wait(lambda: face._controller._desired_request is None)
            assert not events
        else:
            wait(lambda: len(events) == 1)
            assert events[0].event == (
                MissionEvent.UNAUTHORIZED if ending == "exhausted" else MissionEvent.AUTHORIZED
            )
            wait(lambda: face._controller._desired_request is None and voice._controller._desired_request is None)
            if ending.endswith("face"):
                wait(lambda: any(item.provider == "face" and item.result == AuthorizationEvidence.PASSED for item in evidence))
                passed = next(item for item in evidence if item.provider == "face" and item.result == AuthorizationEvidence.PASSED)
                assert passed.stage == AuthorizationEvidence.DUAL_FIRST
                assert passed.person_id == "employee-1"
                assert Path(unquote(urlparse(passed.image_url).path)).is_file()
                assert task.cancelled.is_set()

        assert hardware.face_resets == 2  # Initial face plus one continuous joint task.
        assert len(hardware.voice_tasks) == rounds
        expected_misses = rounds if ending == "exhausted" else rounds - 1
        assert sum(item.event == VoiceVerificationEvent.NOT_PASSED for item in voice_events) == expected_misses
    finally:
        for task in hardware.voice_tasks:
            task.cancel()
        executor.shutdown()
        for node in reversed(nodes):
            node.destroy_node()
        rclpy.shutdown()
