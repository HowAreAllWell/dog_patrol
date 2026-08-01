import time
from contextlib import contextmanager

import pytest
import rclpy
from dog_patrol_interfaces.msg import (
    MissionEvent,
    MissionState,
    TargetBoundingBox,
)
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_srvs.srv import Trigger

from dog_patrol_perception.fake_perception_node import FakePerceptionNode


STATE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
EVENT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
BBOX_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


class FakePerceptionHarness:
    def __init__(self, case_name, **overrides):
        rclpy.init()
        prefix = f"/test/fake_perception/{case_name}"
        self.state_topic = f"{prefix}/mission_state"
        self.event_topic = f"{prefix}/mission_event"
        self.bbox_topic = f"{prefix}/selected_target_bbox"
        parameters = {
            "state_topic": self.state_topic,
            "event_topic": self.event_topic,
            "bbox_topic": self.bbox_topic,
            "auto_ready": False,
        }
        parameters.update(overrides)
        self.fake = FakePerceptionNode(
            parameter_overrides=[
                Parameter(name, value=value)
                for name, value in parameters.items()
            ]
        )
        self.observer = Node(f"fake_perception_{case_name}_test_observer")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.fake)
        self.executor.add_node(self.observer)

        self.events = []
        self.boxes = []
        self.observer.create_subscription(
            MissionEvent, self.event_topic, self.events.append, EVENT_QOS
        )
        self.observer.create_subscription(
            TargetBoundingBox, self.bbox_topic, self.boxes.append, BBOX_QOS
        )
        self.state_pub = self.observer.create_publisher(
            MissionState, self.state_topic, STATE_QOS
        )

    def close(self):
        self.executor.remove_node(self.observer)
        self.executor.remove_node(self.fake)
        self.observer.destroy_node()
        self.fake.destroy_node()
        rclpy.shutdown()

    def spin_until(self, predicate, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.05)
            if predicate():
                return True
        return False

    def publish_state(
        self,
        state_seq,
        state,
        target_id=0,
        blocked=False,
        block_cause=MissionState.BLOCK_NONE,
    ):
        message = MissionState()
        message.state_seq = state_seq
        message.state = state
        message.target_id = target_id
        message.blocked = blocked
        message.block_cause = block_cause
        self.state_pub.publish(message)
        self.executor.spin_once(timeout_sec=0.1)

    def service_client(self, name):
        return self.observer.create_client(Trigger, f"/fake_perception/{name}")

    def call(self, client):
        assert self.spin_until(client.service_is_ready)
        future = client.call_async(Trigger.Request())
        assert self.spin_until(future.done)
        return future.result()


@contextmanager
def fake_perception(case_name, **overrides):
    harness = FakePerceptionHarness(case_name, **overrides)
    try:
        yield harness
    finally:
        harness.close()


def test_startup_state_publishes_one_perception_ready_event():
    with fake_perception("ready", auto_ready=True) as harness:
        for _ in range(3):
            harness.publish_state(7, MissionState.STARTUP)

        assert harness.spin_until(lambda: len(harness.events) >= 1)
        assert len(harness.events) == 1
        event = harness.events[0]
        assert event.source == MissionEvent.SOURCE_PERCEPTION
        assert event.event == MissionEvent.READY
        assert event.observed_state_seq == 7
        assert event.target_id == 0


def test_confirm_target_service_publishes_event_for_current_patrol_state():
    with fake_perception("confirm", initial_target_id=42) as harness:
        confirm_client = harness.service_client("confirm_target")
        harness.publish_state(11, MissionState.PATROL)

        assert harness.call(confirm_client).success
        assert harness.spin_until(lambda: len(harness.events) == 1)
        event = harness.events[0]
        assert event.source == MissionEvent.SOURCE_PERCEPTION
        assert event.event == MissionEvent.TARGET_CONFIRMED
        assert event.observed_state_seq == 11
        assert event.target_id == 42


def test_active_target_state_publishes_configured_bbox():
    with fake_perception(
        "bbox",
        bbox_publish_rate=20.0,
        image_width=1280,
        image_height=720,
        bbox_x_min=320,
        bbox_y_min=120,
        bbox_x_max=760,
        bbox_y_max=680,
        bbox_confidence=0.88,
        camera_frame_id="fake_camera_optical_frame",
    ) as harness:
        harness.publish_state(12, MissionState.CONFIRM_TARGET, target_id=42)

        assert harness.spin_until(lambda: len(harness.boxes) >= 1)
        box = harness.boxes[-1]
        assert box.target_id == 42
        assert box.header.frame_id == "fake_camera_optical_frame"
        assert (box.image_width, box.image_height) == (1280, 720)
        assert (box.x_min, box.y_min, box.x_max, box.y_max) == (
            320,
            120,
            760,
            680,
        )
        assert abs(box.confidence - 0.88) < 1e-6


def test_target_recovery_and_error_follow_current_mission_context():
    with fake_perception("recovery") as harness:
        lost_client = harness.service_client("target_lost")
        reacquired_client = harness.service_client("target_reacquired")
        error_client = harness.service_client("execution_error")
        harness.publish_state(13, MissionState.APPROACH_TARGET, target_id=42)

        assert harness.call(lost_client).success
        assert harness.spin_until(lambda: len(harness.events) == 1)
        assert harness.events[0].event == MissionEvent.TARGET_LOST
        assert harness.events[0].observed_state_seq == 13
        assert harness.events[0].target_id == 42

        harness.publish_state(
            14,
            MissionState.APPROACH_TARGET,
            target_id=42,
            blocked=True,
            block_cause=MissionState.BLOCK_TARGET_LOST,
        )
        assert harness.call(reacquired_client).success
        assert harness.spin_until(lambda: len(harness.events) == 2)
        assert harness.events[1].event == MissionEvent.TARGET_REACQUIRED
        assert harness.events[1].observed_state_seq == 14
        assert harness.events[1].target_id == 42

        harness.publish_state(15, MissionState.APPROACH_TARGET, target_id=42)
        assert harness.call(error_client).success
        assert harness.spin_until(lambda: len(harness.events) == 3)
        assert harness.events[2].event == MissionEvent.EXECUTION_ERROR
        assert harness.events[2].observed_state_seq == 15
        assert harness.events[2].target_id == 42


def test_two_fake_not_passed_results_publish_one_unauthorized_event():
    with fake_perception("not_passed") as harness:
        client = harness.service_client("authorization_not_passed")
        harness.publish_state(21, MissionState.VERIFY_IDENTITY, target_id=42)

        assert harness.call(client).success
        harness.executor.spin_once(timeout_sec=0.1)
        assert harness.events == []

        assert harness.call(client).success
        assert harness.spin_until(lambda: len(harness.events) == 1)
        event = harness.events[0]
        assert event.event == MissionEvent.UNAUTHORIZED
        assert event.observed_state_seq == 21
        assert event.target_id == 42


@pytest.mark.parametrize(
    ("service_name", "expected_event"),
    [
        ("authorization_passed", MissionEvent.AUTHORIZED),
        ("authorization_error", MissionEvent.EXECUTION_ERROR),
        ("authorization_cancelled", None),
    ],
)
def test_terminal_authorization_results_map_to_public_events(
    service_name, expected_event
):
    with fake_perception(service_name) as harness:
        client = harness.service_client(service_name)
        harness.publish_state(31, MissionState.VERIFY_IDENTITY, target_id=52)

        assert harness.call(client).success
        if expected_event is None:
            harness.executor.spin_once(timeout_sec=0.1)
            assert harness.events == []
        else:
            assert harness.spin_until(lambda: len(harness.events) == 1)
            event = harness.events[0]
            assert event.event == expected_event
            assert event.observed_state_seq == 31
            assert event.target_id == 52


def test_target_lost_immediately_cancels_active_authorization():
    with fake_perception("lost_auth") as harness:
        lost_client = harness.service_client("target_lost")
        passed_client = harness.service_client("authorization_passed")
        harness.publish_state(41, MissionState.VERIFY_IDENTITY, target_id=62)

        assert harness.call(lost_client).success
        assert harness.spin_until(lambda: len(harness.events) == 1)
        assert harness.events[0].event == MissionEvent.TARGET_LOST

        assert not harness.call(passed_client).success
        harness.executor.spin_once(timeout_sec=0.1)
        assert [message.event for message in harness.events] == [
            MissionEvent.TARGET_LOST
        ]
