#!/usr/bin/env python3
"""Minimal fake navigation and face nodes for local integration tests.

These nodes deliberately model only the ROS contract needed by the fixed
integration flow. They do not contain navigation or face algorithms.
"""

from __future__ import annotations

import argparse
import rclpy
from dog_patrol_interfaces.msg import MissionEvent, MissionState, TargetBoundingBox
from dog_patrol_perception_interfaces.msg import CapabilityStatus, TrackedTargetImage
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def state_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


def event_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


def best_effort_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
    )


def capability_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=16,
    )


class FakeNavigation(Node):
    """Advance the minimum navigation states without commanding hardware."""

    def __init__(self, delay: float, state_topic: str, event_topic: str, bbox_topic: str) -> None:
        super().__init__("fake_navigation")
        self._delay = max(0.0, delay)
        # The supervisor uses initial_state_seq=1. Seed a startup snapshot so
        # READY can be retried even if this node starts before the latched state.
        self._state: MissionState | None = MissionState()
        self._state.state_seq = 1
        self._state.state = MissionState.STARTUP
        self._bbox: TargetBoundingBox | None = None
        self._sent: set[tuple[int, int]] = set()
        self._event_pub = self.create_publisher(
            MissionEvent, event_topic, event_qos()
        )
        self.create_subscription(
            MissionState, state_topic, self._on_state, state_qos()
        )
        self.create_subscription(
            MissionEvent, event_topic, self._on_event, event_qos()
        )
        self.create_subscription(
            TargetBoundingBox,
            bbox_topic,
            self._on_bbox,
            best_effort_qos(),
        )
        self._timer = self.create_timer(0.1, self._tick)

    def _on_state(self, msg: MissionState) -> None:
        self._state = msg

    def _on_event(self, msg: MissionEvent) -> None:
        if msg.event == MissionEvent.TARGET_CONFIRMED:
            self.get_logger().info(f"observed TARGET_CONFIRMED target={msg.target_id}")

    def _on_bbox(self, msg: TargetBoundingBox) -> None:
        self._bbox = msg

    def _publish(self, event: int, target_id: int = 0, detail: str = "") -> None:
        assert self._state is not None
        msg = MissionEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.observed_state_seq = self._state.state_seq
        msg.target_id = target_id
        msg.source = MissionEvent.SOURCE_NAVIGATION
        msg.event = event
        msg.detail = detail
        self._event_pub.publish(msg)
        self._sent.add((self._state.state_seq, event))

    def _tick(self) -> None:
        state = self._state
        if state is None:
            return
        key = (int(state.state_seq), int(state.state))
        if state.state == MissionState.STARTUP:
            # MissionEvent is volatile; retry until the supervisor records it.
            self._publish(MissionEvent.READY, detail="fake navigation ready")
            return
        if state.target_id <= 0:
            return
        if state.state == MissionState.CONFIRM_TARGET:
            bbox = self._bbox
            if bbox is None or bbox.target_id != state.target_id:
                return
            stamp = int(bbox.header.stamp.sec) + int(bbox.header.stamp.nanosec) / 1e9
            if self.get_clock().now().nanoseconds / 1e9 - stamp > 0.3:
                return
            if (state.state_seq, MissionEvent.TARGET_POSITION_READY) not in self._sent:
                self._publish(MissionEvent.TARGET_POSITION_READY, state.target_id,
                              "fake target position ready")
        elif state.state == MissionState.APPROACH_TARGET:
            if (state.state_seq, MissionEvent.ARRIVED_AND_STOPPED) not in self._sent:
                self._sent.add((state.state_seq, MissionEvent.ARRIVED_AND_STOPPED))
                self.create_timer(self._delay, self._delayed_arrival)

    def _delayed_arrival(self) -> None:
        if self._state is not None and self._state.state == MissionState.APPROACH_TARGET:
            self._publish(MissionEvent.ARRIVED_AND_STOPPED, self._state.target_id,
                          "fake navigation arrived and stopped")


class FakeFace(Node):
    """Publish only face readiness and observe target crops."""

    def __init__(self, state_topic: str, capability_topic: str, image_topic: str) -> None:
        super().__init__("fake_face")
        self._last_startup = 0
        self._state: MissionState | None = None
        self._last_publish_ns = 0
        self._status_pub = self.create_publisher(
            CapabilityStatus, capability_topic, capability_qos()
        )
        self.create_subscription(
            MissionState, state_topic, self._on_state, state_qos()
        )
        self.create_subscription(
            TrackedTargetImage,
            image_topic,
            self._on_image,
            best_effort_qos(),
        )
        self._timer = self.create_timer(0.5, self._retry_startup_ready)

    def _on_state(self, msg: MissionState) -> None:
        self._state = msg
        if msg.state != MissionState.STARTUP:
            return
        self._publish_ready(msg)

    def _retry_startup_ready(self) -> None:
        msg = self._state
        if msg is None or msg.state != MissionState.STARTUP:
            return
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_publish_ns >= 1_000_000_000:
            self._publish_ready(msg)

    def _publish_ready(self, msg: MissionState) -> None:
        first_publish = msg.state_seq != self._last_startup
        self._last_startup = msg.state_seq
        self._last_publish_ns = self.get_clock().now().nanoseconds
        status = CapabilityStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.capability = "face"
        status.status = CapabilityStatus.READY
        status.observed_startup_state_seq = msg.state_seq
        status.diagnostic = "minimal fake face: crop observer only"
        self._status_pub.publish(status)
        if first_publish:
            self.get_logger().info(f"published fake face READY for startup seq={msg.state_seq}")

    def _on_image(self, msg: TrackedTargetImage) -> None:
        if msg.target_id > 0:
            self.get_logger().debug(f"observed target crop target={msg.target_id}")


class FakeCapabilities(Node):
    """Publish only face/voice READY so tracking can exercise mission states."""

    def __init__(self, state_topic: str, capability_topic: str) -> None:
        super().__init__("fake_capabilities")
        self._last_startup = 0
        self._pub = self.create_publisher(CapabilityStatus, capability_topic, capability_qos())
        self.create_subscription(MissionState, state_topic, self._on_state, state_qos())

    def _on_state(self, msg: MissionState) -> None:
        if msg.state != MissionState.STARTUP or msg.state_seq == self._last_startup:
            return
        self._last_startup = msg.state_seq
        for capability in ("face", "voice"):
            status = CapabilityStatus()
            status.header.stamp = self.get_clock().now().to_msg()
            status.capability = capability
            status.status = CapabilityStatus.READY
            status.observed_startup_state_seq = msg.state_seq
            status.diagnostic = "test-only capability readiness; algorithm disabled"
            self._pub.publish(status)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--role", choices=("navigation", "face", "capabilities"), required=True)
    parser.add_argument("--arrival-delay", type=float, default=1.0)
    parser.add_argument("--state-topic", default="/mission/state")
    parser.add_argument("--event-topic", default="/mission/event")
    parser.add_argument("--bbox-topic", default="/perception/selected_target_bbox")
    parser.add_argument("--capability-topic", default="/perception/capability_status")
    parser.add_argument("--image-topic", default="/perception/tracked_target_image")
    args = parser.parse_args()
    rclpy.init()
    if args.role == "navigation":
        node = FakeNavigation(args.arrival_delay, args.state_topic, args.event_topic, args.bbox_topic)
    elif args.role == "face":
        node = FakeFace(args.state_topic, args.capability_topic, args.image_topic)
    else:
        node = FakeCapabilities(args.state_topic, args.capability_topic)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
