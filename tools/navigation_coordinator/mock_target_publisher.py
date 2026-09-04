#!/usr/bin/env python3
"""Publish a synthetic perception target for navigation coordinator tests.

This script deliberately does not fake lidar, TF, Nav2, or navigation events.
It only publishes the perception-side TARGET_CONFIRMED event and a timestamped
TargetBoundingBox. The real coordinator must still fuse the box with the live
PointCloud2 and must publish TARGET_POSITION_READY itself.
"""

from __future__ import annotations

import argparse

import rclpy
from dog_patrol_interfaces.msg import MissionEvent, MissionState, TargetBoundingBox
from rclpy.executors import ExternalShutdownException
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


def bbox_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
    )


class MockTargetPublisher(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("mock_target_publisher")
        self._args = args
        self._state: MissionState | None = None
        self._confirmed_seq: int | None = None
        self._terminal_seq: int | None = None
        self._event_pub = self.create_publisher(
            MissionEvent, args.event_topic, event_qos()
        )
        self._bbox_pub = self.create_publisher(
            TargetBoundingBox, args.bbox_topic, bbox_qos()
        )
        self.create_subscription(
            MissionState, args.state_topic, self._on_state, state_qos()
        )
        self.create_timer(1.0 / args.rate, self._tick)
        self.get_logger().info(
            "mock target ready: target_id=%d bbox=(%d,%d)-(%d,%d) %dx%d frame=%s",
            args.target_id,
            args.x_min,
            args.y_min,
            args.x_max,
            args.y_max,
            args.image_width,
            args.image_height,
            args.frame_id,
        )

    def _on_state(self, msg: MissionState) -> None:
        if self._state is not None and msg.state_seq < self._state.state_seq:
            return
        self._state = msg

    def _publish_event(self, event: int, detail: str) -> None:
        if self._state is None:
            return
        msg = MissionEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.observed_state_seq = self._state.state_seq
        msg.target_id = self._args.target_id if event != MissionEvent.READY else 0
        msg.source = MissionEvent.SOURCE_PERCEPTION
        msg.event = event
        msg.detail = detail
        self._event_pub.publish(msg)

    def _publish_bbox(self) -> None:
        msg = TargetBoundingBox()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._args.frame_id
        msg.target_id = self._args.target_id
        msg.image_width = self._args.image_width
        msg.image_height = self._args.image_height
        msg.x_min = self._args.x_min
        msg.y_min = self._args.y_min
        msg.x_max = self._args.x_max
        msg.y_max = self._args.y_max
        msg.confidence = self._args.confidence
        self._bbox_pub.publish(msg)

    def _tick(self) -> None:
        state = self._state
        if state is None:
            return

        if state.state == MissionState.PATROL and state.target_id == 0:
            if self._confirmed_seq != state.state_seq:
                self._publish_event(
                    MissionEvent.TARGET_CONFIRMED,
                    "synthetic perception target for coordinator test",
                )
                self._confirmed_seq = state.state_seq
            return

        if state.target_id != self._args.target_id:
            return

        if state.state in (
            MissionState.CONFIRM_TARGET,
            MissionState.APPROACH_TARGET,
            MissionState.VERIFY_IDENTITY,
            MissionState.TRACK_INTRUDER,
        ):
            self._publish_bbox()

        if (
            self._args.terminal_event != "none"
            and state.state == MissionState.VERIFY_IDENTITY
            and self._terminal_seq != state.state_seq
        ):
            event = (
                MissionEvent.AUTHORIZED
                if self._args.terminal_event == "authorized"
                else MissionEvent.UNAUTHORIZED
            )
            self._publish_event(event, "synthetic perception result")
            self._terminal_seq = state.state_seq


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--state-topic", default="/mission/state")
    parser.add_argument("--event-topic", default="/mission/event")
    parser.add_argument("--bbox-topic", default="/perception/selected_target_bbox")
    parser.add_argument("--target-id", type=int, default=1001)
    parser.add_argument("--image-width", type=int, default=1280)
    parser.add_argument("--image-height", type=int, default=1024)
    parser.add_argument("--x-min", type=int, default=500)
    parser.add_argument("--y-min", type=int, default=220)
    parser.add_argument("--x-max", type=int, default=780)
    parser.add_argument("--y-max", type=int, default=900)
    parser.add_argument("--confidence", type=float, default=0.99)
    parser.add_argument("--frame-id", default="camera_link")
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument(
        "--terminal-event",
        choices=("none", "authorized", "unauthorized"),
        default="none",
        help="optional synthetic result after real navigation reports arrival",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.rate <= 0.0:
        raise SystemExit("--rate must be positive")
    if not (0 < args.target_id):
        raise SystemExit("--target-id must be positive")
    if not (0 <= args.x_min < args.x_max <= args.image_width):
        raise SystemExit("bbox x coordinates are outside image bounds")
    if not (0 <= args.y_min < args.y_max <= args.image_height):
        raise SystemExit("bbox y coordinates are outside image bounds")

    rclpy.init()
    node = MockTargetPublisher(args)
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
