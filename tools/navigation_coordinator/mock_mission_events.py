#!/usr/bin/env python3
"""Publish only synthetic mission events for a real navigation test.

Use this with real tracking when face and voice are intentionally disabled. It
does not publish a target box. Tracking remains the only publisher of the
perception bbox; this helper only supplies the final business decision and,
optionally, the operator completion event.
"""

from __future__ import annotations

import argparse
import time

import rclpy
from dog_patrol_interfaces.msg import MissionEvent, MissionState
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


class MockMissionEvents(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("mock_mission_events")
        self._args = args
        self._state: MissionState | None = None
        self._verification_seq: int | None = None
        self._tracking_seq: int | None = None
        self._tracking_started: float | None = None
        self._completed_seq: int | None = None
        self._pub = self.create_publisher(MissionEvent, args.event_topic, event_qos())
        self.create_subscription(MissionState, args.state_topic, self._on_state, state_qos())
        self.create_timer(0.1, self._tick)

    def _on_state(self, msg: MissionState) -> None:
        if self._state is not None and msg.state_seq < self._state.state_seq:
            return
        self._state = msg
        if msg.state != MissionState.TRACK_INTRUDER:
            self._tracking_started = None
        elif self._tracking_seq != msg.state_seq:
            self._tracking_seq = msg.state_seq
            self._tracking_started = time.monotonic()

    def _publish(self, event: int, source: int, detail: str) -> None:
        if self._state is None:
            return
        msg = MissionEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.observed_state_seq = self._state.state_seq
        msg.target_id = self._state.target_id
        msg.source = source
        msg.event = event
        msg.detail = detail
        self._pub.publish(msg)

    def _tick(self) -> None:
        state = self._state
        if state is None or state.target_id <= 0:
            return

        if state.state == MissionState.VERIFY_IDENTITY and self._verification_seq != state.state_seq:
            event = (
                MissionEvent.AUTHORIZED
                if self._args.result == "authorized"
                else MissionEvent.UNAUTHORIZED
            )
            self._publish(event, MissionEvent.SOURCE_PERCEPTION, "synthetic test result")
            self._verification_seq = state.state_seq
            return

        if (
            state.state == MissionState.TRACK_INTRUDER
            and self._args.complete_after >= 0.0
            and self._tracking_started is not None
            and self._completed_seq != state.state_seq
            and time.monotonic() - self._tracking_started >= self._args.complete_after
        ):
            self._publish(
                MissionEvent.HANDLING_COMPLETE,
                MissionEvent.SOURCE_OPERATOR,
                "synthetic operator completion for navigation test",
            )
            self._completed_seq = state.state_seq


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--state-topic", default="/mission/state")
    parser.add_argument("--event-topic", default="/mission/event")
    parser.add_argument("--result", choices=("authorized", "unauthorized"), default="unauthorized")
    parser.add_argument(
        "--complete-after",
        type=float,
        default=-1.0,
        help="seconds after TRACK_INTRUDER before HANDLING_COMPLETE; negative disables it",
    )
    args = parser.parse_args()
    if args.complete_after < -1.0:
        raise SystemExit("--complete-after must be -1 or non-negative")

    rclpy.init()
    node = MockMissionEvents(args)
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
