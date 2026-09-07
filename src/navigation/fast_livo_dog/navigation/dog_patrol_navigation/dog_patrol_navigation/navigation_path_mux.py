#!/usr/bin/env python3
"""Select exactly one navigation path source for the downstream control chain."""

from __future__ import annotations

import copy

import rclpy
from dog_patrol_interfaces.msg import MissionState
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


WAYPOINT_SOURCE = "waypoint"
MISSION_SOURCE = "mission"
NO_SOURCE = "none"


def path_source_for_mission_state(state: int) -> str:
    """Return the path source allowed to drive the control chain."""
    if int(state) in {
        MissionState.APPROACH_TARGET,
        MissionState.VERIFY_IDENTITY,
        MissionState.TRACK_INTRUDER,
        MissionState.RECOVER_PATROL,
    }:
        return MISSION_SOURCE
    if int(state) in {
        MissionState.PATROL,
        MissionState.CONFIRM_TARGET,
    }:
        return WAYPOINT_SOURCE
    return NO_SOURCE


class NavigationPathMux(Node):
    """Publish one state-selected path to Pure Pursuit and the RL planner."""

    def __init__(self) -> None:
        super().__init__("navigation_path_mux")

        self.declare_parameter("waypoint_path_topic", "waypoint_global_path")
        self.declare_parameter("mission_path_topic", "mission_global_path")
        self.declare_parameter("output_path_topic", "global_path")
        self.declare_parameter("pure_pursuit_path_topic", "global_path")
        self.declare_parameter("mission_state_topic", "/mission/state")

        self.waypoint_path_topic = str(self.get_parameter("waypoint_path_topic").value)
        self.mission_path_topic = str(self.get_parameter("mission_path_topic").value)
        self.output_path_topic = str(self.get_parameter("output_path_topic").value)
        self.pure_pursuit_path_topic = str(
            self.get_parameter("pure_pursuit_path_topic").value
        )
        self.mission_state_topic = str(self.get_parameter("mission_state_topic").value)
        path_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
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

        # The task-navigation mux must fail closed until the authoritative
        # mission state arrives. A waypoint path can otherwise win a
        # cross-topic startup race while the supervisor is still in STARTUP.
        self._active_source = NO_SOURCE
        self._mission_state_seen = False
        self._last_state_seq = -1
        self._wait_for_fresh_waypoint_path = False
        self._waypoint_min_stamp_ns = 0
        self._path_pub = self.create_publisher(Path, self.output_path_topic, path_qos)
        same_output = (
            self.output_path_topic.lstrip("/")
            == self.pure_pursuit_path_topic.lstrip("/")
        )
        self._pure_pursuit_path_pub = (
            None
            if same_output
            else self.create_publisher(Path, self.pure_pursuit_path_topic, path_qos)
        )

        self.create_subscription(
            Path, self.waypoint_path_topic, self._on_waypoint_path, path_qos
        )
        self.create_subscription(
            Path, self.mission_path_topic, self._on_mission_path, path_qos
        )
        self.create_subscription(
            MissionState,
            self.mission_state_topic,
            self._on_mission_state,
            state_qos,
        )
        self.get_logger().info(
            "navigation path mux ready: "
            f"waypoint={self.waypoint_path_topic} "
            f"mission={self.mission_path_topic} "
            f"output={self.output_path_topic}"
        )

    def _on_waypoint_path(self, msg: Path) -> None:
        if self._active_source == WAYPOINT_SOURCE:
            if self._wait_for_fresh_waypoint_path and not msg.poses:
                self._publish(msg)
                return
            if self._wait_for_fresh_waypoint_path:
                path_stamp_ns = NavigationPathMux._stamp_ns(msg)
                if (
                    self._waypoint_min_stamp_ns > 0
                    and path_stamp_ns > 0
                    and path_stamp_ns <= self._waypoint_min_stamp_ns
                ):
                    return
            self._publish(msg)
            if self._wait_for_fresh_waypoint_path and msg.poses:
                self._wait_for_fresh_waypoint_path = False

    def _on_mission_path(self, msg: Path) -> None:
        if self._active_source == MISSION_SOURCE:
            self._publish(msg)

    def _on_mission_state(self, msg: MissionState) -> None:
        state_seq = int(msg.state_seq)
        if self._mission_state_seen and state_seq < self._last_state_seq:
            self.get_logger().warning(
                f"ignoring stale mission state seq={state_seq}"
            )
            return

        source = path_source_for_mission_state(int(msg.state))
        previous_source = self._active_source
        source_changed = (
            not self._mission_state_seen or source != self._active_source
        )
        self._mission_state_seen = True
        self._last_state_seq = state_seq
        if not source_changed:
            return

        self._active_source = source
        # Cross-topic delivery order is not defined. Never replay a waypoint
        # path cached before a mission-path state. Wait for a waypoint message
        # delivered after PATROL becomes authoritative; the waypoint publisher
        # emits one on resume and continues periodic replanning.
        self._wait_for_fresh_waypoint_path = (
            source == WAYPOINT_SOURCE and previous_source != WAYPOINT_SOURCE
        )
        self._waypoint_min_stamp_ns = (
            NavigationPathMux._stamp_ns(msg)
            if self._wait_for_fresh_waypoint_path
            else 0
        )
        self._publish_empty()
        self.get_logger().info(
            f"path source {previous_source} -> {source}, mission seq={state_seq}"
        )

    @staticmethod
    def _stamp_ns(message: Path) -> int:
        stamp = message.header.stamp
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _publish(self, msg: Path) -> None:
        selected = copy.deepcopy(msg)
        self._path_pub.publish(selected)
        if self._pure_pursuit_path_pub is not None:
            self._pure_pursuit_path_pub.publish(copy.deepcopy(selected))

    def _publish_empty(self) -> None:
        self._publish(Path())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavigationPathMux()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
