#!/usr/bin/env python3
"""Select the one path source authorized by PatrolTaskState."""

from __future__ import annotations

import copy
import time

import rclpy
from dog_patrol_interfaces.msg import PatrolTaskState
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


def sequence_is_newer(candidate: int, current: int) -> bool:
    delta = (int(candidate) - int(current)) & 0xFFFFFFFF
    return 0 < delta < 0x80000000


def _home_pose_signature(message: PatrolTaskState):
    if not bool(getattr(message, "home_pose_valid", False)):
        return None
    pose = getattr(message, "home_pose", None)
    if pose is None:
        return None
    body = getattr(pose, "pose", pose)
    header = getattr(pose, "header", None)
    stamp = getattr(header, "stamp", None)
    return (
        str(getattr(header, "frame_id", "")),
        int(getattr(stamp, "sec", 0)),
        int(getattr(stamp, "nanosec", 0)),
        float(body.position.x),
        float(body.position.y),
        float(body.position.z),
        float(body.orientation.x),
        float(body.orientation.y),
        float(body.orientation.z),
        float(body.orientation.w),
    )


def path_source_for_task_state(message: PatrolTaskState) -> str:
    """Return the authorized source, failing closed on an invalid snapshot."""
    state_seq = int(message.state_seq)
    task_state = int(message.task_state)
    outcome = int(message.last_task_outcome)
    mode = int(message.navigation_mode)
    home_valid = bool(getattr(message, "home_pose_valid", task_state != PatrolTaskState.TASK_NO_TASK))

    valid_tasks = {
        PatrolTaskState.TASK_NO_TASK,
        PatrolTaskState.TASK_RUNNING,
        PatrolTaskState.TASK_PAUSED,
        PatrolTaskState.TASK_RETURNING,
    }
    valid_outcomes = {
        PatrolTaskState.TASK_OUTCOME_NONE,
        PatrolTaskState.TASK_OUTCOME_COMPLETED,
        PatrolTaskState.TASK_OUTCOME_RETURNED,
        PatrolTaskState.TASK_OUTCOME_TERMINATED,
        PatrolTaskState.TASK_OUTCOME_FAILED,
    }
    if state_seq == 0 or task_state not in valid_tasks or outcome not in valid_outcomes:
        return NO_SOURCE
    if task_state == PatrolTaskState.TASK_NO_TASK:
        return NO_SOURCE if mode == PatrolTaskState.NAVIGATION_STOP and not home_valid else NO_SOURCE
    if outcome != PatrolTaskState.TASK_OUTCOME_NONE or not home_valid:
        return NO_SOURCE
    if task_state == PatrolTaskState.TASK_PAUSED:
        return NO_SOURCE
    if task_state == PatrolTaskState.TASK_RETURNING:
        return MISSION_SOURCE if mode == PatrolTaskState.NAVIGATION_MISSION else NO_SOURCE
    if mode == PatrolTaskState.NAVIGATION_WAYPOINT:
        return WAYPOINT_SOURCE
    if mode == PatrolTaskState.NAVIGATION_MISSION:
        return MISSION_SOURCE
    return NO_SOURCE


class NavigationPathMux(Node):
    """Publish only paths created after the current parent-task barrier."""

    def __init__(self) -> None:
        super().__init__("navigation_path_mux")
        self.declare_parameter("waypoint_path_topic", "waypoint_global_path")
        self.declare_parameter("mission_path_topic", "mission_global_path")
        self.declare_parameter("output_path_topic", "global_path")
        self.declare_parameter("pure_pursuit_path_topic", "global_path")
        self.declare_parameter("task_state_topic", "/patrol_task/state")
        self.declare_parameter("state_timeout", 0.75)
        self.declare_parameter("max_future_path_skew", 0.25)

        value = self.get_parameter
        self.waypoint_path_topic = str(value("waypoint_path_topic").value)
        self.mission_path_topic = str(value("mission_path_topic").value)
        self.output_path_topic = str(value("output_path_topic").value)
        self.pure_pursuit_path_topic = str(value("pure_pursuit_path_topic").value)
        self.task_state_topic = str(value("task_state_topic").value)
        self.state_timeout = max(0.1, float(value("state_timeout").value))
        self.max_future_path_skew = max(0.0, float(value("max_future_path_skew").value))

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

        self._active_source = NO_SOURCE
        self._task_state_seen = False
        self._last_state_seq = 0
        self._state_signature = None
        self._state_conflicted = False
        self._barrier_stamp_ns = 0
        self._last_state_monotonic = None
        self._state_timed_out = False

        self._path_pub = self.create_publisher(Path, self.output_path_topic, path_qos)
        same_output = self.output_path_topic.lstrip("/") == self.pure_pursuit_path_topic.lstrip("/")
        self._pure_pursuit_path_pub = (
            None
            if same_output
            else self.create_publisher(Path, self.pure_pursuit_path_topic, path_qos)
        )
        self.create_subscription(Path, self.waypoint_path_topic, self._on_waypoint_path, path_qos)
        self.create_subscription(Path, self.mission_path_topic, self._on_mission_path, path_qos)
        self.create_subscription(
            PatrolTaskState,
            self.task_state_topic,
            self._on_task_state,
            state_qos,
        )
        self.create_timer(min(0.1, self.state_timeout / 2.0), self._check_state_timeout)
        self.get_logger().info(
            "navigation path mux ready: "
            f"waypoint={self.waypoint_path_topic} mission={self.mission_path_topic} "
            f"task_state={self.task_state_topic} output={self.output_path_topic}"
        )

    def _on_waypoint_path(self, msg: Path) -> None:
        self._on_source_path(WAYPOINT_SOURCE, msg)

    def _on_mission_path(self, msg: Path) -> None:
        self._on_source_path(MISSION_SOURCE, msg)

    def _on_source_path(self, source: str, msg: Path) -> None:
        if not self._task_state_seen or self._state_timed_out or source != self._active_source:
            return
        if not msg.poses:
            self._publish(msg)
            return
        if self._path_is_after_barrier(msg):
            self._publish(msg)

    def _path_is_after_barrier(self, msg: Path) -> bool:
        stamp_ns = self._stamp_ns(msg)
        if stamp_ns <= 0 or stamp_ns <= self._barrier_stamp_ns:
            return False
        get_clock = getattr(self, "get_clock", None)
        if callable(get_clock) and hasattr(self, "max_future_path_skew"):
            now_ns = int(get_clock().now().nanoseconds)
            future_limit = now_ns + int(self.max_future_path_skew * 1_000_000_000)
            if stamp_ns > future_limit:
                return False
        return True

    def _on_task_state(self, msg: PatrolTaskState) -> None:
        state_seq = int(msg.state_seq)
        if state_seq == 0:
            self.get_logger().warning("ignoring patrol task state seq=0")
            return
        signature = (
            int(msg.task_state),
            int(msg.last_task_outcome),
            int(msg.navigation_mode),
            bool(getattr(msg, "home_pose_valid", False)),
            _home_pose_signature(msg),
        )
        if self._task_state_seen:
            if state_seq == self._last_state_seq:
                self._refresh_heartbeat()
                if self._state_conflicted:
                    return
                if self._state_signature is not None and signature != self._state_signature:
                    self._state_conflicted = True
                    self._active_source = NO_SOURCE
                    self._publish_empty()
                    self.get_logger().error(
                        f"conflicting patrol task state fields at seq={state_seq}; stopping"
                    )
                    return
                if not self._state_timed_out:
                    return
            elif not sequence_is_newer(state_seq, self._last_state_seq):
                self.get_logger().warning(f"ignoring stale patrol task state seq={state_seq}")
                return
            else:
                self._refresh_heartbeat()
        else:
            self._refresh_heartbeat()

        previous_source = self._active_source
        self._task_state_seen = True
        self._last_state_seq = state_seq
        self._state_signature = signature
        self._state_conflicted = False
        self._barrier_stamp_ns = self._stamp_ns(msg)
        self._active_source = path_source_for_task_state(msg)
        self._state_timed_out = False
        self._publish_empty()
        self.get_logger().info(
            f"path source {previous_source} -> {self._active_source}, task seq={state_seq}"
        )

    def _refresh_heartbeat(self) -> None:
        self._last_state_monotonic = time.monotonic()

    def _check_state_timeout(self) -> None:
        if self._last_state_monotonic is None or self._state_timed_out:
            return
        if time.monotonic() - self._last_state_monotonic <= self.state_timeout:
            return
        self._state_timed_out = True
        self._active_source = NO_SOURCE
        self._publish_empty()
        self.get_logger().warning("patrol task state heartbeat timed out; path mux stopped")

    @staticmethod
    def _stamp_ns(message) -> int:
        stamp = message.header.stamp
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _publish(self, msg: Path) -> None:
        selected = copy.deepcopy(msg)
        self._path_pub.publish(selected)
        if self._pure_pursuit_path_pub is not None:
            self._pure_pursuit_path_pub.publish(copy.deepcopy(selected))

    def _publish_empty(self) -> None:
        empty = Path()
        empty.header.stamp = self.get_clock().now().to_msg()
        self._publish(empty)


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
