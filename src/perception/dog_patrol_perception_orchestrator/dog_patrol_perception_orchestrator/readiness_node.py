#!/usr/bin/env python3

import threading
import time
from typing import Optional, Sequence

import rclpy
from dog_patrol_interfaces.msg import MissionEvent, MissionState
from dog_patrol_perception_interfaces.msg import CapabilityStatus
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from dog_patrol_perception_orchestrator.readiness import CapabilitySample, ReadinessCoordinator


class PerceptionReadinessNode(Node):
    REQUIRED_CAPABILITIES = ("detection_tracking", "face", "voice", "reid")

    def __init__(self, *, parameter_overrides: Optional[Sequence[Parameter]] = None) -> None:
        super().__init__("perception_readiness", parameter_overrides=parameter_overrides)
        self.declare_parameter("mission_state_topic", "/mission/state")
        self.declare_parameter("mission_event_topic", "/mission/event")
        self.declare_parameter("capability_status_topic", "/perception/capability_status")
        self.declare_parameter("state_timeout", 0.75)

        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        capability_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=16,
        )
        event_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._lock = threading.RLock()
        self._state_timeout = max(
            0.1, float(self.get_parameter("state_timeout").value)
        )
        self._last_state_monotonic = None
        self._state_timed_out = False
        self._coordinator = ReadinessCoordinator(
            self.REQUIRED_CAPABILITIES, startup_state=MissionState.STARTUP
        )
        self._event_pub = self.create_publisher(
            MissionEvent, str(self.get_parameter("mission_event_topic").value), event_qos
        )
        self._state_sub = self.create_subscription(
            MissionState,
            str(self.get_parameter("mission_state_topic").value),
            self._on_mission,
            state_qos,
        )
        self._capability_sub = self.create_subscription(
            CapabilityStatus,
            str(self.get_parameter("capability_status_topic").value),
            self._on_capability,
            capability_qos,
        )
        self._state_watchdog = self.create_timer(
            min(0.1, self._state_timeout / 2.0), self._check_state_timeout
        )

    def _on_mission(self, msg: MissionState) -> None:
        with self._lock:
            if int(msg.state_seq) <= 0:
                self._coordinator.expire_mission()
                return
            self._last_state_monotonic = time.monotonic()
            self._state_timed_out = False
            ready_startup_seq = self._coordinator.observe_mission(
                int(msg.state_seq),
                int(msg.state),
                int(msg.target_id),
                int(msg.handled_target_id),
            )
            self._publish_if_ready(ready_startup_seq)

    def _check_state_timeout(self) -> None:
        with self._lock:
            if (
                self._last_state_monotonic is None
                or self._state_timed_out
                or time.monotonic() - self._last_state_monotonic
                <= self._state_timeout
            ):
                return
            self._state_timed_out = True
            self._coordinator.expire_mission()
        self.get_logger().warning(
            "mission state heartbeat timed out; aggregate readiness invalidated"
        )

    def _on_capability(self, msg: CapabilityStatus) -> None:
        with self._lock:
            ready_startup_seq = self._coordinator.observe_capability(
                CapabilitySample(
                    capability=str(msg.capability),
                    status=int(msg.status),
                    observed_startup_state_seq=int(msg.observed_startup_state_seq),
                    diagnostic=str(msg.diagnostic),
                )
            )
            self._publish_if_ready(ready_startup_seq)

    def _publish_if_ready(self, ready_startup_seq: Optional[int]) -> None:
        if ready_startup_seq is None:
            return
        msg = MissionEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.observed_state_seq = ready_startup_seq
        msg.target_id = 0
        msg.source = MissionEvent.SOURCE_PERCEPTION
        msg.event = MissionEvent.READY
        msg.detail = "all required perception capabilities ready"
        self._event_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PerceptionReadinessNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
