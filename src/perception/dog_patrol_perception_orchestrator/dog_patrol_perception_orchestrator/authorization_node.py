#!/usr/bin/env python3

import threading
from typing import Optional, Sequence

import rclpy
from dog_patrol_interfaces.msg import MissionEvent, MissionState
from dog_patrol_perception_interfaces.msg import AuthorizationEvidence
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from dog_patrol_perception_orchestrator.authorization import (
    AuthorizationCoordinator,
    AuthorizationResult,
    AuthorizationSession,
)


class PerceptionAuthorizationNode(Node):
    def __init__(self, *, parameter_overrides: Optional[Sequence[Parameter]] = None) -> None:
        super().__init__("perception_authorization", parameter_overrides=parameter_overrides)
        self.declare_parameter("mission_state_topic", "/mission/state")
        self.declare_parameter("mission_event_topic", "/mission/event")
        self.declare_parameter(
            "authorization_evidence_topic", "/perception/authorization_evidence"
        )
        self.declare_parameter("required_not_passed", 2)

        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        event_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._lock = threading.RLock()
        self._coordinator = AuthorizationCoordinator(
            int(self.get_parameter("required_not_passed").value)
        )
        self._mission_session: Optional[AuthorizationSession] = None
        self._mission_started_at_ns = 0
        self._event_pub = self.create_publisher(
            MissionEvent, str(self.get_parameter("mission_event_topic").value), event_qos
        )
        self._state_sub = self.create_subscription(
            MissionState,
            str(self.get_parameter("mission_state_topic").value),
            self._on_mission,
            state_qos,
        )
        self._evidence_sub = self.create_subscription(
            AuthorizationEvidence,
            str(self.get_parameter("authorization_evidence_topic").value),
            self._on_evidence,
            event_qos,
        )

    def _on_mission(self, msg: MissionState) -> None:
        with self._lock:
            session = self._session_from_mission(msg)
            if session == self._mission_session:
                return
            self._mission_session = session
            if session is None:
                self._mission_started_at_ns = 0
                self._coordinator.reset()
            else:
                self._mission_started_at_ns = self._stamp_ns(msg.header.stamp)
                self._coordinator.start(session)

    @staticmethod
    def _session_from_mission(msg: MissionState) -> Optional[AuthorizationSession]:
        if (
            int(msg.state) != MissionState.VERIFY_IDENTITY
            or bool(msg.blocked)
            or int(msg.target_id) <= 0
        ):
            return None
        return AuthorizationSession(int(msg.state_seq), int(msg.target_id))

    def _on_evidence(self, msg: AuthorizationEvidence) -> None:
        result = self._parse_result(int(msg.result))
        if result is None:
            self.get_logger().warning(
                f"ignoring unknown authorization result {int(msg.result)}"
            )
            return

        session = AuthorizationSession(int(msg.observed_state_seq), int(msg.target_id))
        with self._lock:
            if session != self._mission_session:
                return
            evidence_stamp_ns = self._stamp_ns(msg.header.stamp)
            if (
                self._mission_started_at_ns > 0
                and evidence_stamp_ns < self._mission_started_at_ns
            ):
                return
            outcome = self._coordinator.record(session, result)
            if outcome is None or outcome.result is AuthorizationResult.CANCELLED:
                return

            event = MissionEvent()
            event.header.stamp = self.get_clock().now().to_msg()
            event.observed_state_seq = outcome.session.observed_state_seq
            event.target_id = outcome.session.target_id
            event.source = MissionEvent.SOURCE_PERCEPTION
            event.event = self._event_for_result(outcome.result)
            provider = str(msg.provider).strip()
            detail = str(msg.detail).strip()
            event.detail = ": ".join(part for part in (provider, detail) if part)
            self._event_pub.publish(event)

    @staticmethod
    def _stamp_ns(stamp) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    @staticmethod
    def _parse_result(value: int) -> Optional[AuthorizationResult]:
        return {
            AuthorizationEvidence.PASSED: AuthorizationResult.PASSED,
            AuthorizationEvidence.NOT_PASSED: AuthorizationResult.NOT_PASSED,
            AuthorizationEvidence.ERROR: AuthorizationResult.ERROR,
            AuthorizationEvidence.CANCELLED: AuthorizationResult.CANCELLED,
        }.get(value)

    @staticmethod
    def _event_for_result(result: AuthorizationResult) -> int:
        return {
            AuthorizationResult.PASSED: MissionEvent.AUTHORIZED,
            AuthorizationResult.NOT_PASSED: MissionEvent.UNAUTHORIZED,
            AuthorizationResult.ERROR: MissionEvent.EXECUTION_ERROR,
        }[result]


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PerceptionAuthorizationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
