#!/usr/bin/env python3

import threading
import time
from typing import Optional, Sequence

import rclpy
from dog_patrol_interfaces.msg import MissionEvent, MissionState
from dog_patrol_perception_interfaces.msg import (
    AuthorizationCommand,
    AuthorizationEvidence,
)
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from dog_patrol_perception_orchestrator.authorization import (
    AuthorizationCoordinator,
    AuthorizationObservation,
    AuthorizationProvider,
    AuthorizationResult,
    AuthorizationSession,
    AuthorizationStage,
    AuthorizationTransition,
)

UINT32_MASK = (1 << 32) - 1
UINT32_HALF_RANGE = 1 << 31


def _is_newer_state_seq(candidate: int, current: int) -> bool:
    delta = (int(candidate) - int(current)) & UINT32_MASK
    return 0 < delta < UINT32_HALF_RANGE


class PerceptionAuthorizationNode(Node):
    def __init__(self, *, parameter_overrides: Optional[Sequence[Parameter]] = None) -> None:
        super().__init__("perception_authorization", parameter_overrides=parameter_overrides)
        self.declare_parameter("mission_state_topic", "/mission/state")
        self.declare_parameter("mission_event_topic", "/mission/event")
        self.declare_parameter(
            "authorization_evidence_topic", "/perception/authorization_evidence"
        )
        self.declare_parameter(
            "authorization_command_topic", "/perception/authorization_command"
        )
        self.declare_parameter("state_timeout", 0.75)
        self.declare_parameter("voice_prompt_count", 2)

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
            voice_prompt_count=self.get_parameter("voice_prompt_count").value
        )
        self.add_on_set_parameters_callback(self._on_policy_parameters)
        self._mission_session: Optional[AuthorizationSession] = None
        self._mission_state_seq: Optional[int] = None
        self._mission_snapshot_session: Optional[AuthorizationSession] = None
        self._mission_state_signature = None
        self._mission_state_conflicted = False
        self._state_timeout = max(
            0.1, float(self.get_parameter("state_timeout").value)
        )
        self._last_state_monotonic = None
        self._state_timed_out = False
        self._mission_started_at_ns = 0
        self._flow_started_monotonic_ns = 0
        self._stage_started_monotonic_ns = 0
        self._event_pub = self.create_publisher(
            MissionEvent, str(self.get_parameter("mission_event_topic").value), event_qos
        )
        self._command_pub = self.create_publisher(
            AuthorizationCommand,
            str(self.get_parameter("authorization_command_topic").value),
            event_qos,
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
        self._state_watchdog = self.create_timer(
            min(0.1, self._state_timeout / 2.0), self._check_state_timeout
        )

    def _on_policy_parameters(self, parameters: Sequence[Parameter]) -> SetParametersResult:
        try:
            with self._lock:
                for parameter in parameters:
                    if parameter.name == "voice_prompt_count":
                        self._coordinator.set_voice_prompt_count(parameter.value)
        except (TypeError, ValueError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        return SetParametersResult(successful=True)

    def _on_mission(self, msg: MissionState) -> None:
        with self._lock:
            state_seq = int(msg.state_seq) & UINT32_MASK
            if state_seq == 0:
                self._state_timed_out = True
                self._cancel_session()
                return
            session = self._session_from_mission(msg)
            signature = (
                int(msg.state),
                int(msg.target_id),
                int(msg.handled_target_id),
            )
            self._last_state_monotonic = time.monotonic()
            if self._mission_state_seq is not None:
                if state_seq == self._mission_state_seq:
                    if self._mission_state_conflicted:
                        return
                    if signature != self._mission_state_signature:
                        self._mission_state_conflicted = True
                        self._cancel_session()
                        self._state_timed_out = False
                        return
                    if self._state_timed_out:
                        self._state_timed_out = False
                        self._mission_session = session
                        if session is not None:
                            self._mission_started_at_ns = self._stamp_ns(msg.header.stamp)
                            self._flow_started_monotonic_ns = time.monotonic_ns()
                        self._publish_transition(self._coordinator.observe(session))
                    return
                if not _is_newer_state_seq(state_seq, self._mission_state_seq):
                    return
            self._mission_state_seq = state_seq
            self._mission_snapshot_session = session
            self._mission_state_signature = signature
            self._mission_state_conflicted = False
            self._state_timed_out = False
            if session == self._mission_session:
                return
            self._mission_session = session
            if session is None:
                self._mission_started_at_ns = 0
                self._flow_started_monotonic_ns = 0
                self._stage_started_monotonic_ns = 0
            else:
                self._mission_started_at_ns = self._stamp_ns(msg.header.stamp)
                self._flow_started_monotonic_ns = time.monotonic_ns()
            self._publish_transition(self._coordinator.observe(session))

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
            self._cancel_session()
        self.get_logger().warning(
            "mission state heartbeat timed out; authorization session cancelled"
        )

    def _cancel_session(self) -> None:
        if self._mission_session is None:
            return
        self._mission_session = None
        self._mission_started_at_ns = 0
        self._flow_started_monotonic_ns = 0
        self._stage_started_monotonic_ns = 0
        self._publish_transition(self._coordinator.observe(None))

    @staticmethod
    def _session_from_mission(msg: MissionState) -> Optional[AuthorizationSession]:
        if (
            int(msg.state_seq) <= 0
            or int(msg.state_seq) > UINT32_MASK
            or int(msg.state) != MissionState.VERIFY_IDENTITY
            or int(msg.target_id) <= 0
        ):
            return None
        return AuthorizationSession(int(msg.state_seq) & UINT32_MASK, int(msg.target_id))

    def _on_evidence(self, msg: AuthorizationEvidence) -> None:
        result = self._parse_result(int(msg.result))
        if result is None:
            self.get_logger().warning(
                f"ignoring unknown authorization result {int(msg.result)}"
            )
            return
        stage = self._parse_stage(int(msg.stage))
        provider = self._parse_provider(str(msg.provider))
        if stage is None or provider is None:
            self.get_logger().warning(
                "ignoring authorization evidence with unknown stage/provider"
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
            transition = self._coordinator.record_observation(
                AuthorizationObservation(session, stage, provider, result, int(msg.round_number))
            )
            stage_elapsed_ms = self._elapsed_ms(self._stage_started_monotonic_ns)
            self.get_logger().info(
                "authorization_evidence "
                f"state_seq={session.observed_state_seq} target={session.target_id} "
                f"stage={stage.value} provider={provider.value} "
                f"result={result.value} detail={str(msg.detail).strip()!r} "
                f"stage_elapsed_ms={stage_elapsed_ms:.3f}"
            )
            self._publish_transition(transition, evidence=msg)

    def _publish_transition(
        self,
        transition: AuthorizationTransition,
        *,
        evidence: Optional[AuthorizationEvidence] = None,
    ) -> None:
        for command in transition.commands:
            msg = AuthorizationCommand()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.observed_state_seq = command.session.observed_state_seq
            msg.target_id = command.session.target_id
            msg.stage = self._command_stage_value(command.stage)
            msg.round_number = command.round_number
            self._command_pub.publish(msg)
            if command.stage is not AuthorizationStage.CANCEL:
                self._stage_started_monotonic_ns = time.monotonic_ns()
            self.get_logger().info(
                "authorization_command "
                f"state_seq={command.session.observed_state_seq} "
                f"target={command.session.target_id} stage={command.stage.value}"
            )

        outcome = transition.outcome
        if outcome is None or outcome.result is AuthorizationResult.CANCELLED:
            return

        event = MissionEvent()
        event.header.stamp = self.get_clock().now().to_msg()
        event.observed_state_seq = outcome.session.observed_state_seq
        event.target_id = outcome.session.target_id
        event.source = MissionEvent.SOURCE_PERCEPTION
        event.event = self._event_for_result(outcome.result)
        provider_name = "" if evidence is None else str(evidence.provider).strip()
        detail = "" if evidence is None else str(evidence.detail).strip()
        event.detail = ": ".join(
            part for part in (provider_name, detail) if part
        )
        self._event_pub.publish(event)
        total_elapsed_ms = self._elapsed_ms(self._flow_started_monotonic_ns)
        self.get_logger().info(
            "authorization_outcome "
            f"state_seq={outcome.session.observed_state_seq} "
            f"target={outcome.session.target_id} result={outcome.result.value} "
            f"total_elapsed_ms={total_elapsed_ms:.3f}"
        )

    @staticmethod
    def _elapsed_ms(started_ns: int) -> float:
        if started_ns <= 0:
            return 0.0
        return (time.monotonic_ns() - started_ns) / 1_000_000.0

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
    def _parse_stage(value: int) -> Optional[AuthorizationStage]:
        return {
            AuthorizationEvidence.INITIAL_FACE: AuthorizationStage.INITIAL_FACE,
            AuthorizationEvidence.DUAL_FIRST: AuthorizationStage.DUAL_FIRST,
            AuthorizationEvidence.DUAL_SECOND: AuthorizationStage.DUAL_SECOND,
        }.get(value)

    @staticmethod
    def _parse_provider(value: str) -> Optional[AuthorizationProvider]:
        return {
            AuthorizationProvider.FACE.value: AuthorizationProvider.FACE,
            AuthorizationProvider.VOICE.value: AuthorizationProvider.VOICE,
        }.get(value.strip().lower())

    @staticmethod
    def _command_stage_value(stage: AuthorizationStage) -> int:
        return {
            AuthorizationStage.INITIAL_FACE: AuthorizationCommand.INITIAL_FACE,
            AuthorizationStage.DUAL_FIRST: AuthorizationCommand.DUAL_FIRST,
            AuthorizationStage.DUAL_SECOND: AuthorizationCommand.DUAL_SECOND,
            AuthorizationStage.CANCEL: AuthorizationCommand.CANCEL,
        }[stage]

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
