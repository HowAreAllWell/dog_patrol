import time

import pytest
import rclpy
from dog_patrol_interfaces.msg import MissionEvent, MissionState
from dog_patrol_perception_interfaces.msg import (
    AuthorizationCommand,
    AuthorizationEvidence,
)
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from dog_patrol_perception_orchestrator.authorization_node import (
    PerceptionAuthorizationNode,
)


def qos(*, transient=False, depth=10):
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=(
            DurabilityPolicy.TRANSIENT_LOCAL
            if transient
            else DurabilityPolicy.VOLATILE
        ),
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


class AuthorizationHarness:
    def __init__(self):
        suffix = f"t{time.monotonic_ns()}"
        self.state_topic = f"/authorization/{suffix}/mission/state"
        self.event_topic = f"/authorization/{suffix}/mission/event"
        self.evidence_topic = f"/authorization/{suffix}/evidence"
        self.command_topic = f"/authorization/{suffix}/command"
        self.source = Node(f"authorization_source_{suffix}")
        self.probe = Node(f"authorization_probe_{suffix}")
        self.node = PerceptionAuthorizationNode(
            parameter_overrides=[
                Parameter("mission_state_topic", value=self.state_topic),
                Parameter("mission_event_topic", value=self.event_topic),
                Parameter("authorization_evidence_topic", value=self.evidence_topic),
                Parameter("authorization_command_topic", value=self.command_topic),
            ]
        )
        self.state_pub = self.source.create_publisher(
            MissionState, self.state_topic, qos(transient=True, depth=1)
        )
        self.evidence_pub = self.source.create_publisher(
            AuthorizationEvidence, self.evidence_topic, qos()
        )
        self.commands = []
        self.events = []
        self.command_sub = self.probe.create_subscription(
            AuthorizationCommand,
            self.command_topic,
            self.commands.append,
            qos(),
        )
        self.event_sub = self.probe.create_subscription(
            MissionEvent, self.event_topic, self.events.append, qos()
        )
        self.executor = SingleThreadedExecutor()
        for node in (self.source, self.probe, self.node):
            self.executor.add_node(node)

    def close(self):
        self.executor.shutdown()
        for node in (self.node, self.probe, self.source):
            node.destroy_node()

    def spin_until(self, predicate, timeout=3.0):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.05)
        return predicate()

    def publish_state(self, seq, target, *, state=MissionState.VERIFY_IDENTITY):
        msg = MissionState()
        msg.header.stamp.sec = seq
        msg.state_seq = seq
        msg.state = state
        msg.target_id = target
        self.state_pub.publish(msg)
        for _ in range(5):
            self.executor.spin_once(timeout_sec=0.05)

    def publish_evidence(
        self,
        seq,
        target,
        stage,
        provider,
        result,
        detail="result",
        *,
        stamp_sec=None,
    ):
        msg = AuthorizationEvidence()
        msg.header.stamp.sec = seq if stamp_sec is None else stamp_sec
        msg.observed_state_seq = seq
        msg.target_id = target
        msg.stage = stage
        msg.provider = provider
        msg.result = result
        msg.detail = detail
        self.evidence_pub.publish(msg)
        for _ in range(5):
            self.executor.spin_once(timeout_sec=0.05)


@pytest.fixture
def harness():
    rclpy.init()
    value = AuthorizationHarness()
    try:
        yield value
    finally:
        value.close()
        if rclpy.ok():
            rclpy.shutdown()


def command_keys(messages):
    return [
        (message.observed_state_seq, message.target_id, message.stage)
        for message in messages
    ]


def test_two_round_internal_flow_only_rejects_after_both_second_round_miss(
    harness,
):
    harness.publish_state(17, 42)
    assert command_keys(harness.commands) == [
        (17, 42, AuthorizationCommand.INITIAL_FACE)
    ]

    harness.publish_evidence(
        17,
        42,
        AuthorizationEvidence.INITIAL_FACE,
        "face",
        AuthorizationEvidence.NOT_PASSED,
    )
    assert command_keys(harness.commands)[-1] == (
        17,
        42,
        AuthorizationCommand.DUAL_FIRST,
    )

    for provider in ("face", "voice"):
        harness.publish_evidence(
            17,
            42,
            AuthorizationEvidence.DUAL_FIRST,
            provider,
            AuthorizationEvidence.NOT_PASSED,
        )
    assert not harness.events
    assert command_keys(harness.commands)[-1] == (
        17,
        42,
        AuthorizationCommand.DUAL_SECOND,
    )

    for provider in ("voice", "face"):
        harness.publish_evidence(
            17,
            42,
            AuthorizationEvidence.DUAL_SECOND,
            provider,
            AuthorizationEvidence.NOT_PASSED,
        )
    assert harness.spin_until(lambda: len(harness.events) == 1)
    assert command_keys(harness.commands)[-1] == (
        17,
        42,
        AuthorizationCommand.CANCEL,
    )
    assert harness.events[0].event == MissionEvent.UNAUTHORIZED


@pytest.mark.parametrize(
    ("stage", "provider"),
    [
        (AuthorizationEvidence.INITIAL_FACE, "face"),
        (AuthorizationEvidence.DUAL_FIRST, "face"),
        (AuthorizationEvidence.DUAL_FIRST, "voice"),
    ],
)
def test_any_expected_pass_authorizes_and_cancels_other_work(
    harness, stage, provider
):
    harness.publish_state(18, 43)
    if stage == AuthorizationEvidence.DUAL_FIRST:
        harness.publish_evidence(
            18,
            43,
            AuthorizationEvidence.INITIAL_FACE,
            "face",
            AuthorizationEvidence.NOT_PASSED,
        )

    harness.publish_evidence(
        18, 43, stage, provider, AuthorizationEvidence.PASSED
    )

    assert harness.spin_until(lambda: len(harness.events) == 1)
    assert harness.events[0].event == MissionEvent.AUTHORIZED
    assert command_keys(harness.commands)[-1] == (
        18,
        43,
        AuthorizationCommand.CANCEL,
    )


def test_stale_wrong_stage_and_wrong_provider_evidence_are_ignored(harness):
    harness.publish_state(19, 44)
    for seq, stage, provider in (
        (18, AuthorizationEvidence.INITIAL_FACE, "face"),
        (19, AuthorizationEvidence.DUAL_FIRST, "face"),
        (19, AuthorizationEvidence.INITIAL_FACE, "voice"),
    ):
        harness.publish_evidence(
            seq, 44, stage, provider, AuthorizationEvidence.PASSED
        )

    assert not harness.events
    assert command_keys(harness.commands) == [
        (19, 44, AuthorizationCommand.INITIAL_FACE)
    ]


def test_error_cancels_round_and_maps_to_execution_error(harness):
    harness.publish_state(20, 45)
    harness.publish_evidence(
        20,
        45,
        AuthorizationEvidence.INITIAL_FACE,
        "face",
        AuthorizationEvidence.ERROR,
        "model failed",
    )

    assert harness.spin_until(lambda: len(harness.events) == 1)
    assert harness.events[0].event == MissionEvent.EXECUTION_ERROR
    assert harness.events[0].detail == "face: model failed"
    assert command_keys(harness.commands)[-1] == (
        20,
        45,
        AuthorizationCommand.CANCEL,
    )


def test_state_replacement_cancels_active_work(harness):
    harness.publish_state(21, 46)
    harness.publish_state(22, 47)
    harness.publish_state(22, 47)

    expected_commands = [
        (21, 46, AuthorizationCommand.INITIAL_FACE),
        (21, 46, AuthorizationCommand.CANCEL),
        (22, 47, AuthorizationCommand.INITIAL_FACE),
    ]
    # A repeated VERIFY snapshot is a heartbeat, not a cancellation request.
    assert command_keys(harness.commands) == expected_commands
    assert not harness.events

    harness.publish_evidence(
        21, 46, AuthorizationEvidence.INITIAL_FACE, "face", AuthorizationEvidence.PASSED
    )
    assert command_keys(harness.commands) == expected_commands
    assert not harness.events

    # Leaving VERIFY cancels the current work exactly once.
    harness.publish_state(23, 47, state=MissionState.RECOVER_PATROL)
    harness.publish_state(23, 47, state=MissionState.RECOVER_PATROL)
    expected_commands.append((22, 47, AuthorizationCommand.CANCEL))
    assert command_keys(harness.commands) == expected_commands

    harness.publish_evidence(
        22, 47, AuthorizationEvidence.INITIAL_FACE, "face", AuthorizationEvidence.PASSED
    )
    assert command_keys(harness.commands) == expected_commands
    assert not harness.events
