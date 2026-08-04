import time

import rclpy
from dog_patrol_interfaces.msg import MissionEvent, MissionState
from dog_patrol_perception_interfaces.msg import AuthorizationEvidence
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


def spin_until(executor, predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
    return predicate()


def test_external_interface_maps_only_current_verify_session_results():
    rclpy.init()
    suffix = f"t{time.monotonic_ns()}"
    state_topic = f"/issue3/{suffix}/mission/state"
    event_topic = f"/issue3/{suffix}/mission/event"
    evidence_topic = f"/issue3/{suffix}/perception/authorization_evidence"
    provider = Node(f"issue3_authorization_provider_{suffix}")
    probe = Node(f"issue3_authorization_probe_{suffix}")
    node = PerceptionAuthorizationNode(
        parameter_overrides=[
            Parameter("mission_state_topic", value=state_topic),
            Parameter("mission_event_topic", value=event_topic),
            Parameter("authorization_evidence_topic", value=evidence_topic),
        ]
    )
    state_pub = provider.create_publisher(
        MissionState, state_topic, qos(transient=True, depth=1)
    )
    evidence_pub = provider.create_publisher(
        AuthorizationEvidence, evidence_topic, qos()
    )
    events = []
    event_sub = probe.create_subscription(
        MissionEvent, event_topic, lambda msg: events.append(msg), qos()
    )
    executor = SingleThreadedExecutor()
    for executor_node in (provider, probe, node):
        executor.add_node(executor_node)

    def publish_state(seq, target, *, blocked=False):
        msg = MissionState()
        msg.state_seq = seq
        msg.state = MissionState.VERIFY_IDENTITY
        msg.target_id = target
        msg.blocked = blocked
        msg.header.stamp.sec = seq
        state_pub.publish(msg)
        for _ in range(5):
            executor.spin_once(timeout_sec=0.05)

    def publish_evidence(seq, target, result, detail="result", *, stamp_sec=None):
        msg = AuthorizationEvidence()
        msg.observed_state_seq = seq
        msg.target_id = target
        msg.result = result
        msg.provider = "test-only-provider"
        msg.detail = detail
        msg.header.stamp.sec = seq if stamp_sec is None else stamp_sec
        evidence_pub.publish(msg)
        for _ in range(5):
            executor.spin_once(timeout_sec=0.05)

    try:
        publish_state(17, 42)
        publish_evidence(16, 42, AuthorizationEvidence.PASSED)
        publish_evidence(17, 99, AuthorizationEvidence.PASSED)
        publish_evidence(
            17, 42, AuthorizationEvidence.PASSED, stamp_sec=16
        )
        publish_evidence(17, 42, AuthorizationEvidence.NOT_PASSED)
        publish_state(17, 42)
        publish_evidence(17, 42, AuthorizationEvidence.NOT_PASSED)
        assert spin_until(executor, lambda: len(events) == 1)

        publish_state(18, 43)
        publish_evidence(18, 43, AuthorizationEvidence.PASSED)
        assert spin_until(executor, lambda: len(events) == 2)

        publish_state(19, 44)
        publish_evidence(19, 44, AuthorizationEvidence.ERROR, "model failed")
        assert spin_until(executor, lambda: len(events) == 3)

        publish_state(20, 45)
        publish_evidence(20, 45, AuthorizationEvidence.CANCELLED)
        publish_evidence(20, 45, AuthorizationEvidence.PASSED)
        publish_state(21, 46, blocked=True)
        publish_evidence(21, 46, AuthorizationEvidence.PASSED)

        assert [event.event for event in events] == [
            MissionEvent.UNAUTHORIZED,
            MissionEvent.AUTHORIZED,
            MissionEvent.EXECUTION_ERROR,
        ]
        assert [(event.observed_state_seq, event.target_id) for event in events] == [
            (17, 42),
            (18, 43),
            (19, 44),
        ]
        assert all(event.source == MissionEvent.SOURCE_PERCEPTION for event in events)
        assert events[-1].detail == "test-only-provider: model failed"
    finally:
        executor.shutdown()
        for executor_node in (node, probe, provider):
            executor_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        del event_sub
