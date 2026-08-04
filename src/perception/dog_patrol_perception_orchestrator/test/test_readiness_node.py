import time

import rclpy
from dog_patrol_interfaces.msg import MissionEvent, MissionState
from dog_patrol_perception_interfaces.msg import CapabilityStatus
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from dog_patrol_perception_orchestrator.readiness_node import PerceptionReadinessNode


def transient_qos(depth=1):
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def volatile_qos(depth=10):
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def test_late_orchestrator_receives_each_adapters_retained_status_and_emits_once():
    rclpy.init()
    suffix = f"t{time.monotonic_ns()}"
    state_topic = f"/issue10/{suffix}/mission/state"
    event_topic = f"/issue10/{suffix}/mission/event"
    status_topic = f"/issue10/{suffix}/perception/capability_status"
    provider = Node(f"issue10_provider_{suffix}")
    probe = Node(f"issue10_probe_{suffix}")
    state_pub = provider.create_publisher(MissionState, state_topic, transient_qos())
    status_pubs = [
        provider.create_publisher(CapabilityStatus, status_topic, transient_qos())
        for _ in PerceptionReadinessNode.REQUIRED_CAPABILITIES
    ]
    events = []
    event_sub = probe.create_subscription(
        MissionEvent, event_topic, lambda msg: events.append(msg), volatile_qos()
    )

    state = MissionState()
    state.state_seq = 77
    state.state = MissionState.STARTUP
    state_pub.publish(state)
    for publisher, capability in zip(
        status_pubs, PerceptionReadinessNode.REQUIRED_CAPABILITIES
    ):
        status = CapabilityStatus()
        status.capability = capability
        status.status = CapabilityStatus.READY
        status.observed_startup_state_seq = 77
        status.diagnostic = "test adapter ready"
        publisher.publish(status)

    orchestrator = PerceptionReadinessNode(
        parameter_overrides=[
            Parameter("mission_state_topic", value=state_topic),
            Parameter("mission_event_topic", value=event_topic),
            Parameter("capability_status_topic", value=status_topic),
        ]
    )
    executor = SingleThreadedExecutor()
    for node in (provider, probe, orchestrator):
        executor.add_node(node)
    deadline = time.monotonic() + 3.0
    while not events and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
    for _ in range(5):
        executor.spin_once(timeout_sec=0.01)

    try:
        assert len(events) == 1
        assert events[0].source == MissionEvent.SOURCE_PERCEPTION
        assert events[0].event == MissionEvent.READY
        assert events[0].observed_state_seq == 77
    finally:
        executor.shutdown()
        for node in (orchestrator, probe, provider):
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        del event_sub
