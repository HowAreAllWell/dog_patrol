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


def test_production_aggregate_requires_reid_model_readiness():
    node = PerceptionReadinessNode
    assert "reid" in node.REQUIRED_CAPABILITIES
    from dog_patrol_perception_orchestrator.readiness import (
        CapabilitySample, ReadinessCoordinator, READY,
    )
    coordinator = ReadinessCoordinator(node.REQUIRED_CAPABILITIES,
                                       startup_state=MissionState.STARTUP)
    assert coordinator.observe_mission(90, MissionState.STARTUP) is None
    for capability in ("detection_tracking", "face", "voice"):
        assert coordinator.observe_capability(CapabilitySample(capability, READY, 90)) is None
    assert coordinator.observe_capability(CapabilitySample("reid", 0, 90)) is None
    assert coordinator.observe_capability(CapabilitySample("reid", READY, 89)) is None
    assert coordinator.observe_capability(CapabilitySample("reid", READY, 90)) == 90


def test_late_orchestrator_applies_retained_adapter_error_and_not_ready_before_recovery():
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
        status.status = (
            CapabilityStatus.ERROR
            if capability == "face"
            else CapabilityStatus.READY
        )
        status.observed_startup_state_seq = 77
        status.diagnostic = (
            "test face adapter failed"
            if capability == "face"
            else "test adapter ready"
        )
        publisher.publish(status)

    orchestrator = PerceptionReadinessNode(
        parameter_overrides=[
            Parameter("mission_state_topic", value=state_topic),
            Parameter("mission_event_topic", value=event_topic),
            Parameter("capability_status_topic", value=status_topic),
            Parameter("state_timeout", value=10.0),
        ]
    )
    executor = SingleThreadedExecutor()
    for node in (provider, probe, orchestrator):
        executor.add_node(node)
    deadline = time.monotonic() + 3.0
    settle_deadline = time.monotonic() + 0.5
    while time.monotonic() < settle_deadline:
        executor.spin_once(timeout_sec=0.05)
    assert events == []

    face_not_ready = CapabilityStatus()
    face_not_ready.capability = "face"
    face_not_ready.status = CapabilityStatus.NOT_READY
    face_not_ready.observed_startup_state_seq = 77
    face_not_ready.diagnostic = "test adapter still initializing"
    status_pubs[1].publish(face_not_ready)
    settle_deadline = time.monotonic() + 0.25
    while time.monotonic() < settle_deadline:
        executor.spin_once(timeout_sec=0.05)
    assert events == []

    face_ready = CapabilityStatus()
    face_ready.capability = "face"
    face_ready.status = CapabilityStatus.READY
    face_ready.observed_startup_state_seq = 77
    face_ready.diagnostic = "test adapter ready"
    status_pubs[1].publish(face_ready)
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


def test_ready_state_blocks_late_startup_capabilities_and_old_startup_snapshot():
    rclpy.init()
    suffix = f"t{time.monotonic_ns()}"
    state_topic = f"/readiness/{suffix}/mission/state"
    event_topic = f"/readiness/{suffix}/mission/event"
    status_topic = f"/readiness/{suffix}/capability_status"
    source = Node(f"readiness_late_source_{suffix}")
    probe = Node(f"readiness_late_probe_{suffix}")
    node = PerceptionReadinessNode(
        parameter_overrides=[
            Parameter("mission_state_topic", value=state_topic),
            Parameter("mission_event_topic", value=event_topic),
            Parameter("capability_status_topic", value=status_topic),
            Parameter("state_timeout", value=10.0),
        ]
    )
    state_pub = source.create_publisher(MissionState, state_topic, transient_qos())
    status_pub = source.create_publisher(
        CapabilityStatus, status_topic, transient_qos(depth=16)
    )
    events = []
    event_sub = probe.create_subscription(
        MissionEvent, event_topic, events.append, volatile_qos()
    )
    executor = SingleThreadedExecutor()
    for current in (source, probe, node):
        executor.add_node(current)
    try:
        startup = MissionState()
        startup.state_seq = 80
        startup.state = MissionState.STARTUP
        state_pub.publish(startup)
        for capability in ("detection_tracking", "face", "reid"):
            status = CapabilityStatus()
            status.capability = capability
            status.status = CapabilityStatus.READY
            status.observed_startup_state_seq = 80
            status_pub.publish(status)
        for _ in range(10):
            executor.spin_once(timeout_sec=0.03)

        ready = MissionState()
        ready.state_seq = 81
        ready.state = MissionState.READY
        state_pub.publish(ready)
        for _ in range(5):
            executor.spin_once(timeout_sec=0.03)

        late_voice = CapabilityStatus()
        late_voice.capability = "voice"
        late_voice.status = CapabilityStatus.READY
        late_voice.observed_startup_state_seq = 80
        status_pub.publish(late_voice)
        state_pub.publish(startup)
        for _ in range(10):
            executor.spin_once(timeout_sec=0.03)

        assert events == []
    finally:
        executor.shutdown()
        for current in (node, probe, source):
            current.destroy_node()
        del state_pub, status_pub, event_sub
        if rclpy.ok():
            rclpy.shutdown()
