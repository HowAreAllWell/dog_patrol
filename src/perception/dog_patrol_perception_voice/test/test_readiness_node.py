from __future__ import annotations

import time

import rclpy
from dog_patrol_interfaces.msg import MissionState
from dog_patrol_perception_interfaces.msg import CapabilityStatus
from dog_patrol_perception_voice.preflight import READY, VoicePreflightOutcome
from dog_patrol_perception_voice.readiness_node import VoiceReadinessNode
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def _transient_qos(depth: int = 16) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def _wait_for(executor, predicate, timeout: float = 3.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
        if predicate():
            return True
    return predicate()


def test_startup_preflight_is_published_as_retained_matching_capability_status() -> None:
    rclpy.init()
    suffix = f"t{time.monotonic_ns()}"
    state_topic = f"/issue35/{suffix}/mission/state"
    status_topic = f"/issue35/{suffix}/perception/capability_status"
    source = Node(f"issue35_source_{suffix}")
    executor = SingleThreadedExecutor()
    state_pub = source.create_publisher(MissionState, state_topic, _transient_qos(depth=1))
    node = VoiceReadinessNode(
        parameter_overrides=[
            Parameter("mission_state_topic", value=state_topic),
            Parameter("capability_status_topic", value=status_topic),
        ],
        preflight=lambda: VoicePreflightOutcome(READY, "voice preflight ready"),
    )
    executor.add_node(source)
    executor.add_node(node)

    state = MissionState()
    state.state_seq = 37
    state.state = MissionState.STARTUP
    state_pub.publish(state)
    statuses: list[CapabilityStatus] = []
    try:
        assert _wait_for(executor, lambda: len(statuses) == 0, timeout=0.1)
        late_probe = Node(f"issue35_probe_{suffix}")
        status_sub = late_probe.create_subscription(
            CapabilityStatus,
            status_topic,
            statuses.append,
            _transient_qos(),
        )
        executor.add_node(late_probe)
        assert _wait_for(executor, lambda: len(statuses) == 1)
        assert statuses[0].capability == "voice"
        assert statuses[0].status == CapabilityStatus.READY
        assert statuses[0].observed_startup_state_seq == 37
        assert statuses[0].diagnostic == "voice preflight ready"
    finally:
        node.destroy_node()
        source.destroy_node()
        if "late_probe" in locals():
            late_probe.destroy_node()
        executor.shutdown()
        del state_pub
        if rclpy.ok():
            rclpy.shutdown()
