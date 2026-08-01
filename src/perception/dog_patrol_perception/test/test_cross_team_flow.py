import time

import rclpy
from dog_patrol_interfaces.msg import MissionEvent, MissionState
from dog_patrol_manager.mission_supervisor import MissionSupervisor
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_srvs.srv import Trigger

from dog_patrol_perception.fake_perception_node import FakePerceptionNode


def _spin_until(executor, predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
        if predicate():
            return True
    return False


def test_fake_perception_drives_authorized_cross_team_flow():
    rclpy.init()
    state_topic = "/test/cross_team_flow/mission_state"
    event_topic = "/test/cross_team_flow/mission_event"
    supervisor = MissionSupervisor(
        parameter_overrides=[
            Parameter("state_topic", value=state_topic),
            Parameter("event_topic", value=event_topic),
            Parameter("state_publish_rate", value=0.0),
        ]
    )
    fake = FakePerceptionNode(
        parameter_overrides=[
            Parameter("state_topic", value=state_topic),
            Parameter("event_topic", value=event_topic),
        ]
    )
    observer = Node("cross_team_flow_test_observer")
    executor = SingleThreadedExecutor()
    executor.add_node(supervisor)
    executor.add_node(fake)
    executor.add_node(observer)

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
    states = []
    observer.create_subscription(
        MissionState, state_topic, states.append, state_qos
    )
    event_pub = observer.create_publisher(MissionEvent, event_topic, event_qos)
    confirm_client = observer.create_client(
        Trigger, "/fake_perception/confirm_target"
    )
    passed_client = observer.create_client(
        Trigger, "/fake_perception/authorization_passed"
    )

    def wait_for_state(expected):
        return _spin_until(
            executor,
            lambda: bool(states)
            and states[-1].state == expected
            and not states[-1].blocked,
        )

    def publish_navigation_event(event):
        current = states[-1]
        message = MissionEvent()
        message.header.stamp = observer.get_clock().now().to_msg()
        message.observed_state_seq = current.state_seq
        message.target_id = current.target_id
        message.source = MissionEvent.SOURCE_NAVIGATION
        message.event = event
        event_pub.publish(message)

    try:
        assert wait_for_state(MissionState.STARTUP)

        navigation_ready = MissionEvent()
        navigation_ready.header.stamp = observer.get_clock().now().to_msg()
        navigation_ready.observed_state_seq = states[-1].state_seq
        navigation_ready.target_id = 0
        navigation_ready.source = MissionEvent.SOURCE_NAVIGATION
        navigation_ready.event = MissionEvent.READY
        event_pub.publish(navigation_ready)
        assert wait_for_state(MissionState.PATROL)

        assert _spin_until(executor, confirm_client.service_is_ready)
        confirm_future = confirm_client.call_async(Trigger.Request())
        assert _spin_until(executor, confirm_future.done)
        assert confirm_future.result().success
        assert wait_for_state(MissionState.CONFIRM_TARGET)

        publish_navigation_event(MissionEvent.TARGET_POSITION_READY)
        assert wait_for_state(MissionState.APPROACH_TARGET)

        publish_navigation_event(MissionEvent.ARRIVED_AND_STOPPED)
        assert wait_for_state(MissionState.VERIFY_IDENTITY)

        assert _spin_until(executor, passed_client.service_is_ready)
        passed_future = passed_client.call_async(Trigger.Request())
        assert _spin_until(executor, passed_future.done)
        assert passed_future.result().success
        assert wait_for_state(MissionState.PATROL)
        assert states[-1].target_id == 0
    finally:
        executor.remove_node(observer)
        executor.remove_node(fake)
        executor.remove_node(supervisor)
        observer.destroy_node()
        fake.destroy_node()
        supervisor.destroy_node()
        rclpy.shutdown()
