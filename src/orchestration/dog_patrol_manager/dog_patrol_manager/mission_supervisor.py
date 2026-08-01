#!/usr/bin/env python3

import threading

import rclpy
from dog_patrol_interfaces.msg import MissionEvent, MissionState
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from dog_patrol_manager.state_machine import (
    EventSource,
    EventType,
    MissionEventData,
    MissionStateMachine,
)


class MissionSupervisor(Node):
    def __init__(self) -> None:
        super().__init__("mission_supervisor")

        self.declare_parameter("state_topic", "/mission/state")
        self.declare_parameter("event_topic", "/mission/event")
        self.declare_parameter("state_publish_rate", 1.0)
        self.declare_parameter("initial_state_seq", 1)
        self.declare_parameter("processed_event_limit", 256)
        self.declare_parameter("max_detail_length", 256)

        self._state_topic = str(self.get_parameter("state_topic").value)
        self._event_topic = str(self.get_parameter("event_topic").value)
        self._state_publish_rate = float(
            self.get_parameter("state_publish_rate").value
        )
        self._max_detail_length = max(
            32, int(self.get_parameter("max_detail_length").value)
        )
        self._lock = threading.RLock()
        self._machine = MissionStateMachine(
            initial_state_seq=int(self.get_parameter("initial_state_seq").value),
            processed_event_limit=int(
                self.get_parameter("processed_event_limit").value
            ),
        )

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

        self._state_pub = self.create_publisher(
            MissionState, self._state_topic, state_qos
        )
        self._event_sub = self.create_subscription(
            MissionEvent, self._event_topic, self._on_event, event_qos
        )

        self._state_timer = None
        if self._state_publish_rate > 0.0:
            self._state_timer = self.create_timer(
                1.0 / self._state_publish_rate, self._publish_state
            )

        self._publish_state()
        self.get_logger().info(
            "mission supervisor ready: "
            f"state_topic={self._state_topic}, event_topic={self._event_topic}, "
            f"state_publish_rate={self._state_publish_rate:.2f}Hz"
        )

    def _on_event(self, msg: MissionEvent) -> None:
        event_data = MissionEventData(
            observed_state_seq=int(msg.observed_state_seq),
            target_id=int(msg.target_id),
            source=int(msg.source),
            event=int(msg.event),
            detail=str(msg.detail)[: self._max_detail_length],
        )
        with self._lock:
            result = self._machine.handle_event(event_data)
            if result.changed:
                self._publish_state_locked()

        source_name = self._enum_name(EventSource, event_data.source)
        event_name = self._enum_name(EventType, event_data.event)
        prefix = (
            f"event {source_name}/{event_name}, "
            f"seq={event_data.observed_state_seq}, target={event_data.target_id}"
        )
        if result.changed:
            self.get_logger().info(f"accepted {prefix}: {result.reason}")
        elif result.accepted:
            self.get_logger().info(f"accepted {prefix}: {result.reason}")
        elif result.duplicate:
            self.get_logger().debug(f"ignored duplicate {prefix}")
        else:
            self.get_logger().warn(f"rejected {prefix}: {result.reason}")

    def _publish_state(self) -> None:
        with self._lock:
            self._publish_state_locked()

    def _publish_state_locked(self) -> None:
        snapshot = self._machine.snapshot
        msg = MissionState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state_seq = int(snapshot.state_seq)
        msg.state = int(snapshot.state)
        msg.target_id = int(snapshot.target_id)
        msg.blocked = bool(snapshot.blocked)
        msg.block_cause = int(snapshot.block_cause)
        msg.detail = str(snapshot.detail)[: self._max_detail_length]
        self._state_pub.publish(msg)

    @staticmethod
    def _enum_name(enum_type, value: int) -> str:
        try:
            return enum_type(int(value)).name
        except ValueError:
            return f"UNKNOWN({value})"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionSupervisor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
