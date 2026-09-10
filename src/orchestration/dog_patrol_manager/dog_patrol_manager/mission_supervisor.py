#!/usr/bin/env python3

import threading
from typing import Optional, Sequence

import rclpy
from dog_patrol_interfaces.msg import MissionEvent, MissionState
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_srvs.srv import Trigger

from dog_patrol_manager.state_machine import (
    EventSource,
    EventType,
    GlobalState,
    MissionEventData,
    MissionStateMachine,
)


class MissionSupervisor(Node):
    def __init__(
        self,
        *,
        parameter_overrides: Optional[Sequence[Parameter]] = None,
    ) -> None:
        super().__init__(
            "mission_supervisor",
            parameter_overrides=parameter_overrides,
        )

        self.declare_parameter("state_topic", "/mission/state")
        self.declare_parameter("event_topic", "/mission/event")
        self.declare_parameter("state_publish_rate", 10.0)
        self.declare_parameter("initial_state_seq", 1)
        self.declare_parameter("processed_event_limit", 256)
        self.declare_parameter("max_detail_length", 256)
        self.declare_parameter("confirm_target_timeout", 5.0)
        self.declare_parameter("patrol_recovery_timeout", 10.0)

        self._state_topic = str(self.get_parameter("state_topic").value)
        self._event_topic = str(self.get_parameter("event_topic").value)
        self._state_publish_rate = float(
            self.get_parameter("state_publish_rate").value
        )
        self._max_detail_length = max(
            32, int(self.get_parameter("max_detail_length").value)
        )
        self._lock = threading.RLock()
        self._confirm_target_timeout = max(
            0.1, float(self.get_parameter("confirm_target_timeout").value)
        )
        self._patrol_recovery_timeout = max(
            0.1, float(self.get_parameter("patrol_recovery_timeout").value)
        )
        self._machine = MissionStateMachine(
            initial_state_seq=int(
                self.get_parameter("initial_state_seq").value
            ),
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
        self._reset_service = self.create_service(
            Trigger, "/mission/reset", self._on_reset
        )

        self._state_timer = None
        if self._state_publish_rate > 0.0:
            self._state_timer = self.create_timer(
                1.0 / self._state_publish_rate, self._publish_state
            )
        self._state_entered_at = self.get_clock().now()
        self._watchdog_timer = self.create_timer(0.1, self._check_state_timeout)

        self._publish_state()
        self.get_logger().info(
            "mission supervisor ready: "
            f"state_topic={self._state_topic}, "
            f"event_topic={self._event_topic}, "
            f"state_publish_rate={self._state_publish_rate:.2f}Hz, "
            f"confirm_timeout={self._confirm_target_timeout:.1f}s, "
            f"recovery_timeout={self._patrol_recovery_timeout:.1f}s"
        )

    def _on_reset(self, request, response):
        del request
        with self._lock:
            snapshot = self._machine.reset_session()
            self._state_entered_at = self.get_clock().now()
            self._publish_state_locked()
        response.success = True
        response.message = (
            f"mission reset to STARTUP, state_seq={snapshot.state_seq}"
        )
        self.get_logger().info(response.message)
        return response

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
                self._state_entered_at = self.get_clock().now()
                self._publish_state_locked()

        source_name = self._enum_name(EventSource, event_data.source)
        event_name = self._enum_name(EventType, event_data.event)
        prefix = (
            f"event {source_name}/{event_name}, "
            f"seq={event_data.observed_state_seq}, "
            f"target={event_data.target_id}"
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

    def _check_state_timeout(self) -> None:
        with self._lock:
            snapshot = self._machine.snapshot
            age = (self.get_clock().now() - self._state_entered_at).nanoseconds * 1e-9
            if (
                snapshot.state == GlobalState.CONFIRM_TARGET
                and age >= self._confirm_target_timeout
            ):
                result = self._machine.begin_patrol_recovery(
                    f"target position confirmation timed out after {age:.1f}s"
                )
            elif (
                snapshot.state == GlobalState.RECOVER_PATROL
                and age >= self._patrol_recovery_timeout
            ):
                result = self._machine.complete_patrol_recovery(
                    f"patrol recovery timed out after {age:.1f}s; resumed previous patrol task"
                )
            else:
                return

            if result.changed:
                self._state_entered_at = self.get_clock().now()
                self._publish_state_locked()
                self.get_logger().warning(result.reason)

    def _publish_state_locked(self) -> None:
        snapshot = self._machine.snapshot
        msg = MissionState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state_seq = int(snapshot.state_seq)
        msg.state = int(snapshot.state)
        msg.target_id = int(snapshot.target_id)
        msg.handled_target_id = int(snapshot.handled_target_id)
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
