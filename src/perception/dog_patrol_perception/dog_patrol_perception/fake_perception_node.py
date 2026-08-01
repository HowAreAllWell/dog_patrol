#!/usr/bin/env python3

from typing import Optional, Sequence

import rclpy
from dog_patrol_interfaces.msg import (
    MissionEvent,
    MissionState,
    TargetBoundingBox,
)
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

from dog_patrol_perception.authorization import (
    AuthorizationCoordinator,
    AuthorizationResult,
    AuthorizationSession,
)


class FakePerceptionNode(Node):
    """Contract-level perception replacement for cross-team integration."""

    _ACTIVE_TARGET_STATES = frozenset(
        {
            MissionState.CONFIRM_TARGET,
            MissionState.APPROACH_TARGET,
            MissionState.VERIFY_IDENTITY,
            MissionState.TRACK_INTRUDER,
        }
    )
    _AUTHORIZATION_EVENTS = {
        AuthorizationResult.PASSED: MissionEvent.AUTHORIZED,
        AuthorizationResult.NOT_PASSED: MissionEvent.UNAUTHORIZED,
        AuthorizationResult.ERROR: MissionEvent.EXECUTION_ERROR,
    }

    def __init__(
        self,
        *,
        parameter_overrides: Optional[Sequence[Parameter]] = None,
    ) -> None:
        super().__init__(
            "fake_perception",
            parameter_overrides=parameter_overrides,
        )

        self.declare_parameter("state_topic", "/mission/state")
        self.declare_parameter("event_topic", "/mission/event")
        self.declare_parameter(
            "bbox_topic", "/perception/selected_target_bbox"
        )
        self.declare_parameter("auto_ready", True)
        self.declare_parameter("ready_retry_period", 1.0)
        self.declare_parameter("initial_target_id", 1)
        self.declare_parameter("bbox_publish_rate", 10.0)
        self.declare_parameter("image_width", 1280)
        self.declare_parameter("image_height", 720)
        self.declare_parameter("bbox_x_min", 320)
        self.declare_parameter("bbox_y_min", 120)
        self.declare_parameter("bbox_x_max", 760)
        self.declare_parameter("bbox_y_max", 680)
        self.declare_parameter("bbox_confidence", 0.9)
        self.declare_parameter("camera_frame_id", "fake_camera_optical_frame")

        state_topic = str(self.get_parameter("state_topic").value)
        event_topic = str(self.get_parameter("event_topic").value)
        bbox_topic = str(self.get_parameter("bbox_topic").value)
        self._auto_ready = bool(self.get_parameter("auto_ready").value)
        self._next_target_id = max(
            1, int(self.get_parameter("initial_target_id").value)
        )
        self._ready_state_seq: Optional[int] = None
        self._state_seq: Optional[int] = None
        self._state: Optional[int] = None
        self._target_id = 0
        self._blocked = False
        self._block_cause = MissionState.BLOCK_NONE
        self._target_lost = False
        self._reacquire_requested = False
        self._authorization = AuthorizationCoordinator(required_not_passed=2)
        self._image_width = max(
            1, int(self.get_parameter("image_width").value)
        )
        self._image_height = max(
            1, int(self.get_parameter("image_height").value)
        )
        self._bbox_x_min = min(
            max(0, int(self.get_parameter("bbox_x_min").value)),
            self._image_width - 1,
        )
        self._bbox_y_min = min(
            max(0, int(self.get_parameter("bbox_y_min").value)),
            self._image_height - 1,
        )
        self._bbox_x_max = min(
            max(
                self._bbox_x_min + 1,
                int(self.get_parameter("bbox_x_max").value),
            ),
            self._image_width,
        )
        self._bbox_y_max = min(
            max(
                self._bbox_y_min + 1,
                int(self.get_parameter("bbox_y_max").value),
            ),
            self._image_height,
        )
        self._bbox_confidence = min(
            1.0, max(0.0, float(self.get_parameter("bbox_confidence").value))
        )
        self._camera_frame_id = str(
            self.get_parameter("camera_frame_id").value
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
        bbox_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._event_pub = self.create_publisher(
            MissionEvent, event_topic, event_qos
        )
        self._bbox_pub = self.create_publisher(
            TargetBoundingBox, bbox_topic, bbox_qos
        )
        self._state_sub = self.create_subscription(
            MissionState, state_topic, self._on_state, state_qos
        )
        self._confirm_target_service = self.create_service(
            Trigger,
            "~/confirm_target",
            self._confirm_target,
        )
        self._target_lost_service = self.create_service(
            Trigger,
            "~/target_lost",
            self._report_target_lost,
        )
        self._target_reacquired_service = self.create_service(
            Trigger,
            "~/target_reacquired",
            self._report_target_reacquired,
        )
        self._execution_error_service = self.create_service(
            Trigger,
            "~/execution_error",
            self._report_execution_error,
        )
        self._authorization_not_passed_service = self.create_service(
            Trigger,
            "~/authorization_not_passed",
            self._report_authorization_not_passed,
        )
        self._authorization_passed_service = self.create_service(
            Trigger,
            "~/authorization_passed",
            self._report_authorization_passed,
        )
        self._authorization_error_service = self.create_service(
            Trigger,
            "~/authorization_error",
            self._report_authorization_error,
        )
        self._authorization_cancelled_service = self.create_service(
            Trigger,
            "~/authorization_cancelled",
            self._report_authorization_cancelled,
        )
        bbox_publish_rate = max(
            0.1, float(self.get_parameter("bbox_publish_rate").value)
        )
        self._bbox_timer = self.create_timer(
            1.0 / bbox_publish_rate,
            self._publish_bbox,
        )
        ready_retry_period = max(
            0.1, float(self.get_parameter("ready_retry_period").value)
        )
        self._ready_timer = self.create_timer(
            ready_retry_period,
            self._retry_ready,
        )

    def _on_state(self, msg: MissionState) -> None:
        state_seq = int(msg.state_seq)
        previous_target_id = self._target_id
        self._state_seq = state_seq
        self._state = int(msg.state)
        self._target_id = int(msg.target_id)
        self._blocked = bool(msg.blocked)
        self._block_cause = int(msg.block_cause)
        if self._target_id == 0 or self._target_id != previous_target_id:
            self._target_lost = False
            self._reacquire_requested = False
        elif self._reacquire_requested and not self._blocked:
            self._target_lost = False
            self._reacquire_requested = False

        authorization_session = AuthorizationSession(
            observed_state_seq=state_seq,
            target_id=self._target_id,
        )
        if (
            self._state == MissionState.VERIFY_IDENTITY
            and self._target_id > 0
            and not self._blocked
        ):
            if self._authorization.active_session != authorization_session:
                self._cancel_authorization()
                self._authorization.start(authorization_session)
        else:
            self._cancel_authorization()
        if (
            self._auto_ready
            and int(msg.state) == MissionState.STARTUP
            and self._ready_state_seq != state_seq
        ):
            self._ready_state_seq = state_seq
            self._publish_event(
                state_seq,
                0,
                MissionEvent.READY,
                "fake perception ready",
            )

    def _confirm_target(self, _request, response):
        if self._state_seq is None:
            response.success = False
            response.message = "no mission state received"
            return response
        if self._state != MissionState.PATROL or self._blocked:
            response.success = False
            response.message = "target confirmation requires unblocked PATROL"
            return response

        target_id = self._next_target_id
        self._next_target_id = (self._next_target_id + 1) & 0xFFFFFFFF
        if self._next_target_id == 0:
            self._next_target_id = 1
        self._publish_event(
            self._state_seq,
            target_id,
            MissionEvent.TARGET_CONFIRMED,
            "fake target confirmed",
        )
        response.success = True
        response.message = f"published TARGET_CONFIRMED for target {target_id}"
        return response

    def _report_target_lost(self, _request, response):
        if (
            self._state_seq is None
            or self._state not in self._ACTIVE_TARGET_STATES
            or self._target_id <= 0
            or self._blocked
            or self._target_lost
        ):
            response.success = False
            response.message = (
                "TARGET_LOST requires an unblocked active target"
            )
            return response

        self._target_lost = True
        self._cancel_authorization()
        self._publish_event(
            self._state_seq,
            self._target_id,
            MissionEvent.TARGET_LOST,
            "fake target lost",
        )
        response.success = True
        response.message = (
            f"published TARGET_LOST for target {self._target_id}"
        )
        return response

    def _report_target_reacquired(self, _request, response):
        if (
            self._state_seq is None
            or not self._blocked
            or self._block_cause != MissionState.BLOCK_TARGET_LOST
            or self._target_id <= 0
        ):
            response.success = False
            response.message = (
                "TARGET_REACQUIRED requires a TARGET_LOST-blocked target"
            )
            return response

        self._reacquire_requested = True
        self._publish_event(
            self._state_seq,
            self._target_id,
            MissionEvent.TARGET_REACQUIRED,
            "fake target reacquired",
        )
        response.success = True
        response.message = (
            f"published TARGET_REACQUIRED for target {self._target_id}"
        )
        return response

    def _report_execution_error(self, _request, response):
        if self._state_seq is None or self._blocked:
            response.success = False
            response.message = (
                "EXECUTION_ERROR requires an unblocked mission state"
            )
            return response

        self._cancel_authorization()
        self._publish_event(
            self._state_seq,
            self._target_id,
            MissionEvent.EXECUTION_ERROR,
            "fake perception execution error",
        )
        response.success = True
        response.message = "published EXECUTION_ERROR"
        return response

    def _report_authorization_not_passed(self, _request, response):
        return self._record_authorization_result(
            AuthorizationResult.NOT_PASSED, response
        )

    def _report_authorization_passed(self, _request, response):
        return self._record_authorization_result(
            AuthorizationResult.PASSED, response
        )

    def _report_authorization_error(self, _request, response):
        return self._record_authorization_result(
            AuthorizationResult.ERROR, response
        )

    def _report_authorization_cancelled(self, _request, response):
        return self._record_authorization_result(
            AuthorizationResult.CANCELLED, response
        )

    def _record_authorization_result(self, result, response):
        session = self._authorization.active_session
        if session is None:
            response.success = False
            response.message = "no active authorization session"
            return response

        outcome = self._authorization.record(session, result)
        response.success = True
        if outcome is None:
            response.message = (
                "authorization result recorded; awaiting next round"
            )
            return response

        event = self._AUTHORIZATION_EVENTS.get(outcome.result)
        if event is None:
            response.message = "authorization cancelled without public event"
            return response

        self._publish_event(
            outcome.session.observed_state_seq,
            outcome.session.target_id,
            event,
            f"fake authorization result: {outcome.result.value}",
        )
        response.message = f"published authorization event {event}"
        return response

    def _cancel_authorization(self) -> None:
        session = self._authorization.active_session
        if session is not None:
            self._authorization.record(session, AuthorizationResult.CANCELLED)

    def _retry_ready(self) -> None:
        if (
            self._auto_ready
            and self._state_seq is not None
            and self._state == MissionState.STARTUP
        ):
            self._publish_event(
                self._state_seq,
                0,
                MissionEvent.READY,
                "fake perception ready retry",
            )

    def _publish_bbox(self) -> None:
        if (
            self._state not in self._ACTIVE_TARGET_STATES
            or self._target_id <= 0
            or self._blocked
            or self._target_lost
        ):
            return

        msg = TargetBoundingBox()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._camera_frame_id
        msg.target_id = self._target_id
        msg.image_width = self._image_width
        msg.image_height = self._image_height
        msg.x_min = self._bbox_x_min
        msg.y_min = self._bbox_y_min
        msg.x_max = self._bbox_x_max
        msg.y_max = self._bbox_y_max
        msg.confidence = self._bbox_confidence
        self._bbox_pub.publish(msg)

    def _publish_event(
        self,
        observed_state_seq: int,
        target_id: int,
        event: int,
        detail: str,
    ) -> None:
        msg = MissionEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.observed_state_seq = int(observed_state_seq)
        msg.target_id = int(target_id)
        msg.source = MissionEvent.SOURCE_PERCEPTION
        msg.event = int(event)
        msg.detail = str(detail)
        self._event_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakePerceptionNode()
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
