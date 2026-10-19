#!/usr/bin/env python3

"""ROS boundary for the patrol mission and parent-task state machine."""

import math
import threading
from typing import Optional, Sequence

import rclpy
from dog_patrol_interfaces.msg import (
    MissionEvent,
    MissionState,
    PatrolTaskState,
    WaypointSequenceState,
)
from dog_patrol_interfaces.srv import PatrolTaskCommand
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from dog_patrol_manager.state_machine import (
    EventSource,
    EventType,
    GlobalState,
    HomePoseData,
    MissionEventData,
    MissionStateMachine,
    ParentTaskState,
    TaskCommandData,
    TaskOutcome,
    WaypointStateData,
)


class MissionSupervisor(Node):
    def __init__(
        self,
        *,
        parameter_overrides: Optional[Sequence[Parameter]] = None,
    ) -> None:
        super().__init__("mission_supervisor", parameter_overrides=parameter_overrides)
        self._declare_parameters()
        self._read_parameters()
        self._lock = threading.RLock()
        self._machine = MissionStateMachine(
            initial_state_seq=self._initial_state_seq,
            processed_event_limit=self._processed_event_limit,
        )
        self._state_entered_at = self.get_clock().now()

        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

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

        self._state_pub = self.create_publisher(MissionState, self._state_topic, state_qos)
        self._task_state_pub = self.create_publisher(
            PatrolTaskState, self._task_state_topic, state_qos
        )
        self._event_sub = self.create_subscription(
            MissionEvent, self._event_topic, self._on_event, event_qos
        )
        self._waypoint_state_sub = self.create_subscription(
            WaypointSequenceState,
            self._waypoint_state_topic,
            self._on_waypoint_state,
            state_qos,
        )
        self._command_service = self.create_service(
            PatrolTaskCommand,
            self._command_service_name,
            self._on_task_command,
        )
        self._reset_service = self.create_service(Trigger, "/mission/reset", self._on_reset)

        self._state_timer = None
        if self._state_publish_rate > 0.0:
            self._state_timer = self.create_timer(
                1.0 / self._state_publish_rate, self._publish_state
            )
        self._watchdog_timer = self.create_timer(0.1, self._check_state_timeout)
        self._publish_state()
        self.get_logger().info(
            "mission supervisor ready: "
            f"state={self._state_topic}, task_state={self._task_state_topic}, "
            f"event={self._event_topic}, command={self._command_service_name}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("state_topic", "/mission/state")
        self.declare_parameter("task_state_topic", "/patrol_task/state")
        self.declare_parameter("event_topic", "/mission/event")
        self.declare_parameter("waypoint_state_topic", "/waypoint_sequence/state")
        self.declare_parameter("task_command_service", "/mission/task_command")
        self.declare_parameter("state_publish_rate", 10.0)
        self.declare_parameter("initial_state_seq", 1)
        self.declare_parameter("processed_event_limit", 256)
        self.declare_parameter("max_detail_length", 256)
        self.declare_parameter("confirm_target_timeout", 5.0)
        self.declare_parameter("intruder_tracking_timeout", 60.0)
        self.declare_parameter("patrol_recovery_timeout", 10.0)
        self.declare_parameter("return_timeout", 60.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("tf_timeout", 0.2)
        self.declare_parameter("home_pose_max_age", 0.5)

    def _read_parameters(self) -> None:
        value = self.get_parameter
        self._state_topic = str(value("state_topic").value)
        self._task_state_topic = str(value("task_state_topic").value)
        self._event_topic = str(value("event_topic").value)
        self._waypoint_state_topic = str(value("waypoint_state_topic").value)
        self._command_service_name = str(value("task_command_service").value)
        self._state_publish_rate = max(0.0, float(value("state_publish_rate").value))
        self._initial_state_seq = int(value("initial_state_seq").value)
        self._processed_event_limit = int(value("processed_event_limit").value)
        self._max_detail_length = max(32, int(value("max_detail_length").value))
        self._confirm_target_timeout = max(0.1, float(value("confirm_target_timeout").value))
        self._intruder_tracking_timeout = float(value("intruder_tracking_timeout").value)
        if not math.isfinite(self._intruder_tracking_timeout) or self._intruder_tracking_timeout <= 0.0:
            raise ValueError("intruder_tracking_timeout must be finite and positive")
        self._patrol_recovery_timeout = max(0.1, float(value("patrol_recovery_timeout").value))
        self._return_timeout = max(0.1, float(value("return_timeout").value))
        self._map_frame = str(value("map_frame").value).lstrip("/")
        self._robot_frame = str(value("robot_frame").value).lstrip("/")
        self._tf_timeout = max(0.01, float(value("tf_timeout").value))
        self._home_pose_max_age = max(0.0, float(value("home_pose_max_age").value))

    def _on_reset(self, request, response):
        del request
        with self._lock:
            snapshot = self._machine.reset_session()
            self._state_entered_at = self.get_clock().now()
            self._publish_state_locked()
        response.success = True
        response.message = f"mission reset to STARTUP, state_seq={snapshot.state_seq}"
        self.get_logger().info(response.message)
        return response

    def _on_task_command(self, request, response):
        with self._lock:
            home_pose = None
            if int(request.command) == PatrolTaskCommand.Request.START:
                home_pose, home_error = self._capture_home_pose()
                if home_error is not None:
                    result = self._machine.handle_task_command(
                        TaskCommandData(command=int(request.command))
                    )
                    result = result.__class__(
                        accepted=False,
                        changed=False,
                        reason=home_error[1],
                        error_code=home_error[0],
                        snapshot=result.snapshot,
                    )
                else:
                    result = self._machine.handle_task_command(
                        self._task_command_data(
                            int(request.command), self._machine.route, home_pose
                        )
                    )
            else:
                result = self._machine.handle_task_command(
                    self._task_command_data(
                        int(request.command), self._machine.route, None
                    )
                )
            if result.changed:
                self._state_entered_at = self.get_clock().now()
                self._publish_state_locked()
            self._fill_command_response(response, result)
        if result.accepted:
            self.get_logger().info(
                f"accepted task command={request.command}: {result.reason}"
            )
        else:
            self.get_logger().warning(
                f"rejected task command={request.command}: {result.error_code}: {result.reason}"
            )
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
        prefix = (
            f"event {self._enum_name(EventSource, event_data.source)}/"
            f"{self._enum_name(EventType, event_data.event)}, "
            f"seq={event_data.observed_state_seq}, target={event_data.target_id}"
        )
        if result.changed or result.accepted:
            self.get_logger().info(f"accepted {prefix}: {result.reason}")
        elif result.duplicate:
            self.get_logger().debug(f"ignored duplicate {prefix}")
        else:
            self.get_logger().warning(f"rejected {prefix}: {result.reason}")

    def _on_waypoint_state(self, msg: WaypointSequenceState) -> None:
        route = WaypointStateData(
            route_revision=int(msg.route_revision),
            waypoint_count=int(msg.waypoint_count),
            current_index=int(msg.current_index),
            completed_count=int(msg.completed_count),
            sequence_done=bool(msg.sequence_done),
            paused=bool(msg.paused),
        )
        with self._lock:
            result = self._machine.handle_waypoint_state(route)
            if result.changed:
                self._state_entered_at = self.get_clock().now()
                self._publish_state_locked()
        if not result.accepted:
            self.get_logger().warning(f"ignored waypoint state: {result.reason}")

    def _publish_state(self) -> None:
        with self._lock:
            self._publish_state_locked()

    def _publish_state_locked(self) -> None:
        snapshot = self._machine.snapshot
        stamp = self.get_clock().now().to_msg()

        mission = MissionState()
        mission.header.stamp = stamp
        mission.state_seq = int(snapshot.state_seq)
        mission.state = int(snapshot.state)
        mission.target_id = int(snapshot.target_id)
        mission.handled_target_id = int(snapshot.handled_target_id)
        mission.detail = str(snapshot.detail)[: self._max_detail_length]

        task = PatrolTaskState()
        task.header.stamp = stamp
        task.state_seq = int(snapshot.state_seq)
        task.task_state = int(snapshot.task_state)
        task.last_task_outcome = int(snapshot.last_task_outcome)
        task.navigation_mode = int(snapshot.navigation_mode)
        task.detail = str(snapshot.detail)[: self._max_detail_length]
        if snapshot.home_pose is not None and snapshot.home_pose.valid:
            task.home_pose_valid = True
            pose = snapshot.home_pose.pose
            if isinstance(pose, PoseStamped):
                task.home_pose = pose
            else:
                task.home_pose.header.frame_id = self._map_frame
        else:
            task.home_pose_valid = False

        self._state_pub.publish(mission)
        self._task_state_pub.publish(task)

    def _check_state_timeout(self) -> None:
        with self._lock:
            snapshot = self._machine.snapshot
            age = (self.get_clock().now() - self._state_entered_at).nanoseconds * 1e-9
            result = None
            if snapshot.task_state == ParentTaskState.TASK_RUNNING:
                if snapshot.state == GlobalState.CONFIRM_TARGET and age >= self._confirm_target_timeout:
                    result = self._machine.begin_patrol_recovery(
                        f"target position confirmation timed out after {age:.1f}s"
                    )
                elif snapshot.state == GlobalState.TRACK_INTRUDER and age >= self._intruder_tracking_timeout:
                    # This clock starts only when UNAUTHORIZED enters tracking.
                    # Heartbeats and temporary visibility changes do not change
                    # the mission state, so they cannot extend the deadline.
                    result = self._machine.begin_patrol_recovery(
                        f"intruder tracking timed out after {age:.1f}s"
                    )
                elif snapshot.state == GlobalState.RECOVER_PATROL and age >= self._patrol_recovery_timeout:
                    result = self._machine.complete_patrol_recovery(
                        f"patrol recovery timed out after {age:.1f}s"
                    )
            elif (
                snapshot.task_state == ParentTaskState.TASK_RETURNING
                and age >= self._return_timeout
            ):
                result = self._machine.handle_event(
                    MissionEventData(
                        observed_state_seq=snapshot.state_seq,
                        target_id=0,
                        source=int(EventSource.NAVIGATION),
                        event=int(EventType.RETURN_FAILED),
                        detail=f"return timed out after {age:.1f}s",
                    )
                )
            if result is not None and result.changed:
                self._state_entered_at = self.get_clock().now()
                self._publish_state_locked()
                self.get_logger().warning(result.reason)

    def _capture_home_pose(self):
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._robot_frame,
                Time(),
                timeout=Duration(seconds=self._tf_timeout),
            )
        except TransformException as exc:
            return None, ("home_pose_unavailable", f"home TF unavailable: {exc}")

        now = self.get_clock().now()
        source_time = Time.from_msg(transform.header.stamp)
        age = (now - source_time).nanoseconds * 1e-9
        if age < 0.0 or age > self._home_pose_max_age:
            return None, ("home_pose_unavailable", f"home TF age {age:.3f}s is outside limit")
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        values = (
            translation.x,
            translation.y,
            translation.z,
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w,
        )
        if not all(math.isfinite(float(value)) for value in values):
            return None, ("invalid_home_pose", "home TF contains non-finite values")
        norm = math.sqrt(rotation.x**2 + rotation.y**2 + rotation.z**2 + rotation.w**2)
        if norm < 1.0e-6:
            return None, ("invalid_home_pose", "home TF quaternion is invalid")

        pose = PoseStamped()
        pose.header.stamp = transform.header.stamp
        pose.header.frame_id = self._map_frame
        pose.pose.position.x = translation.x
        pose.pose.position.y = translation.y
        pose.pose.position.z = translation.z
        pose.pose.orientation = rotation
        return HomePoseData(pose=pose, frame_id=self._map_frame), None

    @staticmethod
    def _task_command_data(command, route, home_pose):
        return TaskCommandData(
            command=int(command),
            route=route,
            home_pose=home_pose,
        )

    @staticmethod
    def _fill_command_response(response, result) -> None:
        response.accepted = bool(result.accepted)
        response.changed = bool(result.changed)
        response.state_seq = int(result.snapshot.state_seq)
        response.task_state = int(result.snapshot.task_state)
        response.mission_state = int(result.snapshot.state)
        response.last_task_outcome = int(result.snapshot.last_task_outcome)
        response.error_code = str(result.error_code)
        response.message = str(result.reason)

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
