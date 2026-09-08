"""State and decisions for returning to a patrol interruption pose."""

from dataclasses import dataclass
from typing import Optional

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


@dataclass(frozen=True)
class RecoveryConfig:
    position_tolerance: float
    linear_speed: float
    angular_speed: float
    odom_timeout: float
    hold_time: float
    republish_period: float
    request_retry_period: float


@dataclass(frozen=True)
class RecoveryCommand:
    should_stop: bool = False
    request_path: bool = False
    publish_path: Optional[Path] = None
    complete: bool = False


class PatrolRecoveryController:
    """Own recovery progress, while the coordinator owns ROS I/O."""

    def __init__(self, config: RecoveryConfig, now_seconds) -> None:
        self.config = config
        self._now_seconds = now_seconds
        self.interruption_pose: Optional[PoseStamped] = None
        self.path_requested = False
        self.path_ready = False
        self.path: Optional[Path] = None
        self.last_path_publish_seconds: Optional[float] = None
        self.last_path_request_seconds: Optional[float] = None
        self.return_arrived = False
        self.hold_start_seconds: Optional[float] = None
        self.resume_sent = False

    def reset(self) -> None:
        self.interruption_pose = None
        self.path_requested = False
        self.path_ready = False
        self.path = None
        self.last_path_publish_seconds = None
        self.last_path_request_seconds = None
        self.return_arrived = False
        self.hold_start_seconds = None
        self.resume_sent = False

    def begin(self, interruption_pose: Optional[PoseStamped]) -> None:
        self.interruption_pose = interruption_pose
        self.path_requested = False
        self.path_ready = False
        self.path = None
        self.last_path_publish_seconds = None
        self.last_path_request_seconds = None
        self.return_arrived = False
        self.hold_start_seconds = None
        self.resume_sent = False

    def set_path_requested(self, requested: bool) -> None:
        self.path_requested = bool(requested)
        if requested:
            self.last_path_request_seconds = float(self._now_seconds())

    def set_path_result(self, path: Optional[Path]) -> None:
        self.path_requested = False
        if path is None or not path.poses:
            self.path_ready = False
            self.path = None
            self.last_path_publish_seconds = None
            return
        self.path_ready = True
        self.path = path
        self.last_path_publish_seconds = None

    def mark_complete(self) -> bool:
        if self.return_arrived:
            return False
        self.return_arrived = True
        self.path_requested = False
        self.path_ready = False
        self.path = None
        self.last_path_publish_seconds = None
        self.last_path_request_seconds = None
        self.hold_start_seconds = None
        self.resume_sent = True
        return True

    def clear_path(self) -> None:
        self.path_requested = False
        self.path_ready = False
        self.path = None
        self.last_path_publish_seconds = None
        self.last_path_request_seconds = None

    def republish_path(self) -> Optional[Path]:
        if not self.path_ready or self.path is None or not self.path.poses:
            return None
        now = float(self._now_seconds())
        if (
            self.last_path_publish_seconds is not None
            and now - self.last_path_publish_seconds < self.config.republish_period
        ):
            return None
        self.last_path_publish_seconds = now
        return self.path

    def tick(
        self,
        current_pose: Optional[PoseStamped],
        linear_speed: float,
        angular_speed: float,
        odom_age: Optional[float],
        planner_in_flight: bool,
    ) -> RecoveryCommand:
        if self.return_arrived:
            return RecoveryCommand()

        now = float(self._now_seconds())
        if self.interruption_pose is None:
            return RecoveryCommand(complete=True)
        if current_pose is None:
            return RecoveryCommand(should_stop=True)

        dx = (
            current_pose.pose.position.x - self.interruption_pose.pose.position.x
        )
        dy = (
            current_pose.pose.position.y - self.interruption_pose.pose.position.y
        )
        position_error = (dx * dx + dy * dy) ** 0.5
        stopped = (
            linear_speed <= self.config.linear_speed
            and angular_speed <= self.config.angular_speed
            and odom_age is not None
            and odom_age <= self.config.odom_timeout
        )
        if position_error <= self.config.position_tolerance:
            if not stopped:
                self.hold_start_seconds = None
                return RecoveryCommand(should_stop=True)
            if self.hold_start_seconds is None:
                self.hold_start_seconds = now
            elif now - self.hold_start_seconds >= self.config.hold_time:
                return RecoveryCommand(complete=True)
            return RecoveryCommand(should_stop=True)

        self.hold_start_seconds = None
        path_to_publish = self.republish_path()
        request_path = (
            not self.path_ready
            and not self.path_requested
            and not planner_in_flight
            and (
                self.last_path_request_seconds is None
                or now - self.last_path_request_seconds
                >= self.config.request_retry_period
            )
        )
        return RecoveryCommand(
            publish_path=path_to_publish,
            request_path=request_path,
        )
