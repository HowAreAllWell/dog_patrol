"""Target approach, tracking and arrival decisions.

This component is intentionally free of ROS subscriptions and mission
transitions.  It turns current measurements into either a stop decision, an
arrival event, or a planner request.  The coordinator owns publication.
"""

from dataclasses import dataclass
from typing import Optional

import numpy as np

from .navigation_policy import StandoffDecision, compute_standoff_decision


@dataclass(frozen=True)
class MotionConfig:
    approach_distance: float
    tracking_distance: float
    planning_goal_distance: float
    arrival_tolerance: float
    arrival_linear_speed: float
    arrival_angular_speed: float
    arrival_hold_time: float
    approach_replan_period: float
    tracking_replan_period: float
    target_replan_distance: float


@dataclass(frozen=True)
class MotionCommand:
    decision: Optional[StandoffDecision] = None
    goal: Optional[np.ndarray] = None
    should_stop: bool = False
    arrived: bool = False


class TargetMotionController:
    """Maintain target motion timers without owning ROS or mission state."""

    def __init__(self, config: MotionConfig, now_seconds) -> None:
        self.config = config
        self._now_seconds = now_seconds
        self._last_planned_target: Optional[np.ndarray] = None
        self._last_plan_request_seconds: Optional[float] = None
        self._arrival_hold_start_seconds: Optional[float] = None
        self._arrival_reported = False
        self._last_log_seconds: Optional[float] = None

    @property
    def arrival_reported(self) -> bool:
        return self._arrival_reported

    @property
    def last_planned_target(self) -> Optional[np.ndarray]:
        return self._last_planned_target

    @property
    def last_plan_request_seconds(self) -> Optional[float]:
        return self._last_plan_request_seconds

    @property
    def last_log_seconds(self) -> Optional[float]:
        return self._last_log_seconds

    def reset(self) -> None:
        self._last_planned_target = None
        self._last_plan_request_seconds = None
        self._arrival_hold_start_seconds = None
        self._arrival_reported = False
        self._last_log_seconds = None

    def reset_arrival(self) -> None:
        """Clear arrival confirmation while preserving planner throttling."""
        self._arrival_hold_start_seconds = None
        self._arrival_reported = False

    def mark_plan_requested(self, target: np.ndarray) -> None:
        """Record a planner attempt after action availability is confirmed."""
        self._last_planned_target = np.asarray(target, dtype=np.float64).copy()
        self._last_plan_request_seconds = float(self._now_seconds())

    def invalidate_last_plan(self) -> None:
        """Allow the next tick to request a plan for newer target data."""
        self._last_planned_target = None
        self._last_plan_request_seconds = None

    def clear_plan_request(self) -> None:
        """Allow an immediate retry without discarding the target baseline."""
        self._last_plan_request_seconds = None

    def mark_log(self) -> None:
        self._last_log_seconds = float(self._now_seconds())

    def tick(
        self,
        mission_state: int,
        approach_state: int,
        tracking_state: int,
        target: Optional[np.ndarray],
        robot: Optional[np.ndarray],
        measured_distance: Optional[float],
        linear_speed: float,
        angular_speed: float,
        odom_age: Optional[float],
        planner_in_flight: bool,
    ) -> MotionCommand:
        now = float(self._now_seconds())
        if mission_state == approach_state and self._arrival_reported:
            return MotionCommand(should_stop=True)
        if target is None or robot is None:
            return MotionCommand(should_stop=True)

        desired = (
            self.config.tracking_distance
            if mission_state == tracking_state
            else self.config.approach_distance
        )
        decision = compute_standoff_decision(
            robot[0], robot[1], target[0], target[1],
            desired,
            self.config.arrival_tolerance,
            planning_goal_distance=self.config.planning_goal_distance,
            measured_distance=measured_distance,
        )
        if decision.should_stop:
            if mission_state == approach_state:
                arrived = self._update_arrival(
                    decision.distance,
                    linear_speed,
                    angular_speed,
                    odom_age,
                    now,
                )
                return MotionCommand(
                    decision=decision, should_stop=True, arrived=arrived
                )
            self._arrival_hold_start_seconds = None
            return MotionCommand(decision=decision, should_stop=True)

        self._arrival_hold_start_seconds = None
        period = (
            self.config.tracking_replan_period
            if mission_state == tracking_state
            else self.config.approach_replan_period
        )
        elapsed = (
            self._last_plan_request_seconds is None
            or now - self._last_plan_request_seconds >= period
        )
        changed = (
            self._last_planned_target is None
            or np.linalg.norm(target[:2] - self._last_planned_target[:2])
            >= self.config.target_replan_distance
        )
        if not planner_in_flight and (elapsed or changed):
            return MotionCommand(
                decision=decision,
                goal=np.asarray([decision.goal_x, decision.goal_y, 0.0]),
            )
        return MotionCommand(decision=decision)

    def _update_arrival(
        self,
        distance: float,
        linear_speed: float,
        angular_speed: float,
        odom_age: Optional[float],
        now: float,
    ) -> bool:
        stopped = (
            linear_speed <= self.config.arrival_linear_speed
            and angular_speed <= self.config.arrival_angular_speed
            and odom_age is not None
            and odom_age <= 0.5
        )
        if not stopped or distance > self.config.approach_distance + self.config.arrival_tolerance:
            self._arrival_hold_start_seconds = None
            return False
        if self._arrival_hold_start_seconds is None:
            self._arrival_hold_start_seconds = now
            return False
        if (
            now - self._arrival_hold_start_seconds >= self.config.arrival_hold_time
            and not self._arrival_reported
        ):
            self._arrival_reported = True
            return True
        return False
