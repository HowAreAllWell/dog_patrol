"""Stateless navigation policies derived from the authoritative mission state."""

from dataclasses import dataclass
from enum import IntEnum
from math import atan2, hypot
from typing import Optional


class MissionStateValue(IntEnum):
    STARTUP = 0
    PATROL = 1
    CONFIRM_TARGET = 2
    APPROACH_TARGET = 3
    VERIFY_IDENTITY = 4
    TRACK_INTRUDER = 5
    RECOVER_PATROL = 6


@dataclass(frozen=True)
class NavigationPolicy:
    """Actions for one mission-state update; it stores no navigation state."""

    description: str
    safe_stop: bool = False
    pause_patrol: bool = True
    resume_patrol: bool = False
    clear_navigation_path: bool = True
    reset_target: bool = False
    allow_target_fusion: bool = False
    allow_target_motion: bool = False


def navigation_policy(
    mission_state: int,
    previous_mission_state: Optional[int] = None,
) -> NavigationPolicy:
    """Map the current global state directly to navigation actions.

    ``previous_mission_state`` is only used to preserve a freshly restored patrol
    path when the global supervisor changes RECOVER_PATROL to PATROL. It is not
    an independent state machine.
    """
    try:
        state = MissionStateValue(int(mission_state))
    except ValueError:
        return NavigationPolicy(
            description="safe stop for unknown mission state", safe_stop=True
        )

    if state == MissionStateValue.STARTUP:
        return NavigationPolicy(description="initializing")
    if state == MissionStateValue.PATROL:
        returning_from_recovery = previous_mission_state == MissionStateValue.RECOVER_PATROL
        return NavigationPolicy(
            description="patrolling",
            pause_patrol=False,
            resume_patrol=True,
            # The path mux selects the waypoint source during ordinary patrol.
            # The coordinator must not clear the shared output directly.
            clear_navigation_path=False,
            reset_target=True,
        )
    if state == MissionStateValue.CONFIRM_TARGET:
        return NavigationPolicy(
            description="acquiring target while patrol continues",
            pause_patrol=False,
            clear_navigation_path=False,
            allow_target_fusion=True,
        )
    if state == MissionStateValue.APPROACH_TARGET:
        return NavigationPolicy(
            description="approaching target",
            allow_target_fusion=True,
            allow_target_motion=True,
        )
    if state == MissionStateValue.VERIFY_IDENTITY:
        return NavigationPolicy(
            description="holding for verification",
            allow_target_fusion=True,
        )
    if state == MissionStateValue.TRACK_INTRUDER:
        return NavigationPolicy(
            description="tracking target",
            clear_navigation_path=False,
            allow_target_fusion=True,
            allow_target_motion=True,
        )
    if state == MissionStateValue.RECOVER_PATROL:
        return NavigationPolicy(
            description="returning to patrol interruption pose",
            pause_patrol=True,
            resume_patrol=False,
            reset_target=True,
        )
    return NavigationPolicy(
        description="safe stop for unsupported mission state", safe_stop=True
    )


@dataclass(frozen=True)
class StandoffDecision:
    """Planar motion decision for approaching a target with sensor stop range."""

    distance: float
    map_distance: float
    should_stop: bool
    goal_x: float
    goal_y: float
    goal_yaw: float


def compute_standoff_decision(
    robot_x: float,
    robot_y: float,
    target_x: float,
    target_y: float,
    standoff_distance: float,
    stop_tolerance: float,
    planning_goal_distance: Optional[float] = None,
    measured_distance: Optional[float] = None,
) -> StandoffDecision:
    """Return a near-target planning goal and an independent stop decision."""
    delta_x = float(target_x) - float(robot_x)
    delta_y = float(target_y) - float(robot_y)
    map_distance = hypot(delta_x, delta_y)
    distance = (
        float(measured_distance)
        if measured_distance is not None and measured_distance >= 0.0
        else map_distance
    )
    yaw = atan2(delta_y, delta_x)
    if distance <= max(0.0, standoff_distance) + max(0.0, stop_tolerance):
        return StandoffDecision(distance, map_distance, True, robot_x, robot_y, yaw)

    goal_distance = (
        max(0.0, standoff_distance)
        if planning_goal_distance is None
        else max(0.0, float(planning_goal_distance))
    )
    scale = max(0.0, map_distance - goal_distance) / max(map_distance, 1.0e-6)
    return StandoffDecision(
        distance,
        map_distance,
        False,
        float(robot_x) + delta_x * scale,
        float(robot_y) + delta_y * scale,
        yaw,
    )
