from dataclasses import dataclass
from enum import IntEnum
from math import atan2, hypot


class MissionStateValue(IntEnum):
    STARTUP = 0
    PATROL = 1
    CONFIRM_TARGET = 2
    APPROACH_TARGET = 3
    VERIFY_IDENTITY = 4
    TRACK_INTRUDER = 5


class NavigationMode(IntEnum):
    INITIALIZING = 0
    PATROLLING = 1
    ACQUIRING_TARGET = 2
    APPROACHING_TARGET = 3
    HOLDING_FOR_VERIFICATION = 4
    TRACKING_TARGET = 5
    SAFE_STOP = 6


@dataclass(frozen=True)
class NavigationEntryActions:
    mode: NavigationMode = NavigationMode.INITIALIZING
    pause_patrol: bool = True
    resume_patrol: bool = False
    clear_navigation_path: bool = True
    reset_target: bool = False
    allow_target_fusion: bool = False
    allow_target_motion: bool = False


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
    planning_goal_distance: float | None = None,
    measured_distance: float | None = None,
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


class NavigationStateMachine:
    """Map authoritative mission states to navigation-only operating modes."""

    def __init__(self) -> None:
        self.mode = NavigationMode.INITIALIZING
        self.state_seq = 0
        self.target_id = 0

    def enter(
        self,
        mission_state: int,
        blocked: bool,
        state_seq: int,
        target_id: int,
    ) -> NavigationEntryActions:
        self.state_seq = int(state_seq)
        self.target_id = int(target_id)

        if blocked:
            self.mode = NavigationMode.SAFE_STOP
            return NavigationEntryActions(mode=self.mode)

        try:
            state = MissionStateValue(int(mission_state))
        except ValueError:
            self.mode = NavigationMode.SAFE_STOP
            return NavigationEntryActions(mode=self.mode)

        if state == MissionStateValue.STARTUP:
            self.mode = NavigationMode.INITIALIZING
            return NavigationEntryActions(mode=self.mode)
        if state == MissionStateValue.PATROL:
            self.mode = NavigationMode.PATROLLING
            return NavigationEntryActions(
                mode=self.mode,
                pause_patrol=False,
                resume_patrol=True,
                reset_target=True,
            )
        if state == MissionStateValue.CONFIRM_TARGET:
            self.mode = NavigationMode.ACQUIRING_TARGET
            return NavigationEntryActions(
                mode=self.mode,
                allow_target_fusion=True,
            )
        if state == MissionStateValue.APPROACH_TARGET:
            self.mode = NavigationMode.APPROACHING_TARGET
            return NavigationEntryActions(
                mode=self.mode,
                clear_navigation_path=False,
                allow_target_fusion=True,
                allow_target_motion=True,
            )
        if state == MissionStateValue.VERIFY_IDENTITY:
            self.mode = NavigationMode.HOLDING_FOR_VERIFICATION
            return NavigationEntryActions(
                mode=self.mode,
                allow_target_fusion=True,
            )

        self.mode = NavigationMode.TRACKING_TARGET
        return NavigationEntryActions(
            mode=self.mode,
            clear_navigation_path=False,
            allow_target_fusion=True,
            allow_target_motion=True,
        )
