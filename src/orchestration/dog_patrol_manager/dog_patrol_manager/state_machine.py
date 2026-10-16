"""Pure patrol mission and parent-task state machine."""

from collections import deque
from dataclasses import dataclass
from enum import IntEnum
from typing import Any, Deque, Dict, FrozenSet, Optional, Set, Tuple


class GlobalState(IntEnum):
    STARTUP = 0
    PATROL = 1
    CONFIRM_TARGET = 2
    APPROACH_TARGET = 3
    VERIFY_IDENTITY = 4
    TRACK_INTRUDER = 5
    RECOVER_PATROL = 6
    READY = 7


class EventSource(IntEnum):
    PERCEPTION = 0
    NAVIGATION = 1


class EventType(IntEnum):
    READY = 0
    TARGET_CONFIRMED = 1
    TARGET_POSITION_READY = 2
    ARRIVED_AND_STOPPED = 3
    AUTHORIZED = 4
    UNAUTHORIZED = 5
    TARGET_LOST = 6
    EXECUTION_ERROR = 7
    PATROL_RECOVERY_COMPLETE = 8
    RETURN_REACHED = 9
    RETURN_FAILED = 10


class ParentTaskState(IntEnum):
    TASK_NO_TASK = 0
    TASK_RUNNING = 1
    TASK_PAUSED = 2
    TASK_RETURNING = 3


class TaskOutcome(IntEnum):
    TASK_OUTCOME_NONE = 0
    TASK_OUTCOME_COMPLETED = 1
    TASK_OUTCOME_RETURNED = 2
    TASK_OUTCOME_TERMINATED = 3
    TASK_OUTCOME_FAILED = 4


class NavigationMode(IntEnum):
    NAVIGATION_STOP = 0
    NAVIGATION_WAYPOINT = 1
    NAVIGATION_MISSION = 2


class TaskCommand(IntEnum):
    START = 0
    PAUSE = 1
    RESUME = 2
    RETURN = 3
    TERMINATE = 4


class ResumeBehavior(IntEnum):
    PATROL = 0
    RECOVER_PATROL = 1


@dataclass(frozen=True)
class MissionEventData:
    observed_state_seq: int
    target_id: int
    source: int
    event: int
    detail: str = ""


@dataclass(frozen=True)
class HomePoseData:
    pose: Any
    frame_id: str = "map"
    valid: bool = True


@dataclass(frozen=True)
class WaypointStateData:
    route_revision: int
    waypoint_count: int
    current_index: int
    completed_count: int
    sequence_done: bool
    paused: bool


@dataclass(frozen=True)
class MissionSnapshot:
    state_seq: int
    state: GlobalState
    target_id: int
    detail: str
    handled_target_id: int = 0
    task_state: ParentTaskState = ParentTaskState.TASK_NO_TASK
    last_task_outcome: TaskOutcome = TaskOutcome.TASK_OUTCOME_NONE
    navigation_mode: NavigationMode = NavigationMode.NAVIGATION_STOP
    home_pose: Optional[HomePoseData] = None
    resume_behavior: Optional[ResumeBehavior] = None


@dataclass(frozen=True)
class EventResult:
    accepted: bool
    changed: bool
    duplicate: bool
    reason: str
    snapshot: MissionSnapshot


@dataclass(frozen=True)
class TaskCommandData:
    command: int
    route: Optional[WaypointStateData] = None
    home_pose: Optional[HomePoseData] = None


@dataclass(frozen=True)
class TaskCommandResult:
    accepted: bool
    changed: bool
    reason: str
    error_code: str
    snapshot: MissionSnapshot


EventKey = Tuple[int, int, int, int]
TransitionKey = Tuple[GlobalState, EventSource, EventType]


class MissionStateMachine:
    """Deterministic mission and parent-task state machine."""

    _EVENT_SOURCES: Dict[EventType, FrozenSet[EventSource]] = {
        EventType.READY: frozenset({EventSource.PERCEPTION, EventSource.NAVIGATION}),
        EventType.TARGET_CONFIRMED: frozenset({EventSource.PERCEPTION}),
        EventType.TARGET_POSITION_READY: frozenset({EventSource.NAVIGATION}),
        EventType.ARRIVED_AND_STOPPED: frozenset({EventSource.NAVIGATION}),
        EventType.AUTHORIZED: frozenset({EventSource.PERCEPTION}),
        EventType.UNAUTHORIZED: frozenset({EventSource.PERCEPTION}),
        EventType.TARGET_LOST: frozenset({EventSource.PERCEPTION}),
        EventType.EXECUTION_ERROR: frozenset(
            {EventSource.PERCEPTION, EventSource.NAVIGATION}
        ),
        EventType.PATROL_RECOVERY_COMPLETE: frozenset({EventSource.NAVIGATION}),
        EventType.RETURN_REACHED: frozenset({EventSource.NAVIGATION}),
        EventType.RETURN_FAILED: frozenset({EventSource.NAVIGATION}),
    }

    _TRANSITIONS: Dict[TransitionKey, GlobalState] = {
        (GlobalState.PATROL, EventSource.PERCEPTION, EventType.TARGET_CONFIRMED): GlobalState.CONFIRM_TARGET,
        (GlobalState.CONFIRM_TARGET, EventSource.NAVIGATION, EventType.TARGET_POSITION_READY): GlobalState.APPROACH_TARGET,
        (GlobalState.APPROACH_TARGET, EventSource.NAVIGATION, EventType.ARRIVED_AND_STOPPED): GlobalState.VERIFY_IDENTITY,
        (GlobalState.VERIFY_IDENTITY, EventSource.PERCEPTION, EventType.AUTHORIZED): GlobalState.RECOVER_PATROL,
        (GlobalState.VERIFY_IDENTITY, EventSource.PERCEPTION, EventType.UNAUTHORIZED): GlobalState.TRACK_INTRUDER,
        (GlobalState.RECOVER_PATROL, EventSource.NAVIGATION, EventType.PATROL_RECOVERY_COMPLETE): GlobalState.PATROL,
    }

    def __init__(self, initial_state_seq: int = 1, processed_event_limit: int = 256):
        self._state_seq = self._normalize_seq(initial_state_seq)
        self._state = GlobalState.STARTUP
        self._target_id = 0
        self._handled_target_id = 0
        self._detail = "waiting for perception and navigation ready"
        self._perception_ready = False
        self._navigation_ready = False
        self._task_state = ParentTaskState.TASK_NO_TASK
        self._last_task_outcome = TaskOutcome.TASK_OUTCOME_NONE
        self._navigation_mode = NavigationMode.NAVIGATION_STOP
        self._home_pose: Optional[HomePoseData] = None
        self._resume_behavior: Optional[ResumeBehavior] = None
        self._recovery_from_pause = False
        self._route: Optional[WaypointStateData] = None
        self._processed_event_limit = max(16, int(processed_event_limit))
        self._processed_events: Set[EventKey] = set()
        self._processed_order: Deque[EventKey] = deque()

    @property
    def snapshot(self) -> MissionSnapshot:
        return MissionSnapshot(
            state_seq=self._state_seq,
            state=self._state,
            target_id=self._target_id,
            detail=self._detail,
            handled_target_id=self._handled_target_id,
            task_state=self._task_state,
            last_task_outcome=self._last_task_outcome,
            navigation_mode=self._navigation_mode,
            home_pose=self._home_pose,
            resume_behavior=self._resume_behavior,
        )

    @property
    def perception_ready(self) -> bool:
        return self._perception_ready

    @property
    def navigation_ready(self) -> bool:
        return self._navigation_ready

    @property
    def route(self) -> Optional[WaypointStateData]:
        return self._route

    def reset_session(self, detail: str = "mission session reset") -> MissionSnapshot:
        self._state = GlobalState.STARTUP
        self._target_id = 0
        self._handled_target_id = 0
        self._perception_ready = False
        self._navigation_ready = False
        self._task_state = ParentTaskState.TASK_NO_TASK
        self._last_task_outcome = TaskOutcome.TASK_OUTCOME_NONE
        self._navigation_mode = NavigationMode.NAVIGATION_STOP
        self._home_pose = None
        self._resume_behavior = None
        self._recovery_from_pause = False
        self._route = None
        self._detail = str(detail).strip() or "mission session reset"
        self._processed_events.clear()
        self._processed_order.clear()
        self._advance_seq()
        return self.snapshot

    def handle_event(self, raw_event: MissionEventData) -> EventResult:
        source = self._parse_source(raw_event.source)
        event = self._parse_event(raw_event.event)
        if source is None:
            return self._reject(f"unknown event source {raw_event.source}")
        if event is None:
            return self._reject(f"unknown event type {raw_event.event}")
        key = (int(source), int(event), int(raw_event.observed_state_seq), int(raw_event.target_id))
        if key in self._processed_events:
            return EventResult(False, False, True, "duplicate event", self.snapshot)
        if source not in self._EVENT_SOURCES[event]:
            return self._reject(f"{source.name} is not allowed to publish {event.name}")
        if int(raw_event.observed_state_seq) != self._state_seq:
            return self._reject(
                f"stale state_seq {raw_event.observed_state_seq}, current is {self._state_seq}"
            )
        if event == EventType.READY:
            return self._handle_ready(source, raw_event, key)
        if event == EventType.RETURN_REACHED:
            return self._handle_return_event(raw_event, key, returned=True)
        if event == EventType.RETURN_FAILED:
            return self._handle_return_event(raw_event, key, returned=False)
        if event == EventType.TARGET_LOST:
            return self._handle_target_lost(source, raw_event, key)
        if event == EventType.EXECUTION_ERROR:
            return self._handle_execution_error(source, raw_event, key)
        if self._task_state != ParentTaskState.TASK_RUNNING:
            return self._reject("mission event requires TASK_RUNNING")

        next_state = self._TRANSITIONS.get((self._state, source, event))
        if next_state is None:
            return self._reject(
                f"{event.name} from {source.name} is invalid in {self._state.name}"
            )
        target_id = int(raw_event.target_id)
        if event == EventType.TARGET_CONFIRMED:
            if target_id <= 0:
                return self._reject("TARGET_CONFIRMED requires a non-zero target_id")
        elif event == EventType.PATROL_RECOVERY_COMPLETE:
            if not self._recovery_target_matches(target_id):
                return self._reject("PATROL_RECOVERY_COMPLETE requires the active recovery target_id")
        elif self._target_id <= 0 or target_id != self._target_id:
            return self._reject(
                f"target_id {target_id} does not match active target {self._target_id}"
            )

        self._remember_event(key)
        previous_state = self._state
        if event == EventType.TARGET_CONFIRMED:
            self._target_id = target_id
            self._handled_target_id = 0
        elif event == EventType.AUTHORIZED:
            # Only successful verification grants the handled exemption.
            self._handled_target_id = target_id
        if next_state == GlobalState.PATROL:
            self._target_id = 0
            self._recovery_from_pause = False
        self._state = next_state
        self._navigation_mode = self._compute_navigation_mode()
        self._detail = f"{previous_state.name} -> {next_state.name}: {source.name}/{event.name}"
        self._advance_seq()
        return EventResult(True, True, False, self._detail, self.snapshot)

    def handle_task_command(self, raw_command: TaskCommandData) -> TaskCommandResult:
        command = self._parse_command(raw_command.command)
        if command is None:
            return self._command_reject("invalid_command", "unknown task command")
        if command == TaskCommand.START:
            return self._start_task(raw_command.route, raw_command.home_pose)
        if command == TaskCommand.PAUSE:
            return self._pause_task()
        if command == TaskCommand.RESUME:
            return self._resume_task(raw_command.route)
        if command == TaskCommand.RETURN:
            return self._return_task()
        return self._terminate_task()

    def handle_waypoint_state(self, route: WaypointStateData) -> EventResult:
        self._route = route
        if not self._valid_route_progress(route):
            return self._reject("invalid_route_progress")
        if (
            self._task_state not in {ParentTaskState.TASK_RUNNING, ParentTaskState.TASK_PAUSED}
            or route.waypoint_count <= 0
            or not route.sequence_done
        ):
            return EventResult(True, False, False, "waypoint state recorded", self.snapshot)
        snapshot = self._finish_task(TaskOutcome.TASK_OUTCOME_COMPLETED, "waypoint sequence completed")
        return EventResult(True, True, False, snapshot.detail, snapshot)

    def begin_patrol_recovery(self, detail: str) -> EventResult:
        if self._task_state != ParentTaskState.TASK_RUNNING:
            return self._reject("patrol recovery requires TASK_RUNNING")
        if self._state not in {
            GlobalState.CONFIRM_TARGET,
            GlobalState.APPROACH_TARGET,
            GlobalState.VERIFY_IDENTITY,
            GlobalState.TRACK_INTRUDER,
        }:
            return self._reject(f"patrol recovery is invalid in {self._state.name}")
        previous_state = self._state
        previous_target = self._target_id
        self._handled_target_id = 0
        self._recovery_from_pause = False
        self._state = GlobalState.RECOVER_PATROL
        self._navigation_mode = NavigationMode.NAVIGATION_MISSION
        reason = str(detail).strip() or "target task ended"
        self._detail = f"{previous_state.name} -> RECOVER_PATROL: {reason}; restoring patrol after target {previous_target}"
        self._advance_seq()
        return EventResult(True, True, False, self._detail, self.snapshot)

    def complete_patrol_recovery(self, detail: str) -> EventResult:
        if self._task_state != ParentTaskState.TASK_RUNNING or self._state != GlobalState.RECOVER_PATROL:
            return self._reject(f"patrol recovery completion is invalid in {self._state.name}")
        previous_target = self._target_id
        self._state = GlobalState.PATROL
        self._target_id = 0
        self._recovery_from_pause = False
        self._navigation_mode = NavigationMode.NAVIGATION_WAYPOINT
        reason = str(detail).strip() or "patrol recovery timeout"
        self._detail = f"RECOVER_PATROL -> PATROL: {reason}; released target {previous_target}"
        self._advance_seq()
        return EventResult(True, True, False, self._detail, self.snapshot)

    def _start_task(self, route: Optional[WaypointStateData], home_pose: Optional[HomePoseData]) -> TaskCommandResult:
        if not (self._task_state == ParentTaskState.TASK_NO_TASK and self._state == GlobalState.READY):
            return self._command_reject("system_not_ready", "START requires NO_TASK + READY")
        route_error = self._route_error(route)
        if route_error is not None:
            return self._command_reject(*route_error)
        if home_pose is None:
            return self._command_reject("home_pose_unavailable", "home pose is unavailable")
        if not home_pose.valid or home_pose.frame_id != "map":
            return self._command_reject("invalid_home_pose", "home pose is invalid")
        self._route = route
        self._home_pose = home_pose
        self._target_id = 0
        self._handled_target_id = 0
        self._resume_behavior = None
        self._recovery_from_pause = False
        self._last_task_outcome = TaskOutcome.TASK_OUTCOME_NONE
        self._task_state = ParentTaskState.TASK_RUNNING
        self._state = GlobalState.PATROL
        self._navigation_mode = NavigationMode.NAVIGATION_WAYPOINT
        self._detail = "START accepted; patrol task started"
        self._advance_seq()
        return self._command_accept("START accepted")

    def _pause_task(self) -> TaskCommandResult:
        if self._task_state != ParentTaskState.TASK_RUNNING:
            return self._command_reject("invalid_state_transition", "PAUSE requires TASK_RUNNING")
        self._resume_behavior = (
            ResumeBehavior.PATROL
            if self._state in {GlobalState.PATROL, GlobalState.CONFIRM_TARGET}
            else ResumeBehavior.RECOVER_PATROL
        )
        self._task_state = ParentTaskState.TASK_PAUSED
        self._state = GlobalState.READY
        self._navigation_mode = NavigationMode.NAVIGATION_STOP
        self._target_id = 0
        self._recovery_from_pause = False
        self._detail = "PAUSE accepted; parent task paused"
        self._advance_seq()
        return self._command_accept("PAUSE accepted")

    def _resume_task(self, route: Optional[WaypointStateData]) -> TaskCommandResult:
        if not (self._task_state == ParentTaskState.TASK_PAUSED and self._state == GlobalState.READY):
            return self._command_reject("invalid_state_transition", "RESUME requires PAUSED + READY")
        route = route if route is not None else self._route
        if route is None:
            return self._command_reject("route_unavailable", "waypoint state is unavailable")
        if route.waypoint_count <= 0:
            return self._command_reject("route_empty", "waypoint route is empty")
        if route.sequence_done:
            return self._command_reject("route_already_complete", "waypoint route is already complete")
        if not self._valid_route_progress(route):
            return self._command_reject("invalid_route_progress", "waypoint progress is invalid")
        behavior = self._resume_behavior or ResumeBehavior.PATROL
        self._route = route
        self._task_state = ParentTaskState.TASK_RUNNING
        self._target_id = 0
        self._state = GlobalState.RECOVER_PATROL if behavior == ResumeBehavior.RECOVER_PATROL else GlobalState.PATROL
        self._navigation_mode = self._compute_navigation_mode()
        self._recovery_from_pause = self._state == GlobalState.RECOVER_PATROL
        self._resume_behavior = None
        self._detail = "RESUME accepted; patrol task resumed"
        self._advance_seq()
        return self._command_accept("RESUME accepted")

    def _return_task(self) -> TaskCommandResult:
        if self._task_state == ParentTaskState.TASK_NO_TASK:
            return self._command_reject("no_active_task", "RETURN requires an active task")
        if self._task_state not in {ParentTaskState.TASK_RUNNING, ParentTaskState.TASK_PAUSED}:
            return self._command_reject("return_not_allowed", "RETURN is not allowed here")
        if self._home_pose is None or not self._home_pose.valid:
            return self._command_reject("return_not_allowed", "home pose is unavailable")
        self._task_state = ParentTaskState.TASK_RETURNING
        self._state = GlobalState.READY
        self._navigation_mode = NavigationMode.NAVIGATION_MISSION
        self._target_id = 0
        self._handled_target_id = 0
        self._resume_behavior = None
        self._recovery_from_pause = False
        self._detail = "RETURN accepted; returning to home pose"
        self._advance_seq()
        return self._command_accept("RETURN accepted")

    def _terminate_task(self) -> TaskCommandResult:
        if self._task_state == ParentTaskState.TASK_NO_TASK:
            return self._command_reject("no_active_task", "TERMINATE requires an active task")
        snapshot = self._finish_task(TaskOutcome.TASK_OUTCOME_TERMINATED, "TERMINATE accepted; parent task stopped")
        return self._command_accept("TERMINATE accepted", snapshot)

    def _handle_return_event(self, raw_event: MissionEventData, key: EventKey, *, returned: bool) -> EventResult:
        if raw_event.target_id != 0:
            return self._reject("return event requires target_id=0")
        if not (self._task_state == ParentTaskState.TASK_RETURNING and self._state == GlobalState.READY):
            return self._reject("return event requires TASK_RETURNING + READY")
        self._remember_event(key)
        outcome = TaskOutcome.TASK_OUTCOME_RETURNED if returned else TaskOutcome.TASK_OUTCOME_FAILED
        detail = str(raw_event.detail).strip() or ("return reached home" if returned else "return failed")
        snapshot = self._finish_task(outcome, detail)
        return EventResult(True, True, False, detail, snapshot)

    def _handle_ready(self, source: EventSource, raw_event: MissionEventData, key: EventKey) -> EventResult:
        if self._task_state != ParentTaskState.TASK_NO_TASK or self._state != GlobalState.STARTUP:
            return self._reject(f"READY is invalid in {self._state.name}")
        if int(raw_event.target_id) != 0:
            return self._reject("READY requires target_id=0")
        if source == EventSource.PERCEPTION:
            self._perception_ready = True
        else:
            self._navigation_ready = True
        self._remember_event(key)
        if not (self._perception_ready and self._navigation_ready):
            return EventResult(True, False, False, f"{source.name} ready recorded", self.snapshot)
        self._state = GlobalState.READY
        self._navigation_mode = NavigationMode.NAVIGATION_STOP
        self._detail = "STARTUP -> READY: perception and navigation ready"
        self._advance_seq()
        return EventResult(True, True, False, self._detail, self.snapshot)

    def _handle_target_lost(self, source: EventSource, raw_event: MissionEventData, key: EventKey) -> EventResult:
        error = self._validate_event_target(raw_event)
        if error:
            return self._reject(error)
        if self._task_state != ParentTaskState.TASK_RUNNING or self._target_id == 0:
            return self._reject("TARGET_LOST requires an active target task")
        if self._state not in {GlobalState.CONFIRM_TARGET, GlobalState.APPROACH_TARGET, GlobalState.VERIFY_IDENTITY, GlobalState.TRACK_INTRUDER}:
            return self._reject("TARGET_LOST requires an active target")
        self._remember_event(key)
        detail = str(raw_event.detail).strip() or "target lost"
        return self.begin_patrol_recovery(
            f"{source.name}/TARGET_LOST: {detail}",
        )

    def _handle_execution_error(self, source: EventSource, raw_event: MissionEventData, key: EventKey) -> EventResult:
        error = self._validate_event_target(raw_event)
        if error:
            return self._reject(error)
        self._remember_event(key)
        detail = str(raw_event.detail).strip() or "unspecified execution error"
        if self._task_state == ParentTaskState.TASK_RUNNING and self._target_id > 0:
            if self._state == GlobalState.RECOVER_PATROL:
                return self._reject(f"ignoring {source.name}/EXECUTION_ERROR during patrol recovery: {detail}")
            return self.begin_patrol_recovery(f"recoverable {source.name}/EXECUTION_ERROR: {detail}")
        return EventResult(True, False, False, f"recorded {source.name}/EXECUTION_ERROR without blocking: {detail}", self.snapshot)

    def _finish_task(self, outcome: TaskOutcome, detail: str) -> MissionSnapshot:
        if outcome == TaskOutcome.TASK_OUTCOME_NONE:
            raise ValueError("terminal outcome cannot be NONE")
        self._task_state = ParentTaskState.TASK_NO_TASK
        self._state = GlobalState.READY
        self._last_task_outcome = outcome
        self._navigation_mode = NavigationMode.NAVIGATION_STOP
        self._target_id = 0
        self._handled_target_id = 0
        self._home_pose = None
        self._resume_behavior = None
        self._recovery_from_pause = False
        self._detail = str(detail).strip() or outcome.name
        self._advance_seq()
        return self.snapshot

    def _recovery_target_matches(self, target_id: int) -> bool:
        if self._state != GlobalState.RECOVER_PATROL:
            return False
        if self._recovery_from_pause:
            return target_id == 0 and self._target_id == 0
        return self._target_id > 0 and target_id == self._target_id

    def _compute_navigation_mode(self) -> NavigationMode:
        if self._task_state == ParentTaskState.TASK_RETURNING:
            return NavigationMode.NAVIGATION_MISSION
        if self._task_state != ParentTaskState.TASK_RUNNING:
            return NavigationMode.NAVIGATION_STOP
        if self._state in {GlobalState.PATROL, GlobalState.CONFIRM_TARGET}:
            return NavigationMode.NAVIGATION_WAYPOINT
        return NavigationMode.NAVIGATION_MISSION

    @staticmethod
    def _route_error(route: Optional[WaypointStateData]) -> Optional[Tuple[str, str]]:
        if route is None:
            return ("route_unavailable", "waypoint state is unavailable")
        if route.waypoint_count <= 0:
            return ("route_empty", "waypoint route is empty")
        if route.sequence_done:
            return ("route_already_complete", "waypoint route is already complete")
        if not MissionStateMachine._valid_route_progress(route):
            return ("invalid_route_progress", "waypoint progress is invalid")
        return None

    @staticmethod
    def _valid_route_progress(route: WaypointStateData) -> bool:
        count = int(route.waypoint_count)
        index = int(route.current_index)
        completed = int(route.completed_count)
        if count < 0 or index < 0 or completed < 0:
            return False
        if count == 0:
            return index == 0 and completed == 0
        if route.sequence_done:
            return index == count - 1 and completed == count
        return 0 <= index < count and completed == index

    def _command_accept(self, message: str, snapshot: Optional[MissionSnapshot] = None) -> TaskCommandResult:
        return TaskCommandResult(True, True, message, "", snapshot or self.snapshot)

    def _command_reject(self, error_code: str, message: str) -> TaskCommandResult:
        return TaskCommandResult(False, False, message, error_code, self.snapshot)

    def _validate_event_target(self, raw_event: MissionEventData) -> Optional[str]:
        target_id = int(raw_event.target_id)
        if self._target_id == 0:
            if target_id != 0:
                return f"event target_id {target_id} is invalid without an active target"
        elif target_id != self._target_id:
            return f"target_id {target_id} does not match active target {self._target_id}"
        return None

    def _reject(self, reason: str) -> EventResult:
        return EventResult(False, False, False, reason, self.snapshot)

    def _remember_event(self, key: EventKey) -> None:
        if key in self._processed_events:
            return
        self._processed_events.add(key)
        self._processed_order.append(key)
        while len(self._processed_order) > self._processed_event_limit:
            old_key = self._processed_order.popleft()
            self._processed_events.discard(old_key)

    def _advance_seq(self) -> None:
        self._state_seq = (self._state_seq + 1) & 0xFFFFFFFF
        if self._state_seq == 0:
            self._state_seq = 1

    @staticmethod
    def _normalize_seq(value: int) -> int:
        normalized = int(value) & 0xFFFFFFFF
        return normalized if normalized else 1

    @staticmethod
    def _parse_source(value: int) -> Optional[EventSource]:
        try:
            return EventSource(int(value))
        except ValueError:
            return None

    @staticmethod
    def _parse_event(value: int) -> Optional[EventType]:
        try:
            return EventType(int(value))
        except ValueError:
            return None

    @staticmethod
    def _parse_command(value: int) -> Optional[TaskCommand]:
        try:
            return TaskCommand(int(value))
        except ValueError:
            return None
