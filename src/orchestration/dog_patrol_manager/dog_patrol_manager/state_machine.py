from collections import deque
from dataclasses import dataclass
from enum import IntEnum
from typing import Deque, Dict, FrozenSet, Optional, Set, Tuple


class GlobalState(IntEnum):
    STARTUP = 0
    PATROL = 1
    CONFIRM_TARGET = 2
    APPROACH_TARGET = 3
    VERIFY_IDENTITY = 4
    TRACK_INTRUDER = 5
    RECOVER_PATROL = 6


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


@dataclass(frozen=True)
class MissionEventData:
    observed_state_seq: int
    target_id: int
    source: int
    event: int
    detail: str = ""


@dataclass(frozen=True)
class MissionSnapshot:
    state_seq: int
    state: GlobalState
    target_id: int
    detail: str
    # Keep the completed outcome through recovery and the following PATROL.
    handled_target_id: int = 0


@dataclass(frozen=True)
class EventResult:
    accepted: bool
    changed: bool
    duplicate: bool
    reason: str
    snapshot: MissionSnapshot


EventKey = Tuple[int, int, int, int]
TransitionKey = Tuple[GlobalState, EventSource, EventType]


class MissionStateMachine:
    """Deterministic business state machine independent of ROS transport."""

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
    }

    _TRANSITIONS: Dict[TransitionKey, GlobalState] = {
        (
            GlobalState.PATROL,
            EventSource.PERCEPTION,
            EventType.TARGET_CONFIRMED,
        ): GlobalState.CONFIRM_TARGET,
        (
            GlobalState.CONFIRM_TARGET,
            EventSource.NAVIGATION,
            EventType.TARGET_POSITION_READY,
        ): GlobalState.APPROACH_TARGET,
        (
            GlobalState.APPROACH_TARGET,
            EventSource.NAVIGATION,
            EventType.ARRIVED_AND_STOPPED,
        ): GlobalState.VERIFY_IDENTITY,
        (
            GlobalState.VERIFY_IDENTITY,
            EventSource.PERCEPTION,
            EventType.AUTHORIZED,
        ): GlobalState.RECOVER_PATROL,
        (
            GlobalState.VERIFY_IDENTITY,
            EventSource.PERCEPTION,
            EventType.UNAUTHORIZED,
        ): GlobalState.TRACK_INTRUDER,
        (
            GlobalState.RECOVER_PATROL,
            EventSource.NAVIGATION,
            EventType.PATROL_RECOVERY_COMPLETE,
        ): GlobalState.PATROL,
    }

    def __init__(self, initial_state_seq: int = 1, processed_event_limit: int = 256):
        self._state_seq = max(1, int(initial_state_seq))
        self._state = GlobalState.STARTUP
        self._target_id = 0
        self._handled_target_id = 0
        self._detail = "waiting for perception and navigation ready"

        self._perception_ready = False
        self._navigation_ready = False

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
        )

    @property
    def perception_ready(self) -> bool:
        return self._perception_ready

    @property
    def navigation_ready(self) -> bool:
        return self._navigation_ready

    def reset_session(self, detail: str = "mission session reset") -> MissionSnapshot:
        """Return to STARTUP without restarting the ROS node."""
        self._state = GlobalState.STARTUP
        self._target_id = 0
        self._handled_target_id = 0
        self._perception_ready = False
        self._navigation_ready = False
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

        key = (
            int(source),
            int(event),
            int(raw_event.observed_state_seq),
            int(raw_event.target_id),
        )
        if key in self._processed_events:
            return EventResult(
                accepted=False,
                changed=False,
                duplicate=True,
                reason="duplicate event",
                snapshot=self.snapshot,
            )

        if source not in self._EVENT_SOURCES[event]:
            return self._reject(
                f"{source.name} is not allowed to publish {event.name}"
            )

        if int(raw_event.observed_state_seq) != self._state_seq:
            return self._reject(
                f"stale state_seq {raw_event.observed_state_seq}, "
                f"current is {self._state_seq}"
            )

        if event == EventType.TARGET_LOST:
            return self._handle_target_lost(source, raw_event, key)

        if event == EventType.EXECUTION_ERROR:
            return self._handle_execution_error(source, raw_event, key)

        if event == EventType.READY:
            return self._handle_ready(source, raw_event, key)

        transition_key = (self._state, source, event)
        next_state = self._TRANSITIONS.get(transition_key)
        if next_state is None:
            return self._reject(
                f"{event.name} from {source.name} is invalid in {self._state.name}"
            )

        target_id = int(raw_event.target_id)
        if event == EventType.TARGET_CONFIRMED:
            if target_id <= 0:
                return self._reject("TARGET_CONFIRMED requires a non-zero target_id")
        elif event == EventType.PATROL_RECOVERY_COMPLETE:
            if (
                self._state != GlobalState.RECOVER_PATROL
                or self._target_id <= 0
                or target_id != self._target_id
            ):
                return self._reject(
                    "PATROL_RECOVERY_COMPLETE requires the active recovery target_id"
                )
        elif self._target_id <= 0 or target_id != self._target_id:
            return self._reject(
                f"target_id {target_id} does not match active target "
                f"{self._target_id}"
            )

        self._remember_event(key)
        previous_state = self._state
        if event == EventType.TARGET_CONFIRMED:
            self._target_id = target_id
            self._handled_target_id = 0
        elif event == EventType.AUTHORIZED:
            self._handled_target_id = target_id
        if next_state == GlobalState.PATROL:
            self._target_id = 0

        self._state = next_state
        self._detail = (
            f"{previous_state.name} -> {next_state.name}: "
            f"{source.name}/{event.name}"
        )
        self._advance_seq()
        return EventResult(
            accepted=True,
            changed=True,
            duplicate=False,
            reason=self._detail,
            snapshot=self.snapshot,
        )

    def _handle_ready(
        self,
        source: EventSource,
        raw_event: MissionEventData,
        key: EventKey,
    ) -> EventResult:
        if self._state != GlobalState.STARTUP:
            return self._reject(f"READY is invalid in {self._state.name}")
        if int(raw_event.target_id) != 0:
            return self._reject("READY requires target_id=0")

        if source == EventSource.PERCEPTION:
            self._perception_ready = True
        elif source == EventSource.NAVIGATION:
            self._navigation_ready = True

        self._remember_event(key)
        if not (self._perception_ready and self._navigation_ready):
            return EventResult(
                accepted=True,
                changed=False,
                duplicate=False,
                reason=f"{source.name} ready recorded",
                snapshot=self.snapshot,
            )

        self._state = GlobalState.PATROL
        self._target_id = 0
        self._detail = "STARTUP -> PATROL: perception and navigation ready"
        self._advance_seq()
        return EventResult(
            accepted=True,
            changed=True,
            duplicate=False,
            reason=self._detail,
            snapshot=self.snapshot,
        )

    def begin_patrol_recovery(
        self, detail: str, *, target_handled: bool = False
    ) -> EventResult:
        """Restore patrol; failed tasks do not grant a handled-target exemption."""
        if self._state not in {
            GlobalState.CONFIRM_TARGET,
            GlobalState.APPROACH_TARGET,
            GlobalState.VERIFY_IDENTITY,
            GlobalState.TRACK_INTRUDER,
        }:
            return self._reject(
                f"patrol recovery is invalid in {self._state.name}"
            )

        previous_state = self._state
        previous_target = self._target_id
        self._handled_target_id = previous_target if target_handled else 0
        self._state = GlobalState.RECOVER_PATROL
        reason = str(detail).strip() or "target task ended"
        self._detail = (
            f"{previous_state.name} -> RECOVER_PATROL: {reason}; "
            f"restoring patrol after target {previous_target}"
        )
        self._advance_seq()
        return EventResult(
            accepted=True,
            changed=True,
            duplicate=False,
            reason=self._detail,
            snapshot=self.snapshot,
        )

    def complete_patrol_recovery(self, detail: str) -> EventResult:
        """Return to patrol even when the recovery path did not arrive in time."""
        if self._state != GlobalState.RECOVER_PATROL:
            return self._reject(
                f"patrol recovery completion is invalid in {self._state.name}"
            )

        previous_target = self._target_id
        self._state = GlobalState.PATROL
        self._target_id = 0
        reason = str(detail).strip() or "patrol recovery timeout"
        self._detail = (
            f"RECOVER_PATROL -> PATROL: {reason}; "
            f"released target {previous_target}"
        )
        self._advance_seq()
        return EventResult(
            accepted=True,
            changed=True,
            duplicate=False,
            reason=self._detail,
            snapshot=self.snapshot,
        )

    def _validate_event_target(self, raw_event: MissionEventData) -> Optional[str]:
        target_id = int(raw_event.target_id)
        if self._target_id == 0:
            if target_id != 0:
                return f"event target_id {target_id} is invalid without an active target"
        elif target_id != self._target_id:
            return (
                f"target_id {target_id} does not match active target "
                f"{self._target_id}"
            )
        return None

    def _handle_target_lost(
        self,
        source: EventSource,
        raw_event: MissionEventData,
        key: EventKey,
    ) -> EventResult:
        error = self._validate_event_target(raw_event)
        if error:
            return self._reject(error)
        if self._target_id == 0 or self._state not in {
            GlobalState.CONFIRM_TARGET,
            GlobalState.APPROACH_TARGET,
            GlobalState.VERIFY_IDENTITY,
            GlobalState.TRACK_INTRUDER,
        }:
            return self._reject("TARGET_LOST requires an active target")

        self._remember_event(key)
        detail = str(raw_event.detail).strip() or "target lost"
        return self.begin_patrol_recovery(
            f"{source.name}/TARGET_LOST: {detail}",
            # Ending an intruder pursuit is a completed business outcome;
            # losing a target before that stage is an unfinished task.
            target_handled=self._state == GlobalState.TRACK_INTRUDER,
        )

    def _handle_execution_error(
        self,
        source: EventSource,
        raw_event: MissionEventData,
        key: EventKey,
    ) -> EventResult:
        error = self._validate_event_target(raw_event)
        if error:
            return self._reject(error)

        self._remember_event(key)
        detail = str(raw_event.detail).strip() or "unspecified execution error"
        if self._target_id > 0:
            if self._state == GlobalState.RECOVER_PATROL:
                return self._reject(
                    f"ignoring {source.name}/EXECUTION_ERROR during patrol recovery: {detail}"
                )
            return self.begin_patrol_recovery(
                f"recoverable {source.name}/EXECUTION_ERROR: {detail}"
            )

        # Outside a target task, EXECUTION_ERROR is diagnostic only. Keep the
        # current mission authoritative and let the navigation stack continue
        # its own retry/recovery behavior instead of creating a permanent block.
        return EventResult(
            accepted=True,
            changed=False,
            duplicate=False,
            reason=(
                f"recorded {source.name}/EXECUTION_ERROR without blocking: {detail}"
            ),
            snapshot=self.snapshot,
        )

    def _advance_seq(self) -> None:
        self._state_seq = (self._state_seq + 1) & 0xFFFFFFFF
        if self._state_seq == 0:
            self._state_seq = 1

    def _remember_event(self, key: EventKey) -> None:
        if key in self._processed_events:
            return
        self._processed_events.add(key)
        self._processed_order.append(key)
        while len(self._processed_order) > self._processed_event_limit:
            old_key = self._processed_order.popleft()
            self._processed_events.discard(old_key)

    def _reject(self, reason: str) -> EventResult:
        return EventResult(
            accepted=False,
            changed=False,
            duplicate=False,
            reason=reason,
            snapshot=self.snapshot,
        )

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
