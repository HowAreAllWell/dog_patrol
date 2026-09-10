from dataclasses import replace

import pytest

from dog_patrol_manager.state_machine import (
    EventSource,
    EventType,
    GlobalState,
    MissionEventData,
    MissionStateMachine,
)


def event(machine, source, event_type, target_id=0, detail=""):
    return MissionEventData(
        observed_state_seq=machine.snapshot.state_seq,
        target_id=target_id,
        source=int(source),
        event=int(event_type),
        detail=detail,
    )


def start_patrol(machine):
    result = machine.handle_event(
        event(machine, EventSource.PERCEPTION, EventType.READY)
    )
    assert result.accepted
    assert not result.changed

    result = machine.handle_event(
        event(machine, EventSource.NAVIGATION, EventType.READY)
    )
    assert result.changed
    assert result.snapshot.state == GlobalState.PATROL


def advance_to_verify(machine, target_id=87):
    start_patrol(machine)
    result = machine.handle_event(
        event(
            machine,
            EventSource.PERCEPTION,
            EventType.TARGET_CONFIRMED,
            target_id,
        )
    )
    assert result.snapshot.state == GlobalState.CONFIRM_TARGET

    result = machine.handle_event(
        event(
            machine,
            EventSource.NAVIGATION,
            EventType.TARGET_POSITION_READY,
            target_id,
        )
    )
    assert result.snapshot.state == GlobalState.APPROACH_TARGET

    result = machine.handle_event(
        event(
            machine,
            EventSource.NAVIGATION,
            EventType.ARRIVED_AND_STOPPED,
            target_id,
        )
    )
    assert result.snapshot.state == GlobalState.VERIFY_IDENTITY


def test_startup_requires_both_ready_events():
    machine = MissionStateMachine()
    first_seq = machine.snapshot.state_seq

    result = machine.handle_event(
        event(machine, EventSource.PERCEPTION, EventType.READY)
    )
    assert result.accepted
    assert machine.snapshot.state == GlobalState.STARTUP
    assert machine.snapshot.state_seq == first_seq

    result = machine.handle_event(
        event(machine, EventSource.NAVIGATION, EventType.READY)
    )
    assert result.changed
    assert machine.snapshot.state == GlobalState.PATROL
    assert machine.snapshot.state_seq == first_seq + 1


def test_authorized_flow_restores_patrol_before_clearing_target():
    machine = MissionStateMachine()
    advance_to_verify(machine)

    result = machine.handle_event(
        event(machine, EventSource.PERCEPTION, EventType.AUTHORIZED, 87)
    )
    assert result.changed
    assert result.snapshot.state == GlobalState.RECOVER_PATROL
    assert result.snapshot.target_id == 87
    assert result.snapshot.handled_target_id == 87

    result = machine.handle_event(
        event(
            machine,
            EventSource.NAVIGATION,
            EventType.PATROL_RECOVERY_COMPLETE,
            87,
        )
    )
    assert result.snapshot.state == GlobalState.PATROL
    assert result.snapshot.target_id == 0
    assert result.snapshot.handled_target_id == 87


def test_unauthorized_flow_recovers_when_target_is_lost():
    machine = MissionStateMachine()
    advance_to_verify(machine)

    result = machine.handle_event(
        event(machine, EventSource.PERCEPTION, EventType.UNAUTHORIZED, 87)
    )
    assert result.snapshot.state == GlobalState.TRACK_INTRUDER
    assert result.snapshot.target_id == 87
    assert result.snapshot.handled_target_id == 0

    result = machine.handle_event(
        event(machine, EventSource.PERCEPTION, EventType.TARGET_LOST, 87)
    )
    assert result.snapshot.state == GlobalState.RECOVER_PATROL
    assert result.snapshot.target_id == 87
    assert result.snapshot.handled_target_id == 87

    result = machine.handle_event(
        event(
            machine,
            EventSource.NAVIGATION,
            EventType.PATROL_RECOVERY_COMPLETE,
            87,
        )
    )
    assert result.snapshot.state == GlobalState.PATROL
    assert result.snapshot.target_id == 0
    assert result.snapshot.handled_target_id == 87


def test_stale_and_wrong_source_events_are_rejected():
    machine = MissionStateMachine()
    start_patrol(machine)

    stale = MissionEventData(
        observed_state_seq=machine.snapshot.state_seq - 1,
        target_id=12,
        source=int(EventSource.PERCEPTION),
        event=int(EventType.TARGET_CONFIRMED),
    )
    assert not machine.handle_event(stale).accepted

    wrong_source = event(
        machine,
        EventSource.NAVIGATION,
        EventType.TARGET_CONFIRMED,
        12,
    )
    assert not machine.handle_event(wrong_source).accepted
    assert machine.snapshot.state == GlobalState.PATROL


def test_target_lost_enters_patrol_recovery():
    machine = MissionStateMachine()
    start_patrol(machine)
    machine.handle_event(
        event(
            machine,
            EventSource.PERCEPTION,
            EventType.TARGET_CONFIRMED,
            42,
        )
    )
    seq_before = machine.snapshot.state_seq

    result = machine.handle_event(
        event(
            machine,
            EventSource.PERCEPTION,
            EventType.TARGET_LOST,
            42,
            "tracking timeout",
        )
    )
    assert result.changed
    assert result.snapshot.state == GlobalState.RECOVER_PATROL
    assert result.snapshot.target_id == 42
    assert result.snapshot.state_seq == seq_before + 1

    restored = machine.handle_event(
        event(
            machine,
            EventSource.NAVIGATION,
            EventType.PATROL_RECOVERY_COMPLETE,
            42,
        )
    )
    assert restored.snapshot.state == GlobalState.PATROL
    assert restored.snapshot.target_id == 0


def test_navigation_cannot_publish_target_lost():
    machine = MissionStateMachine()
    start_patrol(machine)
    machine.handle_event(
        event(
            machine,
            EventSource.PERCEPTION,
            EventType.TARGET_CONFIRMED,
            42,
        )
    )
    state_before = machine.snapshot

    result = machine.handle_event(
        event(
            machine,
            EventSource.NAVIGATION,
            EventType.TARGET_LOST,
            42,
            "bbox timeout",
        )
    )

    assert not result.accepted
    assert result.reason == "NAVIGATION is not allowed to publish TARGET_LOST"
    assert machine.snapshot == state_before


def test_recovery_complete_rejects_wrong_target_stale_and_out_of_order():
    machine = MissionStateMachine()
    start_patrol(machine)
    machine.handle_event(
        event(
            machine,
            EventSource.PERCEPTION,
            EventType.TARGET_CONFIRMED,
            42,
        )
    )

    out_of_order = machine.handle_event(
        event(
            machine,
            EventSource.NAVIGATION,
            EventType.PATROL_RECOVERY_COMPLETE,
            42,
        )
    )
    assert not out_of_order.accepted

    recovery = machine.begin_patrol_recovery("target lost")
    wrong_target = machine.handle_event(
        event(
            machine,
            EventSource.NAVIGATION,
            EventType.PATROL_RECOVERY_COMPLETE,
            99,
        )
    )
    assert not wrong_target.accepted

    stale = MissionEventData(
        observed_state_seq=recovery.snapshot.state_seq - 1,
        target_id=42,
        source=int(EventSource.NAVIGATION),
        event=int(EventType.PATROL_RECOVERY_COMPLETE),
    )
    assert not machine.handle_event(stale).accepted

    restored = machine.handle_event(
        event(
            machine,
            EventSource.NAVIGATION,
            EventType.PATROL_RECOVERY_COMPLETE,
            42,
        )
    )
    assert restored.accepted
    assert restored.snapshot.state == GlobalState.PATROL
    assert restored.snapshot.target_id == 0


def test_target_task_execution_error_enters_patrol_recovery():
    machine = MissionStateMachine()
    start_patrol(machine)
    machine.handle_event(
        event(
            machine,
            EventSource.PERCEPTION,
            EventType.TARGET_CONFIRMED,
            42,
        )
    )
    result = machine.handle_event(
        event(
            machine,
            EventSource.NAVIGATION,
            EventType.EXECUTION_ERROR,
            42,
            "planner unavailable",
        )
    )
    assert result.accepted
    assert result.snapshot.state == GlobalState.RECOVER_PATROL
    assert result.snapshot.target_id == 42


def test_recovery_execution_error_is_ignored_until_recovery_timeout():
    machine = MissionStateMachine()
    start_patrol(machine)
    machine.handle_event(
        event(
            machine,
            EventSource.PERCEPTION,
            EventType.TARGET_CONFIRMED,
            42,
        )
    )
    machine.begin_patrol_recovery("target lost")

    result = machine.handle_event(
        event(
            machine,
            EventSource.NAVIGATION,
            EventType.EXECUTION_ERROR,
            42,
            "restored patrol path unavailable",
        )
    )
    assert not result.accepted
    assert result.snapshot.state == GlobalState.RECOVER_PATROL


def test_system_execution_error_without_target_is_diagnostic_only():
    machine = MissionStateMachine()
    start_patrol(machine)
    before = machine.snapshot
    result = machine.handle_event(
        event(
            machine,
            EventSource.NAVIGATION,
            EventType.EXECUTION_ERROR,
            0,
            "localization unavailable",
        )
    )
    assert result.accepted
    assert not result.changed
    assert result.snapshot.state == GlobalState.PATROL
    assert result.snapshot.target_id == 0
    assert result.snapshot.state_seq == before.state_seq
    assert result.snapshot.detail == before.detail
    assert "without blocking" in result.reason


def test_confirm_timeout_uses_patrol_recovery_path():
    machine = MissionStateMachine()
    start_patrol(machine)
    machine.handle_event(
        event(machine, EventSource.PERCEPTION, EventType.TARGET_CONFIRMED, 42)
    )

    result = machine.begin_patrol_recovery("target position confirmation timed out")
    assert result.accepted
    assert result.snapshot.state == GlobalState.RECOVER_PATROL
    assert result.snapshot.target_id == 42
    assert result.snapshot.handled_target_id == 0


def test_recovery_timeout_returns_to_patrol_without_blocking():
    machine = MissionStateMachine()
    start_patrol(machine)
    machine.handle_event(
        event(machine, EventSource.PERCEPTION, EventType.TARGET_CONFIRMED, 42)
    )
    machine.begin_patrol_recovery("target position confirmation timed out")

    result = machine.complete_patrol_recovery("navigation did not restore patrol")
    assert result.accepted
    assert result.snapshot.state == GlobalState.PATROL
    assert result.snapshot.target_id == 0
    assert result.snapshot.handled_target_id == 0


def test_duplicate_ready_event_is_idempotent():
    machine = MissionStateMachine()
    ready = event(machine, EventSource.PERCEPTION, EventType.READY)
    assert machine.handle_event(ready).accepted

    duplicate = machine.handle_event(ready)
    assert not duplicate.accepted
    assert duplicate.duplicate


def test_reset_session_invalidates_readiness_and_active_target():
    machine = MissionStateMachine()
    start_patrol(machine)
    confirmed = machine.handle_event(
        event(machine, EventSource.PERCEPTION, EventType.TARGET_CONFIRMED, 42)
    )
    assert confirmed.accepted

    previous_seq = machine.snapshot.state_seq
    snapshot = machine.reset_session("navigation stopped")

    assert snapshot.state == GlobalState.STARTUP
    assert snapshot.target_id == 0
    assert not machine.perception_ready
    assert not machine.navigation_ready
    assert snapshot.state_seq == previous_seq + 1


def advance_to_state(machine, state, target_id=87):
    start_patrol(machine)
    transitions = (
        (EventSource.PERCEPTION, EventType.TARGET_CONFIRMED),
        (EventSource.NAVIGATION, EventType.TARGET_POSITION_READY),
        (EventSource.NAVIGATION, EventType.ARRIVED_AND_STOPPED),
        (EventSource.PERCEPTION, EventType.UNAUTHORIZED),
    )
    for source, event_type in transitions:
        result = machine.handle_event(
            event(machine, source, event_type, target_id)
        )
        assert result.accepted
        if result.snapshot.state == state:
            return
    raise AssertionError(f"cannot advance to {state}")


@pytest.mark.parametrize(
    "state",
    [
        GlobalState.CONFIRM_TARGET,
        GlobalState.APPROACH_TARGET,
        GlobalState.VERIFY_IDENTITY,
    ],
)
@pytest.mark.parametrize(
    "source,event_type",
    [
        (EventSource.PERCEPTION, EventType.TARGET_LOST),
        (EventSource.PERCEPTION, EventType.EXECUTION_ERROR),
        (EventSource.NAVIGATION, EventType.EXECUTION_ERROR),
    ],
)
def test_unfinished_target_tasks_never_grant_handled_exemption(
    state, source, event_type
):
    machine = MissionStateMachine()
    advance_to_state(machine, state)

    result = machine.handle_event(event(machine, source, event_type, 87))
    assert result.accepted
    assert result.snapshot.state == GlobalState.RECOVER_PATROL
    assert result.snapshot.handled_target_id == 0

    result = machine.handle_event(
        event(
            machine, EventSource.NAVIGATION, EventType.PATROL_RECOVERY_COMPLETE, 87
        )
    )
    assert result.accepted
    assert result.snapshot.state == GlobalState.PATROL
    assert result.snapshot.handled_target_id == 0


@pytest.mark.parametrize(
    "source", [EventSource.PERCEPTION, EventSource.NAVIGATION]
)
def test_intruder_execution_error_does_not_count_as_completed_pursuit(source):
    machine = MissionStateMachine()
    advance_to_state(machine, GlobalState.TRACK_INTRUDER)

    result = machine.handle_event(
        event(machine, source, EventType.EXECUTION_ERROR, 87)
    )
    assert result.accepted
    assert result.snapshot.state == GlobalState.RECOVER_PATROL
    assert result.snapshot.handled_target_id == 0


@pytest.mark.parametrize(
    "target_state", [GlobalState.VERIFY_IDENTITY, GlobalState.TRACK_INTRUDER]
)
@pytest.mark.parametrize(
    "source", [EventSource.PERCEPTION, EventSource.NAVIGATION]
)
def test_handled_outcome_survives_recovery_errors_and_watchdog(
    target_state, source
):
    machine = MissionStateMachine()
    advance_to_state(machine, target_state)
    outcome = (
        EventType.AUTHORIZED
        if target_state == GlobalState.VERIFY_IDENTITY
        else EventType.TARGET_LOST
    )
    outcome_event = event(machine, EventSource.PERCEPTION, outcome, 87)
    assert machine.handle_event(outcome_event).snapshot.handled_target_id == 87
    assert machine.handle_event(outcome_event).duplicate
    recovery = machine.snapshot

    rejected = machine.handle_event(
        event(machine, source, EventType.EXECUTION_ERROR, 87)
    )
    assert not rejected.accepted
    assert rejected.snapshot == recovery
    restored = machine.complete_patrol_recovery("recovery watchdog expired")
    assert restored.accepted
    assert restored.snapshot.state == GlobalState.PATROL
    assert restored.snapshot.target_id == 0
    assert restored.snapshot.handled_target_id == 87


@pytest.mark.parametrize(
    "bad_fields",
    [
        {"target_id": 99},
        {"source": int(EventSource.NAVIGATION)},
        {"observed_state_seq": 1},
    ],
)
@pytest.mark.parametrize(
    "target_state", [GlobalState.VERIFY_IDENTITY, GlobalState.TRACK_INTRUDER]
)
def test_invalid_outcome_cannot_mark_a_target_handled(target_state, bad_fields):
    machine = MissionStateMachine()
    advance_to_state(machine, target_state)
    outcome = (
        EventType.AUTHORIZED
        if target_state == GlobalState.VERIFY_IDENTITY
        else EventType.TARGET_LOST
    )
    before = machine.snapshot
    rejected = machine.handle_event(
        replace(event(machine, EventSource.PERCEPTION, outcome, 87), **bad_fields)
    )
    assert not rejected.accepted
    assert rejected.snapshot == before
    assert rejected.snapshot.handled_target_id == 0


def test_authorized_before_verification_is_not_a_handled_outcome():
    machine = MissionStateMachine()
    advance_to_state(machine, GlobalState.CONFIRM_TARGET)
    before = machine.snapshot
    result = machine.handle_event(
        event(machine, EventSource.PERCEPTION, EventType.AUTHORIZED, 87)
    )
    assert not result.accepted
    assert result.snapshot == before
    assert result.snapshot.handled_target_id == 0


@pytest.mark.parametrize("next_action", ["new_target", "reset"])
def test_handled_outcome_is_cleared_for_next_task_or_session(next_action):
    machine = MissionStateMachine()
    advance_to_verify(machine)
    authorized = event(machine, EventSource.PERCEPTION, EventType.AUTHORIZED, 87)
    machine.handle_event(authorized)
    restored = machine.complete_patrol_recovery("recovery watchdog expired")
    assert restored.snapshot.handled_target_id == 87

    if next_action == "new_target":
        result = machine.handle_event(
            event(machine, EventSource.PERCEPTION, EventType.TARGET_CONFIRMED, 99)
        )
        assert result.accepted
        assert result.snapshot.target_id == 99
        assert result.snapshot.state == GlobalState.CONFIRM_TARGET
    else:
        assert machine.reset_session().state == GlobalState.STARTUP

    assert machine.snapshot.handled_target_id == 0
    assert not machine.handle_event(authorized).accepted
    assert machine.snapshot.handled_target_id == 0


@pytest.mark.parametrize(
    "state",
    [
        GlobalState.CONFIRM_TARGET,
        GlobalState.APPROACH_TARGET,
        GlobalState.VERIFY_IDENTITY,
        GlobalState.TRACK_INTRUDER,
    ],
)
def test_generic_recovery_does_not_infer_handled_outcome_from_detail(state):
    machine = MissionStateMachine()
    advance_to_state(machine, state)

    result = machine.begin_patrol_recovery("failed to send AUTHORIZED event")
    assert result.accepted
    assert result.snapshot.state == GlobalState.RECOVER_PATROL
    assert result.snapshot.handled_target_id == 0
