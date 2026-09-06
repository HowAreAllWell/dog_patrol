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


def test_unauthorized_flow_recovers_when_target_is_lost():
    machine = MissionStateMachine()
    advance_to_verify(machine)

    result = machine.handle_event(
        event(machine, EventSource.PERCEPTION, EventType.UNAUTHORIZED, 87)
    )
    assert result.snapshot.state == GlobalState.TRACK_INTRUDER
    assert result.snapshot.target_id == 87

    result = machine.handle_event(
        event(machine, EventSource.PERCEPTION, EventType.TARGET_LOST, 87)
    )
    assert result.snapshot.state == GlobalState.RECOVER_PATROL
    assert result.snapshot.target_id == 87

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
