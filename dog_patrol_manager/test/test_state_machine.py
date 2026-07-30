from dog_patrol_manager.state_machine import (
    BlockCause,
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


def test_authorized_flow_returns_to_patrol_and_clears_target():
    machine = MissionStateMachine()
    advance_to_verify(machine)

    result = machine.handle_event(
        event(machine, EventSource.PERCEPTION, EventType.AUTHORIZED, 87)
    )
    assert result.changed
    assert result.snapshot.state == GlobalState.PATROL
    assert result.snapshot.target_id == 0
    assert not result.snapshot.blocked


def test_unauthorized_flow_tracks_until_operator_completes():
    machine = MissionStateMachine()
    advance_to_verify(machine)

    result = machine.handle_event(
        event(machine, EventSource.PERCEPTION, EventType.UNAUTHORIZED, 87)
    )
    assert result.snapshot.state == GlobalState.TRACK_INTRUDER
    assert result.snapshot.target_id == 87

    result = machine.handle_event(
        event(machine, EventSource.OPERATOR, EventType.HANDLING_COMPLETE, 87)
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


def test_target_lost_blocks_without_changing_business_state():
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
    state_before = machine.snapshot.state
    seq_before = machine.snapshot.state_seq

    result = machine.handle_event(
        event(
            machine,
            EventSource.NAVIGATION,
            EventType.TARGET_LOST,
            42,
            "bbox timeout",
        )
    )
    assert result.changed
    assert result.snapshot.state == state_before
    assert result.snapshot.state_seq == seq_before + 1
    assert result.snapshot.blocked

    follow_up = MissionEventData(
        observed_state_seq=result.snapshot.state_seq,
        target_id=42,
        source=int(EventSource.NAVIGATION),
        event=int(EventType.TARGET_POSITION_READY),
    )
    assert not machine.handle_event(follow_up).accepted


def test_same_target_reacquired_clears_only_target_lost_block():
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
    state_before_loss = machine.snapshot.state

    loss = machine.handle_event(
        event(
            machine,
            EventSource.PERCEPTION,
            EventType.TARGET_LOST,
            42,
        )
    )
    assert loss.snapshot.block_cause == BlockCause.TARGET_LOST

    reacquired = machine.handle_event(
        event(
            machine,
            EventSource.PERCEPTION,
            EventType.TARGET_REACQUIRED,
            42,
        )
    )
    assert reacquired.accepted
    assert reacquired.changed
    assert reacquired.snapshot.state == state_before_loss
    assert reacquired.snapshot.target_id == 42
    assert not reacquired.snapshot.blocked
    assert reacquired.snapshot.block_cause == BlockCause.NONE
    assert reacquired.snapshot.state_seq == loss.snapshot.state_seq + 1


def test_target_reacquired_rejects_wrong_target_stale_duplicate_and_out_of_order():
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
            EventSource.PERCEPTION,
            EventType.TARGET_REACQUIRED,
            42,
        )
    )
    assert not out_of_order.accepted
    assert not out_of_order.changed

    loss = machine.handle_event(
        event(
            machine,
            EventSource.PERCEPTION,
            EventType.TARGET_LOST,
            42,
        )
    )
    state_before_rejections = machine.snapshot

    wrong_target = machine.handle_event(
        event(
            machine,
            EventSource.PERCEPTION,
            EventType.TARGET_REACQUIRED,
            99,
        )
    )
    assert not wrong_target.accepted
    assert machine.snapshot == state_before_rejections

    stale = MissionEventData(
        observed_state_seq=loss.snapshot.state_seq - 2,
        target_id=42,
        source=int(EventSource.PERCEPTION),
        event=int(EventType.TARGET_REACQUIRED),
    )
    assert not machine.handle_event(stale).accepted
    assert machine.snapshot == state_before_rejections

    reacquired_event = event(
        machine,
        EventSource.PERCEPTION,
        EventType.TARGET_REACQUIRED,
        42,
    )
    assert machine.handle_event(reacquired_event).accepted
    state_after_reacquisition = machine.snapshot

    duplicate = machine.handle_event(reacquired_event)
    assert not duplicate.accepted
    assert duplicate.duplicate
    assert machine.snapshot == state_after_reacquisition


def test_target_reacquired_cannot_clear_execution_error_block():
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
    error = machine.handle_event(
        event(
            machine,
            EventSource.NAVIGATION,
            EventType.EXECUTION_ERROR,
            42,
            "planner unavailable",
        )
    )
    assert error.snapshot.block_cause == BlockCause.EXECUTION_ERROR

    reacquired = machine.handle_event(
        event(
            machine,
            EventSource.PERCEPTION,
            EventType.TARGET_REACQUIRED,
            42,
        )
    )
    assert not reacquired.accepted
    assert machine.snapshot == error.snapshot


def test_blocked_mission_rejects_a_new_error_event():
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
    loss = machine.handle_event(
        event(
            machine,
            EventSource.PERCEPTION,
            EventType.TARGET_LOST,
            42,
        )
    )

    error = machine.handle_event(
        event(
            machine,
            EventSource.NAVIGATION,
            EventType.EXECUTION_ERROR,
            42,
        )
    )
    assert not error.accepted
    assert machine.snapshot == loss.snapshot


def test_target_lost_and_reacquired_cycles_preserve_active_mission():
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
    state_before_cycles = machine.snapshot.state
    seq_before_cycles = machine.snapshot.state_seq

    for cycle in range(2):
        loss = machine.handle_event(
            event(
                machine,
                EventSource.PERCEPTION,
                EventType.TARGET_LOST,
                42,
                f"cycle {cycle}",
            )
        )
        assert loss.accepted
        assert loss.snapshot.block_cause == BlockCause.TARGET_LOST

        reacquired = machine.handle_event(
            event(
                machine,
                EventSource.PERCEPTION,
                EventType.TARGET_REACQUIRED,
                42,
            )
        )
        assert reacquired.accepted
        assert reacquired.snapshot.state == state_before_cycles
        assert reacquired.snapshot.target_id == 42
        assert not reacquired.snapshot.blocked

    assert machine.snapshot.state_seq == seq_before_cycles + 4


def test_duplicate_ready_event_is_idempotent():
    machine = MissionStateMachine()
    ready = event(machine, EventSource.PERCEPTION, EventType.READY)
    assert machine.handle_event(ready).accepted

    duplicate = machine.handle_event(ready)
    assert not duplicate.accepted
    assert duplicate.duplicate
