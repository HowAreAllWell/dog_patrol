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


def test_duplicate_ready_event_is_idempotent():
    machine = MissionStateMachine()
    ready = event(machine, EventSource.PERCEPTION, EventType.READY)
    assert machine.handle_event(ready).accepted

    duplicate = machine.handle_event(ready)
    assert not duplicate.accepted
    assert duplicate.duplicate
