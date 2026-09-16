import pytest

from dog_patrol_manager.state_machine import (
    EventSource,
    EventType,
    GlobalState,
    HomePoseData,
    MissionEventData,
    MissionStateMachine,
    NavigationMode,
    ParentTaskState,
    TaskCommand,
    TaskCommandData,
    TaskOutcome,
    WaypointStateData,
)


def event(machine, source, event_type, target_id=0):
    return MissionEventData(
        observed_state_seq=machine.snapshot.state_seq,
        target_id=target_id,
        source=int(source),
        event=int(event_type),
    )


def route(count=3, index=0, completed=0, done=False, revision=1):
    return WaypointStateData(revision, count, index, completed, done, True)


def home():
    return HomePoseData(pose=(1.0, 2.0, 0.0))


def make_ready(machine):
    machine.handle_event(event(machine, EventSource.PERCEPTION, EventType.READY))
    result = machine.handle_event(event(machine, EventSource.NAVIGATION, EventType.READY))
    assert result.snapshot.state == GlobalState.READY


def start(machine):
    make_ready(machine)
    result = machine.handle_task_command(
        TaskCommandData(TaskCommand.START, route(), home())
    )
    assert result.accepted
    return result


def advance_to(machine, target_state, target_id=87):
    start(machine)
    transitions = (
        (EventSource.PERCEPTION, EventType.TARGET_CONFIRMED),
        (EventSource.NAVIGATION, EventType.TARGET_POSITION_READY),
        (EventSource.NAVIGATION, EventType.ARRIVED_AND_STOPPED),
        (EventSource.PERCEPTION, EventType.UNAUTHORIZED),
    )
    if target_state == GlobalState.PATROL:
        return
    for source, event_type in transitions:
        result = machine.handle_event(event(machine, source, event_type, target_id))
        assert result.accepted
        if result.snapshot.state == target_state:
            return
    raise AssertionError(f"cannot reach {target_state}")


def test_ready_does_not_start_parent_task():
    machine = MissionStateMachine()
    make_ready(machine)
    snapshot = machine.snapshot
    assert snapshot.task_state == ParentTaskState.TASK_NO_TASK
    assert snapshot.state == GlobalState.READY
    assert snapshot.navigation_mode == NavigationMode.NAVIGATION_STOP


@pytest.mark.parametrize(
    "route_state,error_code",
    [
        (None, "route_unavailable"),
        (route(count=0), "route_empty"),
        (route(index=2, completed=3, done=True), "route_already_complete"),
        (route(index=2, completed=0), "invalid_route_progress"),
    ],
)
def test_start_rejects_unusable_route_without_side_effect(route_state, error_code):
    machine = MissionStateMachine()
    make_ready(machine)
    before = machine.snapshot
    result = machine.handle_task_command(
        TaskCommandData(TaskCommand.START, route_state, home())
    )
    assert not result.accepted
    assert result.error_code == error_code
    assert result.snapshot == before


def test_start_requires_valid_home_pose():
    machine = MissionStateMachine()
    make_ready(machine)
    before = machine.snapshot
    result = machine.handle_task_command(TaskCommandData(TaskCommand.START, route()))
    assert result.error_code == "home_pose_unavailable"
    assert result.snapshot == before
    result = machine.handle_task_command(
        TaskCommandData(
            TaskCommand.START,
            route(),
            HomePoseData(pose=None, frame_id="odom", valid=False),
        )
    )
    assert result.error_code == "invalid_home_pose"
    assert result.snapshot == before


def test_start_creates_running_patrol_and_captures_home():
    machine = MissionStateMachine()
    result = start(machine)
    snapshot = result.snapshot
    assert snapshot.task_state == ParentTaskState.TASK_RUNNING
    assert snapshot.state == GlobalState.PATROL
    assert snapshot.navigation_mode == NavigationMode.NAVIGATION_WAYPOINT
    assert snapshot.last_task_outcome == TaskOutcome.TASK_OUTCOME_NONE
    assert snapshot.home_pose == home()


@pytest.mark.parametrize(
    "paused_from,resumed_state,resumed_mode",
    [
        (GlobalState.PATROL, GlobalState.PATROL, NavigationMode.NAVIGATION_WAYPOINT),
        (GlobalState.CONFIRM_TARGET, GlobalState.PATROL, NavigationMode.NAVIGATION_WAYPOINT),
        (GlobalState.APPROACH_TARGET, GlobalState.RECOVER_PATROL, NavigationMode.NAVIGATION_MISSION),
        (GlobalState.VERIFY_IDENTITY, GlobalState.RECOVER_PATROL, NavigationMode.NAVIGATION_MISSION),
        (GlobalState.TRACK_INTRUDER, GlobalState.RECOVER_PATROL, NavigationMode.NAVIGATION_MISSION),
    ],
)
def test_pause_resume_follows_documented_recovery_policy(
    paused_from, resumed_state, resumed_mode
):
    machine = MissionStateMachine()
    advance_to(machine, paused_from)
    pause = machine.handle_task_command(TaskCommandData(TaskCommand.PAUSE))
    assert pause.accepted
    assert pause.snapshot.task_state == ParentTaskState.TASK_PAUSED
    assert pause.snapshot.state == GlobalState.READY
    assert pause.snapshot.target_id == 0
    assert pause.snapshot.navigation_mode == NavigationMode.NAVIGATION_STOP
    assert pause.snapshot.home_pose == home()

    resume = machine.handle_task_command(
        TaskCommandData(TaskCommand.RESUME, route())
    )
    assert resume.accepted
    assert resume.snapshot.task_state == ParentTaskState.TASK_RUNNING
    assert resume.snapshot.state == resumed_state
    assert resume.snapshot.navigation_mode == resumed_mode
    assert resume.snapshot.target_id == 0

    if resumed_state == GlobalState.RECOVER_PATROL:
        completed = machine.handle_event(
            event(machine, EventSource.NAVIGATION, EventType.PATROL_RECOVERY_COMPLETE, 0)
        )
        assert completed.accepted
        assert completed.snapshot.state == GlobalState.PATROL


@pytest.mark.parametrize(
    "route_state,error_code",
    [
        (route(count=0), "route_empty"),
        (route(index=2, completed=3, done=True), "route_already_complete"),
        (route(index=1, completed=0), "invalid_route_progress"),
    ],
)
def test_resume_rejects_invalid_route_and_stays_paused(route_state, error_code):
    machine = MissionStateMachine()
    start(machine)
    machine.handle_task_command(TaskCommandData(TaskCommand.PAUSE))
    before = machine.snapshot
    result = machine.handle_task_command(
        TaskCommandData(TaskCommand.RESUME, route_state)
    )
    assert not result.accepted
    assert result.error_code == error_code
    assert result.snapshot == before


@pytest.mark.parametrize(
    "final_event,outcome",
    [
        (EventType.RETURN_REACHED, TaskOutcome.TASK_OUTCOME_RETURNED),
        (EventType.RETURN_FAILED, TaskOutcome.TASK_OUTCOME_FAILED),
    ],
)
def test_return_events_form_one_terminal_outcome(final_event, outcome):
    machine = MissionStateMachine()
    start(machine)
    returning = machine.handle_task_command(TaskCommandData(TaskCommand.RETURN))
    assert returning.snapshot.task_state == ParentTaskState.TASK_RETURNING
    assert returning.snapshot.navigation_mode == NavigationMode.NAVIGATION_MISSION
    returned_seq = returning.snapshot.state_seq
    result = machine.handle_event(
        event(machine, EventSource.NAVIGATION, final_event, 0)
    )
    assert result.accepted
    assert result.snapshot.task_state == ParentTaskState.TASK_NO_TASK
    assert result.snapshot.last_task_outcome == outcome
    assert result.snapshot.navigation_mode == NavigationMode.NAVIGATION_STOP
    assert result.snapshot.home_pose is None
    assert result.snapshot.state_seq != returned_seq

    late = MissionEventData(
        observed_state_seq=returned_seq,
        target_id=0,
        source=EventSource.NAVIGATION,
        event=EventType.RETURN_REACHED,
    )
    assert not machine.handle_event(late).accepted
    assert machine.snapshot.last_task_outcome == outcome


@pytest.mark.parametrize(
    "before_command",
    [None, TaskCommand.PAUSE, TaskCommand.RETURN],
)
def test_terminate_closes_running_paused_and_returning(before_command):
    machine = MissionStateMachine()
    start(machine)
    if before_command is not None:
        machine.handle_task_command(TaskCommandData(before_command))
    result = machine.handle_task_command(TaskCommandData(TaskCommand.TERMINATE))
    assert result.accepted
    assert result.snapshot.task_state == ParentTaskState.TASK_NO_TASK
    assert result.snapshot.last_task_outcome == TaskOutcome.TASK_OUTCOME_TERMINATED
    assert result.snapshot.home_pose is None


@pytest.mark.parametrize("paused", [False, True])
def test_nonempty_completion_closes_running_or_paused_task(paused):
    machine = MissionStateMachine()
    start(machine)
    if paused:
        machine.handle_task_command(TaskCommandData(TaskCommand.PAUSE))
    before_seq = machine.snapshot.state_seq
    result = machine.handle_waypoint_state(
        route(index=2, completed=3, done=True)
    )
    assert result.changed
    assert result.snapshot.task_state == ParentTaskState.TASK_NO_TASK
    assert result.snapshot.last_task_outcome == TaskOutcome.TASK_OUTCOME_COMPLETED
    assert result.snapshot.state_seq != before_seq


def test_empty_route_never_means_completed():
    machine = MissionStateMachine()
    start(machine)
    before = machine.snapshot
    result = machine.handle_waypoint_state(route(count=0, done=True))
    assert result.accepted
    assert not result.changed
    assert result.snapshot == before


def test_state_sequence_wraps_to_one_on_terminal_transition():
    machine = MissionStateMachine(initial_state_seq=0xFFFFFFFD)
    make_ready(machine)
    started = machine.handle_task_command(
        TaskCommandData(TaskCommand.START, route(), home())
    )
    assert started.snapshot.state_seq == 0xFFFFFFFF
    paused = machine.handle_task_command(TaskCommandData(TaskCommand.PAUSE))
    assert paused.snapshot.state_seq == 1
