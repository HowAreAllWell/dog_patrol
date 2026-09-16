from collections import deque
import os
from pathlib import Path
import subprocess
import sys
import time
from types import MethodType, SimpleNamespace
import threading

from dog_patrol_interfaces.msg import MissionState, PatrolTaskState
from dog_patrol_interfaces.srv import PatrolTaskCommand

from robot_console.ros2_node import RobotBackendNode
from robot_console.main_window import RobotMainWindow, task_command_permissions


class RecordingSignal:
    def __init__(self):
        self.values = []

    def emit(self, *values):
        self.values.append(values)


def backend_state_node():
    state_signal = RecordingSignal()
    log_signal = RecordingSignal()
    node = SimpleNamespace(
        signals=SimpleNamespace(
            task_state_updated=state_signal,
            log_msg=log_signal,
        ),
        _task_state_seq=None,
        _task_state_signature=None,
        _task_state_conflicted=False,
        _task_state_timed_out=False,
        _last_task_state_monotonic=None,
        _mission_state_seq=None,
        _mission_state_signature=None,
        _mission_state_conflicted=False,
        _last_mission_state_monotonic=None,
        _task_state_timeout=0.75,
        _route_valid=True,
    )
    node._states_fresh = MethodType(RobotBackendNode._states_fresh, node)
    node._emit_invalid_task_state = MethodType(
        RobotBackendNode._emit_invalid_task_state, node
    )
    node._combined_snapshot_is_valid = RobotBackendNode._combined_snapshot_is_valid
    node._emit_combined_task_state = MethodType(
        RobotBackendNode._emit_combined_task_state, node
    )
    return node, state_signal, log_signal


def test_task_state_display_rejects_stale_sequence_across_wrap() -> None:
    node, signal, _ = backend_state_node()

    newest_before_wrap = SimpleNamespace(
        state_seq=0xFFFFFFFF,
        task_state=PatrolTaskState.TASK_RUNNING,
        last_task_outcome=PatrolTaskState.TASK_OUTCOME_NONE,
        navigation_mode=PatrolTaskState.NAVIGATION_WAYPOINT,
        home_pose_valid=True,
    )
    newest_after_wrap = SimpleNamespace(
        state_seq=1,
        task_state=PatrolTaskState.TASK_PAUSED,
        last_task_outcome=PatrolTaskState.TASK_OUTCOME_NONE,
        navigation_mode=PatrolTaskState.NAVIGATION_STOP,
        home_pose_valid=True,
    )

    RobotBackendNode.mission_state_callback(
        node,
        SimpleNamespace(
            state_seq=0xFFFFFFFF,
            state=MissionState.PATROL,
            target_id=0,
            handled_target_id=0,
        ),
    )
    RobotBackendNode.task_state_callback(node, newest_before_wrap)
    RobotBackendNode.mission_state_callback(
        node,
        SimpleNamespace(
            state_seq=1,
            state=MissionState.READY,
            target_id=0,
            handled_target_id=0,
        ),
    )
    RobotBackendNode.task_state_callback(node, newest_after_wrap)
    RobotBackendNode.task_state_callback(node, newest_before_wrap)

    valid = [item for item in signal.values if item[1] >= 0]
    assert valid == [
        (
            0xFFFFFFFF,
            PatrolTaskState.TASK_RUNNING,
            PatrolTaskState.TASK_OUTCOME_NONE,
            PatrolTaskState.NAVIGATION_WAYPOINT,
            MissionState.PATROL,
            True,
        ),
        (
            1,
            PatrolTaskState.TASK_PAUSED,
            PatrolTaskState.TASK_OUTCOME_NONE,
            PatrolTaskState.NAVIGATION_STOP,
            MissionState.READY,
            True,
        ),
    ]


class PendingFuture:
    def __init__(self):
        self.callback = None

    def add_done_callback(self, callback):
        self.callback = callback

    def result(self):
        return SimpleNamespace(accepted=True, error_code="", message="accepted")

    def complete(self):
        self.callback(self)


class RecordingClient:
    def __init__(self):
        self.futures = []

    def service_is_ready(self):
        return True

    def call_async(self, request):
        future = PendingFuture()
        self.futures.append((request, future))
        return future


def test_task_commands_are_sent_serially() -> None:
    client = RecordingClient()
    result_signal = RecordingSignal()
    guard = SimpleNamespace(trigger=lambda: None)
    node = SimpleNamespace(
        _task_commands=deque(
            [PatrolTaskCommand.Request.START, PatrolTaskCommand.Request.PAUSE]
        ),
        _task_command_lock=threading.Lock(),
        _task_command_in_flight=None,
        task_command_client=client,
        task_command_guard=guard,
        signals=SimpleNamespace(task_command_result=result_signal),
    )
    node._on_task_command_result = MethodType(
        RobotBackendNode._on_task_command_result, node
    )

    RobotBackendNode._dispatch_task_command(node)
    RobotBackendNode._dispatch_task_command(node)
    assert len(client.futures) == 1

    client.futures[0][1].complete()
    RobotBackendNode._dispatch_task_command(node)
    assert [item[0].command for item in client.futures] == [
        PatrolTaskCommand.Request.START,
        PatrolTaskCommand.Request.PAUSE,
    ]
    assert result_signal.values == [(True, "accepted")]


def test_console_disables_commands_for_unknown_or_illegal_snapshot() -> None:
    assert not any(task_command_permissions(99, 99).values())
    assert not any(task_command_permissions(0, PatrolTaskState.NAVIGATION_WAYPOINT).values())
    assert not task_command_permissions(0, PatrolTaskState.NAVIGATION_STOP)["start"]
    assert not task_command_permissions(
        0,
        PatrolTaskState.NAVIGATION_STOP,
        mission_state=MissionState.STARTUP,
        route_valid=True,
    )["start"]
    assert task_command_permissions(
        0,
        PatrolTaskState.NAVIGATION_STOP,
        mission_state=MissionState.READY,
        route_valid=True,
    )["start"]


def test_same_sequence_conflict_disables_console_until_newer_snapshot() -> None:
    node, state_signal, log_signal = backend_state_node()
    running = SimpleNamespace(
        state_seq=30,
        task_state=PatrolTaskState.TASK_RUNNING,
        last_task_outcome=PatrolTaskState.TASK_OUTCOME_NONE,
        navigation_mode=PatrolTaskState.NAVIGATION_WAYPOINT,
        home_pose_valid=True,
    )
    conflicting = SimpleNamespace(
        state_seq=30,
        task_state=PatrolTaskState.TASK_PAUSED,
        last_task_outcome=PatrolTaskState.TASK_OUTCOME_NONE,
        navigation_mode=PatrolTaskState.NAVIGATION_STOP,
        home_pose_valid=True,
    )
    newer = SimpleNamespace(
        state_seq=31,
        task_state=PatrolTaskState.TASK_PAUSED,
        last_task_outcome=PatrolTaskState.TASK_OUTCOME_NONE,
        navigation_mode=PatrolTaskState.NAVIGATION_STOP,
        home_pose_valid=True,
    )

    RobotBackendNode.mission_state_callback(
        node,
        SimpleNamespace(
            state_seq=30,
            state=MissionState.PATROL,
            target_id=0,
            handled_target_id=0,
        ),
    )
    RobotBackendNode.task_state_callback(node, running)
    RobotBackendNode.task_state_callback(node, conflicting)
    RobotBackendNode.task_state_callback(node, running)
    RobotBackendNode.mission_state_callback(
        node,
        SimpleNamespace(
            state_seq=31,
            state=MissionState.READY,
            target_id=0,
            handled_target_id=0,
        ),
    )
    RobotBackendNode.task_state_callback(node, newer)

    assert any(item[0] == 30 and item[1] == -1 for item in state_signal.values)
    assert state_signal.values[-1] == (
        31,
        PatrolTaskState.TASK_PAUSED,
        PatrolTaskState.TASK_OUTCOME_NONE,
        PatrolTaskState.NAVIGATION_STOP,
        MissionState.READY,
        True,
    )
    assert log_signal.values


def test_task_state_heartbeat_timeout_disables_and_same_snapshot_recovers() -> None:
    node, state_signal, log_signal = backend_state_node()
    node._task_state_timeout = 0.01
    RobotBackendNode.mission_state_callback(
        node,
        SimpleNamespace(
            state_seq=40,
            state=MissionState.PATROL,
            target_id=0,
            handled_target_id=0,
        ),
    )
    heartbeat = SimpleNamespace(
        state_seq=40,
        task_state=PatrolTaskState.TASK_RUNNING,
        last_task_outcome=PatrolTaskState.TASK_OUTCOME_NONE,
        navigation_mode=PatrolTaskState.NAVIGATION_WAYPOINT,
        home_pose_valid=True,
    )
    RobotBackendNode.task_state_callback(node, heartbeat)
    state_signal.values.clear()
    node._last_task_state_monotonic = time.monotonic() - 1.0
    node._last_mission_state_monotonic = time.monotonic() - 1.0

    RobotBackendNode._check_task_state_timeout(node)
    assert state_signal.values == [(40, -1, -1, -1, -1, False)]
    assert node._task_state_timed_out

    RobotBackendNode.task_state_callback(node, heartbeat)
    assert node._task_state_timed_out
    RobotBackendNode.mission_state_callback(
        node,
        SimpleNamespace(
            state_seq=40,
            state=MissionState.PATROL,
            target_id=0,
            handled_target_id=0,
        ),
    )
    assert state_signal.values[-1] == (
        40,
        PatrolTaskState.TASK_RUNNING,
        PatrolTaskState.TASK_OUTCOME_NONE,
        PatrolTaskState.NAVIGATION_WAYPOINT,
        MissionState.PATROL,
        True,
    )
    assert not node._task_state_timed_out


def test_direct_app_bootstraps_workspace_environment() -> None:
    workspace = Path(__file__).resolve().parents[4]
    app = workspace / "src/orchestration/robot_console/robot_console/app.py"
    environment = os.environ.copy()
    workspace_prefix = str(workspace)
    for variable in (
        "AMENT_PREFIX_PATH",
        "CMAKE_PREFIX_PATH",
        "COLCON_PREFIX_PATH",
        "LD_LIBRARY_PATH",
        "PYTHONPATH",
    ):
        environment[variable] = os.pathsep.join(
            entry
            for entry in environment.get(variable, "").split(os.pathsep)
            if entry and not entry.startswith(workspace_prefix)
        )
    environment.pop("DOG_PATROL_WORKSPACE_BOOTSTRAPPED", None)

    result = subprocess.run(
        [sys.executable, str(app), "--check-environment"],
        cwd=app.parent,
        env=environment,
        check=False,
        capture_output=True,
        text=True,
        timeout=20,
    )

    assert result.returncode == 0, result.stderr
    assert "robot_console environment OK" in result.stdout
    assert "TASK_RUNNING=1" in result.stdout
    assert "START=0" in result.stdout


def test_workspace_path_uses_valid_bootstrap_root(monkeypatch) -> None:
    workspace = Path(__file__).resolve().parents[4]
    monkeypatch.setenv("DOG_PATROL_WORKSPACE_ROOT", str(workspace))

    assert RobotMainWindow.get_workspace_path(SimpleNamespace()) == str(workspace)


def test_manager_exit_stops_old_business_stack_before_restart(monkeypatch) -> None:
    calls = []
    monkeypatch.setattr(
        "robot_console.main_window.QTimer.singleShot",
        lambda delay, callback: calls.append(("restart", delay, callback)),
    )
    window = SimpleNamespace(
        _closing=False,
        _manager_should_run=True,
        process_manager=object(),
        is_nav_running=True,
        is_perception_running=True,
        log_engine=SimpleNamespace(
            parse_and_append_log=lambda *args, **kwargs: calls.append(("log", args))
        ),
        append_log=lambda message: calls.append(("append", message)),
        stop_navigation=lambda: calls.append(("stop_navigation",)),
        stop_perception=lambda: calls.append(("stop_perception",)),
        start_mission_manager=lambda: None,
    )

    RobotMainWindow._mission_manager_finished(window, 1, None)

    assert window.process_manager is None
    assert ("stop_navigation",) in calls
    assert ("stop_perception",) in calls
    assert any(item[0] == "restart" and item[1] == 1000 for item in calls)


def test_unexpected_navigation_exit_stops_perception_before_reset() -> None:
    calls = []
    window = SimpleNamespace(
        _closing=False,
        _stopping_navigation=False,
        process_nav=object(),
        is_nav_running=True,
        is_perception_running=True,
        current_state="IDLE",
        append_log=lambda *args, **kwargs: calls.append(("log", args)),
        stop_perception=lambda: calls.append(("stop_perception",)),
        reset_mission_session=lambda reason: calls.append(("reset", reason)),
        update_ui_state=lambda state: calls.append(("ui", state)),
    )

    RobotMainWindow._navigation_finished(window, 1, None)

    assert window.process_nav is None
    assert not window.is_nav_running
    assert ("stop_perception",) in calls
    assert not any(item[0] == "reset" for item in calls)


def test_unexpected_perception_exit_stops_navigation_before_reset() -> None:
    calls = []
    window = SimpleNamespace(
        _closing=False,
        _stopping_perception=False,
        process_perception=object(),
        is_perception_running=True,
        is_nav_running=True,
        current_state="IDLE",
        append_log=lambda *args, **kwargs: calls.append(("log", args)),
        stop_navigation=lambda: calls.append(("stop_navigation",)),
        reset_mission_session=lambda reason: calls.append(("reset", reason)),
        update_ui_state=lambda state: calls.append(("ui", state)),
    )

    RobotMainWindow._perception_finished(window, 1, None)

    assert window.process_perception is None
    assert not window.is_perception_running
    assert ("stop_navigation",) in calls
    assert not any(item[0] == "reset" for item in calls)
