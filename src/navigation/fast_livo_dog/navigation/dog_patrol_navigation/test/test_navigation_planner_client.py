from types import SimpleNamespace

import numpy as np
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from dog_patrol_navigation.navigation_planner_client import NavigationPlannerClient


class FakeFuture:
    def __init__(self):
        self._callbacks = []
        self._result = None
        self._done = False

    def add_done_callback(self, callback):
        if self._done:
            callback(self)
        else:
            self._callbacks.append(callback)

    def resolve(self, result):
        self._result = result
        self._done = True
        for callback in self._callbacks:
            callback(self)

    def result(self):
        return self._result


class FakeLogger:
    def debug(self, _message):
        pass

    def warning(self, _message):
        pass


class FakeGoalHandle:
    accepted = True

    def __init__(self):
        self.result_future = FakeFuture()
        self.cancel_count = 0

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancel_count += 1
        future = FakeFuture()
        future.resolve(SimpleNamespace(goals_canceling=[object()]))
        return future


class FakeActionClient:
    def __init__(self):
        self.ready = True
        self.sent_goals = []
        self.response_futures = []

    def server_is_ready(self):
        return self.ready

    def send_goal_async(self, goal):
        self.sent_goals.append(goal)
        future = FakeFuture()
        self.response_futures.append(future)
        return future


def make_client(action_client, results, failures, target_goals, dispatched=None):
    node = SimpleNamespace(get_logger=lambda: FakeLogger())
    now = SimpleNamespace(to_msg=lambda: Time(sec=1, nanosec=2))
    return NavigationPlannerClient(
        node,
        action_client,
        "GridBased",
        "map",
        lambda: now,
        target_goals.append,
        lambda request, path: results.append((request, path)),
        lambda reason, request: failures.append((reason, request)),
        None if dispatched is None else dispatched.append,
    )


def usable_path():
    path = Path()
    path.poses.append(PoseStamped())
    return path


def test_planner_forwards_only_a_successful_current_result():
    action_client = FakeActionClient()
    results = []
    failures = []
    target_goals = []
    client = make_client(action_client, results, failures, target_goals)
    goal = PoseStamped()
    goal.pose.position.x = 4.0

    assert client.request(goal, np.asarray([5.0, 0.0, 0.0]), 12, 7, "target")
    assert client.in_flight
    assert action_client.sent_goals[0].goal.pose.position.x == 4.0
    assert len(target_goals) == 1

    handle = FakeGoalHandle()
    action_client.response_futures[0].resolve(handle)
    path = usable_path()
    handle.result_future.resolve(SimpleNamespace(result=SimpleNamespace(path=path)))

    assert not client.in_flight
    assert not failures
    assert len(results) == 1
    assert results[0][0].state_seq == 12
    assert results[0][0].target_id == 7
    assert results[0][0].plan_kind == "target"
    assert results[0][1] is path


def test_invalidation_cancels_a_late_accepted_goal_and_drops_its_result():
    action_client = FakeActionClient()
    results = []
    failures = []
    client = make_client(action_client, results, failures, [])
    assert client.request(PoseStamped(), np.zeros(3), 8, 7, "recovery")

    client.invalidate("mission state changed")
    handle = FakeGoalHandle()
    action_client.response_futures[0].resolve(handle)

    assert handle.cancel_count == 1
    assert not client.in_flight
    assert not results
    assert not failures


def test_empty_planner_result_reports_failure_without_publishing():
    action_client = FakeActionClient()
    results = []
    failures = []
    client = make_client(action_client, results, failures, [])
    assert client.request(PoseStamped(), np.zeros(3), 3, 7, "recovery")

    handle = FakeGoalHandle()
    action_client.response_futures[0].resolve(handle)
    handle.result_future.resolve(
        SimpleNamespace(result=SimpleNamespace(path=Path()))
    )

    assert not client.in_flight
    assert not results
    assert failures[0][0] == "ComputePathToPose returned no usable path"
    assert failures[0][1].plan_kind == "recovery"


def test_unavailable_planner_reports_the_operation_context():
    action_client = FakeActionClient()
    action_client.ready = False
    results = []
    failures = []
    client = make_client(action_client, results, failures, [])

    assert not client.request(PoseStamped(), np.zeros(3), 19, 11, "recovery")
    assert failures[0][0] == "ComputePathToPose action unavailable"
    assert failures[0][1].state_seq == 19
    assert failures[0][1].target_id == 11
    assert failures[0][1].plan_kind == "recovery"


def test_unavailable_planner_is_not_reported_as_dispatched():
    action_client = FakeActionClient()
    action_client.ready = False
    results = []
    failures = []
    dispatched = []
    client = make_client(action_client, results, failures, [], dispatched)

    assert not client.request(PoseStamped(), np.zeros(3), 19, 11, "recovery")
    assert not dispatched


def test_dispatched_callback_runs_before_async_goal_response():
    action_client = FakeActionClient()
    results = []
    failures = []
    dispatched = []
    client = make_client(action_client, results, failures, [], dispatched)

    assert client.request(PoseStamped(), np.zeros(3), 20, 11, "recovery")
    assert len(dispatched) == 1
    assert dispatched[0].state_seq == 20
    assert dispatched[0].plan_kind == "recovery"


def test_invalidated_empty_result_does_not_report_failure_to_new_context():
    action_client = FakeActionClient()
    results = []
    failures = []
    client = make_client(action_client, results, failures, [])
    assert client.request(PoseStamped(), np.zeros(3), 20, 11, "recovery")

    handle = FakeGoalHandle()
    action_client.response_futures[0].resolve(handle)
    client.invalidate("mission state changed")
    handle.result_future.resolve(
        SimpleNamespace(result=SimpleNamespace(path=Path()))
    )

    assert not results
    assert not failures
