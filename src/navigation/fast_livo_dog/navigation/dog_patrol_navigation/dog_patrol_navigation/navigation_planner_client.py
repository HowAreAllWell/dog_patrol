"""Small async wrapper around Nav2 ComputePathToPose.

The client owns action generations and cancellation.  It deliberately does
not know the mission state machine; the coordinator supplies a validation
callback for state_seq and target freshness before accepting a result.
"""

from dataclasses import dataclass
from typing import Callable, Optional

import numpy as np
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient


@dataclass(frozen=True)
class PlannerRequest:
    generation: int
    state_seq: int
    target_id: int
    planned_target: np.ndarray
    plan_kind: str


class NavigationPlannerClient:
    """Serialize planner requests and reject results from old generations."""

    def __init__(
        self,
        node,
        action_client: ActionClient,
        planner_id: str,
        global_frame: str,
        now: Callable[[], object],
        publish_target_goal: Callable[[PoseStamped], None],
        result_callback: Callable[[PlannerRequest, Path], None],
        failure_callback: Callable[[str, PlannerRequest], None],
        dispatch_callback: Optional[Callable[[PlannerRequest], None]] = None,
    ) -> None:
        self._node = node
        self._action_client = action_client
        self._planner_id = str(planner_id)
        self._global_frame = str(global_frame)
        self._now = now
        self._publish_target_goal = publish_target_goal
        self._result_callback = result_callback
        self._failure_callback = failure_callback
        self._dispatch_callback = dispatch_callback
        self._generation = 0
        self._in_flight = False
        self._active_goal_handle = None
        self._active_goal_generation = -1
        self._last_request: Optional[PlannerRequest] = None

    @property
    def in_flight(self) -> bool:
        return self._in_flight

    @property
    def last_request(self) -> Optional[PlannerRequest]:
        return self._last_request

    def invalidate(self, reason: str) -> None:
        self._generation += 1
        self._in_flight = False
        self.cancel_active(reason)

    def request(
        self,
        goal_pose: PoseStamped,
        planned_target: np.ndarray,
        state_seq: int,
        target_id: int,
        plan_kind: str,
    ) -> bool:
        self._generation += 1
        planner_request = PlannerRequest(
            generation=self._generation,
            state_seq=int(state_seq),
            target_id=int(target_id),
            planned_target=np.asarray(planned_target, dtype=np.float64).copy(),
            plan_kind=str(plan_kind),
        )
        self._last_request = planner_request
        if not self._action_client.server_is_ready():
            self._failure_callback(
                "ComputePathToPose action unavailable", planner_request
            )
            return False

        request = ComputePathToPose.Goal()
        request.goal = PoseStamped()
        request.goal.header.frame_id = self._global_frame
        request.goal.header.stamp = self._now().to_msg()
        request.goal.pose.position.x = float(goal_pose.pose.position.x)
        request.goal.pose.position.y = float(goal_pose.pose.position.y)
        request.goal.pose.position.z = float(goal_pose.pose.position.z)
        request.goal.pose.orientation = goal_pose.pose.orientation
        request.use_start = False
        request.planner_id = self._planner_id
        if plan_kind == "target":
            self._publish_target_goal(request.goal)

        if self._dispatch_callback is not None:
            # The action server is available, so this is a real planner
            # attempt even if the transport call below raises immediately.
            self._dispatch_callback(planner_request)
        self._in_flight = True
        try:
            response_future = self._action_client.send_goal_async(request)
        except Exception as exc:
            self._in_flight = False
            self._failure_callback(
                f"planner goal send failed: {exc}", planner_request
            )
            return False
        response_future.add_done_callback(
            lambda future: self._on_goal_response(future, planner_request)
        )
        return True

    def _on_goal_response(self, future, planner_request: PlannerRequest) -> None:
        try:
            handle = future.result()
        except Exception as exc:
            if self._is_current(planner_request):
                self._in_flight = False
                self._failure_callback(
                    f"planner goal send failed: {exc}", planner_request
                )
            return

        if not self._is_current(planner_request):
            if handle is not None and handle.accepted:
                self._cancel_goal_handle(
                    handle, f"stale {planner_request.plan_kind} planner goal"
                )
            return
        if handle is None or not handle.accepted:
            self._in_flight = False
            self._failure_callback(
                "ComputePathToPose goal rejected", planner_request
            )
            return

        self._active_goal_handle = handle
        self._active_goal_generation = planner_request.generation
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda result: self._on_plan_result(result, planner_request)
        )

    def _on_plan_result(self, future, planner_request: PlannerRequest) -> None:
        if self._active_goal_generation == planner_request.generation:
            self._active_goal_handle = None
            self._active_goal_generation = -1
        if not self._is_current(planner_request):
            return
        self._in_flight = False
        try:
            wrapped = future.result()
            if wrapped is None or wrapped.result is None or not wrapped.result.path.poses:
                raise RuntimeError("ComputePathToPose returned no usable path")
            path = wrapped.result.path
        except Exception as exc:
            self._failure_callback(str(exc), planner_request)
            return
        self._result_callback(planner_request, path)

    def _is_current(self, request: PlannerRequest) -> bool:
        return request.generation == self._generation

    def _cancel_goal_handle(self, goal_handle, reason: str) -> None:
        try:
            future = goal_handle.cancel_goal_async()
            future.add_done_callback(
                lambda done_future: self._on_cancel_response(done_future, reason)
            )
            self._node.get_logger().debug(f"Cancel planner goal: {reason}")
        except Exception as exc:
            self._node.get_logger().warning(
                f"ComputePathToPose cancel failed ({reason}): {exc}"
            )

    def cancel_active(self, reason: str) -> None:
        goal_handle = self._active_goal_handle
        self._active_goal_handle = None
        self._active_goal_generation = -1
        if goal_handle is not None:
            self._cancel_goal_handle(goal_handle, reason)

    def _on_cancel_response(self, future, reason: str) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self._node.get_logger().warning(
                f"ComputePathToPose cancel response failed ({reason}): {exc}"
            )
            return
        if response is not None and not getattr(response, "goals_canceling", []):
            self._node.get_logger().debug(
                f"ComputePathToPose cancel had no active goal ({reason})"
            )
