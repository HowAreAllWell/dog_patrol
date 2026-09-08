#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Publish Nav2 global paths for a sequence of map-frame goals.

This node keeps the original contract of publishing nav_msgs/Path on
`global_path`, but it can now execute multiple terminal goals in order:

  1. Ask Nav2 planner_server /compute_path_to_pose for the current goal.
  2. Publish the returned path to the configured waypoint path topic.
  3. Watch robot pose from TF.
  4. Once the robot is within `goal_tolerance`, advance to the next goal.

It does not send velocity commands by itself. Your RL/PRIEST local planner
continues to consume `global_path`, generate `local_path`, and the adapter
continues to command the robot.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from typing import List, Optional
import copy

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseArray, PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Empty, String
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros

try:
    from interactive_markers.interactive_marker_server import InteractiveMarkerServer
    from interactive_markers.menu_handler import MenuHandler
    from visualization_msgs.msg import (
        InteractiveMarker,
        InteractiveMarkerControl,
        InteractiveMarkerFeedback,
    )
except ImportError:
    InteractiveMarkerServer = None
    MenuHandler = None
    InteractiveMarker = None
    InteractiveMarkerControl = None
    InteractiveMarkerFeedback = None


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    yaw: float = 0.0  # radians


class GlobalPathSequencePublisher(Node):
    def __init__(self):
        super().__init__("global_path_sequence_publisher")

        # Main behavior.
        self.declare_parameter("use_static_goals", False)
        self.declare_parameter("goals", "")
        self.declare_parameter("goals_xy", [])
        self.declare_parameter("goal_yaw_unit", "deg")  # deg / rad, only for `goals` third column
        self.declare_parameter("goal_tolerance", 1.0)
        self.declare_parameter("replan_period", 2.0)
        self.declare_parameter("loop", False)
        self.declare_parameter("auto_advance", True)
        self.declare_parameter("start_index", 0)
        self.declare_parameter("auto_start_on_click", True)
        self.declare_parameter("min_clicked_spacing", 0.25)

        # Frames/topics.
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("path_topic", "global_path")
        self.declare_parameter("publish_pure_pursuit_plan", True)
        self.declare_parameter("pure_pursuit_plan_topic", "global_path")
        self.declare_parameter("goal_topic", "goal_pose")
        self.declare_parameter("waypoints_topic", "waypoints")
        self.declare_parameter("waypoints_pose_topic", "waypoints_pose_array")
        self.declare_parameter("clicked_point_topic", "")
        self.declare_parameter("delete_clicked_point_topic", "waypoint_sequence/delete_nearest")
        self.declare_parameter("replace_clicked_point_topic", "waypoint_sequence/replace_nearest")
        self.declare_parameter("status_topic", "waypoint_sequence/status")
        self.declare_parameter("clear_topic", "waypoint_sequence/clear")
        self.declare_parameter("undo_topic", "waypoint_sequence/undo")
        self.declare_parameter("pause_topic", "waypoint_sequence/pause")
        self.declare_parameter("resume_topic", "waypoint_sequence/resume")
        self.declare_parameter("resume_from_current_topic", "waypoint_sequence/resume_from_current")
        self.declare_parameter("planner_id", "")
        self.declare_parameter("tf_timeout", 0.2)
        self.declare_parameter("edit_radius", 1.5)
        self.declare_parameter("marker_line_width", 0.0)
        self.declare_parameter("marker_point_diameter", 0.7)
        self.declare_parameter("marker_point_height", 0.18)
        self.declare_parameter("marker_label_height", 1.25)
        self.declare_parameter("marker_label_z", 2.5)
        self.declare_parameter("enable_interactive_markers", True)
        self.declare_parameter("interactive_marker_namespace", "waypoint_editor")

        self.use_static_goals = bool(self.get_parameter("use_static_goals").value)
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.robot_frame = str(self.get_parameter("robot_frame").value)
        self.path_topic = str(self.get_parameter("path_topic").value)
        self.publish_pure_pursuit_plan = bool(self.get_parameter("publish_pure_pursuit_plan").value)
        self.pure_pursuit_plan_topic = str(self.get_parameter("pure_pursuit_plan_topic").value)
        self.goal_topic = str(self.get_parameter("goal_topic").value)
        self.waypoints_topic = str(self.get_parameter("waypoints_topic").value)
        self.waypoints_pose_topic = str(self.get_parameter("waypoints_pose_topic").value)
        self.clicked_point_topic = str(self.get_parameter("clicked_point_topic").value)
        self.delete_clicked_point_topic = str(self.get_parameter("delete_clicked_point_topic").value)
        self.replace_clicked_point_topic = str(self.get_parameter("replace_clicked_point_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.clear_topic = str(self.get_parameter("clear_topic").value)
        self.undo_topic = str(self.get_parameter("undo_topic").value)
        self.pause_topic = str(self.get_parameter("pause_topic").value)
        self.resume_topic = str(self.get_parameter("resume_topic").value)
        self.resume_from_current_topic = str(self.get_parameter("resume_from_current_topic").value)
        self.planner_id = str(self.get_parameter("planner_id").value)
        self.tf_timeout = float(self.get_parameter("tf_timeout").value)
        self.edit_radius = max(0.05, float(self.get_parameter("edit_radius").value))
        self.marker_line_width = max(0.01, float(self.get_parameter("marker_line_width").value))
        self.marker_point_diameter = max(0.05, float(self.get_parameter("marker_point_diameter").value))
        self.marker_point_height = max(0.02, float(self.get_parameter("marker_point_height").value))
        self.marker_label_height = max(0.1, float(self.get_parameter("marker_label_height").value))
        self.marker_label_z = max(0.1, float(self.get_parameter("marker_label_z").value))
        self.enable_interactive_markers = bool(self.get_parameter("enable_interactive_markers").value)
        self.interactive_marker_namespace = str(self.get_parameter("interactive_marker_namespace").value)

        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.replan_period = max(0.5, float(self.get_parameter("replan_period").value))
        self.loop = bool(self.get_parameter("loop").value)
        self.auto_advance = bool(self.get_parameter("auto_advance").value)
        self.auto_start_on_click = bool(self.get_parameter("auto_start_on_click").value)
        self.min_clicked_spacing = max(0.0, float(self.get_parameter("min_clicked_spacing").value))

        self.waypoints = self._load_waypoints()
        self.current_index = int(self.get_parameter("start_index").value)
        if self.waypoints:
            self.current_index = int(np.clip(self.current_index, 0, len(self.waypoints) - 1))
        self.sequence_done = False
        self.paused = False
        self.request_in_flight = False
        self.sequence_version = 0
        self.request_version = -1
        self._active_goal_handle = None
        self._active_goal_version = -1
        self._last_path: Optional[Path] = None
        self._interactive_markers_dirty = True
        self._interactive_label_distance_bucket: Optional[int] = None
        self._interactive_label_distance_text: Optional[str] = None

        self.compute_path_client = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        self.path_pub = self.create_publisher(Path, self.path_topic, 20)
        same_plan_topic = self.path_topic.lstrip("/") == self.pure_pursuit_plan_topic.lstrip("/")
        self.pure_pursuit_plan_pub = (
            self.create_publisher(Path, self.pure_pursuit_plan_topic, 20)
            if self.publish_pure_pursuit_plan and not same_plan_topic else None
        )
        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.waypoint_marker_pub = self.create_publisher(MarkerArray, self.waypoints_topic, 10)
        self.waypoints_pose_pub = self.create_publisher(PoseArray, self.waypoints_pose_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        self.clicked_point_sub = None
        if self.clicked_point_topic:
            self.clicked_point_sub = self.create_subscription(
                PointStamped, self.clicked_point_topic, self._on_clicked_point, 10
            )
        self.delete_clicked_point_sub = None
        if self.delete_clicked_point_topic:
            self.delete_clicked_point_sub = self.create_subscription(
                PointStamped, self.delete_clicked_point_topic, self._on_delete_clicked_point, 10
            )
        self.replace_clicked_point_sub = None
        if self.replace_clicked_point_topic:
            self.replace_clicked_point_sub = self.create_subscription(
                PointStamped, self.replace_clicked_point_topic, self._on_replace_clicked_point, 10
            )
        self.clear_sub = self.create_subscription(Empty, self.clear_topic, self._on_clear, 10)
        self.undo_sub = self.create_subscription(Empty, self.undo_topic, self._on_undo, 10)
        self.pause_sub = self.create_subscription(Empty, self.pause_topic, self._on_pause, 10)
        self.resume_sub = self.create_subscription(Empty, self.resume_topic, self._on_resume, 10)
        self.resume_from_current_sub = self.create_subscription(
            Empty,
            self.resume_from_current_topic,
            self._on_resume_from_current,
            10,
        )

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.interactive_marker_server = None
        self.interactive_menu_handler = None
        self.interactive_delete_entry = None
        self.interactive_clear_entry = None
        if self.enable_interactive_markers and InteractiveMarkerServer is not None:
            self.interactive_marker_server = InteractiveMarkerServer(
                self,
                self.interactive_marker_namespace,
            )
            self.interactive_menu_handler = MenuHandler()
            self.interactive_delete_entry = self.interactive_menu_handler.insert(
                "Delete this waypoint",
                callback=self._on_interactive_menu,
            )
            self.interactive_clear_entry = self.interactive_menu_handler.insert(
                "Clear all waypoints",
                callback=self._on_interactive_menu,
            )
        elif self.enable_interactive_markers:
            self.get_logger().warn(
                "interactive_markers Python package is not available; RViz drag/edit is disabled."
            )

        self.timer = self.create_timer(self.replan_period, self._on_timer)
        self.marker_timer = self.create_timer(0.1, self._publish_waypoints)
        self.status_timer = self.create_timer(1.0, self._publish_status)

        self.get_logger().debug(

            "[global_path_sequence] ready\n"
            f"  goals={[(round(w.x, 3), round(w.y, 3), round(w.yaw, 3)) for w in self.waypoints]}\n"
            f"  path_topic={self.path_topic}, pure_pursuit_plan_topic={self.pure_pursuit_plan_topic}\n"
            f"  clicked_point_topic={self.clicked_point_topic}, waypoints_topic={self.waypoints_topic}\n"
            f"  delete_topic={self.delete_clicked_point_topic}, replace_topic={self.replace_clicked_point_topic}\n"
            f"  interactive_marker_update=/{self.interactive_marker_namespace}/update\n"
            f"  frame={self.global_frame}, robot_frame={self.robot_frame}\n"
            f"  tolerance={self.goal_tolerance:.2f} m, replan_period={self.replan_period:.2f} s, "
            f"loop={self.loop}, auto_advance={self.auto_advance}"
        )
        self._publish_empty_paths()

    def _load_waypoints(self) -> List[Waypoint]:
        if not self.use_static_goals:
            return []

        goals_text = str(self.get_parameter("goals").value or "").strip()
        yaw_unit = str(self.get_parameter("goal_yaw_unit").value or "deg").lower()
        yaw_scale = math.pi / 180.0 if yaw_unit.startswith("deg") else 1.0

        if goals_text:
            return self._parse_goals_string(goals_text, yaw_scale)

        flat = list(self.get_parameter("goals_xy").value)
        if len(flat) % 2 != 0:
            raise ValueError("goals_xy must be a flat [x1, y1, x2, y2, ...] list")
        if len(flat) == 0:
            return []
        return [
            Waypoint(float(flat[i]), float(flat[i + 1]), 0.0)
            for i in range(0, len(flat), 2)
        ]

    @staticmethod
    def _parse_goals_string(value: str, yaw_scale: float) -> List[Waypoint]:
        """
        Parse examples:
          "1.0,2.0; 3.0,4.0; 5.0,6.0"
          "1.0,2.0,90; 3.0,4.0,180"

        Third column is yaw, default unit configured by `goal_yaw_unit`.
        """
        waypoints: List[Waypoint] = []
        for chunk in re.split(r"[;\n]+", value):
            chunk = chunk.strip()
            if not chunk:
                continue
            parts = [p for p in re.split(r"[,\s]+", chunk) if p]
            if len(parts) not in (2, 3):
                raise ValueError(f"Invalid goal '{chunk}', expected x,y or x,y,yaw")
            x = float(parts[0])
            y = float(parts[1])
            yaw = float(parts[2]) * yaw_scale if len(parts) == 3 else 0.0
            waypoints.append(Waypoint(x, y, yaw))
        return waypoints

    def _on_timer(self):
        if not self.waypoints:
            self.get_logger().debug("No waypoints configured.")
            self._publish_waypoints()
            self._publish_status()
            return
        if self.paused:
            self._publish_status()
            return
        if self.sequence_done:
            self._publish_status()
            return

        if self.auto_advance:
            self._advance_if_reached()
            if self.sequence_done:
                self._publish_status()
                return

        if self.request_in_flight:
            self._publish_status()
            return
        if not self.compute_path_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().debug("ComputePathToPose server not available yet.")
            self._publish_status()
            return

        self._request_plan_to_current_goal()
        self._publish_status()

    def _advance_if_reached(self) -> bool:
        robot_xy = self._lookup_robot_xy()
        if robot_xy is None:
            return False

        goal = self.waypoints[self.current_index]
        dist = math.hypot(robot_xy[0] - goal.x, robot_xy[1] - goal.y)
        if dist > self.goal_tolerance:
            return False

        self.get_logger().debug(
            f"Reached waypoint {self.current_index + 1}/{len(self.waypoints)}: "
            f"({goal.x:.3f}, {goal.y:.3f}), dist={dist:.3f} m"
        )

        if self.current_index + 1 < len(self.waypoints):
            self.current_index += 1
            self.sequence_version += 1
            self._invalidate_planning_request("advanced to next waypoint")
            self._interactive_markers_dirty = True
            next_goal = self.waypoints[self.current_index]
            self.get_logger().debug(
                f"Advance to waypoint {self.current_index + 1}/{len(self.waypoints)}: "
                f"({next_goal.x:.3f}, {next_goal.y:.3f})"
            )
            self._last_path = None
            self._publish_empty_paths()
            self._publish_waypoints()
            return True

        if self.loop:
            self.current_index = 0
            self.sequence_version += 1
            self._invalidate_planning_request("looped waypoint sequence")
            self._interactive_markers_dirty = True
            self._last_path = None
            self._publish_empty_paths()
            self._publish_waypoints()
            self.get_logger().debug("Waypoint sequence completed; looping to waypoint 1.")
            return True
        else:
            self.sequence_done = True
            self.sequence_version += 1
            self._invalidate_planning_request("completed waypoint sequence")
            self._last_path = None
            self._interactive_markers_dirty = True
            self._publish_empty_paths()
            self._publish_waypoints()
            self.get_logger().debug("Waypoint sequence completed.")
            return True

    def _request_plan_to_current_goal(self):
        goal = self.waypoints[self.current_index]
        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = self._to_pose_stamped(goal)
        goal_msg.use_start = False
        if self.planner_id:
            goal_msg.planner_id = self.planner_id

        self.request_in_flight = True
        request_version = self.sequence_version
        self.request_version = request_version
        self._publish_current_goal()

        self.get_logger().debug(
            f"Planning single path to waypoint {self.current_index + 1}/{len(self.waypoints)}: "
            f"({goal.x:.3f}, {goal.y:.3f})"
        )

        future = self.compute_path_client.send_goal_async(goal_msg)
        future.add_done_callback(
            lambda done_future, v=request_version: self._goal_response_callback(done_future, v)
        )

    def _goal_response_callback(self, future, request_version: int):
        try:
            goal_handle = future.result()
        except Exception as exc:
            if self.request_version == request_version:
                self.request_in_flight = False
            self.get_logger().error(f"ComputePathToPose send failed: {exc}")
            return

        if request_version != self.sequence_version:
            if self.request_version == request_version:
                self.request_in_flight = False
            if goal_handle is not None and goal_handle.accepted:
                self._cancel_planner_goal_handle(
                    goal_handle,
                    f"stale ComputePathToPose goal version {request_version}",
                )
            return

        if goal_handle is None or not goal_handle.accepted:
            if self.request_version == request_version:
                self.request_in_flight = False
            self.get_logger().warn("ComputePathToPose goal rejected by server")
            return

        self._active_goal_handle = goal_handle
        self._active_goal_version = request_version
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda done_future, v=request_version: self._get_result_callback(done_future, v)
        )

    def _get_result_callback(self, future, request_version: int):
        if self.request_version == request_version:
            self.request_in_flight = False
        if self._active_goal_version == request_version:
            self._active_goal_handle = None
            self._active_goal_version = -1
        if request_version != self.sequence_version:
            return
        if not self.waypoints or self.sequence_done or self.paused:
            return
        
        try:
            result = future.result().result
        except Exception as exc:
            self.get_logger().error(f"ComputePathToPose result failed: {exc}")
            return

        if result is None or result.path is None or len(result.path.poses) == 0:
            self.get_logger().warn("No valid path returned to current waypoint")
            return

        # Manually extract and instantiate poses to entirely bypass rclpy's C-level memory reuse
        extracted_poses = []
        for p in result.path.poses:
            new_p = PoseStamped()
            new_p.header.stamp = p.header.stamp
            new_p.header.frame_id = p.header.frame_id
            new_p.pose.position.x = p.pose.position.x
            new_p.pose.position.y = p.pose.position.y
            new_p.pose.position.z = p.pose.position.z
            new_p.pose.orientation.x = p.pose.orientation.x
            new_p.pose.orientation.y = p.pose.orientation.y
            new_p.pose.orientation.z = p.pose.orientation.z
            new_p.pose.orientation.w = p.pose.orientation.w
            extracted_poses.append(new_p)

        if request_version != self.sequence_version:
            return
        if not self.waypoints or self.sequence_done or self.paused:
            return

        path_msg = Path()
        path_msg.header = extracted_poses[0].header
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.global_frame
        path_msg.poses = extracted_poses

        self._last_path = path_msg
        self.path_pub.publish(path_msg)
        if self.pure_pursuit_plan_pub is not None:
            self.pure_pursuit_plan_pub.publish(path_msg)
        self.get_logger().debug(
            f"Published single-segment global path to waypoint "
            f"{self.current_index + 1}/{len(self.waypoints)} with {len(path_msg.poses)} poses"
        )

    def _cancel_planner_goal_handle(self, goal_handle, reason: str):
        try:
            cancel_future = goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(
                lambda done_future, cancel_reason=reason: self._planner_cancel_callback(
                    done_future,
                    cancel_reason,
                )
            )
            self.get_logger().debug(f"Cancel ComputePathToPose goal: {reason}")
        except Exception as exc:
            self.get_logger().warn(f"ComputePathToPose cancel failed ({reason}): {exc}")

    def _planner_cancel_callback(self, future, reason: str):
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f"ComputePathToPose cancel response failed ({reason}): {exc}")
            return

        goals_canceling = getattr(response, "goals_canceling", [])
        if not goals_canceling:
            self.get_logger().debug(f"ComputePathToPose cancel had no active goal ({reason})")

    def _cancel_active_planner_goal(self, reason: str):
        goal_handle = self._active_goal_handle
        self._active_goal_handle = None
        self._active_goal_version = -1
        if goal_handle is not None:
            self._cancel_planner_goal_handle(goal_handle, reason)

    def _invalidate_planning_request(self, reason: str):
        self.request_in_flight = False
        self.request_version = -1
        if hasattr(self, '_cached_segments'):
            self._cached_segments.clear()
        self._cancel_active_planner_goal(reason)

    def _on_clicked_point(self, msg: PointStamped):
        waypoint = self._point_to_waypoint(msg)
        if waypoint is None:
            return

        was_sequence_done = self.sequence_done

        if self.waypoints:
            last = self.waypoints[-1]
            dist = math.hypot(waypoint.x - last.x, waypoint.y - last.y)
            if dist < self.min_clicked_spacing:
                return

        self.waypoints.append(waypoint)
        self.sequence_version += 1
        self._invalidate_planning_request("waypoint list changed")
        self._interactive_markers_dirty = True
        if len(self.waypoints) == 1 or was_sequence_done:
            self.current_index = len(self.waypoints) - 1 if was_sequence_done else 0
            self.sequence_done = False
        if self.auto_start_on_click:
            self.paused = False
        self._last_path = None
        self._publish_waypoints()
        self._publish_status()
        if was_sequence_done and self.auto_start_on_click and not self.paused:
            self._on_timer()

    def _on_delete_clicked_point(self, msg: PointStamped):
        waypoint = self._point_to_waypoint(msg)
        if waypoint is None:
            return
        nearest = self._nearest_waypoint_index(waypoint)
        if nearest is None:
            return

        idx, dist = nearest
        if dist > self.edit_radius:
            self._publish_status(f"delete ignored: nearest waypoint is {dist:.2f} m away")
            return

        self._delete_waypoint(idx, f"deleted waypoint {idx + 1}, nearest click dist={dist:.2f} m")

    @staticmethod
    def _progress_after_waypoint_removal(
        current_index: int,
        sequence_done: bool,
        removed_index: int,
        remaining_count: int,
    ) -> tuple[int, bool, bool]:
        """Keep completion progress stable after removing one waypoint.

        The publisher represents completed points implicitly: points before
        ``current_index`` are complete, and ``sequence_done`` means every
        remaining point is complete.  Removing a point must therefore adjust
        only the index needed to keep the same logical target.
        """
        if remaining_count <= 0:
            return 0, True, True

        if sequence_done:
            return min(current_index, remaining_count - 1), True, False

        if removed_index < current_index:
            return current_index - 1, False, False

        if removed_index == current_index:
            # The removed point was the final pending point.  All remaining
            # points were already completed, so do not reopen the last one.
            if current_index >= remaining_count:
                return remaining_count - 1, True, True
            return current_index, False, True

        return current_index, False, False

    @staticmethod
    def _waypoint_edit_affects_active_goal(
        current_index: int,
        sequence_done: bool,
        edited_index: int,
    ) -> bool:
        """Return whether editing a waypoint requires a new active path."""
        return not sequence_done and edited_index == current_index

    def _delete_waypoint(self, idx: int, detail: str):
        if idx < 0 or idx >= len(self.waypoints):
            return

        previous_index = self.current_index
        previous_sequence_done = self.sequence_done
        removed = self.waypoints.pop(idx)
        (
            self.current_index,
            self.sequence_done,
            plan_affected,
        ) = self._progress_after_waypoint_removal(
            previous_index,
            previous_sequence_done,
            idx,
            len(self.waypoints),
        )

        if plan_affected:
            self.sequence_version += 1
            self._invalidate_planning_request("waypoint deleted")
            self._last_path = None

        self._interactive_markers_dirty = True
        if plan_affected:
            self._publish_empty_paths()
        self._publish_waypoints()
        self._publish_status(f"{detail}: ({removed.x:.2f}, {removed.y:.2f})")

    def _on_replace_clicked_point(self, msg: PointStamped):
        waypoint = self._point_to_waypoint(msg)
        if waypoint is None:
            return
        nearest = self._nearest_waypoint_index(waypoint)
        if nearest is None:
            return

        idx, dist = nearest
        if dist > self.edit_radius:
            self._publish_status(f"replace ignored: nearest waypoint is {dist:.2f} m away")
            return

        old = self.waypoints[idx]
        self.waypoints[idx] = Waypoint(waypoint.x, waypoint.y, old.yaw)
        self._interactive_markers_dirty = True

        if self._waypoint_edit_affects_active_goal(
            self.current_index,
            self.sequence_done,
            idx,
        ):
            self.sequence_version += 1
            self._invalidate_planning_request("waypoint replaced")
            self._last_path = None
            self._publish_empty_paths()

        self._publish_waypoints()
        self._publish_status(
            f"replaced waypoint {idx + 1}: ({old.x:.2f}, {old.y:.2f}) -> "
            f"({waypoint.x:.2f}, {waypoint.y:.2f})"
        )

    def _nearest_waypoint_index(self, waypoint: Waypoint) -> Optional[tuple[int, float]]:
        if not self.waypoints:
            self._publish_status("edit ignored: no waypoints")
            return None

        distances = [
            math.hypot(waypoint.x - candidate.x, waypoint.y - candidate.y)
            for candidate in self.waypoints
        ]
        idx = int(np.argmin(distances))
        return idx, float(distances[idx])

    def _point_to_waypoint(self, msg: PointStamped) -> Optional[Waypoint]:
        src_frame = (msg.header.frame_id or self.global_frame).lstrip("/")
        point = np.array([msg.point.x, msg.point.y], dtype=np.float64)

        if src_frame and src_frame != self.global_frame:
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    self.global_frame,
                    src_frame,
                    Time(),
                    timeout=Duration(seconds=self.tf_timeout),
                )
            except Exception as exc:
                self.get_logger().debug(f"TF lookup failed: {self.global_frame} <- {src_frame}: {exc}")
                return None
            point = self._apply_tf_xy(point, tf_msg)

        return Waypoint(float(point[0]), float(point[1]), 0.0)

    def _lookup_robot_xy(self) -> Optional[np.ndarray]:
        try:
            trans = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout),
            )
        except Exception as exc:
            self.get_logger().debug(f"TF lookup failed: {self.global_frame} <- {self.robot_frame}: {exc}")
            return None

        return np.array(
            [trans.transform.translation.x, trans.transform.translation.y],
            dtype=np.float64,
        )

    def _current_goal_distance(self) -> Optional[float]:
        if not self.waypoints or self.sequence_done:
            return None
        if self.current_index < 0 or self.current_index >= len(self.waypoints):
            return None

        robot_xy = self._lookup_robot_xy()
        if robot_xy is None:
            return None

        goal = self.waypoints[self.current_index]
        return float(math.hypot(robot_xy[0] - goal.x, robot_xy[1] - goal.y))

    def _format_status(self) -> str:
        if not self.waypoints:
            return (
                "empty: click RViz Publish Point on /clicked_point to add waypoints; "
                f"edit_radius={self.edit_radius:.2f}m"
            )
        if self.sequence_done:
            return f"done: {len(self.waypoints)} waypoint(s), tolerance={self.goal_tolerance:.2f}m"
        if self.paused:
            prefix = "paused"
        else:
            prefix = "active"

        dist = self._current_goal_distance()
        dist_text = "dist=unknown" if dist is None else f"dist={dist:.2f}m"
        return (
            f"{prefix}: waypoint={self.current_index + 1}/{len(self.waypoints)}, "
            f"{dist_text}, tolerance={self.goal_tolerance:.2f}m, "
            f"replan_period={self.replan_period:.2f}s"
        )

    def _publish_status(self, detail: Optional[str] = None):
        msg = String()
        msg.data = self._format_status()
        if detail:
            msg.data = f"{msg.data}; {detail}"
        self.status_pub.publish(msg)

    @staticmethod
    def _apply_tf_xy(point_xy: np.ndarray, tf_msg) -> np.ndarray:
        tx = tf_msg.transform.translation.x
        ty = tf_msg.transform.translation.y
        q = tf_msg.transform.rotation
        yaw = 2.0 * math.atan2(q.z, q.w)
        c, s = math.cos(yaw), math.sin(yaw)
        return np.array(
            [
                c * point_xy[0] - s * point_xy[1] + tx,
                s * point_xy[0] + c * point_xy[1] + ty,
            ],
            dtype=np.float64,
        )

    def _to_pose_stamped(self, waypoint: Waypoint) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = self.global_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(waypoint.x)
        msg.pose.position.y = float(waypoint.y)
        msg.pose.position.z = 0.0
        msg.pose.orientation.z = math.sin(waypoint.yaw * 0.5)
        msg.pose.orientation.w = math.cos(waypoint.yaw * 0.5)
        return msg

    def _publish_current_goal(self):
        if not self.waypoints or self.sequence_done:
            return
        self.goal_pub.publish(self._to_pose_stamped(self.waypoints[self.current_index]))

    def _publish_waypoints(self):
        arr = PoseArray()
        arr.header.frame_id = self.global_frame
        arr.header.stamp = self.get_clock().now().to_msg()
        for wp in self.waypoints:
            ps = self._to_pose_stamped(wp)
            arr.poses.append(ps.pose)
        self.waypoints_pose_pub.publish(arr)
        self.waypoint_marker_pub.publish(self._build_waypoint_markers(arr.header))
        self._refresh_interactive_distance_label()
        self._sync_interactive_markers(arr.header)
        self._publish_current_goal()
        self._publish_status()

    def _build_waypoint_markers(self, header) -> MarkerArray:
        markers = MarkerArray()

        clear = Marker()
        clear.header = header
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        return markers

    def _sync_interactive_markers(self, header):
        if self.interactive_marker_server is None or not self._interactive_markers_dirty:
            return

        self.interactive_marker_server.clear()
        if not self.waypoints:
            self.interactive_marker_server.insert(self._make_interactive_ready_marker(header))
            self.interactive_marker_server.applyChanges()
            self._interactive_markers_dirty = False
            return

        for idx, wp in enumerate(self.waypoints):
            int_marker = self._make_interactive_waypoint_marker(idx, wp, header)
            self.interactive_marker_server.insert(
                int_marker,
                feedback_callback=self._on_interactive_feedback,
            )
            if self.interactive_menu_handler is not None:
                self.interactive_menu_handler.apply(
                    self.interactive_marker_server,
                    int_marker.name,
                )
        self.interactive_marker_server.applyChanges()
        self._interactive_markers_dirty = False

    def _refresh_interactive_distance_label(self):
        if self.interactive_marker_server is None:
            return

        bucket = None
        distance_text = None
        distance = self._current_goal_distance()
        if distance is not None:
            bucket = int(round(distance * 10.0))
            distance_text = f"{bucket / 10.0:.1f}m"

        if bucket == self._interactive_label_distance_bucket:
            return

        self._interactive_label_distance_bucket = bucket
        self._interactive_label_distance_text = distance_text
        self._interactive_markers_dirty = True

    def _make_interactive_ready_marker(self, header) -> InteractiveMarker:
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = header.frame_id or self.global_frame
        int_marker.header.stamp = header.stamp
        int_marker.name = "waypoint_editor_ready"
        int_marker.description = "Waypoint editor ready; use Publish Point to add waypoints"
        int_marker.pose.position.z = -1000.0
        int_marker.pose.orientation.w = 1.0
        int_marker.scale = 0.01

        marker = Marker()
        marker.type = Marker.SPHERE
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.a = 0.0

        control = InteractiveMarkerControl()
        control.name = "ready"
        control.interaction_mode = InteractiveMarkerControl.NONE
        control.always_visible = True
        control.markers.append(marker)
        int_marker.controls.append(control)
        return int_marker

    def _make_interactive_waypoint_marker(self, idx: int, wp: Waypoint, header) -> InteractiveMarker:
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = header.frame_id or self.global_frame
        int_marker.header.stamp = header.stamp
        int_marker.name = f"waypoint_{idx}"
        int_marker.description = ""
        int_marker.pose.position.x = float(wp.x)
        int_marker.pose.position.y = float(wp.y)
        int_marker.pose.position.z = 0.0
        int_marker.pose.orientation.w = 1.0
        int_marker.scale = max(0.9, self.marker_point_diameter * 1.6)

        marker = Marker()
        marker.type = Marker.CYLINDER
        marker.pose.position.z = 0.08
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.marker_point_diameter
        marker.scale.y = self.marker_point_diameter
        marker.scale.z = self.marker_point_height
        if self.sequence_done or idx < self.current_index:
            marker.color.r = 0.2
            marker.color.g = 0.85
            marker.color.b = 0.2
        elif idx == self.current_index and not self.sequence_done:
            marker.color.r = 1.0
            marker.color.g = 0.35
            marker.color.b = 0.0
        else:
            marker.color.r = 0.0
            marker.color.g = 0.55
            marker.color.b = 1.0
        marker.color.a = 0.85

        label_text = self._interactive_label_text(idx)
        text = Marker()
        text.type = Marker.TEXT_VIEW_FACING
        text.pose.position.x = 1.0
        text.pose.position.y = 1.0
        text.pose.position.z = self.marker_label_z
        text.pose.orientation.w = 1.0
        text.scale.z = self.marker_label_height
        if idx == self.current_index and not self.sequence_done:
            text.color.r = 1.0
            text.color.g = 0.25
            text.color.b = 0.0
        elif self.sequence_done or idx < self.current_index:
            text.color.r = 0.0
            text.color.g = 0.65
            text.color.b = 0.18
        else:
            text.color.r = 0.0
            text.color.g = 0.25
            text.color.b = 1.0
        text.color.a = 1.0
        text.text = label_text

        move_control = InteractiveMarkerControl()
        move_control.name = "drag_xy"
        move_control.orientation_mode = InteractiveMarkerControl.FIXED
        move_control.orientation.w = 0.0
        move_control.orientation.x = 0.7071
        move_control.orientation.y = 0.0
        move_control.orientation.z = 0.7071
        move_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        move_control.always_visible = True
        move_control.markers.append(marker)
        move_control.markers.append(text)
        int_marker.controls.append(move_control)

        menu_marker = Marker()
        menu_marker.type = Marker.CUBE
        menu_marker.pose.position.x = 0.45
        menu_marker.pose.position.y = 0.45
        menu_marker.pose.position.z = 0.28
        menu_marker.pose.orientation.w = 1.0
        menu_marker.scale.x = 0.22
        menu_marker.scale.y = 0.22
        menu_marker.scale.z = 0.08
        menu_marker.color.r = 1.0
        menu_marker.color.g = 0.15
        menu_marker.color.b = 0.0
        menu_marker.color.a = 0.8

        menu_control = InteractiveMarkerControl()
        menu_control.name = "menu"
        menu_control.interaction_mode = InteractiveMarkerControl.MENU
        menu_control.always_visible = True
        menu_control.markers.append(menu_marker)
        int_marker.controls.append(menu_control)
        return int_marker

    def _interactive_label_text(self, idx: int) -> str:
        label = str(idx + 1)
        if idx == self.current_index and not self.sequence_done and self._interactive_label_distance_text:
            return f"{label} {self._interactive_label_distance_text}"
        return label

    @staticmethod
    def _interactive_waypoint_index(marker_name: str) -> Optional[int]:
        prefix = "waypoint_"
        if not marker_name.startswith(prefix):
            return None
        try:
            return int(marker_name[len(prefix):])
        except ValueError:
            return None

    def _on_interactive_feedback(self, feedback):
        if InteractiveMarkerFeedback is None:
            return

        idx = self._interactive_waypoint_index(feedback.marker_name)
        if idx is None or idx < 0 or idx >= len(self.waypoints):
            return

        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            self._on_interactive_menu(feedback)
            return

        if feedback.event_type not in (
            InteractiveMarkerFeedback.POSE_UPDATE,
            InteractiveMarkerFeedback.MOUSE_UP,
        ):
            return

        old = self.waypoints[idx]
        new_wp = Waypoint(
            float(feedback.pose.position.x),
            float(feedback.pose.position.y),
            old.yaw,
        )
        self.waypoints[idx] = new_wp

        if feedback.event_type != InteractiveMarkerFeedback.MOUSE_UP:
            return

        if self._waypoint_edit_affects_active_goal(
            self.current_index,
            self.sequence_done,
            idx,
        ):
            self.sequence_version += 1
            self._invalidate_planning_request("waypoint dragged")
            self._last_path = None
            self._publish_empty_paths()

        self._interactive_markers_dirty = True
        self._publish_waypoints()
        self._publish_status(
            f"dragged waypoint {idx + 1}: ({old.x:.2f}, {old.y:.2f}) -> "
            f"({new_wp.x:.2f}, {new_wp.y:.2f})"
        )

    def _on_interactive_menu(self, feedback):
        idx = self._interactive_waypoint_index(feedback.marker_name)
        if idx is None:
            return

        if feedback.menu_entry_id == self.interactive_delete_entry:
            self._delete_waypoint(idx, f"deleted waypoint {idx + 1} from RViz menu")
        elif feedback.menu_entry_id == self.interactive_clear_entry:
            self._clear_waypoints("cleared all waypoints from RViz menu")

    def _on_clear(self, _msg: Empty):
        self._clear_waypoints("cleared all waypoints")

    def _clear_waypoints(self, detail: str):
        self.waypoints.clear()
        self.sequence_version += 1
        self.current_index = 0
        self.sequence_done = True
        self._invalidate_planning_request("cleared all waypoints")
        self._last_path = None
        self._interactive_markers_dirty = True
        self._publish_empty_paths()
        self._publish_waypoints()
        self._publish_status(detail)

    def _on_undo(self, _msg: Empty):
        if not self.waypoints:
            return
        removed_index = len(self.waypoints) - 1
        previous_index = self.current_index
        previous_sequence_done = self.sequence_done
        removed = self.waypoints.pop()
        (
            self.current_index,
            self.sequence_done,
            plan_affected,
        ) = self._progress_after_waypoint_removal(
            previous_index,
            previous_sequence_done,
            removed_index,
            len(self.waypoints),
        )

        if plan_affected:
            self.sequence_version += 1
            self._invalidate_planning_request("waypoint undo")

        self._interactive_markers_dirty = True
        if plan_affected:
            self._last_path = None
            self._publish_empty_paths()
        self._publish_waypoints()
        self._publish_status(
            f"undid waypoint {removed_index + 1}: ({removed.x:.2f}, {removed.y:.2f})"
        )

    def _on_pause(self, _msg: Empty):
        self.paused = True
        self.sequence_version += 1
        self._invalidate_planning_request("waypoint sequence paused")
        # Keep the last valid patrol path as a short bridge for normal resume.
        # It is never used for timeout recovery, which explicitly replans from
        # the robot's current pose.
        self._publish_empty_paths()
        self._publish_waypoints()
        self._publish_status("paused")

    def _on_resume(self, _msg: Empty):
        if self.waypoints:
            self.paused = False
            self.sequence_version += 1
            self._invalidate_planning_request("waypoint sequence resumed")
            self._publish_waypoints()
            if self.sequence_done:
                self._last_path = None
                self._publish_empty_paths()
                self._publish_status("resumed; waypoint sequence already complete")
                return
            cached_pose_count = self._publish_cached_path()
            if cached_pose_count == 0:
                self._publish_empty_paths()
            self._publish_status(
                f"resumed; restored {cached_pose_count} cached path poses"
                if cached_pose_count > 0
                else "resumed; requesting a fresh path"
            )
            # Do not wait for the next periodic tick. request_in_flight guards
            # the timer from submitting a duplicate request.
            self._on_timer()

    def _on_resume_from_current(self, _msg: Empty):
        """Resume patrol without replaying the path cached before interruption."""
        if not self.waypoints:
            return

        self.paused = False
        self.sequence_version += 1
        self._invalidate_planning_request(
            "waypoint sequence resumed from current pose after recovery timeout"
        )
        self._last_path = None
        self._publish_empty_paths()
        self._publish_waypoints()
        if self.sequence_done:
            self._publish_status(
                "recovery timed out; waypoint sequence already complete"
            )
            return
        self._publish_status(
            "recovery timed out; replanning from current pose"
        )
        self._on_timer()

    def _publish_cached_path(self) -> int:
        if self._last_path is None or not self._last_path.poses:
            return 0

        path = copy.deepcopy(self._last_path)
        stamp = self.get_clock().now().to_msg()
        path.header.stamp = stamp
        path.header.frame_id = path.header.frame_id or self.global_frame
        for pose in path.poses:
            pose.header.stamp = stamp
            pose.header.frame_id = pose.header.frame_id or path.header.frame_id

        self.path_pub.publish(path)
        if self.pure_pursuit_plan_pub is not None:
            self.pure_pursuit_plan_pub.publish(path)
        return len(path.poses)

    def _publish_empty_paths(self):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.global_frame
        
        self.path_pub.publish(msg)
        if self.pure_pursuit_plan_pub is not None:
            self.pure_pursuit_plan_pub.publish(msg)


    def cleanup(self):
        # Explicitly tell RViz to delete all interactive markers
        if self.interactive_marker_server is not None:
            self.interactive_marker_server.clear()
            self.interactive_marker_server.applyChanges()
        # Explicitly tell RViz to delete all static markers
        from visualization_msgs.msg import Marker, MarkerArray
        empty_markers = MarkerArray()
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        empty_markers.markers.append(delete_all)
        self.waypoint_marker_pub.publish(empty_markers)
        # Give the publisher a tiny moment to flush the messages
        import time
        time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = GlobalPathSequencePublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
