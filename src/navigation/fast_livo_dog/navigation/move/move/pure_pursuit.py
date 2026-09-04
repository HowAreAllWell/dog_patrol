#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# ROS2 port of Pure Pursuit sub-goal publisher
#
# Original authorship notes retained in spirit:
#  revision history: xzt
#   20210604 (TE): first version (ROS1)
#
# This ROS2 node publishes a sub-goal point using the pure pursuit algorithm.
# Topics:
#   Subscribes:  path (nav_msgs/Path)
#   Publishes:   subgoal (geometry_msgs/PoseStamped), final_goal (geometry_msgs/PoseStamped)
#
# Frames:
#   world_frame (default: "map")
#   robot_frame (default: "base_footprint")
#
# Parameters (ROS2):
#   lookahead   [double, default 1.0]  Lookahead distance (m)
#   rate        [double, default 20.0] Control loop rate (Hz)
#   goal_margin [double, default 0.9]  Distance threshold to final goal (m)  # kept for parity
#   wheel_base  [double, default 0.23] Robot wheel base (m)                  # kept for parity
#   wheel_radius[double, default 0.025]Wheel radius (m)                      # kept for parity
#   v_max       [double, default 0.5]  Max linear velocity (m/s)             # kept for parity
#   w_max       [double, default 5.0]  Max angular velocity (rad/s)          # kept for parity
#   world_frame [string, default "map"]
#   robot_frame [string, default "base_footprint"]
#
# Notes:
# - Uses tf2_ros Buffer/TransformListener to query robot pose.
# - Applies TRANSIENT_LOCAL QoS on the Path subscription so late-joiners can
#   receive the last path (ROS1 latched-topic behavior).
# - Adds multiple numerical/logic guards to avoid runtime errors.

import math
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header

from tf2_ros import Buffer, TransformListener

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit')

        # ---------------- Parameters ----------------
        self.declare_parameter('lookahead', 1.8)
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('goal_margin', 0.45)

        self.declare_parameter('wheel_base', 0.23)
        self.declare_parameter('wheel_radius', 0.025)
        self.declare_parameter('v_max', 0.3)
        self.declare_parameter('w_max', 0.5)

        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('robot_frame', 'base_footprint')

        self.lookahead = float(self.get_parameter('lookahead').value)
        self.rate = float(self.get_parameter('rate').value)
        self.goal_margin = float(self.get_parameter('goal_margin').value)

        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.v_max = float(self.get_parameter('v_max').value)
        self.w_max = float(self.get_parameter('w_max').value)

        self.world_frame = str(self.get_parameter('world_frame').value)
        self.robot_frame = str(self.get_parameter('robot_frame').value)

        # ---------------- TF2 ----------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------------- Data & Lock ----------------
        self.path = None
        self.path_points = np.empty((0, 2), dtype=np.float64)
        self.lock = threading.Lock()
        self.timer = None
        self._waiting_for_path_logged = False

        # ---------------- QoS ----------------
        # Path is often published once and should be latched-like for late subscribers.
        path_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        pub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ---------------- Sub/Pub ----------------
        self.path_sub = self.create_subscription(Path, 'plan', self.path_callback, path_qos)
        self.cnn_goal_pub = self.create_publisher(PoseStamped, 'subgoal', pub_qos)
        self.final_goal_pub = self.create_publisher(PoseStamped, 'final_goal', pub_qos)

        self.get_logger().debug('PurePursuitNode initialized (ROS2).')

    # --------------- Path Callback ---------------
    def path_callback(self, msg: Path):
        self.get_logger().debug('PurePursuit: Got path')
        with self.lock:
            self.path = msg
            if len(msg.poses) >= 2:
                self.path_points = np.asarray(
                    [
                        (pose.pose.position.x, pose.pose.position.y)
                        for pose in msg.poses
                    ],
                    dtype=np.float64,
                )
            else:
                self.path_points = np.empty((0, 2), dtype=np.float64)

        if self.path is None or len(self.path.poses) < 2:
            if not self._waiting_for_path_logged:
                self.get_logger().debug('PurePursuit: Path cleared, waiting for a new plan')
                self._waiting_for_path_logged = True
        else:
            self._waiting_for_path_logged = False

        # Start timer upon first path reception
        if self.timer is None:
            self.start()

    # --------------- Timer ---------------
    def start(self):
        # run controller at specified rate
        period = 1.0 / max(self.rate, 1e-3)
        self.timer = self.create_timer(period, self.timer_callback)
        self.get_logger().debug(f'PurePursuit control loop started at {self.rate:.1f} Hz')

    # --------------- TF Pose ---------------
    def get_current_pose(self):
        try:
            t: Time = Time()  # latest available
            trans = self.tf_buffer.lookup_transform(self.world_frame, self.robot_frame, t)
        except Exception as ex:
            self.get_logger().debug(f'Could not get robot pose: {ex}')
            return np.array([np.nan, np.nan]), np.nan

        x = np.array([trans.transform.translation.x, trans.transform.translation.y], dtype=float)
        qx = trans.transform.rotation.x
        qy = trans.transform.rotation.y
        qz = trans.transform.rotation.z
        qw = trans.transform.rotation.w
        theta = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )
        self.get_logger().debug(f'x = {x[0]:.3f}, y = {x[1]:.3f}, theta = {theta:.3f}')
        return x, theta

    # --------------- Geometry Helpers ---------------
    def find_closest_point(self, x: np.ndarray, seg: int = -1):
        """
        Find the closest point on the path to x.

        Returns:
            pt_min: np.array([x, y])
            dist_min: float
            seg_min: int
        """
        pt_min = np.array([np.nan, np.nan], dtype=float)
        dist_min = np.inf
        seg_min = -1

        points = self.path_points
        if points.shape[0] < 2:
            return pt_min, dist_min, seg_min

        if seg >= 0:
            if seg >= points.shape[0] - 1:
                return pt_min, dist_min, seg_min
            starts = points[seg : seg + 1]
            vectors = points[seg + 1 : seg + 2] - starts
            indices = np.array([seg], dtype=np.int64)
        else:
            starts = points[:-1]
            vectors = points[1:] - starts
            indices = np.arange(starts.shape[0], dtype=np.int64)

        lengths_sq = np.einsum('ij,ij->i', vectors, vectors)
        offsets = x - starts
        projection = np.divide(
            np.einsum('ij,ij->i', offsets, vectors),
            lengths_sq,
            out=np.zeros_like(lengths_sq),
            where=lengths_sq > 1e-18,
        )
        projection = np.clip(projection, 0.0, 1.0)
        closest_points = starts + projection[:, None] * vectors
        deltas = closest_points - x
        distances_sq = np.einsum('ij,ij->i', deltas, deltas)
        best = int(np.argmin(distances_sq))
        pt_min = closest_points[best]
        dist_min = math.sqrt(float(distances_sq[best]))
        seg_min = int(indices[best])

        return pt_min, dist_min, seg_min

    def find_goal(self, x: np.ndarray, pt: np.ndarray, dist: float, seg: int):
        """
        Determine the goal point along the path.
        Returns the target point and the final path point, both in the world frame.
        """
        goal = None

        points = self.path_points
        if points.shape[0] < 2:
            return None, None

        end_goal_pos = points[-1]

        if dist > self.lookahead:
            # far from path: drive toward closest point
            goal = pt
        else:
            seg_max = points.shape[0] - 2

            # end of current segment
            p_end = points[seg + 1]
            dist_end = np.linalg.norm(x - p_end)

            # advance until leaving the lookahead circle or reaching last segment
            while dist_end < self.lookahead and seg < seg_max:
                seg += 1
                p_end = points[seg + 1]
                dist_end = np.linalg.norm(x - p_end)

            if dist_end < self.lookahead:
                # searched whole path: goal is the path end
                pt2 = points[seg_max + 1]
                goal = pt2
            else:
                # find intersection with the lookahead circle on this segment
                pt2, _, seg2 = self.find_closest_point(x, seg)

                p_start = points[seg2]
                p_end = points[seg2 + 1]

                v = p_end - p_start
                length_seg = np.linalg.norm(v)
                if length_seg < 1e-9:
                    goal = pt2
                else:
                    v = v / length_seg
                    dist_proj_x = np.dot(x - pt2, v)
                    delta = x - pt2
                    dist_proj_y = abs(delta[0] * v[1] - delta[1] * v[0])
                    under_radical = self.lookahead ** 2 - dist_proj_y ** 2
                    if under_radical < 0.0:
                        # numerical guard
                        under_radical = 0.0
                    goal = pt2 + (np.sqrt(under_radical) + dist_proj_x) * v

        return goal, end_goal_pos

    # --------------- Main Control Loop ---------------
    def timer_callback(self):
        with self.lock:
            # get current pose
            x, theta = self.get_current_pose()
            if np.isnan(x[0]):
                return

            # closest point
            pt, dist, seg = self.find_closest_point(x)
            if np.isnan(pt).any() or seg < 0:
                return

            # goal
            goal, end_goal_pos = self.find_goal(x, pt, dist, seg)
            if goal is None or end_goal_pos is None:
                return

        # ---- Transform goal to robot(local) coordinates ----
        c, s = math.cos(theta), math.sin(theta)

        def to_robot_frame(point):
            dx = point[0] - x[0]
            dy = point[1] - x[1]
            return np.array([c * dx + s * dy, -s * dx + c * dy], dtype=np.float64)

        goal_local = to_robot_frame(goal)
        relative_goal = to_robot_frame(end_goal_pos)

        # ---- Publish subgoal ----
        hdr = Header()
        hdr.stamp = self.get_clock().now().to_msg()
        hdr.frame_id = self.robot_frame

        cnn_goal = PoseStamped()
        cnn_goal.header = hdr
        cnn_goal.pose.position.x = float(goal_local[0])
        cnn_goal.pose.position.y = float(goal_local[1])

        if not np.isnan(cnn_goal.pose.position.x) and not np.isnan(cnn_goal.pose.position.y):
            self.cnn_goal_pub.publish(cnn_goal)
            self.get_logger().debug(f'Subgoal (local): {goal_local[0]:.3f}, {goal_local[1]:.3f}')

        # ---- Publish final goal (relative position only) ----
        final_goal = PoseStamped()
        final_goal.header = hdr
        final_goal.pose.position.x = float(relative_goal[0])
        final_goal.pose.position.y = float(relative_goal[1])

        if not np.isnan(final_goal.pose.position.x) and not np.isnan(final_goal.pose.position.y):
            self.final_goal_pub.publish(final_goal)
            self.get_logger().debug(f'Final goal (local): {relative_goal[0]:.3f}, {relative_goal[1]:.3f}')


def main():
    rclpy.init()
    node = PurePursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
