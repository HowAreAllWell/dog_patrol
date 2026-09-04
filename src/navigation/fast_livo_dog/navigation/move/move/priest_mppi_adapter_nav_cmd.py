#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
from typing import Optional

import numpy as np
import rclpy
from drdds.msg import NavCmd
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from std_msgs.msg import Float32
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

import tf2_ros


def yaw_to_quat(yaw: float):
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


def clamp_abs(value: float, limit: float) -> float:
    if limit <= 0.0 or not math.isfinite(limit):
        return value
    return max(-limit, min(limit, value))


class PriestMppiAdapterNavCmd(Node):
    def __init__(self):
        super().__init__("priest_mppi_adapter_nav_cmd")

        self.declare_parameter("priest_path_topic", "local_path")
        self.declare_parameter("path_target_frame", "camera_init_footprint")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("tf_timeout", 0.03)
        self.declare_parameter("path_timeout", 1.0)
        self.declare_parameter("goal_send_hz", 5.0)
        self.declare_parameter("min_goal_resend_interval", 0.5)
        self.declare_parameter("min_path_points", 4)
        self.declare_parameter("publish_mppi_path", True)
        self.declare_parameter("mppi_path_topic", "mppi_path")
        self.declare_parameter("localization_confidence_topic", "/localization_3d_confidence")
        self.declare_parameter("localization_confidence_threshold", 0.65)
        self.declare_parameter("localization_confidence_timeout", 1.5)

        self.declare_parameter("follow_path_action", "follow_path")
        self.declare_parameter("controller_id", "FollowPath")
        self.declare_parameter("goal_checker_id", "general_goal_checker")
        self.declare_parameter("progress_checker_id", "progress_checker")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("nav_cmd_topic", "/NAV_CMD")
        self.declare_parameter("cmd_frame_id", 0)
        self.declare_parameter("nav_cmd_publish_hz", 30.0)
        self.declare_parameter("cmd_timeout", 1.0)
        self.declare_parameter("scale_x", 1.0)
        self.declare_parameter("scale_y", 1.0)
        self.declare_parameter("scale_yaw", 1.0)
        self.declare_parameter("max_x_vel", 0.8)
        self.declare_parameter("max_y_vel", 0.8)
        self.declare_parameter("max_yaw_vel", 1.0)

        self.priest_path_topic = self.get_parameter("priest_path_topic").value
        self.path_target_frame = self._normalize_frame_id(self.get_parameter("path_target_frame").value)
        self.base_frame = self._normalize_frame_id(self.get_parameter("base_frame").value)
        self.tf_timeout = float(self.get_parameter("tf_timeout").value)
        self.path_timeout = float(self.get_parameter("path_timeout").value)
        self.goal_send_hz = float(self.get_parameter("goal_send_hz").value)
        self.min_goal_resend_interval = float(self.get_parameter("min_goal_resend_interval").value)
        self.min_path_points = int(self.get_parameter("min_path_points").value)
        self.publish_mppi_path = bool(self.get_parameter("publish_mppi_path").value)
        self.mppi_path_topic = self.get_parameter("mppi_path_topic").value
        self.localization_confidence_topic = self.get_parameter("localization_confidence_topic").value
        self.localization_confidence_threshold = float(self.get_parameter("localization_confidence_threshold").value)
        self.localization_confidence_timeout = float(self.get_parameter("localization_confidence_timeout").value)

        self.follow_path_action = self.get_parameter("follow_path_action").value
        self.controller_id = self.get_parameter("controller_id").value
        self.goal_checker_id = self.get_parameter("goal_checker_id").value
        self.progress_checker_id = self.get_parameter("progress_checker_id").value

        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.nav_cmd_topic = self.get_parameter("nav_cmd_topic").value
        self.cmd_frame_id = int(self.get_parameter("cmd_frame_id").value)
        self.nav_cmd_publish_hz = float(self.get_parameter("nav_cmd_publish_hz").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        self.scale_x = float(self.get_parameter("scale_x").value)
        self.scale_y = float(self.get_parameter("scale_y").value)
        self.scale_yaw = float(self.get_parameter("scale_yaw").value)
        self.max_x_vel = float(self.get_parameter("max_x_vel").value)
        self.max_y_vel = float(self.get_parameter("max_y_vel").value)
        self.max_yaw_vel = float(self.get_parameter("max_yaw_vel").value)

        qos_path = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.path_sub = self.create_subscription(Path, self.priest_path_topic, self._on_path, qos_path)
        self.cmd_vel_sub = self.create_subscription(Twist, self.cmd_vel_topic, self._on_cmd_vel, 10)
        self.localization_conf_sub = self.create_subscription(
            Float32, self.localization_confidence_topic, self._on_localization_confidence, 10
        )

        self.mppi_path_pub = self.create_publisher(Path, self.mppi_path_topic, 10) if self.publish_mppi_path else None
        self.nav_cmd_pub = self.create_publisher(NavCmd, self.nav_cmd_topic, 10)

        self.follow_path_client = ActionClient(self, FollowPath, self.follow_path_action)

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.latest_path: Optional[Path] = None
        self.latest_path_time = None
        self.latest_path_seq = 0
        self.last_sent_seq = -1

        self.latest_cmd_vel = Twist()
        self.latest_cmd_time = None
        self._cmd_timeout_active = False

        self._last_feedback_log = 0.0
        self._active_goal_handle = None
        self._active_goal_seq = -1
        self._pending_goal = False
        self._cancel_requested = False
        self._last_goal_send_time = None
        self.localization_confidence = 0.0
        self.localization_ready = False
        self.localization_confidence_time = None
        self._localization_timeout_active = False

        send_period = 1.0 / max(1e-3, self.goal_send_hz)
        self.goal_timer = self.create_timer(send_period, self._on_goal_timer)

        cmd_bridge_period = 1.0 / max(1e-3, self.nav_cmd_publish_hz)
        self.cmd_bridge_timer = self.create_timer(cmd_bridge_period, self._on_cmd_bridge_timer)

        self.get_logger().debug(
            "[priest_mppi_adapter_nav_cmd] ready\n"
            f"  priest_path_topic={self.priest_path_topic}\n"
            f"  path_target_frame={self.path_target_frame}\n"
            f"  follow_path_action={self.follow_path_action}\n"
            f"  controller_id={self.controller_id}\n"
            f"  min_goal_resend_interval={self.min_goal_resend_interval}\n"
            f"  mppi_path_topic={self.mppi_path_topic} (publish={self.publish_mppi_path})\n"
            f"  localization_confidence_topic={self.localization_confidence_topic}\n"
            f"  localization_confidence_threshold={self.localization_confidence_threshold}\n"
            f"  localization_confidence_timeout={self.localization_confidence_timeout}\n"
            f"  cmd_vel_topic={self.cmd_vel_topic} -> nav_cmd_topic={self.nav_cmd_topic}\n"
            f"  nav_cmd_publish_hz={self.nav_cmd_publish_hz}, cmd_timeout={self.cmd_timeout}"
        )

    @staticmethod
    def _normalize_frame_id(frame_id: str) -> str:
        return (frame_id or "").lstrip("/")

    def _on_path(self, msg: Path):
        if len(msg.poses) < self.min_path_points:
            self.latest_path = None
            self.latest_path_time = None
            self.latest_path_seq += 1
            self.last_sent_seq = -1
            self.latest_cmd_time = None
            self._publish_nav_cmd(0.0, 0.0, 0.0)
            self._cancel_active_follow_path("local path cleared")
            self._publish_empty_mppi_path()
            return
        self.latest_path = msg
        self.latest_path_time = self.get_clock().now()
        self.latest_path_seq += 1
        self._cancel_requested = False

    def _cancel_follow_path_goal_handle(self, goal_handle, reason: str):
        try:
            goal_handle.cancel_goal_async()
            self.get_logger().debug(f"Cancel FollowPath goal: {reason}")
        except Exception as exc:
            self.get_logger().warn(f"FollowPath cancel failed ({reason}): {exc}")

    def _cancel_active_follow_path(self, reason: str):
        self.latest_cmd_time = None
        self._publish_nav_cmd(0.0, 0.0, 0.0)
        goal_handle = self._active_goal_handle
        if goal_handle is None:
            return

        self._active_goal_handle = None
        self._active_goal_seq = -1
        self._cancel_requested = True
        self._cancel_follow_path_goal_handle(goal_handle, reason)

    def _publish_empty_mppi_path(self):
        if self.mppi_path_pub is None:
            return
        empty = Path()
        empty.header.frame_id = self.path_target_frame
        empty.header.stamp = self.get_clock().now().to_msg()
        self.mppi_path_pub.publish(empty)

    def _on_cmd_vel(self, msg: Twist):
        self.latest_cmd_vel = msg
        self.latest_cmd_time = self.get_clock().now()
        self._cmd_timeout_active = False

    def _set_localization_ready(self, ready: bool, reason: str = ""):
        if self.localization_ready == ready:
            return
        self.localization_ready = ready
        if ready:
            self._localization_timeout_active = False
            self.get_logger().debug(
                f"Localization confidence recovered to {self.localization_confidence:.3f}, navigation output enabled"
            )
        else:
            self.last_sent_seq = -1
            self.get_logger().warn(
                f"Localization not ready ({reason}), freeze navigation output"
            )

    def _refresh_localization_state(self):
        if self.localization_confidence_time is None:
            self._set_localization_ready(False, "no confidence message received")
            return
        age = (self.get_clock().now() - self.localization_confidence_time).nanoseconds * 1e-9
        if age > self.localization_confidence_timeout:
            if not self._localization_timeout_active:
                self._localization_timeout_active = True
                self._set_localization_ready(False, f"confidence timeout {age:.2f}s")
            return
        ready = math.isfinite(self.localization_confidence) and (
            self.localization_confidence >= self.localization_confidence_threshold
        )
        self._set_localization_ready(ready, f"confidence {self.localization_confidence:.3f}")

    def _on_localization_confidence(self, msg: Float32):
        self.localization_confidence = float(msg.data)
        self.localization_confidence_time = self.get_clock().now()
        self._localization_timeout_active = False
        self._refresh_localization_state()

    def _lookup_tf(self, target: str, source: str, time=None):
        if time is None:
            time = Time()
        try:
            return self.tf_buffer.lookup_transform(
                target, source, time, timeout=Duration(seconds=self.tf_timeout)
            )
        except Exception as e:
            self.get_logger().debug(f"TF lookup failed: {target} <- {source}: {e}")
            return None

    @staticmethod
    def _apply_tf_xy(points_xy: np.ndarray, tf_msg) -> np.ndarray:
        tx = tf_msg.transform.translation.x
        ty = tf_msg.transform.translation.y
        q = tf_msg.transform.rotation
        yaw = 2.0 * math.atan2(q.z, q.w)
        c, s = math.cos(yaw), math.sin(yaw)
        rot = np.array([[c, -s], [s, c]], dtype=np.float64)
        pts = (rot @ points_xy.T).T
        pts[:, 0] += tx
        pts[:, 1] += ty
        return pts.astype(np.float64)

    def _path_to_target_frame(self, path_msg: Path) -> Optional[Path]:
        if len(path_msg.poses) < self.min_path_points:
            return None

        src_frame = self._normalize_frame_id(path_msg.header.frame_id) or self.base_frame
        pts_src = np.array(
            [(ps.pose.position.x, ps.pose.position.y) for ps in path_msg.poses],
            dtype=np.float64,
        )

        if src_frame == self.path_target_frame:
            pts_target = pts_src
        else:
            tf_msg = self._lookup_tf(self.path_target_frame, src_frame, path_msg.header.stamp)
            if tf_msg is None:
                return None
            pts_target = self._apply_tf_xy(pts_src, tf_msg)

        path_out = Path()
        path_out.header.frame_id = self.path_target_frame
        path_out.header.stamp = self.get_clock().now().to_msg()

        if pts_target.shape[0] < 2:
            return None

        headings = np.zeros((pts_target.shape[0],), dtype=np.float64)
        diffs = pts_target[1:] - pts_target[:-1]
        seg_yaws = np.arctan2(diffs[:, 1], diffs[:, 0])
        headings[:-1] = seg_yaws
        headings[-1] = seg_yaws[-1]

        for i in range(pts_target.shape[0]):
            ps = PoseStamped()
            ps.header = path_out.header
            ps.pose.position.x = float(pts_target[i, 0])
            ps.pose.position.y = float(pts_target[i, 1])
            ps.pose.position.z = 0.0
            qx, qy, qz, qw = yaw_to_quat(float(headings[i]))
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            path_out.poses.append(ps)

        return path_out

    def _publish_nav_cmd(self, x_vel: float, y_vel: float, yaw_vel: float):
        msg = NavCmd()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = int(self.cmd_frame_id)
        msg.data.x_vel = float(x_vel)
        msg.data.y_vel = float(y_vel)
        msg.data.yaw_vel = float(yaw_vel)
        self.nav_cmd_pub.publish(msg)

    def _on_cmd_bridge_timer(self):
        self._refresh_localization_state()
        x_vel = 0.0
        y_vel = 0.0
        yaw_vel = 0.0

        if self.localization_ready and self.latest_cmd_time is not None:
            age = (self.get_clock().now() - self.latest_cmd_time).nanoseconds * 1e-9
            if age <= self.cmd_timeout:
                x_vel = clamp_abs(self.latest_cmd_vel.linear.x * self.scale_x, self.max_x_vel)
                y_vel = clamp_abs(self.latest_cmd_vel.linear.y * self.scale_y, self.max_y_vel)
                yaw_vel = clamp_abs(self.latest_cmd_vel.angular.z * self.scale_yaw, self.max_yaw_vel)
            elif not self._cmd_timeout_active:
                self._cmd_timeout_active = True
                self.get_logger().warn(
                    f"/cmd_vel timeout ({age:.2f}s > {self.cmd_timeout:.2f}s), publish zero NAV_CMD"
                )
        elif not self.localization_ready:
            self._cmd_timeout_active = False

        self._publish_nav_cmd(x_vel, y_vel, yaw_vel)

    def _on_goal_timer(self):
        self._refresh_localization_state()
        if not self.localization_ready:
            self._cancel_active_follow_path("localization not ready")
            return
        if self.latest_path is None or self.latest_path_time is None or self._pending_goal:
            return

        age = (self.get_clock().now() - self.latest_path_time).nanoseconds * 1e-9
        if age > self.path_timeout:
            self._cancel_active_follow_path(f"local path timeout {age:.2f}s")
            return

        if self.latest_path_seq == self.last_sent_seq:
            return

        if self._last_goal_send_time is not None:
            resend_age = (self.get_clock().now() - self._last_goal_send_time).nanoseconds * 1e-9
            if resend_age < self.min_goal_resend_interval:
                return

        if not self.follow_path_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().debug("follow_path action server not available yet")
            return

        path_goal = self._path_to_target_frame(self.latest_path)
        if path_goal is None:
            return

        goal_msg = FollowPath.Goal()
        goal_msg.path = path_goal
        goal_msg.controller_id = self.controller_id
        goal_msg.goal_checker_id = self.goal_checker_id
        if hasattr(goal_msg, "progress_checker_id"):
            goal_msg.progress_checker_id = self.progress_checker_id

        if self.mppi_path_pub is not None:
            self.mppi_path_pub.publish(path_goal)

        self._pending_goal = True
        self._last_goal_send_time = self.get_clock().now()
        goal_seq = self.latest_path_seq
        future = self.follow_path_client.send_goal_async(goal_msg, feedback_callback=self._on_feedback)
        future.add_done_callback(lambda f, goal_seq=goal_seq: self._on_goal_response(f, goal_seq))

    def _on_goal_response(self, future, goal_seq: int):
        self._pending_goal = False
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().warn(f"FollowPath goal request failed: {e}")
            return
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn("FollowPath goal rejected")
            return
        if goal_seq != self.latest_path_seq or self.latest_path is None:
            self._cancel_follow_path_goal_handle(goal_handle, f"stale path seq {goal_seq}")
            return

        self._active_goal_handle = goal_handle
        self._active_goal_seq = goal_seq
        self._cancel_requested = False
        self.last_sent_seq = goal_seq
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f, goal_handle=goal_handle, goal_seq=goal_seq: self._on_result(f, goal_handle, goal_seq)
        )

    def _on_result(self, future, goal_handle, goal_seq: int):
        try:
            result_wrap = future.result()
        except Exception as e:
            self.get_logger().warn(f"FollowPath result failed: {e}")
            return

        is_current_goal = goal_handle == self._active_goal_handle
        if is_current_goal:
            self._active_goal_handle = None
            self._active_goal_seq = -1
            self._cancel_requested = False
        status = getattr(result_wrap, "status", None)
        result = getattr(result_wrap, "result", None)
        if not is_current_goal:
            return
        if result is not None and hasattr(result, "error_code") and result.error_code != 0:
            self.get_logger().warn(
                f"FollowPath finished with error_code={result.error_code}, "
                f"error_msg='{getattr(result, 'error_msg', '')}' status={status}"
            )

    def _on_feedback(self, feedback_msg):
        now = time.time()
        if now - self._last_feedback_log < 1.0:
            return
        self._last_feedback_log = now
        fb = feedback_msg.feedback
        dist = float(getattr(fb, "distance_to_goal", 0.0))
        speed = float(getattr(fb, "speed", 0.0))
        self.get_logger().debug(f"[follow_path] distance_to_goal={dist:.3f}, speed={speed:.3f}")

    def destroy_node(self):
        try:
            self._publish_nav_cmd(0.0, 0.0, 0.0)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PriestMppiAdapterNavCmd()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
