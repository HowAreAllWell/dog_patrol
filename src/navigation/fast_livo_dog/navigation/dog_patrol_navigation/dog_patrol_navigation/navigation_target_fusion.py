"""Target bbox and lidar fusion for the navigation mission coordinator.

This module owns the sensor-side target measurement lifecycle.  It does not
create ROS subscriptions or change mission state; the coordinator remains the
ROS boundary and decides when fusion is allowed.
"""

from collections import deque
from dataclasses import dataclass
from math import hypot
from typing import Callable, Deque, Optional, Tuple

import numpy as np
from builtin_interfaces.msg import Time as RosTime
from geometry_msgs.msg import PointStamped, Quaternion
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException

from dog_patrol_interfaces.msg import MissionState, TargetBoundingBox

from .target_estimator import (
    BoundingBox,
    PreparedPointCloud,
    TargetEstimator,
    TargetObservationUnavailable,
    TargetPositionFilter,
)


def normalize_frame(frame: str) -> str:
    return str(frame).lstrip("/")


def stamp_ns(stamp: RosTime) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def seconds_between(lhs: RosTime, rhs: RosTime) -> float:
    return (stamp_ns(lhs) - stamp_ns(rhs)) * 1.0e-9


def point_cloud_xyz(cloud: PointCloud2) -> np.ndarray:
    """Extract finite XYZ points from a PointCloud2 with mixed field types."""
    structured = point_cloud2.read_points(
        cloud, field_names=("x", "y", "z"), skip_nans=False
    )
    if structured.size == 0:
        return np.empty((0, 3), dtype=np.float32)

    points = np.column_stack(
        (structured["x"], structured["y"], structured["z"])
    ).astype(np.float32, copy=False)
    return points[np.all(np.isfinite(points), axis=1)]


def now_time(node: Node) -> Time:
    return node.get_clock().now()


def time_from_msg(node: Node, stamp: RosTime) -> Time:
    return Time.from_msg(stamp, clock_type=node.get_clock().clock_type)


def quaternion_to_rotation(q: Quaternion) -> np.ndarray:
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    norm = hypot(hypot(x, y), hypot(z, w))
    if norm < 1.0e-9:
        raise ValueError("invalid zero-length TF quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def device_lidar_to_camera(values) -> np.ndarray:
    """Return the runtime lidar-to-camera matrix used by FAST-LIVO."""
    matrix = np.asarray(values, dtype=np.float64)
    if matrix.size != 16:
        raise ValueError("T_camera_lidar must contain 16 row-major values")
    matrix = matrix.reshape(4, 4)
    if not np.all(np.isfinite(matrix)):
        raise ValueError("T_camera_lidar contains non-finite values")
    return matrix


def device_lidar_to_base(values) -> np.ndarray:
    """Convert the historical device matrix to lidar -> base_link."""
    matrix = np.asarray(values, dtype=np.float64)
    if matrix.size != 16:
        raise ValueError("T_lidar_base must contain 16 row-major values")
    matrix = matrix.reshape(4, 4)
    if not np.all(np.isfinite(matrix)):
        raise ValueError("T_lidar_base contains non-finite values")
    return np.linalg.inv(matrix)


@dataclass(frozen=True)
class TargetFusionConfig:
    camera_frame: str
    lidar_frame: str
    base_frame: str
    global_frame: str
    max_bbox_age: float
    sync_tolerance: float
    cloud_buffer_size: int
    target_timeout: float
    failure_timeout: float
    tf_timeout: float
    stable_samples: int


class TargetFusionController:
    """Fuse the current semantic target with the closest synchronized cloud."""

    def __init__(
        self,
        node: Node,
        tf_buffer: Buffer,
        estimator: TargetEstimator,
        target_filter: TargetPositionFilter,
        lidar_to_base: np.ndarray,
        config: TargetFusionConfig,
        target_point_publish: Callable[[PointStamped], None],
        position_ready: Callable[[], None],
        fusion_recovered: Callable[[], None],
    ) -> None:
        self._node = node
        self._tf_buffer = tf_buffer
        self._estimator = estimator
        self._target_filter = target_filter
        self._lidar_to_base = np.asarray(lidar_to_base, dtype=np.float64).reshape(4, 4)
        self.config = config
        self._target_point_publish = target_point_publish
        self._position_ready = position_ready
        self._fusion_recovered = fusion_recovered

        self._clouds: Deque[PointCloud2] = deque(maxlen=max(2, config.cloud_buffer_size))
        self._latest_bbox: Optional[TargetBoundingBox] = None
        self._last_bbox_time: Optional[Time] = None
        self._last_bbox_received_time: Optional[Time] = None
        self._last_cloud_time: Optional[Time] = None
        self._last_fused_cloud_stamp: Optional[int] = None
        self._last_estimate_pair: Optional[Tuple[int, int]] = None
        self._last_estimate_point_lidar: Optional[np.ndarray] = None
        self._last_estimate_range: Optional[float] = None
        self._last_estimate_roi_points = 0
        self._last_estimate_cluster_points = 0
        self._prepared_cloud_stamp: Optional[int] = None
        self._prepared_cloud: Optional[PreparedPointCloud] = None
        self._last_target_update: Optional[Time] = None
        self._last_target_sensor_distance: Optional[float] = None
        self._last_target_sensor_time: Optional[Time] = None
        self._target_motion_ready = False
        self._last_measurement_log: Optional[Time] = None
        self._last_fusion_error = ""
        self._last_fusion_error_recoverable = False
        self._last_fusion_warning: Optional[Time] = None
        self._last_tf_fallback_warning: Optional[Time] = None

    @property
    def last_cloud_time(self) -> Optional[Time]:
        return self._last_cloud_time

    @property
    def last_fusion_error(self) -> str:
        return self._last_fusion_error

    @property
    def target_filter(self) -> TargetPositionFilter:
        return self._target_filter

    @property
    def stable(self) -> bool:
        return self._target_filter.stable

    @property
    def last_measurement_range(self) -> Optional[float]:
        return self._last_estimate_range

    def reset(self) -> None:
        self._target_filter.reset()
        self._clouds.clear()
        self._latest_bbox = None
        self._last_bbox_time = None
        self._last_bbox_received_time = None
        self._last_fused_cloud_stamp = None
        self._last_estimate_pair = None
        self._last_estimate_point_lidar = None
        self._last_estimate_range = None
        self._last_estimate_roi_points = 0
        self._last_estimate_cluster_points = 0
        self._prepared_cloud_stamp = None
        self._prepared_cloud = None
        self._last_target_update = None
        self._last_target_sensor_distance = None
        self._last_target_sensor_time = None
        self._target_motion_ready = False
        self._last_fusion_error = ""
        self._last_fusion_error_recoverable = False
        self._last_fusion_warning = None
        self._last_measurement_log = None

    def on_bbox(
        self,
        msg: TargetBoundingBox,
        active_target_id: int,
        accepts_fusion: bool,
        mission_state: Optional[int],
    ) -> None:
        if not accepts_fusion or int(msg.target_id) != int(active_target_id):
            return
        self._last_bbox_received_time = now_time(self._node)
        if normalize_frame(msg.header.frame_id) != self.config.camera_frame:
            self._record_error(
                f"bbox frame '{msg.header.frame_id}' does not match "
                f"'{self.config.camera_frame}'"
            )
            return
        age = seconds_between(self._node.get_clock().now().to_msg(), msg.header.stamp)
        if age < -self.config.sync_tolerance or age > self.config.max_bbox_age:
            self._record_error(
                f"bbox timestamp age {age:.3f}s outside "
                f"[-{self.config.sync_tolerance:.3f}, {self.config.max_bbox_age:.3f}]s"
            )
            return
        self._latest_bbox = msg
        self._last_bbox_time = now_time(self._node)
        self._try_fuse(msg, active_target_id, mission_state)

    def on_cloud(
        self,
        msg: PointCloud2,
        active_target_id: int,
        accepts_fusion: bool,
        mission_state: Optional[int],
    ) -> None:
        if normalize_frame(msg.header.frame_id) != self.config.lidar_frame:
            self._node.get_logger().warning(
                f"lidar frame '{msg.header.frame_id}' does not match "
                f"'{self.config.lidar_frame}'"
            )
            return
        self._last_cloud_time = now_time(self._node)
        self._clouds.append(msg)
        if self._latest_bbox is not None and accepts_fusion:
            self._try_fuse(self._latest_bbox, active_target_id, mission_state)

    def latest_target(self) -> Optional[np.ndarray]:
        now = now_time(self._node)
        if self._last_bbox_time is None or self._last_target_update is None:
            return None
        if (
            (now - self._last_bbox_time).nanoseconds * 1e-9 > self.config.target_timeout
            or (now - self._last_target_update).nanoseconds * 1e-9
            > self.config.target_timeout
            or not self._target_motion_ready
        ):
            return None
        position = self._target_filter.position
        return None if position is None else np.asarray(position, dtype=np.float64)

    def sensor_distance(self) -> Optional[float]:
        if (
            self._last_target_sensor_distance is None
            or self._last_target_sensor_time is None
        ):
            return None
        age = (
            now_time(self._node) - self._last_target_sensor_time
        ).nanoseconds * 1.0e-9
        if age > self.config.target_timeout:
            return None
        return float(self._last_target_sensor_distance)

    def health_fault(self) -> Optional[str]:
        """Return a persistent non-recoverable fusion fault, if one exists."""
        if self._last_bbox_received_time is None:
            return None
        now = now_time(self._node)
        receive_silence = (
            now - self._last_bbox_received_time
        ).nanoseconds * 1e-9
        if receive_silence > self.config.target_timeout:
            return None
        if self._last_bbox_time is None:
            return self._last_fusion_error or "received bbox is not valid for fusion"
        bbox_silence = (now - self._last_bbox_time).nanoseconds * 1e-9
        if bbox_silence > self.config.target_timeout:
            return None
        fusion_age = (
            float("inf")
            if self._last_target_update is None
            else (now - self._last_target_update).nanoseconds * 1e-9
        )
        if (
            fusion_age > self.config.failure_timeout
            and not self._last_fusion_error_recoverable
        ):
            return self._last_fusion_error or "fresh bbox cannot be fused with lidar"
        return None

    def _try_fuse(
        self,
        bbox_msg: TargetBoundingBox,
        active_target_id: int,
        mission_state: Optional[int],
    ) -> None:
        if not self._clouds:
            self._record_error("waiting for buffered /livox/lidar cloud")
            return
        bbox_age = seconds_between(self._node.get_clock().now().to_msg(), bbox_msg.header.stamp)
        if bbox_age < -self.config.sync_tolerance or bbox_age > self.config.max_bbox_age:
            self._record_error(f"bbox became stale before fusion (age={bbox_age:.3f}s)")
            return
        matched = min(
            self._clouds,
            key=lambda cloud: abs(seconds_between(cloud.header.stamp, bbox_msg.header.stamp)),
        )
        offset = abs(seconds_between(matched.header.stamp, bbox_msg.header.stamp))
        if offset > self.config.sync_tolerance:
            self._record_error(
                f"camera-lidar timestamp offset {offset:.3f}s exceeds "
                f"{self.config.sync_tolerance:.3f}s"
            )
            return
        cloud_stamp = stamp_ns(matched.header.stamp)
        if cloud_stamp == self._last_fused_cloud_stamp:
            return
        bbox_stamp = stamp_ns(bbox_msg.header.stamp)
        pair = (cloud_stamp, bbox_stamp)
        if pair != self._last_estimate_pair:
            try:
                if self._prepared_cloud_stamp != cloud_stamp:
                    self._prepared_cloud = self._estimator.prepare(point_cloud_xyz(matched))
                    self._prepared_cloud_stamp = cloud_stamp
                if self._prepared_cloud is None:
                    self._record_error("prepared lidar cloud is unavailable")
                    return
                estimate = self._estimator.estimate_prepared(
                    self._prepared_cloud,
                    BoundingBox(
                        int(bbox_msg.image_width), int(bbox_msg.image_height),
                        int(bbox_msg.x_min), int(bbox_msg.y_min),
                        int(bbox_msg.x_max), int(bbox_msg.y_max),
                    ),
                )
                self._last_estimate_point_lidar = np.asarray(
                    estimate.point_lidar, dtype=np.float64
                )
                self._last_estimate_range = float(estimate.range)
                self._last_estimate_roi_points = int(estimate.roi_points)
                self._last_estimate_cluster_points = int(estimate.cluster_points)
                self._last_estimate_pair = pair
            except TargetObservationUnavailable as exc:
                self._last_estimate_pair = pair
                self._last_estimate_point_lidar = None
                self._last_estimate_range = None
                self._record_error(str(exc), recoverable=True)
                return
            except (AssertionError, ValueError, RuntimeError) as exc:
                self._last_estimate_pair = pair
                self._last_estimate_point_lidar = None
                self._last_estimate_range = None
                self._record_error(str(exc))
                return

        if self._last_estimate_point_lidar is None:
            return
        try:
            point_base = (
                self._lidar_to_base[:3, :3] @ self._last_estimate_point_lidar
                + self._lidar_to_base[:3, 3]
            )
            point_base_msg = PointStamped()
            point_base_msg.header.stamp = matched.header.stamp
            point_base_msg.header.frame_id = self.config.base_frame
            point_base_msg.point.x = float(point_base[0])
            point_base_msg.point.y = float(point_base[1])
            point_base_msg.point.z = float(point_base[2])
            point_map = self._transform_point(
                point_base_msg, self._lookup_fusion_transform(matched.header.stamp)
            )
        except (ValueError, RuntimeError, TransformException) as exc:
            self._record_error(str(exc))
            return

        sample = np.asarray(
            [point_map.point.x, point_map.point.y, point_map.point.z], dtype=np.float64
        )
        if not self._target_filter.add(sample):
            update_age = (
                float("inf")
                if self._last_target_update is None
                else (now_time(self._node) - self._last_target_update).nanoseconds * 1e-9
            )
            if update_age <= self.config.target_timeout:
                self._record_error("target map position rejected as jump outlier")
                return
            self._target_filter.reset()
            self._target_motion_ready = False
            if not self._target_filter.add(sample):
                self._record_error("target map position could not be re-seeded")
                return

        self._last_fused_cloud_stamp = cloud_stamp
        self._last_target_update = now_time(self._node)
        self._last_target_sensor_distance = hypot(float(point_base[0]), float(point_base[1]))
        self._last_target_sensor_time = self._last_target_update
        if self._target_filter.stable:
            self._target_motion_ready = True
            self._last_fusion_error = ""
            self._fusion_recovered()
        else:
            self._last_fusion_error = (
                f"stabilizing fused target samples "
                f"{self._target_filter.sample_count}/{self.config.stable_samples}"
            )

        point_map.header.stamp = now_time(self._node).to_msg()
        point_map.header.frame_id = self.config.global_frame
        self._target_point_publish(point_map)
        measurement_log_age = (
            float("inf")
            if self._last_measurement_log is None
            else (now_time(self._node) - self._last_measurement_log).nanoseconds * 1e-9
        )
        if measurement_log_age >= 1.0 and self._last_estimate_range is not None:
            self._node.get_logger().info(
                f"target measurement id={active_target_id} "
                f"range={self._last_estimate_range:.2f}m "
                f"base_planar={self._last_target_sensor_distance:.2f}m "
                f"base_z={float(point_base[2]):.2f}m "
                f"roi_points={self._last_estimate_roi_points} "
                f"cluster_points={self._last_estimate_cluster_points} "
                f"stable={self._target_filter.stable}"
            )
            self._last_measurement_log = now_time(self._node)
        if (
            mission_state == MissionState.CONFIRM_TARGET
            and self._target_filter.stable
        ):
            self._position_ready()

    def _lookup_fusion_transform(self, stamp: RosTime):
        try:
            return self._tf_buffer.lookup_transform(
                self.config.global_frame,
                self.config.base_frame,
                time_from_msg(self._node, stamp),
                timeout=Duration(seconds=self.config.tf_timeout),
            )
        except TransformException as exc:
            if "future" not in str(exc).lower():
                raise
            latest = self._tf_buffer.lookup_transform(
                self.config.global_frame,
                self.config.base_frame,
                Time(seconds=0, clock_type=self._node.get_clock().clock_type),
                timeout=Duration(seconds=self.config.tf_timeout),
            )
            now = now_time(self._node)
            warning_age = (
                float("inf")
                if self._last_tf_fallback_warning is None
                else (now - self._last_tf_fallback_warning).nanoseconds * 1e-9
            )
            if warning_age >= 2.0:
                lag = seconds_between(stamp, latest.header.stamp)
                self._node.get_logger().warning(
                    "target fusion used latest map -> base TF because the "
                    f"sensor timestamp is ahead of the TF buffer by {lag:.3f}s"
                )
                self._last_tf_fallback_warning = now
            return latest

    @staticmethod
    def _transform_point(point: PointStamped, transform) -> PointStamped:
        rotation = quaternion_to_rotation(transform.transform.rotation)
        vector = np.asarray(
            [point.point.x, point.point.y, point.point.z], dtype=np.float64
        )
        translated = rotation @ vector + np.asarray(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ],
            dtype=np.float64,
        )
        result = PointStamped()
        result.header = transform.header
        result.point.x, result.point.y, result.point.z = map(float, translated)
        return result

    def _record_error(self, reason: str, recoverable: bool = False) -> None:
        reason = str(reason).strip() or "unknown target fusion error"
        changed = reason != self._last_fusion_error
        self._last_fusion_error = reason
        self._last_fusion_error_recoverable = bool(recoverable)
        now = now_time(self._node)
        warning_age = (
            float("inf")
            if self._last_fusion_warning is None
            else (now - self._last_fusion_warning).nanoseconds * 1e-9
        )
        if changed and warning_age >= 2.0:
            self._node.get_logger().warning(f"target fusion waiting: {reason}")
            self._last_fusion_warning = now
