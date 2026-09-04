from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Sequence, Tuple

import numpy as np


Point3 = Tuple[float, float, float]


@dataclass(frozen=True)
class BoundingBox:
    image_width: int
    image_height: int
    x_min: int
    y_min: int
    x_max: int
    y_max: int


@dataclass(frozen=True)
class CameraCalibration:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    distortion: Sequence[float]
    lidar_to_camera: Sequence[float]


@dataclass(frozen=True)
class EstimatorConfig:
    min_range: float = 0.4
    max_range: float = 15.0
    roi_horizontal_inset: float = 0.15
    roi_top_inset: float = 0.08
    roi_bottom_inset: float = 0.15
    fallback_roi_horizontal_inset: float = 0.05
    fallback_roi_top_inset: float = 0.05
    fallback_roi_bottom_inset: float = 0.20
    cluster_gap: float = 0.40
    min_cluster_points: int = 3
    min_cluster_support_ratio: float = 0.50
    point_stride: int = 1
    projection_margin_pixels: float = 32.0
    aspect_ratio_tolerance: float = 0.02


@dataclass(frozen=True)
class TargetEstimate:
    point_lidar: Point3
    range: float
    roi_points: int
    cluster_points: int


class TargetObservationUnavailable(ValueError):
    """A valid bbox currently has too little lidar support for navigation."""


@dataclass(frozen=True)
class PreparedPointCloud:
    """Point cloud data shared by repeated bbox estimates for one timestamp."""

    points_lidar: np.ndarray
    points_camera: np.ndarray
    normalized_x: np.ndarray
    normalized_y: np.ndarray
    range_sq: np.ndarray


class TargetEstimator:
    """Project a PointCloud2-derived XYZ array and select foreground depth."""

    def __init__(
        self,
        calibration: CameraCalibration,
        config: EstimatorConfig,
    ) -> None:
        if calibration.width <= 0 or calibration.height <= 0:
            raise ValueError("camera calibration dimensions must be positive")
        if calibration.fx <= 0.0 or calibration.fy <= 0.0:
            raise ValueError("camera focal lengths must be positive")
        if len(calibration.distortion) != 4:
            raise ValueError("distortion must contain k1, k2, p1, p2")
        if len(calibration.lidar_to_camera) != 16:
            raise ValueError("lidar_to_camera must contain 16 row-major values")
        self._calibration = calibration
        self._config = config
        matrix = np.asarray(calibration.lidar_to_camera, dtype=np.float32)
        self._lidar_to_camera = matrix.reshape(4, 4)

    def prepare(self, lidar_points: np.ndarray) -> PreparedPointCloud:
        """Transform and range-filter a cloud once before bbox-specific selection."""
        points = np.asarray(lidar_points)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError("lidar_points must be an N x 3 array")

        stride = max(1, int(self._config.point_stride))
        points = np.asarray(points[::stride], dtype=np.float32)
        points = points[np.all(np.isfinite(points), axis=1)]
        if points.size == 0:
            raise TargetObservationUnavailable("point cloud has no finite XYZ points")

        rotation = self._lidar_to_camera[:3, :3]
        translation = self._lidar_to_camera[:3, 3]
        camera_points = points @ rotation.T + translation
        depth = camera_points[:, 2]
        range_sq = np.einsum("ij,ij->i", camera_points, camera_points)
        valid = (
            (depth > 1.0e-6)
            & (range_sq >= self._config.min_range ** 2)
            & (range_sq <= self._config.max_range ** 2)
        )
        points = points[valid]
        camera_points = camera_points[valid]
        range_sq = range_sq[valid]
        if points.shape[0] < self._config.min_cluster_points:
            raise TargetObservationUnavailable("not enough points in camera range")
        depth = camera_points[:, 2]
        return PreparedPointCloud(
            points,
            camera_points,
            camera_points[:, 0] / depth,
            camera_points[:, 1] / depth,
            range_sq,
        )

    def estimate(
        self,
        lidar_points: np.ndarray,
        bbox: BoundingBox,
    ) -> TargetEstimate:
        points = np.asarray(lidar_points)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError("lidar_points must be an N x 3 array")
        if (
            bbox.image_width <= 0
            or bbox.image_height <= 0
            or bbox.x_min < 0
            or bbox.y_min < 0
            or bbox.x_min >= bbox.x_max
            or bbox.y_min >= bbox.y_max
            or bbox.x_max > bbox.image_width
            or bbox.y_max > bbox.image_height
        ):
            raise ValueError("invalid bounding box")

        calibration_aspect = (
            float(self._calibration.width) / self._calibration.height
        )
        image_aspect = float(bbox.image_width) / bbox.image_height
        aspect_error = abs(calibration_aspect - image_aspect) / calibration_aspect
        if aspect_error > self._config.aspect_ratio_tolerance:
            raise ValueError("bbox and calibration aspect ratios do not match")

        prepared = self.prepare(points)
        return self.estimate_prepared(prepared, bbox)

    def estimate_prepared(
        self,
        prepared: PreparedPointCloud,
        bbox: BoundingBox,
    ) -> TargetEstimate:
        """Select a target from prepared points without repeating cloud transforms."""
        if (
            bbox.image_width <= 0
            or bbox.image_height <= 0
            or bbox.x_min < 0
            or bbox.y_min < 0
            or bbox.x_min >= bbox.x_max
            or bbox.y_min >= bbox.y_max
            or bbox.x_max > bbox.image_width
            or bbox.y_max > bbox.image_height
        ):
            raise ValueError("invalid bounding box")

        calibration_aspect = (
            float(self._calibration.width) / self._calibration.height
        )
        image_aspect = float(bbox.image_width) / bbox.image_height
        aspect_error = abs(calibration_aspect - image_aspect) / calibration_aspect
        if aspect_error > self._config.aspect_ratio_tolerance:
            raise ValueError("bbox and calibration aspect ratios do not match")

        scale_x = float(bbox.image_width) / self._calibration.width
        scale_y = float(bbox.image_height) / self._calibration.height
        fx = np.float32(self._calibration.fx * scale_x)
        fy = np.float32(self._calibration.fy * scale_y)
        cx = np.float32(self._calibration.cx * scale_x)
        cy = np.float32(self._calibration.cy * scale_y)

        # Lens distortion is small for this camera. Use an expanded pinhole
        # rectangle first, then apply the exact distortion test only to those
        # candidates. This keeps the result exact while avoiding work on most
        # points outside a narrow bbox.
        normalized_x = prepared.normalized_x
        normalized_y = prepared.normalized_y
        rough_x = fx * normalized_x + cx
        rough_y = fy * normalized_y + cy
        margin = max(0.0, float(self._config.projection_margin_pixels))
        margin_x = max(margin, 0.08 * float(bbox.x_max - bbox.x_min))
        margin_y = max(margin, 0.08 * float(bbox.y_max - bbox.y_min))
        rough_inside = (
            (rough_x >= bbox.x_min - margin_x)
            & (rough_x < bbox.x_max + margin_x)
            & (rough_y >= bbox.y_min - margin_y)
            & (rough_y < bbox.y_max + margin_y)
        )
        if int(np.count_nonzero(rough_inside)) < self._config.min_cluster_points:
            raise TargetObservationUnavailable(
                "not enough projected lidar points near bbox "
                f"({int(np.count_nonzero(rough_inside))}/"
                f"{self._config.min_cluster_points})"
            )

        points = prepared.points_lidar[rough_inside]
        range_sq = prepared.range_sq[rough_inside]

        normalized_x = prepared.normalized_x[rough_inside]
        normalized_y = prepared.normalized_y[rough_inside]
        radius2 = normalized_x * normalized_x + normalized_y * normalized_y
        k1, k2, p1, p2 = self._calibration.distortion
        radial = 1.0 + k1 * radius2 + k2 * radius2 * radius2
        distorted_x = (
            normalized_x * radial
            + 2.0 * p1 * normalized_x * normalized_y
            + p2 * (radius2 + 2.0 * normalized_x * normalized_x)
        )
        distorted_y = (
            normalized_y * radial
            + p1 * (radius2 + 2.0 * normalized_y * normalized_y)
            + 2.0 * p2 * normalized_x * normalized_y
        )
        image_x = fx * distorted_x + cx
        image_y = fy * distorted_y + cy

        box_width = float(bbox.x_max - bbox.x_min)
        box_height = float(bbox.y_max - bbox.y_min)
        def roi_mask(horizontal: float, top: float, bottom: float) -> np.ndarray:
            horizontal = float(np.clip(horizontal, 0.0, 0.49))
            top = float(np.clip(top, 0.0, 0.49))
            bottom = float(np.clip(bottom, 0.0, 0.49))
            return (
                (image_x >= bbox.x_min + box_width * horizontal)
                & (image_x < bbox.x_max - box_width * horizontal)
                & (image_y >= bbox.y_min + box_height * top)
                & (image_y < bbox.y_max - box_height * bottom)
            )

        minimum = max(1, int(self._config.min_cluster_points))
        inside = roi_mask(
            self._config.roi_horizontal_inset,
            self._config.roi_top_inset,
            self._config.roi_bottom_inset,
        )
        if int(np.count_nonzero(inside)) < minimum:
            inside = roi_mask(
                self._config.fallback_roi_horizontal_inset,
                self._config.fallback_roi_top_inset,
                self._config.fallback_roi_bottom_inset,
            )
        candidates = points[inside]
        candidate_ranges = np.sqrt(range_sq[inside])
        if candidates.shape[0] < minimum:
            raise TargetObservationUnavailable(
                "not enough projected lidar points inside bbox ROI "
                f"({candidates.shape[0]}/{minimum})"
            )

        order = np.argsort(candidate_ranges)
        candidates = candidates[order]
        candidate_ranges = candidate_ranges[order]
        boundaries = np.flatnonzero(
            np.diff(candidate_ranges) > self._config.cluster_gap
        ) + 1
        starts = np.concatenate((np.array([0]), boundaries))
        ends = np.concatenate((boundaries, np.array([len(candidate_ranges)])))

        clusters = [
            (int(begin), int(end))
            for begin, end in zip(starts, ends)
            if int(end - begin) >= minimum
        ]
        selected = None
        selected_ranges = None
        if clusters:
            largest = max(end - begin for begin, end in clusters)
            support_ratio = float(
                np.clip(self._config.min_cluster_support_ratio, 0.0, 1.0)
            )
            required_support = max(minimum, int(np.ceil(largest * support_ratio)))
            for begin, end in clusters:
                if end - begin >= required_support:
                    selected = candidates[begin:end]
                    selected_ranges = candidate_ranges[begin:end]
                    break
        if selected is None or selected_ranges is None:
            raise TargetObservationUnavailable(
                "no sufficiently supported lidar depth cluster inside bbox ROI "
                f"(points={candidates.shape[0]}, largest={largest if clusters else 0}, "
                f"required={required_support if clusters else minimum})"
            )

        median_point = np.median(selected, axis=0)
        return TargetEstimate(
            point_lidar=(
                float(median_point[0]),
                float(median_point[1]),
                float(median_point[2]),
            ),
            range=float(np.median(selected_ranges)),
            roi_points=int(candidates.shape[0]),
            cluster_points=int(selected.shape[0]),
        )


@dataclass(frozen=True)
class TargetFilterConfig:
    stable_samples: int = 3
    stability_window: int = 5
    max_sample_jump: float = 0.60
    max_stability_spread: float = 0.50
    smoothing_alpha: float = 0.55


class TargetPositionFilter:
    def __init__(self, config: TargetFilterConfig) -> None:
        self._config = config
        self._recent: Deque[np.ndarray] = deque(
            maxlen=max(1, int(config.stability_window))
        )
        self._filtered: Optional[np.ndarray] = None
        self._accepted_samples = 0

    def add(self, sample: Sequence[float]) -> bool:
        point = np.asarray(sample, dtype=np.float64)
        if point.shape != (3,) or not np.all(np.isfinite(point)):
            return False
        if (
            self._filtered is not None
            and np.linalg.norm(point - self._filtered)
            > self._config.max_sample_jump
        ):
            return False
        if self._filtered is None:
            self._filtered = point.copy()
        else:
            alpha = float(np.clip(self._config.smoothing_alpha, 0.0, 1.0))
            self._filtered = alpha * point + (1.0 - alpha) * self._filtered
        self._accepted_samples += 1
        self._recent.append(point.copy())
        return True

    def reset(self) -> None:
        self._recent.clear()
        self._filtered = None
        self._accepted_samples = 0

    @property
    def stable(self) -> bool:
        minimum = max(1, int(self._config.stable_samples))
        if (
            self._filtered is None
            or self._accepted_samples < minimum
            or len(self._recent) < min(minimum, self._recent.maxlen or minimum)
        ):
            return False
        spread = max(
            float(np.linalg.norm(sample - self._filtered))
            for sample in self._recent
        )
        return spread <= self._config.max_stability_spread

    @property
    def position(self) -> Optional[Point3]:
        if self._filtered is None:
            return None
        return tuple(float(value) for value in self._filtered)

    @property
    def sample_count(self) -> int:
        return self._accepted_samples
