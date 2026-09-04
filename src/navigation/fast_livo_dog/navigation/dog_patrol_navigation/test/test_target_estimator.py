import numpy as np
import pytest

from dog_patrol_navigation.target_estimator import (
    BoundingBox,
    CameraCalibration,
    EstimatorConfig,
    TargetEstimator,
    TargetFilterConfig,
    TargetObservationUnavailable,
    TargetPositionFilter,
)


def identity_calibration():
    return CameraCalibration(
        width=640,
        height=512,
        fx=500.0,
        fy=500.0,
        cx=320.0,
        cy=256.0,
        distortion=[0.0, 0.0, 0.0, 0.0],
        lidar_to_camera=[
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0,
        ],
    )


def test_scales_calibration_and_selects_nearest_valid_cluster():
    estimator = TargetEstimator(
        identity_calibration(),
        EstimatorConfig(
            min_cluster_points=3,
            cluster_gap=0.35,
            roi_horizontal_inset=0.0,
            roi_top_inset=0.0,
            roi_bottom_inset=0.0,
        ),
    )
    bbox = BoundingBox(1280, 1024, 560, 430, 720, 590)
    points = np.asarray(
        [
            [0.00, 0.00, 3.9], [0.02, 0.01, 4.0], [-0.02, -0.01, 4.1],
            [0.00, 0.00, 7.8], [0.05, 0.00, 8.0], [-0.05, 0.00, 8.1],
            [0.03, 0.02, 8.2],
        ]
    )
    result = estimator.estimate(points, bbox)
    assert result.range == pytest.approx(4.0, abs=0.15)
    assert result.cluster_points == 3

    prepared_result = estimator.estimate_prepared(estimator.prepare(points), bbox)
    assert prepared_result.point_lidar == pytest.approx(result.point_lidar, abs=1.0e-5)
    assert prepared_result.range == pytest.approx(result.range, abs=1.0e-5)


def test_prepared_cloud_reuses_transform_for_multiple_boxes():
    estimator = TargetEstimator(
        identity_calibration(),
        EstimatorConfig(
            min_cluster_points=2,
            cluster_gap=0.35,
            roi_horizontal_inset=0.0,
            roi_top_inset=0.0,
            roi_bottom_inset=0.0,
        ),
    )
    points = np.asarray(
        [[-0.1, 0.0, 4.0], [0.1, 0.0, 4.1], [0.0, 0.0, 8.0]],
        dtype=np.float32,
    )
    prepared = estimator.prepare(points)
    result = estimator.estimate_prepared(
        prepared, BoundingBox(640, 512, 300, 230, 340, 280)
    )
    assert result.range == pytest.approx(4.05, abs=0.1)
    assert prepared.points_camera.shape[0] == 3


def test_rejects_incompatible_image_aspect_ratio():
    estimator = TargetEstimator(identity_calibration(), EstimatorConfig())
    bbox = BoundingBox(1280, 720, 500, 200, 700, 600)
    with pytest.raises(ValueError, match="aspect"):
        estimator.estimate(np.asarray([[0.0, 0.0, 4.0]]), bbox)


def test_filter_requires_stable_samples_and_rejects_jump():
    position_filter = TargetPositionFilter(
        TargetFilterConfig(
            stable_samples=3,
            stability_window=3,
            max_sample_jump=0.6,
            max_stability_spread=0.5,
        )
    )
    assert position_filter.add((4.0, 1.0, 0.0))
    assert position_filter.add((4.1, 1.0, 0.0))
    assert not position_filter.stable
    assert position_filter.add((4.0, 1.1, 0.0))
    assert position_filter.stable
    assert not position_filter.add((8.0, 8.0, 0.0))
    assert position_filter.sample_count == 3


def test_fallback_roi_recovers_sparse_edge_points():
    estimator = TargetEstimator(
        identity_calibration(),
        EstimatorConfig(
            min_cluster_points=3,
            roi_horizontal_inset=0.25,
            roi_top_inset=0.25,
            roi_bottom_inset=0.25,
            fallback_roi_horizontal_inset=0.05,
            fallback_roi_top_inset=0.05,
            fallback_roi_bottom_inset=0.20,
        ),
    )
    bbox = BoundingBox(640, 512, 200, 100, 440, 412)
    pixels_x = np.asarray([225.0, 240.0, 255.0])
    depth = np.asarray([4.0, 4.05, 4.1])
    points = np.column_stack(
        [
            (pixels_x - 320.0) / 500.0 * depth,
            np.zeros(3),
            depth,
        ]
    )

    result = estimator.estimate(points, bbox)

    assert result.cluster_points == 3
    assert result.range == pytest.approx(4.05, abs=0.1)


def test_tiny_near_cluster_does_not_override_supported_target():
    estimator = TargetEstimator(
        identity_calibration(),
        EstimatorConfig(
            min_cluster_points=3,
            min_cluster_support_ratio=0.50,
            roi_horizontal_inset=0.0,
            roi_top_inset=0.0,
            roi_bottom_inset=0.0,
        ),
    )
    near_clutter = np.column_stack(
        [np.linspace(-0.01, 0.01, 3), np.zeros(3), np.full(3, 2.0)]
    )
    person = np.column_stack(
        [np.linspace(-0.08, 0.08, 8), np.zeros(8), np.full(8, 5.0)]
    )

    result = estimator.estimate(
        np.vstack([near_clutter, person]),
        BoundingBox(640, 512, 250, 180, 390, 340),
    )

    assert result.cluster_points == 8
    assert result.range == pytest.approx(5.0, abs=0.1)


def test_insufficient_bbox_points_is_recoverable_observation_gap():
    estimator = TargetEstimator(identity_calibration(), EstimatorConfig())

    with pytest.raises(TargetObservationUnavailable):
        estimator.estimate(
            np.asarray([[0.0, 0.0, 4.0], [0.1, 0.0, 4.0]]),
            BoundingBox(640, 512, 250, 180, 390, 340),
        )
