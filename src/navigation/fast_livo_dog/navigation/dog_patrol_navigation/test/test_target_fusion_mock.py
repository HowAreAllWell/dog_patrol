from pathlib import Path

import numpy as np
import pytest
import yaml
from std_msgs.msg import Header
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2

from dog_patrol_navigation.navigation_mission_coordinator import (
    device_lidar_to_camera,
    device_lidar_to_base,
    point_cloud_xyz,
)
from dog_patrol_navigation.target_estimator import (
    BoundingBox,
    CameraCalibration,
    EstimatorConfig,
    TargetEstimator,
)


def calibration():
    return CameraCalibration(
        width=1280,
        height=1024,
        fx=1291.9363,
        fy=1291.9620,
        cx=627.2076,
        cy=515.7676,
        distortion=[
            -0.09454293973133182,
            0.2549060543551655,
            -0.001344248648876502,
            -0.0019596226081252314,
        ],
        lidar_to_camera=[
            0.011685, -0.999924, 0.003980, -0.029332,
            0.497425, 0.002360, -0.867504, -0.149256,
            0.867428, 0.012116, 0.497414, 0.014702,
            0.0, 0.0, 0.0, 1.0,
        ],
    )


def project(camera_points, camera):
    x = camera_points[:, 0] / camera_points[:, 2]
    y = camera_points[:, 1] / camera_points[:, 2]
    r2 = x * x + y * y
    k1, k2, p1, p2 = camera.distortion
    radial = 1.0 + k1 * r2 + k2 * r2 * r2
    xd = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
    yd = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
    return np.column_stack(
        [camera.fx * xd + camera.cx, camera.fy * yd + camera.cy]
    )


def make_mock_cloud(camera):
    transform = np.asarray(camera.lidar_to_camera, dtype=np.float64).reshape(4, 4)
    camera_to_lidar = np.linalg.inv(transform)

    # A person-sized foreground cluster around 4 m.
    x, y = np.meshgrid(np.linspace(0.05, 0.35, 8), np.linspace(-0.45, 0.45, 12))
    target_camera = np.column_stack(
        [x.reshape(-1), y.reshape(-1), np.full(x.size, 4.0)]
    )

    target_pixels = project(target_camera, camera)
    x_min = max(0, int(np.floor(target_pixels[:, 0].min() - 24)))
    y_min = max(0, int(np.floor(target_pixels[:, 1].min() - 24)))
    x_max = min(camera.width, int(np.ceil(target_pixels[:, 0].max() + 24)))
    y_max = min(camera.height, int(np.ceil(target_pixels[:, 1].max() + 24)))
    bbox = BoundingBox(camera.width, camera.height, x_min, y_min, x_max, y_max)

    # Background points project into the same bbox but are 8 m away. The
    # estimator must retain the nearer contiguous depth cluster.
    background_x, background_y = np.meshgrid(
        np.linspace(0.12, 0.28, 5), np.linspace(-0.25, 0.25, 5)
    )
    background_camera = np.column_stack(
        [background_x.reshape(-1), background_y.reshape(-1), np.full(background_x.size, 8.0)]
    )
    all_camera = np.vstack([target_camera, background_camera])
    homogeneous = np.column_stack([all_camera, np.ones(all_camera.shape[0])])
    lidar = (camera_to_lidar @ homogeneous.T).T[:, :3]
    return lidar.astype(np.float32), bbox


def test_real_calibration_mock_recovers_foreground_target():
    camera = calibration()
    estimator = TargetEstimator(
        camera,
        EstimatorConfig(
            min_cluster_points=3,
            cluster_gap=0.40,
            projection_margin_pixels=32.0,
        ),
    )
    points, bbox = make_mock_cloud(camera)
    estimate = estimator.estimate(points, bbox)

    assert estimate.range == pytest.approx(4.0, abs=0.08)
    assert estimate.cluster_points >= 3
    assert estimate.range < 5.0


def test_fast_livo_device_matrix_is_consumed_as_lidar_to_camera():
    nav_root = Path(__file__).resolve().parents[3]
    device_file = nav_root / "config" / "device_parameters.yaml"
    params = yaml.safe_load(device_file.read_text(encoding="utf-8"))["/**"][
        "ros__parameters"
    ]
    configured = np.asarray(
        params["extrin_calib"]["T_camera_lidar"], dtype=np.float64
    ).reshape(4, 4)

    runtime = device_lidar_to_camera(configured.reshape(-1))

    assert np.allclose(runtime, configured)
    assert not np.allclose(runtime, np.linalg.inv(configured))


def test_fast_livo_device_body_matrix_is_inverted_for_lidar_points():
    nav_root = Path(__file__).resolve().parents[3]
    device_file = nav_root / "config" / "device_parameters.yaml"
    params = yaml.safe_load(device_file.read_text(encoding="utf-8"))["/**"][
        "ros__parameters"
    ]
    configured = np.asarray(params["T_lidar_base"], dtype=np.float64).reshape(4, 4)

    runtime = device_lidar_to_base(configured.reshape(-1))

    assert np.allclose(runtime, np.linalg.inv(configured))
    assert runtime[:3, 3] == pytest.approx([0.327138, 0.014138, 0.312380], abs=1e-5)


def test_seven_metre_camera_target_remains_near_seven_metres_in_base():
    nav_root = Path(__file__).resolve().parents[3]
    params = yaml.safe_load(
        (nav_root / "config" / "device_parameters.yaml").read_text(encoding="utf-8")
    )["/**"]["ros__parameters"]
    lidar_to_camera = device_lidar_to_camera(
        params["extrin_calib"]["T_camera_lidar"]
    )
    lidar_to_base = device_lidar_to_base(params["T_lidar_base"])
    point_camera = np.asarray([0.0, 0.0, 7.0, 1.0])
    point_lidar = np.linalg.inv(lidar_to_camera) @ point_camera
    point_base = lidar_to_base @ point_lidar

    assert np.linalg.norm(point_base[:2]) == pytest.approx(7.31, abs=0.05)
    assert point_base[2] == pytest.approx(0.36, abs=0.05)


def test_map_standoff_geometry_is_three_metres():
    robot_map = np.array([10.0, -2.0])
    yaw = 0.4
    rotation = np.array(
        [[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]]
    )
    target_base = np.array([4.0, 0.3])
    target_map = robot_map + rotation @ target_base
    direction = target_map - robot_map
    goal_map = target_map - direction / np.linalg.norm(direction) * 3.0

    assert np.linalg.norm(target_map - robot_map) - 3.0 == pytest.approx(
        np.linalg.norm(goal_map - robot_map), abs=1.0e-6
    )
    assert np.linalg.norm(target_map - goal_map) == pytest.approx(3.0, abs=1.0e-6)


def test_extracts_xyz_from_livox_mixed_datatype_cloud():
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="tag", offset=16, datatype=PointField.UINT8, count=1),
        PointField(name="line", offset=17, datatype=PointField.UINT8, count=1),
        PointField(name="timestamp", offset=18, datatype=PointField.FLOAT64, count=1),
    ]
    cloud = point_cloud2.create_cloud(
        Header(frame_id="livox_frame"),
        fields,
        [
            (1.0, 2.0, 3.0, 10.0, 1, 2, 100.0),
            (4.0, 5.0, 6.0, 20.0, 3, 4, 101.0),
            (float("nan"), 8.0, 9.0, 30.0, 5, 6, 102.0),
        ],
    )

    points = point_cloud_xyz(cloud)

    assert points.dtype == np.float32
    assert points == pytest.approx(
        np.asarray([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float32)
    )
