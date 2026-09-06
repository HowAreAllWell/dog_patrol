"""Bridge the patrol mission contract to the existing M20 Nav2 control chain."""

from collections import deque
from math import hypot
import os
from typing import Deque, Dict, Optional, Tuple

import numpy as np
import yaml
import rclpy
from action_msgs.srv import CancelGoal
from builtin_interfaces.msg import Time as RosTime
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Empty
from tf2_ros import Buffer, TransformException, TransformListener

from dog_patrol_interfaces.msg import (
    MissionEvent,
    MissionState,
    TargetBoundingBox,
    TargetNavigationStatus,
)

from .navigation_policy import (
    compute_standoff_decision,
    navigation_policy,
)
from .target_estimator import (
    BoundingBox,
    CameraCalibration,
    EstimatorConfig,
    PreparedPointCloud,
    TargetEstimator,
    TargetFilterConfig,
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
    # fast_livo_dog composes this matrix as T_body_base, so despite the YAML
    # name it maps base_link coordinates into the lidar/body frame. Target
    # points arrive in livox_frame and therefore need the inverse transform.
    return np.linalg.inv(matrix)


class NavigationMissionCoordinator(Node):
    """Coordinate target approach while leaving motion control to existing DWB."""

    def __init__(self) -> None:
        super().__init__("navigation_mission_coordinator")
        self._declare_parameters()
        self._read_parameters()

        self._mission_state: Optional[MissionState] = None
        self._active_target_id = 0
        self._clouds: Deque[PointCloud2] = deque()
        self._latest_bbox: Optional[TargetBoundingBox] = None
        self._last_bbox_time: Optional[Time] = None
        self._last_cloud_time: Optional[Time] = None
        self._last_odom_time: Optional[Time] = None
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
        self._last_planned_target: Optional[np.ndarray] = None
        self._last_plan_request: Optional[Time] = None
        self._arrival_hold_start: Optional[Time] = None
        self._last_stop_publish: Optional[Time] = None
        self._controller_cancel_pending = False
        self._last_controller_cancel_warning: Optional[Time] = None
        self._last_ready_event: Optional[Time] = None
        self._technical_fault_start: Optional[Time] = None
        self._planner_failure_start: Optional[Time] = None
        self._technical_fault_reason = ""
        self._planner_failure_reason = ""
        self._last_fusion_error = ""
        self._last_fusion_error_recoverable = False
        self._last_fusion_warning: Optional[Time] = None
        self._last_tf_fallback_warning: Optional[Time] = None
        self._last_measurement_log: Optional[Time] = None
        self._last_motion_log: Optional[Time] = None
        self._last_bbox_received_time: Optional[Time] = None
        self._last_startup_reason = "waiting for mission state"
        self._latest_linear_speed = float("inf")
        self._latest_angular_speed = float("inf")
        self._plan_in_flight = False
        self._plan_generation = 0
        self._active_planner_goal_handle = None
        self._active_planner_goal_generation = -1
        self._arrival_reported = False
        self._sent_events: Dict[Tuple[int, int], bool] = {}
        self._patrol_interruption_pose: Optional[PoseStamped] = None
        self._recovery_return_path_requested = False
        self._recovery_return_path_ready = False
        self._recovery_return_arrived = False
        self._recovery_hold_start: Optional[Time] = None
        self._patrol_resume_sent_during_recovery = False

        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        event_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        bbox_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self._event_pub = self.create_publisher(MissionEvent, self.event_topic, event_qos)
        self._status_pub = self.create_publisher(
            TargetNavigationStatus, self.status_topic, event_qos
        )
        self._path_pub = self.create_publisher(Path, self.global_path_topic, 10)
        self._target_point_pub = self.create_publisher(
            PointStamped, self.target_point_topic, 10
        )
        self._target_goal_pub = self.create_publisher(
            PoseStamped, self.target_goal_topic, 10
        )
        self._pause_pub = self.create_publisher(Empty, self.pause_topic, 10)
        self._resume_pub = self.create_publisher(Empty, self.resume_topic, 10)
        self._resume_from_current_pub = self.create_publisher(
            Empty, self.resume_from_current_topic, 10
        )

        self._state_sub = self.create_subscription(
            MissionState, self.state_topic, self._on_mission_state, state_qos
        )
        self._bbox_sub = self.create_subscription(
            TargetBoundingBox, self.bbox_topic, self._on_bbox, bbox_qos
        )
        self._cloud_sub = self.create_subscription(
            PointCloud2, self.lidar_topic, self._on_cloud, qos_profile_sensor_data
        )
        self._odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self._on_odom, qos_profile_sensor_data
        )
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._planner_client = ActionClient(self, ComputePathToPose, self.planner_action)
        cancel_service = f"{self.follow_path_action.rstrip('/')}/_action/cancel_goal"
        self._controller_cancel_client = self.create_client(CancelGoal, cancel_service)
        self._tick_timer = self.create_timer(1.0 / self.tick_rate, self._tick)
        self._status_timer = self.create_timer(
            1.0 / self.status_rate, self._publish_status
        )

        self.get_logger().info(
            "navigation coordinator ready: "
            f"state={self.state_topic} bbox={self.bbox_topic} "
            f"lidar={self.lidar_topic} path={self.global_path_topic}"
        )

    def _declare_parameters(self) -> None:
        string_defaults = {
            "topics.mission_state": "/mission/state",
            "topics.mission_event": "/mission/event",
            "topics.target_bbox": "/perception/selected_target_bbox",
            "topics.target_status": "/navigation/target_status",
            "topics.lidar": "/livox/lidar",
            "topics.odom": "/odom",
            "topics.global_path": "/global_path",
            "topics.pause_patrol": "/waypoint_sequence/pause",
            "topics.resume_patrol": "/waypoint_sequence/resume",
            "topics.resume_patrol_from_current": "/waypoint_sequence/resume_from_current",
            "topics.nav_cmd": "/NAV_CMD",
            "topics.target_point": "/navigation/target_point",
            "topics.target_goal": "/navigation/target_goal",
            "planner.action_name": "/compute_path_to_pose",
            "planner.id": "GridBased",
            "controller.action_name": "/follow_path",
            "frames.global": "map",
            "frames.base": "base_link",
            "frames.robot": "base_footprint",
            "frames.lidar": "livox_frame",
            "frames.camera_optical": "camera_link",
            "calibration.device_parameters_file": "",
        }
        for name, value in string_defaults.items():
            self.declare_parameter(name, value)
        numeric_defaults = {
            "calibration.image_width": 1280,
            "calibration.image_height": 1024,
            "calibration.fx": 1291.9363,
            "calibration.fy": 1291.9620,
            "calibration.cx": 627.2076,
            "calibration.cy": 515.7676,
            # Perception adds about 0.31 s from image capture to bbox
            # publication; tolerate a short scheduling/inference delay.
            "fusion.max_bbox_age": 0.80,
            "fusion.sync_tolerance": 0.12,
            "fusion.cloud_buffer_size": 20,
            "fusion.min_range": 0.40,
            "fusion.max_range": 15.0,
            "fusion.roi_horizontal_inset": 0.15,
            "fusion.roi_top_inset": 0.08,
            "fusion.roi_bottom_inset": 0.15,
            "fusion.fallback_roi_horizontal_inset": 0.05,
            "fusion.fallback_roi_top_inset": 0.05,
            "fusion.fallback_roi_bottom_inset": 0.20,
            "fusion.cluster_gap": 0.40,
            "fusion.min_cluster_points": 3,
            "fusion.min_cluster_support_ratio": 0.50,
            "fusion.point_stride": 1,
            "fusion.projection_margin_pixels": 32.0,
            "fusion.aspect_ratio_tolerance": 0.02,
            "fusion.stable_samples": 3,
            "fusion.stability_window": 5,
            "fusion.max_sample_jump": 0.60,
            "fusion.max_stability_spread": 0.50,
            "fusion.smoothing_alpha": 0.55,
            "fusion.target_timeout": 0.90,
            "fusion.failure_timeout": 3.0,
            "motion.approach_distance": 3.0,
            "motion.tracking_distance": 3.0,
            "motion.planning_goal_distance": 1.0,
            "motion.arrival_distance_tolerance": 0.10,
            "motion.arrival_linear_speed": 0.05,
            "motion.arrival_angular_speed": 0.10,
            "motion.arrival_hold_time": 0.50,
            "motion.recovery_position_tolerance": 0.25,
            "motion.approach_replan_period": 0.50,
            "motion.tracking_replan_period": 0.50,
            "motion.target_replan_distance": 0.25,
            "motion.stop_republish_period": 0.20,
            "runtime.tick_rate": 10.0,
            "runtime.status_rate": 10.0,
            "runtime.ready_lidar_timeout": 0.50,
            "runtime.ready_event_period": 1.0,
            "runtime.technical_error_timeout": 3.0,
            "runtime.tf_timeout": 0.05,
            "runtime.strict_ready_checks": True,
        }
        for name, value in numeric_defaults.items():
            self.declare_parameter(name, value)
        self.declare_parameter(
            "calibration.distortion",
            [-0.09454293973133182, 0.2549060543551655,
             -0.001344248648876502, -0.0019596226081252314],
        )
        self.declare_parameter(
            "calibration.lidar_to_camera",
            [0.011685, -0.999924, 0.003980, -0.029332,
             0.497425, 0.002360, -0.867504, -0.149256,
             0.867428, 0.012116, 0.497414, 0.014702,
             0.0, 0.0, 0.0, 1.0],
        )
        self.declare_parameter(
            "calibration.lidar_to_base",
            [0.881055, -0.020738, 0.472559, 0.327138,
             0.016898, 0.999780, 0.012370, 0.014138,
             -0.472713, -0.002913, 0.881212, 0.312380,
             0.0, 0.0, 0.0, 1.0],
        )

    def _read_parameters(self) -> None:
        value = self.get_parameter
        self.state_topic = value("topics.mission_state").value
        self.event_topic = value("topics.mission_event").value
        self.bbox_topic = value("topics.target_bbox").value
        self.status_topic = value("topics.target_status").value
        self.lidar_topic = value("topics.lidar").value
        self.odom_topic = value("topics.odom").value
        self.global_path_topic = value("topics.global_path").value
        self.pause_topic = value("topics.pause_patrol").value
        self.resume_topic = value("topics.resume_patrol").value
        self.resume_from_current_topic = value(
            "topics.resume_patrol_from_current"
        ).value
        self.nav_cmd_topic = value("topics.nav_cmd").value
        self.target_point_topic = value("topics.target_point").value
        self.target_goal_topic = value("topics.target_goal").value
        self.planner_action = value("planner.action_name").value
        self.planner_id = value("planner.id").value
        self.follow_path_action = value("controller.action_name").value
        self.global_frame = normalize_frame(value("frames.global").value)
        self.base_frame = normalize_frame(value("frames.base").value)
        self.robot_frame = normalize_frame(value("frames.robot").value)
        self.lidar_frame = normalize_frame(value("frames.lidar").value)
        self.camera_frame = normalize_frame(value("frames.camera_optical").value)

        calibration_values = self._load_calibration(value)
        calibration = CameraCalibration(
            width=int(calibration_values["width"]),
            height=int(calibration_values["height"]),
            fx=float(calibration_values["fx"]),
            fy=float(calibration_values["fy"]),
            cx=float(calibration_values["cx"]),
            cy=float(calibration_values["cy"]),
            distortion=list(calibration_values["distortion"]),
            lidar_to_camera=list(calibration_values["lidar_to_camera"]),
        )
        self._lidar_to_base = np.asarray(
            calibration_values["lidar_to_base"], dtype=np.float64
        ).reshape(4, 4)
        self._estimator = TargetEstimator(
            calibration,
            EstimatorConfig(
                min_range=float(value("fusion.min_range").value),
                max_range=float(value("fusion.max_range").value),
                roi_horizontal_inset=float(value("fusion.roi_horizontal_inset").value),
                roi_top_inset=float(value("fusion.roi_top_inset").value),
                roi_bottom_inset=float(value("fusion.roi_bottom_inset").value),
                fallback_roi_horizontal_inset=float(
                    value("fusion.fallback_roi_horizontal_inset").value
                ),
                fallback_roi_top_inset=float(
                    value("fusion.fallback_roi_top_inset").value
                ),
                fallback_roi_bottom_inset=float(
                    value("fusion.fallback_roi_bottom_inset").value
                ),
                cluster_gap=float(value("fusion.cluster_gap").value),
                min_cluster_points=max(1, int(value("fusion.min_cluster_points").value)),
                min_cluster_support_ratio=float(
                    value("fusion.min_cluster_support_ratio").value
                ),
                point_stride=max(1, int(value("fusion.point_stride").value)),
                projection_margin_pixels=float(
                    value("fusion.projection_margin_pixels").value
                ),
                aspect_ratio_tolerance=float(value("fusion.aspect_ratio_tolerance").value),
            ),
        )
        self._target_filter = TargetPositionFilter(
            TargetFilterConfig(
                stable_samples=max(1, int(value("fusion.stable_samples").value)),
                stability_window=max(1, int(value("fusion.stability_window").value)),
                max_sample_jump=float(value("fusion.max_sample_jump").value),
                max_stability_spread=float(value("fusion.max_stability_spread").value),
                smoothing_alpha=float(value("fusion.smoothing_alpha").value),
            )
        )
        self.stable_samples = max(1, int(value("fusion.stable_samples").value))
        self.max_bbox_age = float(value("fusion.max_bbox_age").value)
        self.sync_tolerance = float(value("fusion.sync_tolerance").value)
        self.cloud_buffer_size = max(2, int(value("fusion.cloud_buffer_size").value))
        self._clouds = deque(maxlen=self.cloud_buffer_size)
        self.target_timeout = float(value("fusion.target_timeout").value)
        self.failure_timeout = float(value("fusion.failure_timeout").value)
        self.approach_distance = float(value("motion.approach_distance").value)
        self.tracking_distance = float(value("motion.tracking_distance").value)
        self.planning_goal_distance = float(
            value("motion.planning_goal_distance").value
        )
        self.arrival_tolerance = float(value("motion.arrival_distance_tolerance").value)
        self.arrival_linear_speed = float(value("motion.arrival_linear_speed").value)
        self.arrival_angular_speed = float(value("motion.arrival_angular_speed").value)
        self.arrival_hold_time = float(value("motion.arrival_hold_time").value)
        self.recovery_position_tolerance = float(
            value("motion.recovery_position_tolerance").value
        )
        self.approach_replan_period = float(value("motion.approach_replan_period").value)
        self.tracking_replan_period = float(value("motion.tracking_replan_period").value)
        self.target_replan_distance = float(value("motion.target_replan_distance").value)
        self.stop_republish_period = float(value("motion.stop_republish_period").value)
        self.tick_rate = max(1.0, float(value("runtime.tick_rate").value))
        self.status_rate = max(1.0, float(value("runtime.status_rate").value))
        self.ready_lidar_timeout = float(value("runtime.ready_lidar_timeout").value)
        self.ready_event_period = max(0.1, float(value("runtime.ready_event_period").value))
        self.technical_error_timeout = float(value("runtime.technical_error_timeout").value)
        self.tf_timeout = float(value("runtime.tf_timeout").value)
        self.strict_ready_checks = bool(value("runtime.strict_ready_checks").value)

    def _load_calibration(self, value) -> Dict[str, object]:
        """Use fast_livo_dog device_parameters.yaml as the calibration source of truth."""
        output_width = max(1, int(value("calibration.image_width").value))
        output_height = max(1, int(value("calibration.image_height").value))
        result = {
            "width": output_width,
            "height": output_height,
            "fx": float(value("calibration.fx").value),
            "fy": float(value("calibration.fy").value),
            "cx": float(value("calibration.cx").value),
            "cy": float(value("calibration.cy").value),
            "distortion": list(value("calibration.distortion").value),
            "lidar_to_camera": list(value("calibration.lidar_to_camera").value),
            "lidar_to_base": list(value("calibration.lidar_to_base").value),
        }
        path = str(value("calibration.device_parameters_file").value or "")
        if not path:
            root = os.environ.get("DOG_PATROL_NAV_ROOT", "")
            path = os.path.join(root, "config", "device_parameters.yaml") if root else ""
        if not path or not os.path.isfile(path):
            return result
        try:
            with open(path, "r", encoding="utf-8") as stream:
                data = yaml.safe_load(stream) or {}
            params = data.get("/**", {}).get("ros__parameters", {})
            camera = params.get("camera", {})
            extrinsic = params.get("extrin_calib", {})
            source_width = max(1, int(camera.get("width", output_width)))
            source_height = max(1, int(camera.get("height", output_height)))
            scale_x = output_width / float(source_width)
            scale_y = output_height / float(source_height)
            lidar_to_camera = device_lidar_to_camera(
                extrinsic.get("T_camera_lidar", [])
            )
            lidar_to_base = device_lidar_to_base(
                params.get("T_lidar_base", [])
            )
            result.update({
                "fx": float(camera.get("fx", result["fx"])) * scale_x,
                "fy": float(camera.get("fy", result["fy"])) * scale_y,
                "cx": float(camera.get("cx", result["cx"])) * scale_x,
                "cy": float(camera.get("cy", result["cy"])) * scale_y,
                "distortion": [
                    float(camera.get("d0", result["distortion"][0])),
                    float(camera.get("d1", result["distortion"][1])),
                    float(camera.get("d2", result["distortion"][2])),
                    float(camera.get("d3", result["distortion"][3])),
                ],
                # Despite its historical name, FAST-LIVO passes this matrix
                # directly to setLidarToCameraExtrinsic(). Do not invert it.
                "lidar_to_camera": lidar_to_camera.reshape(-1).tolist(),
                # The device matrix is composed as T_body_base by the original
                # odom bridge; invert it for livox_frame point -> base_link.
                "lidar_to_base": lidar_to_base.reshape(-1).tolist(),
            })
            result["width"] = output_width
            result["height"] = output_height
            self.get_logger().info(
                f"loaded navigation calibration from {path} ({output_width}x{output_height})"
            )
        except (OSError, TypeError, ValueError, yaml.YAMLError) as exc:
            self.get_logger().warning(f"failed to load calibration file {path}: {exc}")
        return result

    def _on_mission_state(self, msg: MissionState) -> None:
        if self._mission_state is not None and msg.state_seq < self._mission_state.state_seq:
            self.get_logger().warn(f"ignoring stale mission state seq={msg.state_seq}")
            return
        if (
            self._mission_state is not None
            and msg.state_seq == self._mission_state.state_seq
            and msg.state == self._mission_state.state
            and msg.target_id == self._mission_state.target_id
        ):
            self._mission_state = msg
            return

        previous_target = self._active_target_id
        previous_mission_state = (
            self._mission_state.state if self._mission_state is not None else None
        )
        entering_approach = (
            int(msg.state) == MissionState.APPROACH_TARGET
            and previous_mission_state != MissionState.APPROACH_TARGET
        )
        entering_recovery = (
            int(msg.state) == MissionState.RECOVER_PATROL
            and previous_mission_state != MissionState.RECOVER_PATROL
        )
        if entering_approach:
            self._capture_patrol_interruption_pose()
        self._mission_state = msg
        self._active_target_id = int(msg.target_id)
        self._arrival_reported = False
        self._arrival_hold_start = None
        if entering_recovery:
            self._recovery_return_path_requested = False
            self._recovery_return_path_ready = False
            self._recovery_return_arrived = False
            self._recovery_hold_start = None
            self._patrol_resume_sent_during_recovery = False
        self._technical_fault_start = None
        self._planner_failure_start = None
        policy = navigation_policy(
            msg.state,
            previous_mission_state=previous_mission_state,
        )
        if (
            policy.reset_target
            or previous_target != self._active_target_id
        ):
            self._reset_target()
        if policy.pause_patrol:
            self._pause_pub.publish(Empty())
        if policy.clear_navigation_path:
            self._publish_empty_path()
        returning_from_recovery = (
            int(msg.state) == MissionState.PATROL
            and previous_mission_state == MissionState.RECOVER_PATROL
        )
        if policy.resume_patrol:
            if returning_from_recovery:
                if not self._patrol_resume_sent_during_recovery:
                    # The supervisor left recovery by timeout. The robot may
                    # still be far from the interruption pose, so a cached
                    # pre-interruption path must not pull it backwards.
                    self._resume_from_current_pub.publish(Empty())
                    self.get_logger().warning(
                        "patrol recovery timed out; discarded interruption path "
                        "and requested a fresh patrol path from current pose"
                    )
            else:
                self._resume_pub.publish(Empty())
        if returning_from_recovery:
            self._patrol_interruption_pose = None
            self._patrol_resume_sent_during_recovery = False
        self.get_logger().info(
            f"mission seq={msg.state_seq} state={msg.state} target={msg.target_id} "
            f"-> navigation={policy.description}"
        )

    def _current_policy(self):
        if self._mission_state is None:
            return navigation_policy(-1)
        return navigation_policy(
            self._mission_state.state,
        )

    def _on_bbox(self, msg: TargetBoundingBox) -> None:
        if not self._accepts_fusion() or int(msg.target_id) != self._active_target_id:
            return
        self._last_bbox_received_time = now_time(self)
        if normalize_frame(msg.header.frame_id) != self.camera_frame:
            self._record_fusion_error(
                f"bbox frame '{msg.header.frame_id}' does not match '{self.camera_frame}'"
            )
            return
        age = seconds_between(self.get_clock().now().to_msg(), msg.header.stamp)
        if age < -self.sync_tolerance or age > self.max_bbox_age:
            self._record_fusion_error(
                f"bbox timestamp age {age:.3f}s outside "
                f"[-{self.sync_tolerance:.3f}, {self.max_bbox_age:.3f}]s"
            )
            return
        self._latest_bbox = msg
        self._last_bbox_time = now_time(self)
        self._try_fuse(msg)

    def _on_cloud(self, msg: PointCloud2) -> None:
        if normalize_frame(msg.header.frame_id) != self.lidar_frame:
            self.get_logger().warn(
                f"lidar frame '{msg.header.frame_id}' does not match '{self.lidar_frame}'"
            )
            return
        self._last_cloud_time = now_time(self)
        self._clouds.append(msg)
        if self._latest_bbox is not None and self._accepts_fusion():
            self._try_fuse(self._latest_bbox)

    def _on_odom(self, msg: Odometry) -> None:
        self._latest_linear_speed = hypot(
            float(msg.twist.twist.linear.x), float(msg.twist.twist.linear.y)
        )
        self._latest_angular_speed = abs(float(msg.twist.twist.angular.z))
        self._last_odom_time = now_time(self)

    def _try_fuse(self, bbox_msg: TargetBoundingBox) -> None:
        if not self._clouds:
            self._record_fusion_error("waiting for buffered /livox/lidar cloud")
            return
        bbox_age = seconds_between(self.get_clock().now().to_msg(), bbox_msg.header.stamp)
        if bbox_age < -self.sync_tolerance or bbox_age > self.max_bbox_age:
            self._record_fusion_error(
                f"bbox became stale before fusion (age={bbox_age:.3f}s)"
            )
            return
        matched = min(
            self._clouds,
            key=lambda cloud: abs(seconds_between(cloud.header.stamp, bbox_msg.header.stamp)),
        )
        offset = abs(seconds_between(matched.header.stamp, bbox_msg.header.stamp))
        if offset > self.sync_tolerance:
            self._record_fusion_error(
                f"camera-lidar timestamp offset {offset:.3f}s exceeds "
                f"{self.sync_tolerance:.3f}s"
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
                    points = point_cloud_xyz(matched)
                    self._prepared_cloud = self._estimator.prepare(points)
                    self._prepared_cloud_stamp = cloud_stamp
                if self._prepared_cloud is None:
                    self._record_fusion_error("prepared lidar cloud is unavailable")
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
                self._record_fusion_error(str(exc), recoverable=True)
                return
            except (AssertionError, ValueError, RuntimeError) as exc:
                self._last_estimate_pair = pair
                self._last_estimate_point_lidar = None
                self._last_estimate_range = None
                self._record_fusion_error(str(exc))
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
            point_base_msg.header.frame_id = self.base_frame
            point_base_msg.point.x = float(point_base[0])
            point_base_msg.point.y = float(point_base[1])
            point_base_msg.point.z = float(point_base[2])
            transform = self._lookup_fusion_transform(matched.header.stamp)
            point_map = self._transform_point(point_base_msg, transform)
        except (ValueError, RuntimeError, TransformException) as exc:
            self._record_fusion_error(str(exc))
            return

        sample = np.asarray(
            [point_map.point.x, point_map.point.y, point_map.point.z], dtype=np.float64
        )
        if not self._target_filter.add(sample):
            update_age = (
                float("inf")
                if self._last_target_update is None
                else (now_time(self) - self._last_target_update).nanoseconds * 1e-9
            )
            if update_age <= self.target_timeout:
                self._record_fusion_error(
                    "target map position rejected as jump outlier"
                )
                return

            # A moving person may cross the fixed jump gate after a short
            # detection gap. Re-seed, then require stable samples again before
            # allowing motion instead of remaining locked to the old position.
            self._target_filter.reset()
            self._target_motion_ready = False
            if not self._target_filter.add(sample):
                self._record_fusion_error(
                    "target map position could not be re-seeded"
                )
                return
        self._last_fused_cloud_stamp = cloud_stamp
        self._last_target_update = now_time(self)
        self._last_target_sensor_distance = hypot(
            float(point_base[0]), float(point_base[1])
        )
        self._last_target_sensor_time = self._last_target_update
        if self._target_filter.stable:
            self._target_motion_ready = True
            self._last_fusion_error = ""
            self._technical_fault_start = None
            self._technical_fault_reason = ""
        else:
            self._last_fusion_error = (
                f"stabilizing fused target samples "
                f"{self._target_filter.sample_count}/{self.stable_samples}"
            )
        point_map.header.stamp = now_time(self).to_msg()
        point_map.header.frame_id = self.global_frame
        self._target_point_pub.publish(point_map)
        measurement_log_age = (
            float("inf")
            if self._last_measurement_log is None
            else (now_time(self) - self._last_measurement_log).nanoseconds * 1e-9
        )
        if measurement_log_age >= 1.0 and self._last_estimate_range is not None:
            self.get_logger().info(
                f"target measurement id={self._active_target_id} "
                f"range={self._last_estimate_range:.2f}m "
                f"base_planar={self._last_target_sensor_distance:.2f}m "
                f"base_z={float(point_base[2]):.2f}m "
                f"roi_points={self._last_estimate_roi_points} "
                f"cluster_points={self._last_estimate_cluster_points} "
                f"stable={self._target_filter.stable}"
            )
            self._last_measurement_log = now_time(self)
        if (
            self._mission_state is not None
            and self._mission_state.state == MissionState.CONFIRM_TARGET
            and self._target_filter.stable
        ):
            self._publish_event(
                MissionEvent.TARGET_POSITION_READY,
                "time-synchronized lidar-camera target position stable",
            )

    def _lookup_fusion_transform(self, stamp: RosTime):
        """Get the target transform without failing on a short TF publication lag.

        The lidar point is expressed in the robot frame, so using the latest
        available map transform is preferable to dropping the measurement when
        the TF buffer has not reached the sensor timestamp yet. Exact-time
        lookup remains the normal path; the fallback is limited to future
        extrapolation errors and still propagates missing/invalid TF errors.
        """
        try:
            return self._tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                time_from_msg(self, stamp),
                timeout=Duration(seconds=self.tf_timeout),
            )
        except TransformException as exc:
            if "future" not in str(exc).lower():
                raise

            latest = self._tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                Time(seconds=0, clock_type=self.get_clock().clock_type),
                timeout=Duration(seconds=self.tf_timeout),
            )
            now = now_time(self)
            warning_age = (
                float("inf")
                if self._last_tf_fallback_warning is None
                else (now - self._last_tf_fallback_warning).nanoseconds * 1e-9
            )
            if warning_age >= 2.0:
                lag = seconds_between(stamp, latest.header.stamp)
                self.get_logger().warning(
                    "target fusion used latest map -> base TF because the "
                    f"sensor timestamp is ahead of the TF buffer by {lag:.3f}s"
                )
                self._last_tf_fallback_warning = now
            return latest

    def _transform_point(self, point: PointStamped, transform) -> PointStamped:
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

    def _robot_pose(self) -> Optional[PoseStamped]:
        try:
            transform = self._tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_frame,
                Time(clock_type=self.get_clock().clock_type),
                timeout=Duration(seconds=self.tf_timeout),
            )
        except TransformException:
            return None
        pose = PoseStamped()
        pose.header.frame_id = self.global_frame
        pose.header.stamp = now_time(self).to_msg()
        pose.pose.position.x = float(transform.transform.translation.x)
        pose.pose.position.y = float(transform.transform.translation.y)
        pose.pose.position.z = float(transform.transform.translation.z)
        pose.pose.orientation = transform.transform.rotation
        return pose

    def _capture_patrol_interruption_pose(self) -> None:
        """Save the patrol pose before target approach pauses the waypoint task."""
        pose = self._robot_pose()
        if pose is None:
            self._patrol_interruption_pose = None
            self.get_logger().warning(
                "cannot save patrol interruption pose: map -> base_footprint TF unavailable"
            )
            return
        self._patrol_interruption_pose = pose
        self.get_logger().info(
            f"saved patrol interruption pose: "
            f"x={pose.pose.position.x:.2f} y={pose.pose.position.y:.2f}"
        )

    def _tick(self) -> None:
        if self._mission_state is None:
            return
        if self._mission_state.state == MissionState.STARTUP:
            self._tick_startup()
            return
        policy = self._current_policy()
        if policy.safe_stop or self._mission_state.state == MissionState.VERIFY_IDENTITY:
            self._republish_stop_path()
        if self._mission_state.state == MissionState.RECOVER_PATROL:
            self._tick_recovery()
            return
        if self._mission_state.state in {
            MissionState.CONFIRM_TARGET,
            MissionState.APPROACH_TARGET,
            MissionState.TRACK_INTRUDER,
        }:
            if self._mission_state.state != MissionState.CONFIRM_TARGET:
                self._tick_target_motion()
            self._check_fusion_health()

    def _tick_recovery(self) -> None:
        """Return to the patrol interruption pose before resuming waypoints."""
        if self._recovery_return_arrived:
            return
        pose = self._patrol_interruption_pose
        if pose is None:
            self.get_logger().warning(
                "patrol interruption pose unavailable; resuming waypoint patrol directly"
            )
            self._finish_recovery_return()
            return

        current = self._robot_pose()
        if current is None:
            self._republish_stop_path()
            return
        position_error = hypot(
            current.pose.position.x - pose.pose.position.x,
            current.pose.position.y - pose.pose.position.y,
        )
        stopped = (
            self._latest_linear_speed <= self.arrival_linear_speed
            and self._latest_angular_speed <= self.arrival_angular_speed
            and self._last_odom_time is not None
            and (now_time(self) - self._last_odom_time).nanoseconds * 1.0e-9 <= 0.5
        )
        within_tolerance = (
            position_error <= self.recovery_position_tolerance
        )
        if within_tolerance and stopped:
            now = now_time(self)
            if self._recovery_hold_start is None:
                self._recovery_hold_start = now
            elif (
                (now - self._recovery_hold_start).nanoseconds * 1.0e-9
                >= self.arrival_hold_time
            ):
                self._finish_recovery_return()
            return

        self._recovery_hold_start = None
        if (
            not self._recovery_return_path_ready
            and not self._recovery_return_path_requested
            and not self._plan_in_flight
        ):
            self._request_recovery_path(pose)

    def _finish_recovery_return(self) -> None:
        if self._recovery_return_arrived:
            return
        self._publish_empty_path()
        self._recovery_return_arrived = True
        self._recovery_return_path_ready = False
        self._recovery_return_path_requested = False
        self._resume_pub.publish(Empty())
        self._patrol_resume_sent_during_recovery = True
        self._publish_event(
            MissionEvent.PATROL_RECOVERY_COMPLETE,
            "patrol interruption pose reached; waypoint resume requested",
        )
        self.get_logger().info(
            "patrol interruption pose reached; resumed waypoint patrol"
        )

    def _tick_startup(self) -> None:
        reason = self._navigation_ready_reason()
        if reason:
            self._last_startup_reason = reason
            self._republish_stop_path()
            return
        self._last_startup_reason = ""
        now = now_time(self)
        ready_age = (
            0.0 if self._last_ready_event is None
            else (now - self._last_ready_event).nanoseconds * 1e-9
        )
        if self._last_ready_event and ready_age < self.ready_event_period:
            return
        self._publish_event(
            MissionEvent.READY,
            "current M20 localization, lidar, planner and DWB control chain ready",
            target_id=0,
            deduplicate=False,
        )
        self._last_ready_event = now

    def _navigation_ready_reason(self) -> str:
        now = now_time(self)
        cloud_age = (
            float("inf") if self._last_cloud_time is None
            else (now - self._last_cloud_time).nanoseconds * 1e-9
        )
        if cloud_age > self.ready_lidar_timeout:
            return "waiting for fresh /livox/lidar PointCloud2"
        try:
            self._tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_frame,
                Time(seconds=0, clock_type=self.get_clock().clock_type),
                timeout=Duration(seconds=self.tf_timeout),
            )
        except TransformException:
            return "waiting for map -> base_footprint TF"
        if self.base_frame != self.robot_frame:
            try:
                self._tf_buffer.lookup_transform(
                    self.global_frame,
                    self.base_frame,
                    Time(seconds=0, clock_type=self.get_clock().clock_type),
                    timeout=Duration(seconds=self.tf_timeout),
                )
            except TransformException:
                return "waiting for map -> base_link TF used by target fusion"
        if not self._planner_client.server_is_ready():
            return "waiting for ComputePathToPose action"
        if self.strict_ready_checks:
            if self.count_subscribers(self.global_path_topic) == 0:
                return "waiting for /global_path consumer"
            if self.count_subscribers(self.pause_topic) == 0:
                return "waiting for waypoint pause subscriber"
            if self.count_subscribers(self.resume_topic) == 0:
                return "waiting for waypoint resume subscriber"
            if self.count_publishers(self.nav_cmd_topic) == 0:
                return "waiting for /NAV_CMD publisher"
        return ""

    def _tick_target_motion(self) -> None:
        if (
            self._mission_state is not None
            and self._mission_state.state == MissionState.APPROACH_TARGET
            and self._arrival_reported
        ):
            self._republish_stop_path()
            return
        target = self._fresh_target()
        if target is None:
            self._republish_stop_path()
            return
        robot = self._robot_position()
        if robot is None:
            self._record_technical_fault("map -> base_footprint TF unavailable")
            self._republish_stop_path()
            return
        desired = (
            self.tracking_distance
            if self._mission_state.state == MissionState.TRACK_INTRUDER
            else self.approach_distance
        )
        measured_distance = self._fresh_target_sensor_distance()
        decision = compute_standoff_decision(
            robot[0], robot[1], target[0], target[1],
            desired, self.arrival_tolerance,
            planning_goal_distance=self.planning_goal_distance,
            measured_distance=measured_distance,
        )
        motion_log_age = (
            float("inf")
            if self._last_motion_log is None
            else (now_time(self) - self._last_motion_log).nanoseconds * 1.0e-9
        )
        if motion_log_age >= 1.0:
            self.get_logger().info(
                f"target standoff state={self._mission_state.state} "
                f"sensor_distance={decision.distance:.2f}m "
                f"map_distance={decision.map_distance:.2f}m "
                f"desired={desired:.2f}m stop={decision.should_stop}"
            )
            self._last_motion_log = now_time(self)
        if decision.should_stop:
            self._republish_stop_path()
            if self._mission_state.state == MissionState.APPROACH_TARGET:
                self._tick_arrival(decision.distance)
            return
        self._arrival_hold_start = None

        goal = np.asarray([decision.goal_x, decision.goal_y, 0.0])
        now = now_time(self)
        period = (
            self.tracking_replan_period
            if self._mission_state.state == MissionState.TRACK_INTRUDER
            else self.approach_replan_period
        )
        elapsed = (
            self._last_plan_request is None
            or (now - self._last_plan_request).nanoseconds * 1e-9 >= period
        )
        changed = (
            self._last_planned_target is None
            or np.linalg.norm(target[:2] - self._last_planned_target[:2])
            >= self.target_replan_distance
        )
        if not self._plan_in_flight and (elapsed or changed):
            self._request_path(goal, decision.goal_yaw, target)

    def _tick_arrival(self, distance: float) -> None:
        now = now_time(self)
        stopped = (
            self._latest_linear_speed <= self.arrival_linear_speed
            and self._latest_angular_speed <= self.arrival_angular_speed
            and self._last_odom_time is not None
            and (now - self._last_odom_time).nanoseconds * 1e-9 <= 0.5
        )
        if not stopped or distance > self.approach_distance + self.arrival_tolerance:
            self._arrival_hold_start = None
            return
        if self._arrival_hold_start is None:
            self._arrival_hold_start = now
            return
        hold_age = (now - self._arrival_hold_start).nanoseconds * 1e-9
        if hold_age >= self.arrival_hold_time and not self._arrival_reported:
            self._publish_event(
                MissionEvent.ARRIVED_AND_STOPPED,
                "target standoff reached and robot velocity held below thresholds",
            )
            self._arrival_reported = True

    def _request_path(self, goal: np.ndarray, yaw: float, target: np.ndarray) -> None:
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.global_frame
        goal_pose.pose.position.x = float(goal[0])
        goal_pose.pose.position.y = float(goal[1])
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.z = float(np.sin(yaw * 0.5))
        goal_pose.pose.orientation.w = float(np.cos(yaw * 0.5))
        self._request_planner_path(goal_pose, target, "target")

    def _request_recovery_path(self, goal_pose: PoseStamped) -> None:
        target = np.asarray(
            [goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z],
            dtype=np.float64,
        )
        self._recovery_return_path_requested = True
        self._request_planner_path(goal_pose, target, "recovery")

    def _request_planner_path(
        self, goal_pose: PoseStamped, planned_target: np.ndarray, plan_kind: str
    ) -> None:
        if not self._planner_client.server_is_ready():
            self._record_planner_failure("ComputePathToPose action unavailable")
            if plan_kind == "recovery":
                self._recovery_return_path_requested = False
            return
        request = ComputePathToPose.Goal()
        request.goal = PoseStamped()
        request.goal.header.frame_id = self.global_frame
        request.goal.header.stamp = now_time(self).to_msg()
        request.goal.pose.position.x = float(goal_pose.pose.position.x)
        request.goal.pose.position.y = float(goal_pose.pose.position.y)
        request.goal.pose.position.z = float(goal_pose.pose.position.z)
        request.goal.pose.orientation = goal_pose.pose.orientation
        request.use_start = False
        request.planner_id = self.planner_id
        if plan_kind == "target":
            self._target_goal_pub.publish(request.goal)

        self._plan_in_flight = True
        self._last_plan_request = now_time(self)
        self._last_planned_target = planned_target.copy()
        self._plan_generation += 1
        generation = self._plan_generation
        state_seq = int(self._mission_state.state_seq)
        planned_target = planned_target.copy()
        try:
            response_future = self._planner_client.send_goal_async(request)
        except Exception as exc:
            self._plan_in_flight = False
            if plan_kind == "recovery":
                self._recovery_return_path_requested = False
            self._record_planner_failure(f"planner goal send failed: {exc}")
            return
        response_future.add_done_callback(
            lambda future: self._on_goal_response(
                future, generation, state_seq, planned_target, plan_kind
            )
        )

    def _on_goal_response(
        self,
        future,
        generation: int,
        state_seq: int,
        planned_target: np.ndarray,
        plan_kind: str,
    ) -> None:
        try:
            handle = future.result()
        except Exception as exc:
            if generation == self._plan_generation:
                self._plan_in_flight = False
                if plan_kind == "recovery":
                    self._recovery_return_path_requested = False
                self._record_planner_failure(f"planner goal send failed: {exc}")
            return

        if generation != self._plan_generation:
            if handle is not None and handle.accepted:
                self._cancel_planner_goal_handle(
                    handle,
                    f"stale {plan_kind} planner goal generation {generation}",
                )
            return
        if handle is None or not handle.accepted:
            self._plan_in_flight = False
            if plan_kind == "recovery":
                self._recovery_return_path_requested = False
            self._record_planner_failure("ComputePathToPose goal rejected")
            return
        self._active_planner_goal_handle = handle
        self._active_planner_goal_generation = generation
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda result: self._on_plan_result(
                result, generation, state_seq, planned_target, plan_kind
            )
        )

    def _on_plan_result(
        self,
        future,
        generation: int,
        state_seq: int,
        planned_target: np.ndarray,
        plan_kind: str,
    ) -> None:
        if self._active_planner_goal_generation == generation:
            self._active_planner_goal_handle = None
            self._active_planner_goal_generation = -1
        if generation != self._plan_generation:
            return
        self._plan_in_flight = False
        if plan_kind == "recovery":
            self._recovery_return_path_requested = False
        if (
            self._mission_state is None
            or int(self._mission_state.state_seq) != state_seq
            or self._mission_state.state not in {
                MissionState.APPROACH_TARGET,
                MissionState.TRACK_INTRUDER,
                MissionState.RECOVER_PATROL,
            }
            or (
                plan_kind == "recovery"
                and self._mission_state.state != MissionState.RECOVER_PATROL
            )
            or (
                plan_kind == "target"
                and self._mission_state.state
                not in {MissionState.APPROACH_TARGET, MissionState.TRACK_INTRUDER}
            )
        ):
            return
        try:
            wrapped = future.result()
            if wrapped is None or wrapped.result is None or not wrapped.result.path.poses:
                raise RuntimeError("ComputePathToPose returned no usable path")
            path = wrapped.result.path
        except Exception as exc:
            self._record_planner_failure(str(exc))
            return
        if plan_kind == "target":
            current_target = self._fresh_target()
            if (
                current_target is None
                or np.linalg.norm(current_target[:2] - planned_target[:2])
                >= self.target_replan_distance
            ):
                # Do not publish a path to a target position that became stale
                # while Nav2 was computing it. The next tick requests a fresh one.
                self._last_plan_request = None
                return
        else:
            self._recovery_return_path_ready = True
        path.header.frame_id = self.global_frame
        path.header.stamp = now_time(self).to_msg()
        self._path_pub.publish(path)
        self._planner_failure_start = None
        self._planner_failure_reason = ""

    def _robot_position(self) -> Optional[np.ndarray]:
        pose = self._robot_pose()
        if pose is None:
            return None
        return np.asarray(
            [
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
            ],
            dtype=np.float64,
        )

    def _fresh_target(self) -> Optional[np.ndarray]:
        now = now_time(self)
        if self._last_bbox_time is None or self._last_target_update is None:
            return None
        if (now - self._last_bbox_time).nanoseconds * 1e-9 > self.target_timeout:
            return None
        if (now - self._last_target_update).nanoseconds * 1e-9 > self.target_timeout:
            return None
        if not self._target_motion_ready:
            return None
        position = self._target_filter.position
        return None if position is None else np.asarray(position, dtype=np.float64)

    def _fresh_target_sensor_distance(self) -> Optional[float]:
        if (
            self._last_target_sensor_distance is None
            or self._last_target_sensor_time is None
        ):
            return None
        age = (now_time(self) - self._last_target_sensor_time).nanoseconds * 1.0e-9
        if age > self.target_timeout:
            return None
        return float(self._last_target_sensor_distance)

    def _check_fusion_health(self) -> None:
        if self._last_bbox_received_time is None:
            return
        now = now_time(self)
        receive_silence = (now - self._last_bbox_received_time).nanoseconds * 1e-9
        if receive_silence > self.target_timeout:
            return
        if self._last_bbox_time is None:
            self._record_technical_fault(
                self._last_fusion_error or "received bbox is not valid for fusion"
            )
            return
        bbox_silence = (now - self._last_bbox_time).nanoseconds * 1e-9
        if bbox_silence > self.target_timeout:
            return
        fusion_age = (
            float("inf") if self._last_target_update is None
            else (now - self._last_target_update).nanoseconds * 1e-9
        )
        if fusion_age > self.failure_timeout:
            if self._last_fusion_error_recoverable:
                return
            self._record_technical_fault(
                self._last_fusion_error or "fresh bbox cannot be fused with lidar"
            )

    def _record_technical_fault(self, reason: str) -> None:
        now = now_time(self)
        if self._technical_fault_start is None:
            self._technical_fault_start = now
            self._technical_fault_reason = reason
            return
        if (now - self._technical_fault_start).nanoseconds * 1e-9 >= self.technical_error_timeout:
            self._publish_event(
                MissionEvent.EXECUTION_ERROR,
                f"navigation technical fault: {self._technical_fault_reason}",
            )

    def _record_planner_failure(self, reason: str) -> None:
        now = now_time(self)
        if self._planner_failure_start is None:
            self._planner_failure_start = now
            self._planner_failure_reason = reason
            return
        if (now - self._planner_failure_start).nanoseconds * 1e-9 >= self.technical_error_timeout:
            self._publish_event(
                MissionEvent.EXECUTION_ERROR,
                f"navigation planner fault: {self._planner_failure_reason}",
            )

    def _publish_event(
        self,
        event: int,
        detail: str,
        target_id: Optional[int] = None,
        deduplicate: bool = True,
    ) -> None:
        if self._mission_state is None:
            return
        key = (int(self._mission_state.state_seq), int(event))
        if deduplicate and key in self._sent_events:
            return
        message = MissionEvent()
        message.header.stamp = now_time(self).to_msg()
        message.observed_state_seq = int(self._mission_state.state_seq)
        message.target_id = (
            0 if event == MissionEvent.READY
            else self._active_target_id if target_id is None else int(target_id)
        )
        message.source = MissionEvent.SOURCE_NAVIGATION
        message.event = int(event)
        message.detail = str(detail)[:256]
        self._event_pub.publish(message)
        if deduplicate:
            self._sent_events[key] = True

    def _publish_status(self) -> None:
        """Publish navigation observability without changing mission state."""
        message = TargetNavigationStatus()
        message.header.stamp = now_time(self).to_msg()
        message.header.frame_id = self.global_frame
        message.target_id = int(self._active_target_id)
        message.distance_valid = False

        target = self._fresh_target()
        robot = self._robot_position()
        if target is not None and robot is not None:
            message.distance_to_target = float(np.linalg.norm(target[:2] - robot[:2]))
            message.distance_valid = True

        if self._mission_state is None:
            message.status = TargetNavigationStatus.WAITING_TARGET
            message.detail = "waiting for mission state"
        elif self._mission_state.state in {
            MissionState.STARTUP,
            MissionState.PATROL,
        }:
            message.status = TargetNavigationStatus.WAITING_TARGET
            message.detail = "patrol is controlled by the waypoint navigation chain"
        elif self._mission_state.state == MissionState.CONFIRM_TARGET:
            message.status = TargetNavigationStatus.WAITING_TARGET
            message.detail = self._last_fusion_error or "waiting for fused target position"
        elif self._mission_state.state == MissionState.APPROACH_TARGET:
            if self._arrival_reported:
                message.status = TargetNavigationStatus.ARRIVED
                message.detail = "3 m standoff reached and stopped"
            elif target is None:
                message.status = TargetNavigationStatus.HOLDING
                message.detail = "holding while waiting for fresh fused target"
            else:
                message.status = TargetNavigationStatus.APPROACHING
                message.detail = "approaching target standoff"
        elif self._mission_state.state == MissionState.VERIFY_IDENTITY:
            message.status = TargetNavigationStatus.HOLDING
            message.detail = "holding position for identity verification"
        elif self._mission_state.state == MissionState.TRACK_INTRUDER:
            if target is None:
                message.status = TargetNavigationStatus.HOLDING
                message.detail = "holding while target data is stale"
            else:
                message.status = TargetNavigationStatus.TRACKING
                message.detail = "tracking target with safe standoff"
        elif self._mission_state.state == MissionState.RECOVER_PATROL:
            message.status = TargetNavigationStatus.HOLDING
            message.detail = "returning to patrol interruption pose"
        else:
            message.status = TargetNavigationStatus.WAITING_TARGET
            message.detail = "unsupported mission state"

        self._status_pub.publish(message)

    def _publish_empty_path(self) -> None:
        empty = Path()
        empty.header.frame_id = self.global_frame
        empty.header.stamp = now_time(self).to_msg()
        self._path_pub.publish(empty)
        self._request_controller_cancel()
        self._plan_generation += 1
        self._plan_in_flight = False
        self._cancel_active_planner_goal("navigation path cleared")

    def _cancel_planner_goal_handle(self, goal_handle, reason: str) -> None:
        try:
            future = goal_handle.cancel_goal_async()
            future.add_done_callback(
                lambda done_future, cancel_reason=reason: self._on_planner_cancel_response(
                    done_future,
                    cancel_reason,
                )
            )
            self.get_logger().debug(f"Cancel target planner goal: {reason}")
        except Exception as exc:
            self.get_logger().warning(
                f"ComputePathToPose cancel failed ({reason}): {exc}"
            )

    def _cancel_active_planner_goal(self, reason: str) -> None:
        goal_handle = self._active_planner_goal_handle
        self._active_planner_goal_handle = None
        self._active_planner_goal_generation = -1
        if goal_handle is not None:
            self._cancel_planner_goal_handle(goal_handle, reason)

    def _on_planner_cancel_response(self, future, reason: str) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warning(
                f"ComputePathToPose cancel response failed ({reason}): {exc}"
            )
            return
        if response is not None and not getattr(response, "goals_canceling", []):
            self.get_logger().debug(
                f"ComputePathToPose cancel had no active goal ({reason})"
            )

    def _request_controller_cancel(self) -> None:
        """Cancel all active FollowPath goals without publishing a competing command."""
        if self._controller_cancel_pending:
            return
        if not self._controller_cancel_client.service_is_ready():
            now = now_time(self)
            warning_age = (
                float("inf")
                if self._last_controller_cancel_warning is None
                else (now - self._last_controller_cancel_warning).nanoseconds * 1.0e-9
            )
            if warning_age >= 1.0:
                self.get_logger().warning(
                    f"FollowPath cancel service for '{self.follow_path_action}' is unavailable"
                )
                self._last_controller_cancel_warning = now
            return

        # An all-zero GoalInfo is the ROS 2 action protocol request to cancel
        # every currently accepted goal. The adapter remains the sole NAV_CMD
        # publisher; controller_server emits its normal zero Twist on cancel.
        self._controller_cancel_pending = True
        future = self._controller_cancel_client.call_async(CancelGoal.Request())
        future.add_done_callback(self._on_controller_cancel_response)

    def _on_controller_cancel_response(self, future) -> None:
        self._controller_cancel_pending = False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warning(f"FollowPath cancel request failed: {exc}")
            return
        if response is not None and response.return_code == CancelGoal.Response.ERROR_REJECTED:
            self.get_logger().warning("FollowPath cancel request was rejected")

    def _republish_stop_path(self) -> None:
        now = now_time(self)
        stop_age = (
            float("inf") if self._last_stop_publish is None
            else (now - self._last_stop_publish).nanoseconds * 1e-9
        )
        if stop_age >= self.stop_republish_period:
            self._publish_empty_path()
            self._last_stop_publish = now

    def _reset_target(self) -> None:
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
        self._last_planned_target = None
        self._last_plan_request = None
        self._arrival_hold_start = None
        self._arrival_reported = False
        self._plan_generation += 1
        self._plan_in_flight = False
        self._cancel_active_planner_goal("target state reset")
        self._last_fusion_error = ""
        self._last_fusion_error_recoverable = False
        self._last_fusion_warning = None
        self._last_measurement_log = None
        self._last_motion_log = None

    def _record_fusion_error(self, reason: str, recoverable: bool = False) -> None:
        reason = str(reason).strip() or "unknown target fusion error"
        changed = reason != self._last_fusion_error
        self._last_fusion_error = reason
        self._last_fusion_error_recoverable = bool(recoverable)
        now = now_time(self)
        warning_age = (
            float("inf")
            if self._last_fusion_warning is None
            else (now - self._last_fusion_warning).nanoseconds * 1e-9
        )
        if changed and warning_age >= 2.0:
            self.get_logger().warning(f"target fusion waiting: {reason}")
            self._last_fusion_warning = now

    def _accepts_fusion(self) -> bool:
        return (
            self._mission_state is not None
            and self._active_target_id > 0
            and self._mission_state.state
            in {
                MissionState.CONFIRM_TARGET,
                MissionState.APPROACH_TARGET,
                MissionState.VERIFY_IDENTITY,
                MissionState.TRACK_INTRUDER,
            }
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavigationMissionCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
