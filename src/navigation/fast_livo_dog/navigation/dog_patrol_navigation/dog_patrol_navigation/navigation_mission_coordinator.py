"""ROS boundary and mission orchestration for the dog patrol navigation stack."""

import copy
from math import hypot
import os
from typing import Dict, Optional

import numpy as np
import yaml
import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PointStamped, PoseStamped
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
from std_msgs.msg import Empty
from tf2_ros import Buffer, TransformException, TransformListener

from dog_patrol_interfaces.msg import (
    MissionEvent,
    MissionState,
    TargetBoundingBox,
    TargetNavigationStatus,
)

from .navigation_motion_controller import MotionConfig, TargetMotionController
from .navigation_planner_client import NavigationPlannerClient, PlannerRequest
from .navigation_policy import navigation_policy
from .navigation_target_fusion import (
    TargetFusionConfig,
    TargetFusionController,
    device_lidar_to_base,
    device_lidar_to_camera,
    normalize_frame,
    now_time,
)
from .patrol_recovery_controller import (
    PatrolRecoveryController,
    RecoveryConfig,
)
from .target_estimator import (
    CameraCalibration,
    EstimatorConfig,
    TargetEstimator,
    TargetFilterConfig,
    TargetPositionFilter,
)


class NavigationMissionCoordinator(Node):
    """Own ROS I/O and route mission-state changes to navigation components."""

    def __init__(self) -> None:
        super().__init__("navigation_mission_coordinator")
        self._declare_parameters()
        self._read_parameters()

        self._mission_state: Optional[MissionState] = None
        self._active_target_id = 0
        self._last_cloud_time = None
        self._last_odom_time = None
        self._last_stop_publish = None
        self._controller_cancel_pending = False
        self._last_controller_cancel_warning = None
        self._last_ready_event = None
        self._technical_fault_start = None
        self._planner_failure_start = None
        self._technical_fault_reason = ""
        self._planner_failure_reason = ""
        self._last_startup_reason = "waiting for mission state"
        self._latest_linear_speed = float("inf")
        self._latest_angular_speed = float("inf")
        self._last_motion_log = None
        self._sent_events: Dict[tuple[int, int], bool] = {}

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
        self._planner_action_client = ActionClient(
            self, ComputePathToPose, self.planner_action
        )
        cancel_service = f"{self.follow_path_action.rstrip('/')}/_action/cancel_goal"
        self._controller_cancel_client = self.create_client(CancelGoal, cancel_service)

        def now_seconds() -> float:
            return now_time(self).nanoseconds * 1.0e-9

        self._fusion = TargetFusionController(
            self,
            self._tf_buffer,
            self._estimator,
            self._target_filter,
            self._lidar_to_base,
            TargetFusionConfig(
                camera_frame=self.camera_frame,
                lidar_frame=self.lidar_frame,
                base_frame=self.base_frame,
                global_frame=self.global_frame,
                max_bbox_age=self.max_bbox_age,
                sync_tolerance=self.sync_tolerance,
                cloud_buffer_size=self.cloud_buffer_size,
                target_timeout=self.target_timeout,
                failure_timeout=self.failure_timeout,
                tf_timeout=self.tf_timeout,
                stable_samples=self.stable_samples,
            ),
            self._target_point_pub.publish,
            self._on_target_position_ready,
            self._on_fusion_recovered,
        )
        self._motion = TargetMotionController(
            MotionConfig(
                approach_distance=self.approach_distance,
                tracking_distance=self.tracking_distance,
                planning_goal_distance=self.planning_goal_distance,
                arrival_tolerance=self.arrival_tolerance,
                arrival_linear_speed=self.arrival_linear_speed,
                arrival_angular_speed=self.arrival_angular_speed,
                arrival_hold_time=self.arrival_hold_time,
                approach_replan_period=self.approach_replan_period,
                tracking_replan_period=self.tracking_replan_period,
                target_replan_distance=self.target_replan_distance,
            ),
            now_seconds,
        )
        self._recovery = PatrolRecoveryController(
            RecoveryConfig(
                position_tolerance=self.recovery_position_tolerance,
                linear_speed=self.arrival_linear_speed,
                angular_speed=self.arrival_angular_speed,
                odom_timeout=0.5,
                hold_time=self.arrival_hold_time,
                republish_period=self.stop_republish_period,
                request_retry_period=self.approach_replan_period,
            ),
            now_seconds,
        )
        self._planner_client = NavigationPlannerClient(
            self,
            self._planner_action_client,
            self.planner_id,
            self.global_frame,
            lambda: now_time(self),
            self._target_goal_pub.publish,
            self._on_planner_result,
            self._record_planner_failure,
            self._on_planner_dispatched,
        )
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
            "topics.global_path": "/mission_global_path",
            "topics.selected_global_path": "/global_path",
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
        self.selected_global_path_topic = value("topics.selected_global_path").value
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
        self.target_timeout = float(value("fusion.target_timeout").value)
        self.failure_timeout = float(value("fusion.failure_timeout").value)
        self.approach_distance = float(value("motion.approach_distance").value)
        self.tracking_distance = float(value("motion.tracking_distance").value)
        self.planning_goal_distance = float(value("motion.planning_goal_distance").value)
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
        """Use fast_livo_dog device_parameters.yaml as calibration source of truth."""
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
            lidar_to_base = device_lidar_to_base(params.get("T_lidar_base", []))
            result.update(
                {
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
                    "lidar_to_camera": lidar_to_camera.reshape(-1).tolist(),
                    "lidar_to_base": lidar_to_base.reshape(-1).tolist(),
                }
            )
            self.get_logger().info(
                f"loaded navigation calibration from {path} ({output_width}x{output_height})"
            )
        except (OSError, TypeError, ValueError, yaml.YAMLError) as exc:
            self.get_logger().warning(f"failed to load calibration file {path}: {exc}")
        return result

    def _on_mission_state(self, msg: MissionState) -> None:
        if self._mission_state is not None and msg.state_seq < self._mission_state.state_seq:
            self.get_logger().warning(f"ignoring stale mission state seq={msg.state_seq}")
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
        previous_state = self._mission_state.state if self._mission_state else None
        previous_seq = (
            int(self._mission_state.state_seq) if self._mission_state else None
        )
        leaving_recovery = (
            previous_state == MissionState.RECOVER_PATROL
            and msg.state != MissionState.RECOVER_PATROL
        )
        entering_approach = (
            msg.state == MissionState.APPROACH_TARGET
            and previous_state != MissionState.APPROACH_TARGET
        )
        starting_recovery_context = (
            msg.state == MissionState.RECOVER_PATROL
            and (
                previous_state != MissionState.RECOVER_PATROL
                or previous_seq != int(msg.state_seq)
                or previous_target != int(msg.target_id)
            )
        )
        if entering_approach:
            self._capture_patrol_interruption_pose()
        if starting_recovery_context:
            self._recovery.begin(self._recovery.interruption_pose)
        elif leaving_recovery:
            # A reset or any other state exit must not leave the recovery
            # request/path latch alive after its action has been invalidated.
            self._recovery.clear_path()
            if msg.state != MissionState.PATROL:
                self._recovery.interruption_pose = None
                self._recovery.return_arrived = False

        self._mission_state = msg
        self._active_target_id = int(msg.target_id)
        # Arrival confirmation belongs to one authoritative state version.
        # Planner throttling belongs to the active target and is only reset
        # when that target context is reset.
        self._motion.reset_arrival()
        self._technical_fault_start = None
        self._planner_failure_start = None
        policy = navigation_policy(msg.state, previous_mission_state=previous_state)
        if policy.reset_target or previous_target != self._active_target_id:
            self._reset_target()
        if policy.pause_patrol:
            self._pause_pub.publish(Empty())
        if policy.clear_navigation_path:
            self._publish_empty_path()

        returning_from_recovery = (
            msg.state == MissionState.PATROL
            and previous_state == MissionState.RECOVER_PATROL
        )
        if returning_from_recovery:
            self._recovery.clear_path()
            self._publish_empty_path()
        if policy.resume_patrol:
            if returning_from_recovery:
                if not self._recovery.resume_sent:
                    self._resume_from_current_pub.publish(Empty())
                    self.get_logger().warning(
                        "patrol recovery timed out; discarded interruption path "
                        "and requested a fresh patrol path from current pose"
                    )
            else:
                self._resume_pub.publish(Empty())
        if returning_from_recovery:
            self._recovery.interruption_pose = None
        self.get_logger().info(
            f"mission seq={msg.state_seq} state={msg.state} target={msg.target_id} "
            f"-> navigation={policy.description}"
        )

    def _on_bbox(self, msg: TargetBoundingBox) -> None:
        self._fusion.on_bbox(
            msg,
            self._active_target_id,
            self._accepts_fusion(),
            self._mission_state.state if self._mission_state else None,
        )

    def _on_cloud(self, msg: PointCloud2) -> None:
        self._fusion.on_cloud(
            msg,
            self._active_target_id,
            self._accepts_fusion(),
            self._mission_state.state if self._mission_state else None,
        )
        self._last_cloud_time = self._fusion.last_cloud_time

    def _on_odom(self, msg: Odometry) -> None:
        self._latest_linear_speed = hypot(
            float(msg.twist.twist.linear.x), float(msg.twist.twist.linear.y)
        )
        self._latest_angular_speed = abs(float(msg.twist.twist.angular.z))
        self._last_odom_time = now_time(self)

    def _on_target_position_ready(self) -> None:
        self._publish_event(
            MissionEvent.TARGET_POSITION_READY,
            "time-synchronized lidar-camera target position stable",
        )

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

    def _robot_position(self) -> Optional[np.ndarray]:
        pose = self._robot_pose()
        if pose is None:
            return None
        return np.asarray(
            [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z],
            dtype=np.float64,
        )

    def _capture_patrol_interruption_pose(self) -> None:
        pose = self._robot_pose()
        if pose is None:
            self._recovery.interruption_pose = None
            self.get_logger().warning(
                "cannot save patrol interruption pose: map -> base_footprint TF unavailable"
            )
            return
        self._recovery.interruption_pose = pose
        self.get_logger().info(
            f"saved patrol interruption pose: "
            f"x={pose.pose.position.x:.2f} y={pose.pose.position.y:.2f}"
        )

    def _tick(self) -> None:
        if self._mission_state is None:
            return
        state = self._mission_state.state
        if state == MissionState.STARTUP:
            self._tick_startup()
            return
        if state == MissionState.RECOVER_PATROL:
            self._tick_recovery()
            return
        policy = navigation_policy(state)
        if policy.safe_stop:
            self._republish_stop_path()
            return
        if state == MissionState.VERIFY_IDENTITY:
            self._republish_stop_path()
        if policy.allow_target_motion:
            self._tick_target_motion()
        # Preserve the original coordinator's health-check scope.  VERIFY_IDENTITY
        # still accepts bbox data for the authorization provider, but fusion
        # failures are not promoted to navigation errors in that holding state.
        if state in {
            MissionState.CONFIRM_TARGET,
            MissionState.APPROACH_TARGET,
            MissionState.TRACK_INTRUDER,
        }:
            self._check_fusion_health()

    def _tick_recovery(self) -> None:
        if self._recovery.return_arrived:
            return
        current_pose = (
            self._robot_pose()
            if self._recovery.interruption_pose is not None
            else None
        )
        odom_age = self._odom_age()
        command = self._recovery.tick(
            current_pose,
            self._latest_linear_speed,
            self._latest_angular_speed,
            odom_age,
            self._planner_client.in_flight,
        )
        if command.publish_path is not None:
            self._publish_recovery_path(command.publish_path)
        if command.should_stop:
            self._republish_stop_path()
        if command.complete and self._recovery.interruption_pose is None:
            self.get_logger().warning(
                "patrol interruption pose unavailable; resuming waypoint patrol directly"
            )
        if command.request_path and self._recovery.interruption_pose is not None:
            self._request_recovery_path(self._recovery.interruption_pose)
        if command.complete:
            self._finish_recovery_return()

    def _finish_recovery_return(self) -> None:
        if self._recovery.return_arrived:
            return
        # Keep the original ordering: clear/cancel the return path first, then
        # latch completion and resume the waypoint source.
        self._publish_empty_path()
        if not self._recovery.mark_complete():
            return
        self._resume_pub.publish(Empty())
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
            0.0
            if self._last_ready_event is None
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
            float("inf")
            if self._last_cloud_time is None
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
        if not self._planner_action_client.server_is_ready():
            return "waiting for ComputePathToPose action"
        if self.strict_ready_checks:
            if self.count_subscribers(self.global_path_topic) == 0:
                return "waiting for navigation_path_mux mission path subscriber"
            if self.count_subscribers(self.selected_global_path_topic) == 0:
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
            and self._motion.arrival_reported
        ):
            self._republish_stop_path()
            return
        target = self._fusion.latest_target()
        # The old coordinator only queried robot TF after a fresh target was
        # available.  It also treated missing TF as a technical fault.
        if target is None:
            self._republish_stop_path()
            return
        robot = self._robot_position()
        if robot is None:
            self._record_technical_fault("map -> base_footprint TF unavailable")
            self._republish_stop_path()
            return
        state = int(self._mission_state.state)
        command = self._motion.tick(
            state,
            MissionState.APPROACH_TARGET,
            MissionState.TRACK_INTRUDER,
            target,
            robot,
            self._fusion.sensor_distance(),
            self._latest_linear_speed,
            self._latest_angular_speed,
            self._odom_age(),
            self._planner_client.in_flight,
        )
        if command.decision is not None:
            now = now_time(self)
            log_age = (
                float("inf")
                if self._last_motion_log is None
                else (now - self._last_motion_log).nanoseconds * 1e-9
            )
            if log_age >= 1.0:
                desired = (
                    self.tracking_distance
                    if state == MissionState.TRACK_INTRUDER
                    else self.approach_distance
                )
                self.get_logger().info(
                    f"target standoff state={state} "
                    f"sensor_distance={command.decision.distance:.2f}m "
                    f"map_distance={command.decision.map_distance:.2f}m "
                    f"desired={desired:.2f}m stop={command.decision.should_stop}"
                )
                self._last_motion_log = now
        if command.should_stop:
            self._republish_stop_path()
        if command.arrived:
            self._publish_event(
                MissionEvent.ARRIVED_AND_STOPPED,
                "target standoff reached and robot velocity held below thresholds",
            )
        if command.goal is not None and target is not None and command.decision is not None:
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = self.global_frame
            goal_pose.pose.position.x = float(command.goal[0])
            goal_pose.pose.position.y = float(command.goal[1])
            goal_pose.pose.orientation.z = float(
                np.sin(command.decision.goal_yaw * 0.5)
            )
            goal_pose.pose.orientation.w = float(
                np.cos(command.decision.goal_yaw * 0.5)
            )
            if not self._planner_client.request(
                goal_pose,
                target,
                int(self._mission_state.state_seq),
                self._active_target_id,
                "target",
            ):
                return

    def _request_recovery_path(self, goal_pose: PoseStamped) -> None:
        target = np.asarray(
            [
                goal_pose.pose.position.x,
                goal_pose.pose.position.y,
                goal_pose.pose.position.z,
            ],
            dtype=np.float64,
        )
        accepted = self._planner_client.request(
            goal_pose,
            target,
            int(self._mission_state.state_seq),
            self._active_target_id,
            "recovery",
        )
        if not accepted:
            self._recovery.set_path_requested(False)

    def _on_planner_dispatched(self, request: PlannerRequest) -> None:
        """Start retry throttles after action availability is confirmed."""
        if not self._planner_request_matches_current_mission(request):
            return
        if request.plan_kind == "target":
            self._motion.mark_plan_requested(request.planned_target)
        elif request.plan_kind == "recovery":
            self._recovery.set_path_requested(True)

    def _on_planner_result(self, request: PlannerRequest, path: Path) -> None:
        if not self._planner_request_matches_current_mission(request):
            return
        self._clear_planner_fault()
        if request.plan_kind == "recovery":
            accepted_path = copy.deepcopy(path)
            self._recovery.set_path_result(accepted_path)
        elif request.plan_kind == "target":
            current_target = self._fusion.latest_target()
            if (
                current_target is None
                or np.linalg.norm(current_target[:2] - request.planned_target[:2])
                >= self.target_replan_distance
            ):
                # The planner is healthy, but this geometry is obsolete. Retry
                # immediately while retaining the previous target as the
                # movement baseline for the next comparison.
                self._motion.clear_plan_request()
                self._publish_empty_path()
                return
        else:
            return

        path.header.frame_id = self.global_frame
        path.header.stamp = now_time(self).to_msg()
        self._path_pub.publish(path)
        self._last_stop_publish = None

    def _publish_recovery_path(self, path: Path) -> None:
        republished = copy.deepcopy(path)
        republished.header.frame_id = self.global_frame
        republished.header.stamp = now_time(self).to_msg()
        self._path_pub.publish(republished)
        self._last_stop_publish = None

    def _odom_age(self) -> Optional[float]:
        if self._last_odom_time is None:
            return None
        return (now_time(self) - self._last_odom_time).nanoseconds * 1.0e-9

    def _check_fusion_health(self) -> None:
        reason = self._fusion.health_fault()
        if reason is None:
            return
        self._record_technical_fault(reason)

    def _on_fusion_recovered(self) -> None:
        # Successful fusion used to clear these coordinator-owned diagnostics
        # inside _try_fuse. Keep that transition at the same callback boundary.
        self._technical_fault_start = None
        self._technical_fault_reason = ""

    def _record_technical_fault(self, reason: str) -> None:
        now = now_time(self)
        if self._technical_fault_start is None:
            self._technical_fault_start = now
            self._technical_fault_reason = reason
            return
        if (
            now - self._technical_fault_start
        ).nanoseconds * 1e-9 >= self.technical_error_timeout:
            self._publish_event(
                MissionEvent.EXECUTION_ERROR,
                f"navigation technical fault: {self._technical_fault_reason}",
            )

    def _record_planner_failure(self, reason: str, request: PlannerRequest) -> None:
        if not self._planner_request_matches_current_mission(request):
            return
        if request.plan_kind == "recovery":
            self._recovery.set_path_requested(False)
        elif request.plan_kind == "target":
            # A failed replan must not leave the previous path driving toward
            # an obsolete position. The motion controller will retry after
            # its normal throttle period.
            self._publish_empty_path()
        now = now_time(self)
        if self._planner_failure_start is None:
            self._planner_failure_start = now
        self._planner_failure_reason = reason
        if (
            now - self._planner_failure_start
        ).nanoseconds * 1e-9 >= self.technical_error_timeout:
            self._publish_event(
                MissionEvent.EXECUTION_ERROR,
                f"navigation planner fault: {self._planner_failure_reason}",
            )

    def _planner_request_matches_current_mission(
        self, request: PlannerRequest
    ) -> bool:
        state = self._mission_state
        if (
            state is None
            or int(state.state_seq) != request.state_seq
            or int(state.target_id) != request.target_id
        ):
            return False
        if request.plan_kind == "recovery":
            return state.state == MissionState.RECOVER_PATROL
        if request.plan_kind == "target":
            return state.state in {
                MissionState.APPROACH_TARGET,
                MissionState.TRACK_INTRUDER,
            }
        return False

    def _clear_planner_fault(self) -> None:
        self._planner_failure_start = None
        self._planner_failure_reason = ""

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
            0
            if event == MissionEvent.READY
            else self._active_target_id if target_id is None else int(target_id)
        )
        message.source = MissionEvent.SOURCE_NAVIGATION
        message.event = int(event)
        message.detail = str(detail)[:256]
        self._event_pub.publish(message)
        if deduplicate:
            self._sent_events[key] = True

    def _publish_status(self) -> None:
        message = TargetNavigationStatus()
        message.header.stamp = now_time(self).to_msg()
        message.header.frame_id = self.global_frame
        message.target_id = int(self._active_target_id)
        message.distance_valid = False

        target = self._fusion.latest_target()
        robot = self._robot_position()
        if target is not None and robot is not None:
            message.distance_to_target = float(np.linalg.norm(target[:2] - robot[:2]))
            message.distance_valid = True

        state = self._mission_state.state if self._mission_state else None
        if state is None:
            message.status = TargetNavigationStatus.WAITING_TARGET
            message.detail = "waiting for mission state"
        elif state in {MissionState.STARTUP, MissionState.PATROL}:
            message.status = TargetNavigationStatus.WAITING_TARGET
            message.detail = "patrol is controlled by the waypoint navigation chain"
        elif state == MissionState.CONFIRM_TARGET:
            message.status = TargetNavigationStatus.WAITING_TARGET
            message.detail = self._fusion.last_fusion_error or "waiting for fused target position"
        elif state == MissionState.APPROACH_TARGET:
            if self._motion.arrival_reported:
                message.status = TargetNavigationStatus.ARRIVED
                message.detail = "3 m standoff reached and stopped"
            elif target is None:
                message.status = TargetNavigationStatus.HOLDING
                message.detail = "holding while waiting for fresh fused target"
            else:
                message.status = TargetNavigationStatus.APPROACHING
                message.detail = "approaching target standoff"
        elif state == MissionState.VERIFY_IDENTITY:
            message.status = TargetNavigationStatus.HOLDING
            message.detail = "holding position for identity verification"
        elif state == MissionState.TRACK_INTRUDER:
            if target is None:
                message.status = TargetNavigationStatus.HOLDING
                message.detail = "holding while target data is stale"
            else:
                message.status = TargetNavigationStatus.TRACKING
                message.detail = "tracking target with safe standoff"
        elif state == MissionState.RECOVER_PATROL:
            message.status = TargetNavigationStatus.HOLDING
            message.detail = "returning to patrol interruption pose"
        else:
            message.status = TargetNavigationStatus.WAITING_TARGET
            message.detail = "unsupported mission state"
        self._status_pub.publish(message)

    def _publish_empty_path(self) -> None:
        now = now_time(self)
        empty = Path()
        empty.header.frame_id = self.global_frame
        empty.header.stamp = now.to_msg()
        self._path_pub.publish(empty)
        # The first stop after a valid motion path must be immediate. The
        # periodic stop timer only throttles reaffirmations of this state.
        self._last_stop_publish = now
        self._request_controller_cancel()
        self._planner_client.invalidate("navigation path cleared")
        # Clearing the output also invalidates a recovery request that may be
        # waiting for an action response. Without releasing this latch, a
        # transient TF loss could leave recovery waiting forever for a result
        # that was deliberately cancelled here.
        if (
            self._mission_state is not None
            and self._mission_state.state == MissionState.RECOVER_PATROL
        ):
            self._recovery.set_path_requested(False)

    def _republish_stop_path(self) -> None:
        now = now_time(self)
        stop_age = (
            float("inf")
            if self._last_stop_publish is None
            else (now - self._last_stop_publish).nanoseconds * 1e-9
        )
        if stop_age >= self.stop_republish_period:
            self._publish_empty_path()
            self._last_stop_publish = now

    def _request_controller_cancel(self) -> None:
        """Cancel FollowPath goals without publishing a competing NAV_CMD."""
        if self._controller_cancel_pending:
            return
        if not self._controller_cancel_client.service_is_ready():
            now = now_time(self)
            warning_age = (
                float("inf")
                if self._last_controller_cancel_warning is None
                else (now - self._last_controller_cancel_warning).nanoseconds * 1e-9
            )
            if warning_age >= 1.0:
                self.get_logger().warning(
                    f"FollowPath cancel service for '{self.follow_path_action}' is unavailable"
                )
                self._last_controller_cancel_warning = now
            return
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

    def _reset_target(self) -> None:
        self._fusion.reset()
        self._motion.reset()
        self._planner_client.invalidate("target state reset")
        self._last_motion_log = None

    def _current_policy(self):
        if self._mission_state is None:
            return navigation_policy(-1)
        return navigation_policy(self._mission_state.state)

    def _accepts_fusion(self) -> bool:
        return (
            self._mission_state is not None
            and self._active_target_id > 0
            and self._current_policy().allow_target_fusion
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
