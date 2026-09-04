#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import warnings

# Matplotlib（可选，用于保存PNG）。无显示环境时强制 'Agg' 后端，避免阻塞/崩溃
try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    HAVE_MPL = True
except Exception:
    HAVE_MPL = False

import math
import numpy as np
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from builtin_interfaces.msg import Time as RosTime

from sensor_msgs.msg import Image, CameraInfo, LaserScan
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion, Vector3
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

from cv_bridge import CvBridge
import cv2

import tf2_ros
from tf_transformations import quaternion_matrix, quaternion_from_euler
from message_filters import TimeSynchronizer, Subscriber, ApproximateTimeSynchronizer  # <<< 严格同步

from ultralytics import YOLO

try:
    from pedestrian_tracker_msgs.msg import Track, TrackArray
    HAVE_CUSTOM_MSGS = True
except Exception:
    HAVE_CUSTOM_MSGS = False


def quat_msg_to_rot_matrix(q: Quaternion):
    return quaternion_matrix([q.x, q.y, q.z, q.w])[0:3, 0:3]

def normalize(v):
    n = np.linalg.norm(v)
    return v / n if n > 1e-9 else v

@dataclass
class KF2D:
    x: np.ndarray  # [x_o, y_o, vx_o, vy_o] in odom
    P: np.ndarray
    q_var: float
    r_var: float
    last_stamp: float

    def predict(self, now: float):
        dt = max(1e-3, now - self.last_stamp)
        F = np.array([[1,0,dt,0],
                      [0,1,0,dt],
                      [0,0,1, 0],
                      [0,0,0, 1]], dtype=float)
        G = np.array([[0.5*dt*dt, 0],
                      [0, 0.5*dt*dt],
                      [dt, 0],
                      [0, dt]], dtype=float)
        Q = (self.q_var) * (G @ G.T)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        self.last_stamp = now

    def update(self, z: np.ndarray):
        H = np.array([[1,0,0,0],
                      [0,1,0,0]], dtype=float)
        R = np.eye(2) * self.r_var
        y = z - (H @ self.x)
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ H) @ self.P


class PedestrianTrackerOdomNode(Node):
    def __init__(self):
        super().__init__('pedestrian_tracker_odom_node')

        # 基本参数
        self.declare_parameter('image_topic', 'oakd/rgb/preview/image_raw')
        self.declare_parameter('camera_info_topic', 'oakd/rgb/preview/camera_info')
        self.declare_parameter('laser_topic', 'scan')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('camera_frame', '')
        self.declare_parameter('lidar_frame', '')

        # YOLO/跟踪
        self.declare_parameter('model_path', 'yolo11n.pt')
        self.declare_parameter('conf_thres', 0.4)
        self.declare_parameter('iou_thres', 0.5)
        self.declare_parameter('class_person', 0)
        self.declare_parameter('skip_rate', 0)

        # 融合/几何
        self.declare_parameter('leg_half_width', 0.25)
        self.declare_parameter('window_deg_min', 2.0)
        self.declare_parameter('window_deg_max', 8.0)
        self.declare_parameter('range_min', 0.3)
        self.declare_parameter('range_max', 12.0)

        # 滤波
        self.declare_parameter('kf_q_var', 2.0)
        self.declare_parameter('kf_r_var', 0.05)
        self.declare_parameter('max_missing_sec', 1.0)

        # 可视化/发布
        self.declare_parameter('viz', True)
        self.declare_parameter('odom_twist_is_in_base_frame', True)  # 保留但不再使用
        self.declare_parameter('vel_arrow_scale', 0.5)

        # ---- 可视化（调试图）参数 ----
        self.declare_parameter('debug_image_topic', 'pedestrians/debug_image')
        self.declare_parameter('viz_draw_lidar_on_image', True)
        self.declare_parameter('viz_lidar_subsample', 1)
        self.declare_parameter('viz_box_thickness', 2)
        self.declare_parameter('viz_text_scale', 0.6)
        self.declare_parameter('viz_text_thickness', 1)
        self.declare_parameter('debug_matplotlib', False)
        self.declare_parameter('debug_save_png', False)
        self.declare_parameter('debug_save_dir', '/tmp')

        self.debug_image_topic   = self.get_parameter('debug_image_topic').get_parameter_value().string_value
        self.viz_draw_lidar_img  = self.get_parameter('viz_draw_lidar_on_image').get_parameter_value().bool_value
        self.viz_lidar_subsample = int(self.get_parameter('viz_lidar_subsample').get_parameter_value().integer_value)
        self.viz_box_thickness   = int(self.get_parameter('viz_box_thickness').get_parameter_value().integer_value)
        self.viz_text_scale      = float(self.get_parameter('viz_text_scale').get_parameter_value().double_value)
        self.viz_text_thickness  = int(self.get_parameter('viz_text_thickness').get_parameter_value().integer_value)
        self.debug_matplotlib    = self.get_parameter('debug_matplotlib').get_parameter_value().bool_value and HAVE_MPL
        self.debug_save_png      = self.get_parameter('debug_save_png').get_parameter_value().bool_value
        self.debug_save_dir      = self.get_parameter('debug_save_dir').get_parameter_value().string_value

        # 取参
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.laser_topic = self.get_parameter('laser_topic').get_parameter_value().string_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.camera_frame_param = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.lidar_frame_param = self.get_parameter('lidar_frame').get_parameter_value().string_value

        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.conf_thres = self.get_parameter('conf_thres').get_parameter_value().double_value
        self.iou_thres = self.get_parameter('iou_thres').get_parameter_value().double_value
        self.class_person = int(self.get_parameter('class_person').get_parameter_value().integer_value)
        self.skip_rate = int(self.get_parameter('skip_rate').get_parameter_value().integer_value)

        self.leg_half_width = self.get_parameter('leg_half_width').get_parameter_value().double_value
        self.window_deg_min = self.get_parameter('window_deg_min').get_parameter_value().double_value
        self.window_deg_max = self.get_parameter('window_deg_max').get_parameter_value().double_value
        self.range_min = self.get_parameter('range_min').get_parameter_value().double_value
        self.range_max = self.get_parameter('range_max').get_parameter_value().double_value

        self.kf_q = self.get_parameter('kf_q_var').get_parameter_value().double_value
        self.kf_r = self.get_parameter('kf_r_var').get_parameter_value().double_value
        self.max_missing_sec = self.get_parameter('max_missing_sec').get_parameter_value().double_value

        self.enable_viz = self.get_parameter('viz').get_parameter_value().bool_value
        self.vel_arrow_scale = self.get_parameter('vel_arrow_scale').get_parameter_value().double_value

        # 模型
        self.model = YOLO(self.model_path)
        self.model.fuse()

        # 订阅（严格同步）
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
        self.image_sub = Subscriber(self, Image, self.image_topic, qos_profile=qos)
        self.scan_sub  = Subscriber(self, LaserScan, self.laser_topic, qos_profile=qos)
        self.odom_sub  = Subscriber(self, Odometry, self.odom_topic, qos_profile=QoSProfile(depth=10))
        self.ts = ApproximateTimeSynchronizer([self.image_sub, self.scan_sub, self.odom_sub], queue_size=50, slop=0.3)  # <<< 严格同步
        self.ts.registerCallback(self.sync_cb)

        self.caminfo_sub = self.create_subscription(CameraInfo, self.camera_info_topic, self.caminfo_cb, qos)

        # 发布
        if HAVE_CUSTOM_MSGS:
            self.tracks_pub = self.create_publisher(TrackArray, 'pedestrians/tracks', 10)
        else:
            self.posearray_pub = self.create_publisher(PoseArray, 'pedestrians/poses', 10)
            self.marker_pub = self.create_publisher(MarkerArray, 'pedestrians/markers', 10)
            self.debug_img_pub = self.create_publisher(Image, self.debug_image_topic, 1)

        # TF
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 其他
        self.bridge = CvBridge()
        self.K = None
        self.frame_count = 0
        self.track_filters = {}  # tid -> KF2D
        self.last_seen = {}
        self._warned_neg_id = False

        self.get_logger().debug("Pedestrian tracker: KF in 'odom', publish ABSOLUTE velocity/position expressed in 'base_link' (strict sync).")

    # ----------------- 工具 -----------------

    def _sec(self, stamp: RosTime):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _drop_negative_tracks(self):
        neg_keys = [k for k in self.track_filters.keys() if k < 0]
        for k in neg_keys:
            self.track_filters.pop(k, None)
            self.last_seen.pop(k, None)

    def _sanitize_topic_suffix(self, tid: int) -> str:
        return str(tid) if tid >= 0 else f"n{abs(tid)}"

    # ----------------- 回调 -----------------

    def caminfo_cb(self, msg: CameraInfo):
        self.K = np.array(msg.k).reshape(3,3)

    # ----------------- 主同步回调（严格同步） -----------------

    def sync_cb(self, img_msg: Image, scan_msg: LaserScan, odom_msg: Odometry):
        self.get_logger().debug("sync_call_back")
        if self.K is None:
            return
        if self.skip_rate > 0 and (self.frame_count % (self.skip_rate + 1) != 0):
            self.frame_count += 1
            return
        self.frame_count += 1

        # 以同步后的统一时间戳进行 TF 查询，严格时序一致
        stamp = odom_msg.header.stamp

        camera_frame = self.camera_frame_param if self.camera_frame_param else (img_msg.header.frame_id or 'camera_optical_frame')
        lidar_frame  = self.lidar_frame_param if self.lidar_frame_param else (scan_msg.header.frame_id or 'laser')

        try:
            T_lidar_cam = self.tf_buffer.lookup_transform(lidar_frame, self._strip_optical(camera_frame), stamp, rclpy.duration.Duration(seconds=0.1))
            R_lidar_cam = quat_msg_to_rot_matrix(T_lidar_cam.transform.rotation)
            t_lidar_cam = np.array([T_lidar_cam.transform.translation.x,
                                    T_lidar_cam.transform.translation.y,
                                    T_lidar_cam.transform.translation.z], dtype=float)
            R_cam_lidar = R_lidar_cam.T
            t_cam_lidar = - R_cam_lidar @ t_lidar_cam

            T_base_lidar = self.tf_buffer.lookup_transform(self.base_frame, lidar_frame, stamp, rclpy.duration.Duration(seconds=0.1))
            R_base_lidar = quat_msg_to_rot_matrix(T_base_lidar.transform.rotation)
            t_base_lidar = np.array([T_base_lidar.transform.translation.x,
                                     T_base_lidar.transform.translation.y,
                                     T_base_lidar.transform.translation.z])

            T_ob = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, stamp, rclpy.duration.Duration(seconds=0.1))
            R_ob = quat_msg_to_rot_matrix(T_ob.transform.rotation)
            t_ob = np.array([T_ob.transform.translation.x,
                             T_ob.transform.translation.y,
                             T_ob.transform.translation.z])
        except Exception as e:
            self.get_logger().debug(f"TF lookup failed: {e}")
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().debug(f"cv_bridge failed: {e}")
            return

        results = self.model.track(
            source=frame,
            conf=self.conf_thres,
            iou=self.iou_thres,
            classes=[self.class_person],
            persist=True,
            verbose=False,
            tracker="bytetrack.yaml"
        )
        if isinstance(results, list) and len(results) > 0:
            res = results[0]
        else:
            return
        if res.boxes is None or res.boxes.xyxy is None:
            self._prune_filters(self._sec(stamp))
            self._publish(stamp, R_ob, t_ob)
            return

        xyxy = res.boxes.xyxy.cpu().numpy()
        ids = res.boxes.id.cpu().numpy() if res.boxes.id is not None else np.full((xyxy.shape[0],), -1)
        confs = res.boxes.conf.cpu().numpy() if res.boxes.conf is not None else np.ones(xyxy.shape[0])

        angle_min = scan_msg.angle_min
        angle_inc = scan_msg.angle_increment
        ranges = np.array(scan_msg.ranges, dtype=float)
        ranges[~np.isfinite(ranges)] = np.inf

        fx, fy = self.K[0,0], self.K[1,1]
        cx, cy = self.K[0,2], self.K[1,2]

        now_sec = self._sec(stamp)
        measurements = []

        for i in range(xyxy.shape[0]):
            if confs[i] < self.conf_thres:
                continue
            x1, y1, x2, y2 = xyxy[i]
            tid = int(ids[i]) if ids is not None else -1

            # 跳过负 id
            if tid < 0:
                if not self._warned_neg_id:
                    self.get_logger().warn("YOLO tracker produced id=-1 (unassigned). Skipping until a valid track id appears.")
                    self._warned_neg_id = True
                continue

            margin = 3.0
            u = 0.5 * (x1 + x2)
            v = max(0.0, y2 - margin)

            d_cam = normalize(np.array([(u - cx)/fx, (v - cy)/fy, 1.0], dtype=float))
            d_lidar = normalize(R_lidar_cam @ d_cam)
            gamma = math.atan2(d_lidar[1], d_lidar[0])

            center_idx = int(round((gamma - angle_min) / angle_inc))
            if center_idx < 0 or center_idx >= ranges.shape[0]:
                continue

            window_deg = 5.0
            if tid in self.track_filters:
                prev_dist = max(0.5, np.linalg.norm(self.track_filters[tid].x[0:2]))
                theta = math.degrees(math.atan(self.leg_half_width / prev_dist))
                window_deg = max(self.window_deg_min, min(self.window_deg_max, 2.0*theta))
            win = max(1, int(round(math.radians(window_deg) / max(1e-6, angle_inc))))

            i0 = max(0, center_idx - win)
            i1 = min(ranges.shape[0]-1, center_idx + win)
            r = np.min(ranges[i0:i1+1])
            if not np.isfinite(r) or r < self.range_min or r > self.range_max or math.isinf(r):
                continue

            p_lidar = np.array([r*math.cos(gamma), r*math.sin(gamma), 0.0], dtype=float)
            p_bl = (R_base_lidar @ p_lidar) + t_base_lidar
            p_o_3d = (R_ob @ p_bl) + t_ob
            z_o = p_o_3d[0:2]

            if tid in self.track_filters:
                pred = self.track_filters[tid].x[0:2]
                if np.linalg.norm(z_o - pred) > 1.5:
                    continue

            measurements.append((tid, z_o[0], z_o[1]))

        # 清掉历史负 id
        self._drop_negative_tracks()

        # 更新滤波器（odom）
        for tid, zx, zy in measurements:
            if tid not in self.track_filters:
                self.track_filters[tid] = KF2D(
                    x=np.array([zx, zy, 0.0, 0.0], dtype=float),
                    P=np.diag([0.2, 0.2, 1.0, 1.0]),
                    q_var=self.kf_q,
                    r_var=self.kf_r,
                    last_stamp=now_sec
                )
            kf = self.track_filters[tid]
            kf.predict(now_sec)
            kf.update(np.array([zx, zy], dtype=float))
            self.last_seen[tid] = now_sec

        # 仅预测缺测轨迹
        present_ids = {t for t, *_ in measurements}
        for tid, kf in list(self.track_filters.items()):
            if tid not in present_ids:
                kf.predict(now_sec)

        try:
            self._render_and_publish_debug_image(
                frame=frame,
                K=self.K,
                R_cam_lidar=R_cam_lidar,
                t_cam_lidar=t_cam_lidar,
                ranges=ranges,
                angle_min=angle_min,
                angle_inc=angle_inc,
                xyxy=xyxy,
                ids=ids,
                confs=confs,
                stamp=stamp,
                R_ob=R_ob,
                t_ob=t_ob
            )
        except Exception as e:
            self.get_logger().debug(f"Debug viz failed: {e}")

        self._prune_filters(now_sec)
        self._publish(stamp, R_ob, t_ob)

    # ----------------- 杂项 -----------------
    def _lidar_points_to_image_uv(self, ranges: np.ndarray, angle_min: float, angle_inc: float,
                                  K: np.ndarray, R_cam_lidar: np.ndarray, t_cam_lidar: np.ndarray,
                                  img_shape, subsample: int = 1):
        """
        将 2D LaserScan 点（假定位于 LiDAR 坐标系平面 z=0）经外参变换到相机，再用内参投影到像素坐标。
        返回 (u,v) 像素整型数组，已经过滤了无效/越界/背面点。
        """
        H, W = img_shape[:2]
        N = ranges.shape[0]
        step = max(1, int(subsample))
        idx = np.arange(0, N, step)

        r = ranges[idx]
        finite = np.isfinite(r)
        idx = idx[finite]
        r = r[finite]

        if r.size == 0:
            return np.empty((0, 2), dtype=np.int32)

        ang = angle_min + idx * angle_inc
        x_l = r * np.cos(ang)
        y_l = r * np.sin(ang)
        z_l = np.zeros_like(x_l)
        P_l = np.stack([x_l, y_l, z_l], axis=0)

        P_c = (R_cam_lidar @ P_l) + t_cam_lidar.reshape(3, 1)
        Z = P_c[2, :]
        front = Z > 1e-3
        if not np.any(front):
            return np.empty((0, 2), dtype=np.int32)
        X = P_c[0, front]
        Y = P_c[1, front]
        Z = Z[front]

        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]
        u = (fx * (X / Z) + cx)
        v = (fy * (Y / Z) + cy)

        in_img = (u >= 0) & (u < (W - 1)) & (v >= 0) & (v < (H - 1))
        u = u[in_img].astype(np.int32)
        v = v[in_img].astype(np.int32)
        if u.size == 0:
            return np.empty((0, 2), dtype=np.int32)
        return np.stack([u, v], axis=1)

    def _make_debug_overlay(self, frame_bgr: np.ndarray, xyxy: np.ndarray, ids: np.ndarray, confs: np.ndarray,
                            uv_lidar: np.ndarray, K: np.ndarray,
                            R_ob: np.ndarray, t_ob: np.ndarray):
        """
        在图像上叠加：LiDAR 投影点、YOLO 检测框、框上的速度/位置（使用当前 KF 状态，速度为绝对速度在 base_link 下的分量）。
        """
        vis = frame_bgr.copy()
        H, W = vis.shape[:2]

        # 1) LiDAR 点
        if uv_lidar is not None and uv_lidar.size > 0:
            for (u, v) in uv_lidar:
                cv2.circle(vis, (int(u), int(v)), 1, (60, 220, 60), -1, lineType=cv2.LINE_AA)

        # 2) 转换矩阵
        R_bo = R_ob.T  # odom->base
        R_bo_2 = R_bo[0:2, 0:2]

        # 3) 画检测框并标注信息（只对 id>=0 且有 KF 的目标）
        if xyxy is None or xyxy.size == 0:
            return vis

        for i in range(xyxy.shape[0]):
            if confs is not None and confs[i] < self.conf_thres:
                continue
            tid = int(ids[i]) if ids is not None else -1
            if tid < 0:
                continue
            if tid not in self.track_filters:
                continue

            x1, y1, x2, y2 = xyxy[i].astype(int)
            x1 = max(0, min(W - 1, x1)); y1 = max(0, min(H - 1, y1))
            x2 = max(0, min(W - 1, x2)); y2 = max(0, min(H - 1, y2))

            # KF 状态（odom）
            kf = self.track_filters[tid]
            p_o = kf.x[0:2]
            v_o = kf.x[2:4]

            # 位置：odom -> base_link
            p_b = (R_bo @ (np.array([p_o[0], p_o[1], 0.0]) - t_ob))[0:2]
            # 速度：绝对速度在 base_link 下的分量（不做自车补偿）
            v_b = (R_bo_2 @ v_o.reshape(2, 1)).flatten()
            speed = float(np.linalg.norm(v_b))

            # 框与文本
            color = (50, 180, 255)
            cv2.rectangle(vis, (x1, y1), (x2, y2), color, thickness=self.viz_box_thickness, lineType=cv2.LINE_AA)

            label = f"id:{tid}  v:{speed:.2f}m/s  p:({p_b[0]:.2f},{p_b[1]:.2f})m"
            (tw, th), base = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, self.viz_text_scale, self.viz_text_thickness)
            pad = 3

            x_text = int((x1 + x2 - tw) / 2)
            x_text = max(pad, min(W - tw - pad, x_text))

            y_text = y2 + th + base + 6
            rect_tl = (x_text - pad, y_text - th - base - pad)
            rect_br = (x_text + tw + pad, y_text + pad)

            if rect_br[1] >= H:
                y_text = y1 - 6
                rect_tl = (x_text - pad, y_text - th - base - pad)
                rect_br = (x_text + tw + pad, y_text + pad)
                if rect_tl[1] < 0:
                    shift = -rect_tl[1]
                    y_text += shift
                    rect_tl = (rect_tl[0], rect_tl[1] + shift)
                    rect_br = (rect_br[0], rect_br[1] + shift)

            cv2.rectangle(vis, rect_tl, rect_br, (0, 0, 0), -1)
            cv2.putText(vis, label, (x_text, y_text),
                        cv2.FONT_HERSHEY_SIMPLEX, self.viz_text_scale,
                        (255, 255, 255), self.viz_text_thickness, cv2.LINE_AA)

        return vis

    def _render_and_publish_debug_image(self, frame: np.ndarray, K: np.ndarray,
                                        R_cam_lidar: np.ndarray, t_cam_lidar: np.ndarray,
                                        ranges: np.ndarray, angle_min: float, angle_inc: float,
                                        xyxy: np.ndarray, ids: np.ndarray, confs: np.ndarray,
                                        stamp, R_ob: np.ndarray, t_ob: np.ndarray):
        """
        统一调度：投影LiDAR→生成叠加→发布ROS图像；可选保存PNG。
        """
        if frame is None or K is None:
            return

        uv = None
        if self.viz_draw_lidar_img:
            uv = self._lidar_points_to_image_uv(
                ranges=ranges,
                angle_min=angle_min,
                angle_inc=angle_inc,
                K=K,
                R_cam_lidar=R_cam_lidar,
                t_cam_lidar=t_cam_lidar,
                img_shape=frame.shape,
                subsample=self.viz_lidar_subsample
            )

        vis = self._make_debug_overlay(
            frame_bgr=frame,
            xyxy=xyxy,
            ids=ids,
            confs=confs,
            uv_lidar=uv,
            K=K,
            R_ob=R_ob,
            t_ob=t_ob
        )

        # 发布 ROS 调试图像
        try:
            img_msg = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
            img_msg.header.stamp = stamp
            img_msg.header.frame_id = 'camera'
            self.debug_img_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().debug(f"Publish debug image failed: {e}")

        # 可选：保存 PNG
        if self.debug_save_png:
            try:
                os.makedirs(self.debug_save_dir, exist_ok=True)
                ts = f"{stamp.sec}.{int(stamp.nanosec):09d}"
                out_file = os.path.join(self.debug_save_dir, f"ped_debug_{ts}.png")

                if self.debug_matplotlib and HAVE_MPL:
                    fig, ax = plt.subplots(figsize=(12, 7), dpi=120)
                    ax.imshow(cv2.cvtColor(vis, cv2.COLOR_BGR2RGB))
                    ax.set_axis_off()
                    fig.tight_layout(pad=0)
                    fig.savefig(out_file, bbox_inches='tight', pad_inches=0)
                    plt.close(fig)
                else:
                    cv2.imwrite(out_file, vis)
            except Exception as e:
                self.get_logger().debug(f"Save debug PNG failed: {e}")

    def _prune_filters(self, now_sec: float):
        to_del = []
        for tid, last in self.last_seen.items():
            if now_sec - last > self.max_missing_sec:
                to_del.append(tid)
        for tid in to_del:
            self.track_filters.pop(tid, None)
            self.last_seen.pop(tid, None)

    def _strip_optical(self, frame_id: str) -> str:
        return frame_id

    # ----------------- 发布 -----------------
    def _publish(self, stamp: RosTime, R_ob: np.ndarray, t_ob: np.ndarray):
        # 仅正 id
        ids = [tid for tid in self.track_filters.keys() if tid >= 0]
        ids.sort()

        R_bo = R_ob.T
        R_bo_2 = R_bo[0:2, 0:2]

        # ---- PoseArray（可保留给下游使用）----
        pa = PoseArray()
        pa.header.stamp = stamp
        pa.header.frame_id = self.base_frame

        poses = []
        v_b_dict = {}
        p_b_dict = {}

        for tid in ids:
            kf = self.track_filters[tid]
            p_o = kf.x[0:2]
            v_o = kf.x[2:4]

            # 位置（base_link 表达）
            p_b = (R_bo @ (np.array([p_o[0], p_o[1], 0.0]) - t_ob))[0:2]
            # 速度（绝对速度在 base_link 下的分量）
            v_b = (R_bo_2 @ v_o.reshape(2, 1)).flatten()

            p = Pose()
            p.position = Point(x=float(p_b[0]), y=float(p_b[1]), z=0.0)
            yaw = math.atan2(float(v_b[1]), float(v_b[0])) if np.linalg.norm(v_b) > 1e-3 else 0.0
            q = quaternion_from_euler(0.0, 0.0, yaw)
            p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

            poses.append(p)
            v_b_dict[tid] = v_b
            p_b_dict[tid] = p_b

        pa.poses = poses
        self.posearray_pub.publish(pa)

        # ---- MarkerArray（位置球 + 文本 + 速度箭头；全部在一个 topic）----
        if self.enable_viz:
            ma = MarkerArray()
            mid = 0

            # 先清空
            clr = Marker()
            clr.header.stamp = stamp
            clr.header.frame_id = self.base_frame
            clr.ns = "pedestrians"
            clr.id = mid; mid += 1
            clr.action = Marker.DELETEALL
            ma.markers.append(clr)

            for tid in ids:
                p_b = p_b_dict[tid]
                v_b = v_b_dict[tid]
                speed = float(np.linalg.norm(v_b))

                # 1) 球形位置 marker
                m = Marker()
                m.header.stamp = stamp
                m.header.frame_id = self.base_frame
                m.ns = "pedestrians"
                m.id = mid; mid += 1
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose.position.x = float(p_b[0])
                m.pose.position.y = float(p_b[1])
                m.pose.position.z = 0.05
                m.scale = Vector3(x=0.25, y=0.25, z=0.25)
                m.color.r = 0.1; m.color.g = 0.8; m.color.b = 0.2; m.color.a = 0.9
                ma.markers.append(m)

                # 2) 文本 marker（包含 id、位置/速度分量与速率）
                t = Marker()
                t.header.stamp = stamp
                t.header.frame_id = self.base_frame
                t.ns = "pedestrians_text"
                t.id = mid; mid += 1
                t.type = Marker.TEXT_VIEW_FACING
                t.action = Marker.ADD
                t.pose.position.x = float(p_b[0])
                t.pose.position.y = float(p_b[1])
                t.pose.position.z = 0.6
                t.scale.z = 0.25
                t.color.r = 1.0; t.color.g = 1.0; t.color.b = 1.0; t.color.a = 1.0
                t.text = (
                    f"id:{tid}\n"
                    f"px:{p_b[0]:.2f} m  py:{p_b[1]:.2f} m\n"
                    f"vx:{v_b[0]:.2f} m/s  vy:{v_b[1]:.2f} m/s\n"
                    f"|v|:{speed:.2f} m/s"
                )
                ma.markers.append(t)

                # 3) 速度箭头 marker（直观表达速度方向与大小）
                a = Marker()
                a.header.stamp = stamp
                a.header.frame_id = self.base_frame
                a.ns = "pedestrians_vel"
                a.id = mid; mid += 1
                a.type = Marker.ARROW
                a.action = Marker.ADD

                p0 = Point(x=float(p_b[0]), y=float(p_b[1]), z=0.1)
                p1 = Point(x=float(p_b[0] + self.vel_arrow_scale * v_b[0]),
                        y=float(p_b[1] + self.vel_arrow_scale * v_b[1]),
                        z=0.1)
                a.points = [p0, p1]

                # shaft 直径 / 箭头直径 / 箭头长度
                a.scale = Vector3(x=0.05, y=0.10, z=0.10)
                a.color.r = 0.2; a.color.g = 0.6; a.color.b = 1.0; a.color.a = 0.9
                ma.markers.append(a)

            self.marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = PedestrianTrackerOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
