#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS2 版本：全局 Navfn 路径 + Pure Pursuit 取局部目标
+ 多局部目标【批量】采样 + （可选）PRIEST 优化（批量）
动态障碍输入已适配从 pedestrians/markers 的 MarkerArray 文本中解析 vx、vy。
默认输出：
    local_path : nav_msgs/msg/Path
"""

import os
import math
import time
import re
import pickle
import sys
import traceback
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple, List

os.environ["XLA_PYTHON_CLIENT_PREALLOCATE"] = "false"

import os
from ament_index_python.packages import get_package_share_directory

try:
    pkg_share = get_package_share_directory('move')
    ws_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(pkg_share))))
except Exception:
    ws_root = str(Path(__file__).resolve().parents[4])

nav_root = Path(
    os.environ.get(
        "DOG_PATROL_NAV_ROOT",
        str(Path(ws_root) / "src" / "navigation" / "fast_livo_dog"),
    )
)
WORKSPACE_SRC = nav_root / "navigation"
RL2PATH_ROOT = WORKSPACE_SRC / "RL2Path"
if RL2PATH_ROOT.is_dir() and str(RL2PATH_ROOT) not in sys.path:
    sys.path.insert(0, str(RL2PATH_ROOT))

MOVE_PACKAGE_ROOT = WORKSPACE_SRC / "move"
DEFAULT_MODEL_PATH = MOVE_PACKAGE_ROOT / "ckpts" / "ckpt_3" / "best_agent.pt"
DEFAULT_AGENT_CFG_PATH = MOVE_PACKAGE_ROOT / "ckpts" / "ckpt_3" / "params" / "agent.pkl"

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration
from rclpy.time import Time as RosTime

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan, PointCloud, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from visualization_msgs.msg import MarkerArray, Marker
from action_msgs.msg import GoalStatusArray

try:
    from pedsim_msgs.msg import AgentStates, TrackedPersons  # type: ignore
except Exception:
    AgentStates = None
    TrackedPersons = None

import tf2_ros
from tf_transformations import quaternion_matrix  # noqa: F401

import torch
from gymnasium import spaces
from skrl.agents.torch.ppo import PPO

# ====== 你项目中的 RL 依赖（保持不变） ======
from RL2Path.utils.math_tools import (
    denormalize_action_tensor,
    polar_to_cartesian,
    batch_cubic_spline_interpolation,
    cartesian_to_polar,
    rotate_xy_tensor,
    normalize_polar_point,
)

from RL2Path.nav.cost_map_2d import Costmap_2d, CostmapCfg
from RL2Path.nav.esdf_map_2d import Costmap_2d as ESDFMap2D, CostmapCfg as ESDFMapCfg
from RL2Path.model.policy import PolicyNet
from RL2Path.model.value import ValueNet
from RL2Path.priest.priest import PriestPlanner
from RL2Path.priest.priest_config import PriestConfig


# ==================== 工具函数 ====================

def laserscan_to_points(scan: LaserScan, min_range: float = 0.2) -> np.ndarray:
    angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
    ranges = np.array(scan.ranges, dtype=np.float32)
    mask = np.isfinite(ranges) & (ranges > min_range)
    ranges = ranges[mask]
    angles = angles[mask]
    xs = ranges * np.cos(angles)
    ys = ranges * np.sin(angles)
    return np.stack([xs, ys], axis=1).astype(np.float32)  # (N, 2)


def process_laser_scan_like_script1(scan: LaserScan, max_useful_range: float = 50.0,
                                    maximum_points: int = 1080) -> np.ndarray:
    ranges = np.array(scan.ranges, dtype=np.float32)
    if abs(scan.angle_increment) < 1e-10:
        num_points = len(ranges)
        angles = np.linspace(scan.angle_min, scan.angle_max, num_points, dtype=np.float32)
    else:
        angles = np.arange(
            scan.angle_min,
            scan.angle_max + scan.angle_increment,
            scan.angle_increment,
            dtype=np.float32
        )
    if len(angles) != len(ranges):
        m = min(len(angles), len(ranges))
        angles = angles[:m]
        ranges = ranges[:m]

    valid = (
        (ranges >= scan.range_min)
        & ((ranges <= scan.range_max) | np.isinf(scan.range_max))
        & (ranges <= max_useful_range)
    )
    r_ok = ranges[valid]
    a_ok = angles[valid]

    r_pad = np.pad(r_ok, (0, maximum_points - len(r_ok)), mode="constant",
                   constant_values=np.nan)[:maximum_points]
    a_pad = np.pad(a_ok, (0, maximum_points - len(a_ok)), mode="constant",
                   constant_values=np.nan)[:maximum_points]

    return np.stack((r_pad, a_pad), axis=-1).astype(np.float32)


# ==================== skrl 包装 ====================

class SkrlPolicyWrapper:
    def __init__(self, model_ckpt_path: str, obs_dim: int, act_dim: int,
                 agent_cfg, costmap_width, device: str = "cuda"):
        self.torch_device = torch.device(
            device if (device == "cuda" and torch.cuda.is_available()) else "cpu"
        )

        observation_space = spaces.Box(low=0.0, high=1.0, shape=(obs_dim,), dtype=np.float32)
        action_space = spaces.Box(low=0.0, high=1.0, shape=(act_dim,), dtype=np.float32)

        self.policy = PolicyNet(
            observation_space=observation_space,
            action_space=action_space,
            device=str(self.torch_device),
            model_version="tcnn_v2",
            clip_actions=False,
            goal_include_theta=False,
            costmap_width=costmap_width
        )

        self.value = ValueNet(
            observation_space=observation_space,
            action_space=action_space,
            device=str(self.torch_device),
            model_version="tcnn_v2",
            clip_actions=False,
            goal_include_theta=False,
            costmap_width=costmap_width
        )

        models = {"policy": self.policy, "value": self.value}

        self.agent = PPO(
            models=models,
            observation_space=observation_space,
            action_space=action_space,
            device=str(self.torch_device),
            cfg=agent_cfg,
        )

        self.agent.load(model_ckpt_path)

    def act(self, obs: torch.Tensor):
        try:
            ctx = torch.inference_mode
        except AttributeError:
            ctx = torch.no_grad
        with ctx():
            return self.agent.act(obs, timestep=0, timesteps=0)


# ==================== 配置与节点（ROS2） ====================

@dataclass
class PlannerConfig:
    num_points: int = 50
    r_max: float = 1.8
    theta_min: float = -np.pi / 2
    theta_max: float = np.pi / 2
    action_with_goal: bool = False
    hz: float = 10.0

    # Pure Pursuit & 虚拟目标相关
    pp_lookahead: float = 3.8
    virt_goal_min: float = 3.5
    virt_goal_max: float = 4.0
    virt_goal_pref: float = 3.8

    # 多局部目标采样
    num_local_goals: int = 5
    local_goal_span_m: float = 1.0
    local_goal_sampling_mode: str = "perp_segment"
    local_goal_angle_deg: float = 120.0
    local_goal_radius_m: float = 4.0

    # 框架/话题
    use_scan: bool = True
    global_frame: str = "map"
    odom_frame: str = "camera_init_footprint"
    base_frame: str = "base_footprint"
    frame_id: str = "base_footprint"
    tf_timeout: float = 0.2
    global_plan_topic: str = "global_path"

    device: str = "cuda"


class RLLocalPlannerNodeROS2(Node):
    def __init__(self):
        super().__init__("rl_priest_local_planner_omni_nav_cmd")

        # ---------- 参数 ----------
        def declare_get(name, default):
            self.declare_parameter(name, default)
            return self.get_parameter(name).get_parameter_value()

        self.priest_cfg = PriestConfig.from_rosparams(self)
        self.get_logger().debug(f"[priest config] {self.priest_cfg}")

        self.cfg = PlannerConfig(
            num_points=int(declare_get("num_points", 100).integer_value),
            r_max=float(declare_get("r_max", 4.0).double_value),
            theta_min=float(declare_get("theta_range_min", -np.pi / 2).double_value),
            theta_max=float(declare_get("theta_range_max",  np.pi / 2).double_value),
            action_with_goal=bool(declare_get("action_with_goal", False).bool_value),
            hz=float(declare_get("hz", 10.0).double_value),

            pp_lookahead=float(declare_get("pp_lookahead", 4.0).double_value),
            virt_goal_min=float(declare_get("virt_goal_min", 3.5).double_value),
            virt_goal_max=float(declare_get("virt_goal_max", 4.0).double_value),
            virt_goal_pref=float(declare_get("virt_goal_pref", 4.0).double_value),

            num_local_goals=int(declare_get("num_local_goals", 1).integer_value),
            local_goal_span_m=float(declare_get("local_goal_span_m", 2.0).double_value),
            local_goal_sampling_mode=str(declare_get("local_goal_sampling_mode", "perp_segment").string_value),
            local_goal_angle_deg=float(declare_get("local_goal_angle_deg", 90.0).double_value),
            local_goal_radius_m=float(declare_get("local_goal_radius_m", 4.0).double_value),

            use_scan=bool(declare_get("use_scan", True).bool_value),
            global_frame=str(declare_get("global_frame", "map").string_value),
            odom_frame=str(declare_get("odom_frame", "camera_init_footprint").string_value),
            base_frame=str(declare_get("base_frame", "base_footprint").string_value),
            frame_id=str(declare_get("frame_id", "base_footprint").string_value),
            tf_timeout=float(declare_get("tf_timeout", 0.08).double_value),
            global_plan_topic=str(declare_get("global_plan_topic", "global_path").string_value),
            device=str(declare_get("device", "cuda").string_value),
        )

        self.declare_parameter("model_path", str(DEFAULT_MODEL_PATH))
        self.declare_parameter("agent_cfg_path", str(DEFAULT_AGENT_CFG_PATH))
        self.declare_parameter("act_dim", 10)   # 2 * 5
        self.declare_parameter("obs_dim", 105 * 105 * 2 + 2 + 10)
        self.declare_parameter("costmap_width", 105)
        self.declare_parameter("rl_sample_batch", 50)

        self.model_path = self.get_parameter("model_path").get_parameter_value().string_value
        self.agent_cfg_path = self.get_parameter("agent_cfg_path").get_parameter_value().string_value
        self.act_dim = int(self.get_parameter("act_dim").get_parameter_value().integer_value)
        self.obs_dim = int(self.get_parameter("obs_dim").get_parameter_value().integer_value)
        self.costmap_width = int(self.get_parameter("costmap_width").get_parameter_value().integer_value)
        self.sample_batch = int(self.get_parameter("rl_sample_batch").get_parameter_value().integer_value)

        if self.sample_batch <= 0:
            self.sample_batch = 1

        if not self.agent_cfg_path or not os.path.isfile(self.agent_cfg_path):
            self.get_logger().error(f"[rl_priest_planner] agent_cfg_path not found: {self.agent_cfg_path}")
            raise FileNotFoundError(self.agent_cfg_path)

        with open(self.agent_cfg_path, "rb") as f:
            agent_cfg = pickle.load(f)

        if not self.model_path or not os.path.isfile(self.model_path):
            self.get_logger().error(f"[rl_priest_planner] model_path not found: {self.model_path}")
            raise FileNotFoundError(self.model_path)

        self.agent = SkrlPolicyWrapper(
            self.model_path, self.obs_dim, self.act_dim, agent_cfg=agent_cfg,
            device=self.cfg.device, costmap_width=self.costmap_width
        )

        # ---------- TF ----------
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---------- 订阅 / 发布 ----------
        q = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.sub_odom = self.create_subscription(Odometry, "odom", self._on_odom, 1)
        if self.cfg.use_scan:
            self.sub_scan = self.create_subscription(LaserScan, "scan", self._on_scan, qos_profile=q)
            self.sub_points = self.create_subscription(PointCloud, "points", self._on_points_pc, 1)
        else:
            self.sub_points = self.create_subscription(PointCloud, "points", self._on_points_pc, 1)
            self.sub_scan = None

        self.sub_global_plan = self.create_subscription(Path, self.cfg.global_plan_topic, self._on_global_plan, 1)

        self.latest_dynamic_raw = None
        if self.priest_cfg.enable:
            t = self.priest_cfg.dynamic_topic
            typ = self.priest_cfg.dynamic_msg_type
            if typ == "marker_array":
                self.sub_dyn = self.create_subscription(MarkerArray, t, self._on_dynamic_obstacles, 1)
            elif typ == "tracked_persons" and TrackedPersons is not None:
                self.sub_dyn = self.create_subscription(TrackedPersons, t, self._on_dynamic_obstacles, 1)
            elif typ == "agent_states" and AgentStates is not None:
                self.sub_dyn = self.create_subscription(AgentStates, t, self._on_dynamic_obstacles, 1)
            else:
                self.get_logger().debug(
                    f"[rl_priest_planner] dynamic_msg_type '{typ}' not available, dynamic obstacles disabled."
                )
                self.priest_cfg.use_obstacle_constraints = False

        self.pub_local_path = self.create_publisher(Path, "local_path", 10)
        self.pub_local_path_rl = self.create_publisher(Path, "local_path_rl_debug", 10) \
            if self.priest_cfg.publish_rl_debug else None

        self.sub_subgoal = self.create_subscription(PoseStamped, "subgoal", self._on_subgoal, 1)
        self.subgoal_position: Optional[Tuple[float, float]] = None

        self.use_esdf = True
        if self.use_esdf:
            costmap_cfg = ESDFMapCfg(
                esdf_clip=self.cfg.r_max,
                x_min=0.0,
                x_max=4.2,
                y_min=-2.1,
                y_max=2.1,
                map_size=self.costmap_width
            )
            self.costmap = ESDFMap2D(costmap_cfg, device=self.cfg.device)
        else:
            costmap_cfg = CostmapCfg()
            self.costmap = Costmap_2d(costmap_cfg, device=self.cfg.device)

        self.last_action = torch.zeros(
            (self.cfg.num_local_goals, self.act_dim),
            device=self.cfg.device,
            dtype=torch.float32
        )
        self.latest_odom: Optional[Odometry] = None
        self.latest_scan: Optional[LaserScan] = None
        self.latest_points_pc: Optional[PointCloud] = None
        self.latest_global_plan: Optional[Path] = None

        self.priest = PriestPlanner(
            num_dynamic_obstacles=self.priest_cfg.num_dynamic_obstacles,
            num_static_obstacles=self.priest_cfg.num_static_obstacles,
            time_horizon=self.priest_cfg.time_horizon,
            tracking_weight=self.priest_cfg.tracking_weight,
            smoothness_weight=self.priest_cfg.smoothness_weight,
            static_obstacle_semi_minor_axis=self.priest_cfg.static_obstacle_semi_minor_axis,
            static_obstacle_semi_major_axis=self.priest_cfg.static_obstacle_semi_major_axis,
            dynamic_obstacle_semi_minor_axis=self.priest_cfg.dynamic_obstacle_semi_minor_axis,
            dynamic_obstacle_semi_major_axis=self.priest_cfg.dynamic_obstacle_semi_major_axis,
            num_waypoints=self.priest_cfg.num_waypoints,
            trajectory_length=self.priest_cfg.trajectory_length,
            trajectory_batch_size=int(self.sample_batch),
            desired_velocity=self.priest_cfg.desired_velocity,
            max_outer_iterations=self.priest_cfg.max_outer_iterations,
            max_inner_iterations=self.priest_cfg.max_inner_iterations,
            max_acceleration=self.priest_cfg.max_acceleration,
            max_velocity=self.priest_cfg.max_velocity,
            min_velocity=self.priest_cfg.min_velocity,
        )

        self.max_dyn = self.priest_cfg.num_dynamic_obstacles
        pad = self.priest_cfg.padding_distance
        self.dyn_hist = np.concatenate(
            (
                np.full((5, 2, self.max_dyn), pad, dtype=np.float32),  # x,y
                np.zeros((5, 2, self.max_dyn), dtype=np.float32),      # vx,vy
            ),
            axis=1,
        )

        self.nav_action_active = False
        self.sub_action_status = self.create_subscription(
            GoalStatusArray,
            "follow_path/_action/status",
            self._on_action_status,
            10
        )

        period = max(1e-3, 1.0 / max(1e-3, self.cfg.hz))
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().debug(
            "[rl_priest_planner ROS2 omni] ready.\n"
            f"  model={self.model_path}\n"
            f"  device={self.cfg.device} (cuda_available={torch.cuda.is_available()})\n"
            f"  hz={self.cfg.hz}, output_frame={self.cfg.frame_id}\n"
            f"  priest_enable={self.priest_cfg.enable}, dyn_msg_type={self.priest_cfg.dynamic_msg_type}\n"
            f"  rl_sample_batch={self.sample_batch}, priest.trajectory_batch_size={self.sample_batch}\n"
            f"  num_local_goals={self.cfg.num_local_goals}, local_goal_span_m={self.cfg.local_goal_span_m}"
        )

    # ---------- 回调 ----------
    def _on_odom(self, msg: Odometry):
        self.latest_odom = msg

    def _on_subgoal(self, msg: PoseStamped):
        self.subgoal_position = (msg.pose.position.x, msg.pose.position.y)

    def _on_action_status(self, msg: GoalStatusArray):
        active = False
        for status in msg.status_list:
            if status.status in [1, 2, 3]:  # ACCEPTED=1, EXECUTING=2, CANCELING=3
                active = True
                break
        self.nav_action_active = active

    def _on_scan(self, msg: LaserScan):
        self.latest_scan = msg

    def _on_points_pc(self, msg: PointCloud):
        self.latest_points_pc = msg

    def _on_points2_pc(self, msg: PointCloud2):
        self.latest_points2_pc = msg

    def _on_global_plan(self, msg: Path):
        self.latest_global_plan = msg
        if len(msg.poses) == 0:
            self.subgoal_position = None
            self._publish_empty_local_paths()

    def _publish_empty_local_paths(self):
        empty = Path()
        empty.header.frame_id = self.cfg.base_frame
        empty.header.stamp = self.get_clock().now().to_msg()
            
        self.pub_local_path.publish(empty)
        if self.pub_local_path_rl is not None:
            self.pub_local_path_rl.publish(empty)

    def _on_dynamic_obstacles(self, msg):
        self.latest_dynamic_raw = msg
        self._update_dyn_hist_once(msg)

    # ---------- 动态障碍解析（支持 MarkerArray 文本 vx,vy） ----------
    def _process_dynamic_obstacles_current(self, msg) -> np.ndarray:
        """
        返回 ndarray (N,4): [x, y, vx, vy]，坐标在 base_footprint。
        MarkerArray: 从 TEXT_VIEW_FACING 的 text 中解析 vx, vy
        """
        if msg is None:
            return np.zeros((0, 4), dtype=np.float32)

        ret = []

        if isinstance(msg, MarkerArray):
            for m in (msg.markers or []):
                if m.type != Marker.TEXT_VIEW_FACING:
                    continue
                text = (m.text or "").strip()
                if not text:
                    continue

                vx = None
                vy = None
                m_vx = re.search(r"vx\s*:\s*([+-]?\d+(?:\.\d+)?)\s*m/s", text, re.IGNORECASE)
                m_vy = re.search(r"vy\s*:\s*([+-]?\d+(?:\.\d+)?)\s*m/s", text, re.IGNORECASE)

                if m_vx:
                    vx = float(m_vx.group(1))
                if m_vy:
                    vy = float(m_vy.group(1))

                if vx is None or vy is None:
                    continue

                x = float(m.pose.position.x)
                y = float(m.pose.position.y)
                ret.append([x, y, vx, vy])

        elif TrackedPersons is not None and isinstance(msg, TrackedPersons):
            for t in (msg.tracks or []):
                ret.append([
                    t.pose.pose.position.x, t.pose.pose.position.y,
                    t.twist.twist.linear.x, t.twist.twist.linear.y
                ])

        elif AgentStates is not None and isinstance(msg, AgentStates):
            for a in (msg.agent_states or []):
                ret.append([
                    a.pose.position.x, a.pose.position.y,
                    a.twist.linear.x, a.twist.linear.y
                ])

        else:
            return np.zeros((0, 4), dtype=np.float32)

        if not ret:
            return np.zeros((0, 4), dtype=np.float32)
        return np.asarray(ret, dtype=np.float32)

    def _update_dyn_hist_once(self, msg):
        pad = self.priest_cfg.padding_distance
        arr = self._process_dynamic_obstacles_current(msg)  # (N,4) base_footprint
        if arr.shape[0] > 0:
            order = np.argsort(np.linalg.norm(arr[:, :2], axis=1))
            arr = arr[order][:self.max_dyn]

        buf = np.full((4, self.max_dyn), 0.0, dtype=np.float32)
        if arr.shape[0] > 0:
            buf[:2, :arr.shape[0]] = arr[:, :2].T
            buf[2:, :arr.shape[0]] = arr[:, 2:].T
        if arr.shape[0] < self.max_dyn:
            buf[0, arr.shape[0]:] = pad
            buf[1, arr.shape[0]:] = pad
            buf[2, arr.shape[0]:] = 0.0
            buf[3, arr.shape[0]:] = 0.0

        self.dyn_hist[:-1] = self.dyn_hist[1:]
        self.dyn_hist[-1] = buf

    # ---------- 静态障碍 ----------
    def _scan_points_ego(self) -> Optional[np.ndarray]:
        scan = self.latest_scan
        if scan is None:
            return None
        return laserscan_to_points(scan)

    def _scan_ranges_angles_ego(self) -> Optional[np.ndarray]:
        scan = self.latest_scan
        if scan is None:
            return None
        return process_laser_scan_like_script1(scan)

    # ---------- TF & Pure Pursuit ----------
    def _lookup_tf(self, target: str, source: str):
        try:
            return self.tf_buffer.lookup_transform(
                target, source, RosTime(), timeout=Duration(seconds=float(self.cfg.tf_timeout))
            )
        except Exception as e:
            self.get_logger().debug(f"TF lookup failed: {target} <- {source}: {e}")
            return None

    def _apply_tf_xy(self, pts_src: np.ndarray, tf_msg) -> np.ndarray:
        tx = tf_msg.transform.translation.x
        ty = tf_msg.transform.translation.y
        q = tf_msg.transform.rotation
        yaw = 2.0 * math.atan2(q.z, q.w)
        c, s = math.cos(yaw), math.sin(yaw)
        R = np.array([[c, -s], [s, c]], dtype=np.float64)
        pts = (R @ pts_src.T).T
        pts[:, 0] += tx
        pts[:, 1] += ty
        return pts.astype(np.float64)

    def _pp_goal_from_global_path(self) -> Optional[Tuple[float, float]]:
        gp = self.latest_global_plan
        if gp is None or len(gp.poses) == 0:
            return None

        src_frame = gp.header.frame_id or self.cfg.global_frame
        tf_to_bl = self._lookup_tf(self.cfg.base_frame, src_frame)
        if tf_to_bl is None:
            self.get_logger().debug(f"global plan TF missing: {src_frame} -> {self.cfg.base_frame}")
            return None

        xs = [ps.pose.position.x for ps in gp.poses]
        ys = [ps.pose.position.y for ps in gp.poses]
        pts_src = np.stack([xs, ys], axis=1).astype(np.float64)
        pts_bl = self._apply_tf_xy(pts_src, tf_to_bl)
        if pts_bl.shape[0] < 1:
            return None

        dists = np.hypot(pts_bl[:, 0], pts_bl[:, 1])
        idx0 = int(np.argmin(dists))

        Ld = float(max(0.05, self.cfg.pp_lookahead))
        acc = 0.0
        for i in range(idx0, pts_bl.shape[0] - 1):
            p = pts_bl[i]
            q = pts_bl[i + 1]
            seg = float(np.hypot(q[0] - p[0], q[1] - p[1]))
            if acc + seg >= Ld:
                remain = Ld - acc
                t = 0.0 if seg < 1e-6 else remain / seg
                gx = float(p[0] + t * (q[0] - p[0]))
                gy = float(p[1] + t * (q[1] - p[1]))
                return (gx, gy)
            acc += seg

        return (float(pts_bl[-1, 0]), float(pts_bl[-1, 1]))

    # ---------- 虚拟目标半径约束 ----------
    def _enforce_virtual_goal_radius(self, gxy: Tuple[float, float]) -> Tuple[float, float]:
        gx, gy = gxy
        d = math.hypot(gx, gy)
        if d <= max(1e-6, self.cfg.virt_goal_min):
            ux, uy = gx / (d + 1e-9), gy / (d + 1e-9)
            r_des = float(np.clip(self.cfg.virt_goal_pref, self.cfg.virt_goal_min, self.cfg.virt_goal_max))
            return (ux * r_des, uy * r_des)
        return (gx, gy)

    # ---------- 多局部目标采样 ----------
    def _sample_local_subgoals(self, gxy: Tuple[float, float]) -> np.ndarray:
        gx, gy = gxy
        N = max(1, int(self.cfg.num_local_goals))
        radius = float(self.cfg.local_goal_radius_m)
        angle_span_deg = float(self.cfg.local_goal_angle_deg)

        if N == 1 or angle_span_deg <= 1e-3:
            return np.array([[gx, gy]], dtype=np.float32)

        theta_center = math.atan2(gy, gx)
        half_span_rad = math.radians(angle_span_deg / 2)
        angles = np.linspace(theta_center - half_span_rad,
                             theta_center + half_span_rad,
                             N, dtype=np.float32)

        pts = np.stack([radius * np.cos(angles),
                        radius * np.sin(angles)], axis=1).astype(np.float32)
        return pts  # (N, 2)

    @staticmethod
    def _distribute_counts(total: int, groups: int) -> List[int]:
        if groups <= 0:
            return [total]
        base = total // groups
        rem = total - base * groups
        return [base + (1 if i < rem else 0) for i in range(groups)]

    # ---------- 定时主逻辑 ----------
    def _on_timer(self):
        # A cleared global plan is authoritative. Clear local output before
        # checking sensor availability so stale local paths cannot survive a
        # final waypoint or mission stop.
        if self.latest_global_plan is None or len(self.latest_global_plan.poses) == 0:
            self.subgoal_position = None
            self._publish_empty_local_paths()
            return

        odom = self.latest_odom
        if odom is None:
            self.get_logger().debug("skip: odom is None")
            return

        if self.cfg.use_scan and self.latest_scan is None:
            self.get_logger().debug("skip: scan is None")
            return

        try:
            path_rl, path_final = self._compute_local_path_and_priest()
            if self.pub_local_path_rl and path_rl is not None:
                self.pub_local_path_rl.publish(path_rl)
            if path_final is not None:
                self.pub_local_path.publish(path_final)
            else:
                self._publish_empty_local_paths()
                self.get_logger().debug("skip: compute_local_path_and_priest returned None")
        except Exception:
            self.get_logger().error("planner exception:\n" + traceback.format_exc())

    # ====== 批量 Bernstein 基与其一二阶导（无 for） ======
    def _bernstein_basis_batched(self, t: np.ndarray, n: int = 10, time_diff: float = 1.0):
        t = np.asarray(t, dtype=np.float32)
        one_minus_t = 1.0 - t

        binom10 = np.array([1, 10, 45, 120, 210, 252, 210, 120, 45, 10, 1], dtype=np.float32)
        k = np.arange(n + 1, dtype=np.float32)
        P = binom10 * (one_minus_t[..., None] ** (n - k)) * (t[..., None] ** k)

        binom9 = np.array([1, 9, 36, 84, 126, 126, 84, 36, 9, 1], dtype=np.float32)
        n1 = n - 1
        k1 = np.arange(n1 + 1, dtype=np.float32)
        Bm1 = binom9 * (one_minus_t[..., None] ** (n1 - k1)) * (t[..., None] ** k1)

        pad_left = np.pad(Bm1, [(0, 0), (0, 0), (1, 0)], mode='constant', constant_values=0.0)
        pad_right = np.pad(Bm1, [(0, 0), (0, 0), (0, 1)], mode='constant', constant_values=0.0)
        Pdot_unit = n * (pad_left - pad_right)

        binom8 = np.array([1, 8, 28, 56, 70, 56, 28, 8, 1], dtype=np.float32)
        n2 = n - 2
        k2 = np.arange(n2 + 1, dtype=np.float32)
        Bm2 = binom8 * (one_minus_t[..., None] ** (n2 - k2)) * (t[..., None] ** k2)

        Bi_2 = np.pad(Bm2, [(0, 0), (0, 0), (2, 0)], mode='constant', constant_values=0.0)
        Bi_1 = np.pad(Bm2, [(0, 0), (0, 0), (1, 1)], mode='constant', constant_values=0.0)
        Bi_0 = np.pad(Bm2, [(0, 0), (0, 0), (0, 2)], mode='constant', constant_values=0.0)
        Pddot_unit = n * (n - 1) * (Bi_2 - 2.0 * Bi_1 + Bi_0)

        denom = time_diff if time_diff != 0 else 1.0
        Pdot = Pdot_unit / denom
        Pddot = Pddot_unit / (denom ** 2)
        return P.astype(np.float32), Pdot.astype(np.float32), Pddot.astype(np.float32)

    # ====== 批量 SVD 伪逆（无 for） ======
    def _batched_pinv(self, P: np.ndarray, rcond: float = 1e-8):
        U, S, Vh = np.linalg.svd(P, full_matrices=False)
        cutoff = rcond * S[..., :1]
        Sinv = np.where(S > cutoff, 1.0 / S, 0.0).astype(P.dtype)

        Ut = np.swapaxes(U, -2, -1)
        Vt = np.swapaxes(Vh, -2, -1)

        Ut_scaled = Ut * Sinv[..., :, None]
        P_pinv = Vt @ Ut_scaled
        return P_pinv

    # ====== 完全批量版拟合（无 for） ======
    def fit_bezier11_coefficients_batch(self, x_batch: np.ndarray,
                                        y_batch: np.ndarray,
                                        time_horizon: float,
                                        rcond: float = 1e-8):
        x = np.asarray(x_batch, dtype=np.float32)
        y = np.asarray(y_batch, dtype=np.float32)

        dx = np.diff(x, axis=1, prepend=x[:, :1])
        dy = np.diff(y, axis=1, prepend=y[:, :1])
        d = np.sqrt(dx * dx + dy * dy)
        s = np.cumsum(d, axis=1)
        s = s / (s[:, -1:, ] + 1e-8)
        t_actual = s * float(time_horizon)

        t_norm = t_actual / (time_horizon if time_horizon != 0 else 1.0)
        P, Pdot, Pddot = self._bernstein_basis_batched(t_norm, n=10, time_diff=time_horizon)

        P_pinv = self._batched_pinv(P, rcond=rcond)
        c_x = (P_pinv @ x[..., None])[..., 0]
        c_y = (P_pinv @ y[..., None])[..., 0]

        P_single = P[-1]
        Pdot_list = list(Pdot)
        Pddot_list = list(Pddot)

        return c_x, c_y, P_single, Pdot_list, Pddot_list

    def fit_bezier11_coefficients(self, x: np.ndarray, y: np.ndarray, time_horizon: float):
        c_x, c_y, P_single, Pdot_list, Pddot_list = self.fit_bezier11_coefficients_batch(
            x[None, :], y[None, :], time_horizon
        )
        return c_x[0], c_y[0], P_single, Pdot_list[0], Pddot_list[0]

    # ---------- RL + PRIEST（全 batch 实现） ----------
    def _compute_local_path_and_priest(self) -> Tuple[Optional[Path], Optional[Path]]:
        from time import perf_counter

        timings_ms = {}
        t_all = perf_counter()

        def _is_cuda(dev) -> bool:
            try:
                return torch.cuda.is_available() and ("cuda" in str(dev))
            except Exception:
                return False

        _on_cuda = _is_cuda(self.cfg.device)

        def _sync_cuda():
            if _on_cuda:
                torch.cuda.synchronize(device=self.cfg.device)

        def _mark(name, t0):
            _sync_cuda()
            timings_ms[name] = (perf_counter() - t0) * 1000.0

        # 1) PP 目标
        t0 = perf_counter()
        goal_pp = self._pp_goal_from_global_path()
        _mark("pp_goal", t0)
        if goal_pp is None:
            timings_ms["total"] = (perf_counter() - t_all) * 1000.0
            return None, None

        # 2) 虚拟半径约束 (仅用于 RL 网络观测，防止近距离抖动)
        t0 = perf_counter()
        g_use_x, g_use_y = self._enforce_virtual_goal_radius(goal_pp)
        _mark("enforce_virtual_goal_radius", t0)

        # 3) 采样子目标
        t0 = perf_counter()
        sub_goals_np = self._sample_local_subgoals((g_use_x, g_use_y))
        Ng = int(sub_goals_np.shape[0])
        sub_goals = torch.from_numpy(sub_goals_np).to(torch.float32).to(self.cfg.device)
        _mark("sample_subgoals_and_to_tensor", t0)

        # 4) delta_yaw 与旋后目标
        t0 = perf_counter()
        delta_yaw = torch.atan2(sub_goals[:, 1], sub_goals[:, 0])
        rel_goal_rot = rotate_xy_tensor(sub_goals.unsqueeze(1), delta_yaw)
        _mark("delta_yaw_and_rel_goal_rot", t0)

        # 5) 扫描点批量旋转
        t0 = perf_counter()
        pts_local = self._scan_points_ego() if self.cfg.use_scan else np.zeros((0, 2), dtype=np.float32)
        pts_local_t = torch.from_numpy(pts_local).to(torch.float32).to(self.cfg.device)
        if pts_local_t.numel() > 0:
            pts_rep = pts_local_t.unsqueeze(0).expand(Ng, -1, -1)
            pts_rot = rotate_xy_tensor(pts_rep, delta_yaw)
        else:
            pts_rot = torch.zeros((Ng, 0, 2), dtype=torch.float32, device=self.cfg.device)
        _mark("scan_prepare_and_rotate_batch", t0)

        # 6) 成本图生成与膨胀
        t0 = perf_counter()
        with torch.inference_mode():
            self.costmap.generate(pts_rot)
            self.costmap.inflate()
        _mark("costmap_generate_and_inflate", t0)

        # 7) 观测构建
        t0 = perf_counter()
        if self.use_esdf:
            cm = self.costmap.inflated_costmaps
            cm = torch.clamp(cm, 0.0, self.costmap.params.esdf_clip) / self.costmap.params.esdf_clip
        else:
            cm = self.costmap.inflated_costmaps / self.costmap.params.lethal_obstacle
            cm = cm.to(torch.float32)

        costmap_flat = cm.view(Ng, -1)

        # self.costmap.debug_plot(0, True)

        dyn_flat = torch.zeros_like(costmap_flat)
        polar_goal = cartesian_to_polar(rel_goal_rot)
        normalized_goal = normalize_polar_point(
            polar_goal, r_max=self.cfg.r_max, theta_range=(self.cfg.theta_min, self.cfg.theta_max)
        )
        last_action_batch = self.last_action
        obs_batch = torch.cat([costmap_flat, dyn_flat, normalized_goal.squeeze(1), last_action_batch], dim=1)
        _mark("build_observations", t0)

        # 8) 策略前向
        t0 = perf_counter()
        self.agent.policy.set_mode("eval")
        with torch.inference_mode():
            _, _, outputs = self.agent.act(obs_batch)
        mean_actions = outputs['mean_actions']
        self.last_action = mean_actions
        _mark("policy_forward", t0)

        # 9) 动作采样与选择
        t0 = perf_counter()
        B_total = int(self.sample_batch)
        counts = self._distribute_counts(B_total, Ng)
        counts_t = torch.tensor(counts, device=self.cfg.device, dtype=torch.long)
        max_cnt = int(counts_t.max().item()) if Ng > 0 else 0

        rand_per_goal = torch.clamp(counts_t - 1, min=0)
        Kmax = int(rand_per_goal.max().item()) if Ng > 0 else 0

        with torch.inference_mode():
            rand_actions_all = self.agent.policy.distribution().rsample((Kmax,)) if Kmax > 0 \
                else torch.empty((0, Ng, mean_actions.shape[1]), dtype=mean_actions.dtype, device=mean_actions.device)

        if max_cnt == 0:
            _mark("sample_and_select_actions", t0)
            timings_ms["total"] = (perf_counter() - t_all) * 1000.0
            return None, None

        pos = torch.arange(max_cnt, device=self.cfg.device).unsqueeze(0).expand(Ng, -1)
        mask = pos < counts_t.unsqueeze(1)
        goal_idx_mat = torch.arange(Ng, device=self.cfg.device).unsqueeze(1).expand(-1, max_cnt)

        goal_idx_flat = goal_idx_mat[mask]
        pos_flat = pos[mask]
        is_mean = (pos_flat == 0)

        mean_selected = mean_actions[goal_idx_flat]
        if Kmax > 0:
            rand_by_goal = rand_actions_all.permute(1, 0, 2)
            k_idx = torch.clamp(pos_flat - 1, min=0)
            rand_selected = rand_by_goal[goal_idx_flat, k_idx]
        else:
            rand_selected = torch.zeros_like(mean_selected)

        actions_selected = torch.where(is_mean.unsqueeze(1), mean_selected, rand_selected)
        _mark("sample_and_select_actions", t0)

        # 10) 反归一化
        t0 = perf_counter()
        steps_polar = denormalize_action_tensor(
            actions_selected.clone(), max_distance=self.cfg.r_max,
            theta_range=(self.cfg.theta_min, self.cfg.theta_max)
        )
        steps_xy = polar_to_cartesian(steps_polar)
        _mark("denormalize_and_to_cartesian", t0)

        # 11) 组装轨迹点并样条重采样
        t0 = perf_counter()
        rel_goal_rot_rows = rel_goal_rot[goal_idx_flat].squeeze()
        neg_yaw_rows = -(torch.atan2(sub_goals[:, 1], sub_goals[:, 0])[goal_idx_flat])

        B = steps_xy.shape[0]
        start_pts = torch.zeros((B, 1, 2), dtype=torch.float32, device=self.cfg.device)
        goal_pts = rel_goal_rot_rows.view(B, 1, 2)
        if self.cfg.action_with_goal:
            path_pts_rel = torch.cat([start_pts, steps_xy], dim=1)
        else:
            path_pts_rel = torch.cat([start_pts, steps_xy, goal_pts], dim=1)

        num_interp = int(self.priest_cfg.trajectory_length) if self.priest_cfg.enable else int(self.cfg.num_points)
        path_pts_rel = batch_cubic_spline_interpolation(path_pts_rel, num_points=num_interp)
        _mark("spline_resample", t0)

        # 12) 旋回 base_footprint
        t0 = perf_counter()
        path_pts_bl_all = rotate_xy_tensor(path_pts_rel, neg_yaw_rows)
        _mark("rotate_back_to_base_footprint", t0)

        # 13) RL 调试路径
        t0 = perf_counter()
        path_rl_for_publish = path_pts_bl_all[0].clone()
        path_rl_for_publish[-1, 0] = float(g_use_x)
        path_rl_for_publish[-1, 1] = float(g_use_y)
        path_rl_msg = self._to_path_msg(path_rl_for_publish, self.cfg.base_frame, stamp=self.latest_odom.header.stamp)
        _mark("to_msg_rl", t0)

        # 14) 未启用 PRIEST
        if not self.priest_cfg.enable:
            timings_ms["total"] = (perf_counter() - t_all) * 1000.0
            return path_rl_msg, path_rl_msg

        # 15) PRIEST 输入
        t0 = perf_counter()
        rl_xy = path_pts_bl_all.detach().cpu().numpy().astype(np.float32)
        x_rl, y_rl = rl_xy[..., 0], rl_xy[..., 1]

        c_x, c_y, _, _, _ = self.fit_bezier11_coefficients_batch(
            x_rl, y_rl, time_horizon=self.priest_cfg.time_horizon
        )
        c_x_batch = c_x.astype(np.float32)
        c_y_batch = c_y.astype(np.float32)

        odom = self.latest_odom
        if odom is not None:
            vx0 = float(odom.twist.twist.linear.x)
            vy0 = float(odom.twist.twist.linear.y)
            ax0 = ay0 = 0.0
        else:
            vx0 = vy0 = ax0 = ay0 = 0.0

        vx0 = np.float32(vx0)
        vy0 = np.float32(vy0)
        ax0 = np.float32(ax0)
        ay0 = np.float32(ay0)

        # 优先使用外部 pure_pursuit 提供的真实目标点作为物理终点和优化目标
        if self.subgoal_position is not None:
            gx, gy = float(self.subgoal_position[0]), float(self.subgoal_position[1])
        else:
            gx, gy = float(goal_pp[0]), float(goal_pp[1])

        latest = self.dyn_hist[-1]
        dyn_x = latest[0].copy()
        dyn_y = latest[1].copy()
        dyn_vx = latest[2].copy()
        dyn_vy = latest[3].copy()

        sta_xy_all_np = self._scan_points_ego() if self.cfg.use_scan else None
        if sta_xy_all_np is None or sta_xy_all_np.shape[0] == 0:
            sta_x = sta_y = None
        else:
            sta_xy_rot_np = sta_xy_all_np
            dist = np.linalg.norm(sta_xy_rot_np, axis=1)
            idx = np.argsort(dist)[:self.priest_cfg.num_static_obstacles]
            ssel = sta_xy_rot_np[idx]
            if ssel.shape[0] < self.priest_cfg.num_static_obstacles:
                padn = self.priest_cfg.num_static_obstacles - ssel.shape[0]
                ssel = np.vstack([
                    ssel,
                    np.full((padn, 2), self.priest_cfg.padding_distance, dtype=np.float32)
                ])
            sta_x = ssel[:, 0].astype(np.float32)
            sta_y = ssel[:, 1].astype(np.float32)
        _mark("priest_prepare_inputs", t0)

        # 16) PRIEST 优化
        t0 = perf_counter()
        try:
            result = self.priest.run_optimization(
                initial_x_position=np.float32(0.0),
                initial_y_position=np.float32(0.0),
                initial_x_velocity=vx0,
                initial_y_velocity=vy0,
                initial_x_acceleration=ax0,
                initial_y_acceleration=ay0,
                goal_x_position=gx,
                goal_y_position=gy,
                dynamic_obstacle_x_positions=dyn_x,
                dynamic_obstacle_y_positions=dyn_y,
                dynamic_obstacle_x_velocities=dyn_vx,
                dynamic_obstacle_y_velocities=dyn_vy,
                static_obstacle_x_positions=sta_x,
                static_obstacle_y_positions=sta_y,
                custom_x_coefficients=c_x_batch,
                custom_y_coefficients=c_y_batch,
            )
        except Exception as e:
            _mark("priest_optimization", t0)
            self.get_logger().debug(f"PRIEST failed: {e}")
            self.get_logger().error("planner exception:\n" + traceback.format_exc())
            timings_ms["total"] = (perf_counter() - t_all) * 1000.0
            return path_rl_msg, path_rl_msg
        _mark("priest_optimization", t0)

        # 17) 最优轨迹消息
        t0 = perf_counter()
        (c_x_best, c_y_best, x_best, y_best,
         c_x_elite, c_y_elite, x_elite, y_elite, idx_min) = result

        traj_best = np.stack([x_best, y_best], axis=1).astype(np.float32)
        path_final_msg = self._to_path_msg(traj_best, self.cfg.base_frame, stamp=self.latest_odom.header.stamp)
        _mark("to_msg_final", t0)

        timings_ms["total"] = (perf_counter() - t_all) * 1000.0
        for name, duration in sorted(timings_ms.items(), key=lambda x: x[1], reverse=True):
            self.get_logger().debug(f"[timing] {name}: {duration:.2f} ms")

        return path_rl_msg, path_final_msg

    # ---------- Path 构造 ----------
    def _to_path_msg(self, xy, frame: str, stamp=None) -> Path:
        msg = Path()
        msg.header.frame_id = frame
        msg.header.stamp = stamp if stamp is not None else self.get_clock().now().to_msg()
        arr = xy.detach().cpu().numpy() if isinstance(xy, torch.Tensor) else np.asarray(xy)
        for i in range(arr.shape[0]):
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(arr[i, 0])
            ps.pose.position.y = float(arr[i, 1])
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        return msg


def main():
    rclpy.init()
    node = RLLocalPlannerNodeROS2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
