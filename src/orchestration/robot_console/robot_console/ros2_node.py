import math
import rclpy
from rclpy.node import Node
from PyQt5.QtCore import QThread, pyqtSignal, QObject
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, Header
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class RobotSignals(QObject):
    pose_updated = pyqtSignal(float, float, float, float, float, float)
    log_msg = pyqtSignal(str)
    status_updated = pyqtSignal(bool) 
    odom_distance_updated = pyqtSignal(float)
    ui_clear = pyqtSignal(bool)

class RobotBackendNode(Node):
    def __init__(self, signals):
        super().__init__('robot_ui_backend') 
        self.signals = signals
        
        self.is_localized = False
        self.total_distance = 0.0
        self.last_odom_x = None
        self.last_odom_y = None
        self.last_odom_z = None
        self.last_gui_x = None
        self.last_gui_y = None
        self.last_gui_z = None
        self.last_gui_yaw = None

        # 主动清空 RViz 可视化的发布者（停止时发送空消息）
        self.pub_clear_path_ = self.create_publisher(Path, '/global_localization/path', 10)
        self.pub_clear_scan_ = self.create_publisher(PointCloud2, '/global_localization/cur_scan', 10)
        # 匹配 Nav2 代价地图的 Transient Local QoS 策略，以确保清空消息成功送达
        costmap_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )
        self.pub_clear_local_costmap_ = self.create_publisher(OccupancyGrid, '/local_costmap/costmap', costmap_qos)
        self.pub_clear_global_costmap_ = self.create_publisher(OccupancyGrid, '/global_costmap/costmap', costmap_qos)

        # 订阅新增的纯净二维投影话题，获取绝对水平投影位姿
        self.sub_odom = self.create_subscription(
            Pose,
            '/global_localization/ui_horizontal_pose',
            self.odom_callback,
            10
        )
        
        # 订阅底层定位状态消息，用于控制轨迹更新与清除
        self.sub_status = self.create_subscription(
            Bool,
            '/localization_status',
            self.status_callback,
            10
        )

        # 订阅用于物理通知 UI 清空画布的消息
        self.sub_ui_clear = self.create_subscription(
            Bool,
            '/global_localization/ui_clear_trigger',
            self.ui_clear_callback,
            10
        )

        # 订阅与 C++ 后端一致的 aft_mapped_to_init 话题用于计算里程计路程 delta_s_
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.sub_odom_raw = self.create_subscription(
            Odometry,
            '/aft_mapped_to_init',
            self.odom_raw_callback,
            odom_qos
        )
        
        # 线程安全保护：创建 GuardCondition 用于跨线程（GUI -> ROS）安全地清除与重置
        self.nav_clear_guard = self.create_guard_condition(self.clear_nav_visualization)
        self.tf_clear_guard = self.create_guard_condition(self.clear_tf_buffer)
        
        self.signals.log_msg.emit("[SYS] ROS 2 话题订阅器已激活。等待数据流接入...")

    def clear_tf_buffer(self):
        # 兼容 UI 的重置清空按钮
        self.is_localized = False
        self.total_distance = 0.0
        self.last_odom_x = None
        self.last_gui_x = None
        self.last_gui_y = None
        self.last_gui_z = None
        self.last_gui_yaw = None
        # 主动发送空消息，同步清空 RViz 里的轨迹和点云 scan
        self._publish_clear_rviz()
        self.signals.odom_distance_updated.emit(0.0)
        self.signals.status_updated.emit(False)
        self.signals.log_msg.emit("[SYS] UI 界面缓存已手动清空。")

    def _publish_clear_rviz(self):
        """向 RViz 发送空消息，主动清除历史轨迹和点云 scan 显示"""
        now = self.get_clock().now().to_msg()
        # 清空轨迹路径
        empty_path = Path()
        empty_path.header = Header()
        empty_path.header.stamp = now
        empty_path.header.frame_id = 'map'
        self.pub_clear_path_.publish(empty_path)
        # 清空点云 scan
        empty_cloud = PointCloud2()
        empty_cloud.header = Header()
        empty_cloud.header.stamp = now
        empty_cloud.header.frame_id = 'map'
        self.pub_clear_scan_.publish(empty_cloud)

    def clear_nav_visualization(self):
        """清理 UI 专用导航可视化，不写入控制链路径 topic。"""
        now = self.get_clock().now().to_msg()
        # 清空代价地图，向 RViz 广播 1x1 大小的空栅格地图 (数据为 0 标识无障碍物)
        empty_grid = OccupancyGrid()
        empty_grid.header = Header()
        empty_grid.header.stamp = now
        empty_grid.header.frame_id = 'map'
        empty_grid.info.resolution = 0.05
        empty_grid.info.width = 1
        empty_grid.info.height = 1
        empty_grid.info.origin = Pose()
        empty_grid.data = [0]
        self.pub_clear_local_costmap_.publish(empty_grid)
        self.pub_clear_global_costmap_.publish(empty_grid)

    def status_callback(self, msg):
        was_localized = self.is_localized
        self.is_localized = msg.data
        if not self.is_localized:
            self.total_distance = 0.0
            self.last_odom_x = None
            self.last_odom_y = None
            self.last_odom_z = None
            self.last_gui_x = None
            self.last_gui_y = None
            self.last_gui_z = None
            self.last_gui_yaw = None
            self.signals.odom_distance_updated.emit(0.0)
            if was_localized:
                # 定位跟踪丢失时，物理清空所有可视化显示（历史轨迹、雷达点云、代价地图、导航绿线）
                self._publish_clear_rviz()
                self.clear_nav_visualization()
                self.signals.ui_clear.emit(True)
        else:
            if not was_localized:
                self.total_distance = 0.0
                self.last_odom_x = None
                self.last_odom_y = None
                self.last_odom_z = None
                self.last_gui_x = None
                self.last_gui_y = None
                self.last_gui_z = None
                self.last_gui_yaw = None
                self.signals.odom_distance_updated.emit(0.0)
        self.signals.status_updated.emit(msg.data)

    def ui_clear_callback(self, msg):
        if msg.data:
            self.total_distance = 0.0
            self.last_odom_x = None
            self.last_odom_y = None
            self.last_odom_z = None
            self.last_gui_x = None
            self.last_gui_y = None
            self.last_gui_z = None
            self.last_gui_yaw = None
            self.signals.odom_distance_updated.emit(0.0)
            # 物理清空所有可视化显示
            self._publish_clear_rviz()
            self.clear_nav_visualization()
            self.signals.ui_clear.emit(True)

    def odom_callback(self, msg):
        # 兜底自愈：如果收到水平投影位姿，说明底层已成功定位，但状态通知可能因 DDS 发现延迟而丢失
        if not self.is_localized:
            self.is_localized = True
            self.signals.status_updated.emit(True)
            self.signals.log_msg.emit("[SYS] 状态自愈：已通过位姿话题恢复定位状态。")
        try:
            x = msg.position.x
            y = msg.position.y
            z = msg.position.z
            q = msg.orientation

            # 校验 NaN 值的侵入，防止数学公式抛出 math domain error 并阻断订阅回调
            if (math.isnan(x) or math.isnan(y) or math.isnan(z) or 
                math.isnan(q.x) or math.isnan(q.y) or math.isnan(q.z) or math.isnan(q.w)):
                self.get_logger().warn("[SYS] 收到包含 NaN (无效值) 的位姿数据。已丢弃该帧。")
                return

            sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
            cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
            roll = math.atan2(sinr_cosp, cosr_cosp)
            sinp = 2.0 * (q.w * q.y - q.z * q.x)
            pitch = math.asin(max(-1.0, min(1.0, sinp)))
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)

            # 抑制原地不动时的微小噪声（死区判定：平移变化 >= 2cm 或 角度变化 >= 3度（0.05弧度））
            if self.last_gui_x is not None:
                dx = x - self.last_gui_x
                dy = y - self.last_gui_y
                dz = z - self.last_gui_z
                dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                dyaw = abs(yaw - self.last_gui_yaw)
                if dyaw > math.pi:
                    dyaw = 2.0 * math.pi - dyaw
                
                if dist < 0.02 and dyaw < 0.05:
                    return

            self.last_gui_x = x
            self.last_gui_y = y
            self.last_gui_z = z
            self.last_gui_yaw = yaw

            # 通知界面绘制，此时由于 x, y 已经是绝对水平投影，且携带了转正后的平展 Yaw 角度
            self.signals.pose_updated.emit(x, y, z, roll, pitch, yaw)
        except Exception as e:
            self.get_logger().error(f"[SYS] 处理里程计数据时发生错误: {str(e)}")

    def odom_raw_callback(self, msg):
        if not self.is_localized:
            self.last_odom_x = None
            self.last_odom_y = None
            self.last_odom_z = None
            return
            
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        
        if self.last_odom_x is not None:
            dx = x - self.last_odom_x
            dy = y - self.last_odom_y
            dz = z - self.last_odom_z
            ds = math.sqrt(dx*dx + dy*dy + dz*dz)
            
            # 原地微动抑制：单步累计漂移小于 2cm 时，不更新参考点，亦不计入总里程
            if ds < 0.02:
                return
                
            self.total_distance += ds
            self.signals.odom_distance_updated.emit(self.total_distance)
            
        self.last_odom_x = x
        self.last_odom_y = y
        self.last_odom_z = z

class Ros2BackendThread(QThread):
    def __init__(self):
        super().__init__()
        self.signals = RobotSignals()
        self.node = None

    def run(self):
        rclpy.init()
        self.node = RobotBackendNode(self.signals)
        try:
            rclpy.spin(self.node) 
        except Exception as e:
            self.signals.log_msg.emit(f"[SYS] 后端发生异常: {str(e)}")
        finally:
            if self.node: self.node.destroy_node()
            rclpy.shutdown()

    def reset_tf(self):
        if self.node:
            # 跨线程安全触发 GuardCondition，使 clear_tf_buffer 在 ROS 2 线程中安全执行
            self.node.tf_clear_guard.trigger()

    def clear_nav_viz(self):
        if self.node:
            # 跨线程安全触发 GuardCondition，使 clear_nav_visualization 在 ROS 2 线程中安全执行
            self.node.nav_clear_guard.trigger()

    def stop(self):
        if self.node: self.node.destroy_node(); self.node = None
        self.quit(); self.wait()
