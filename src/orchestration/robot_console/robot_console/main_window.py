import os
import sys
import signal
import subprocess
import html
import math
import yaml
from datetime import datetime
import shutil
import re
import time
import shlex
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                             QPushButton, QTextEdit, QLabel, QCheckBox, QTabWidget, 
                             QFileDialog, QDialog, QLineEdit, QDialogButtonBox, 
                             QFrame, QGraphicsView, QGraphicsScene, 
                             QGraphicsPathItem, QGraphicsPolygonItem, QGroupBox,
                             QGridLayout, QMessageBox)
from PyQt5.QtGui import QPainter, QPen, QColor, QPainterPath, QPolygonF
from PyQt5.QtCore import QProcess, Qt, QPointF, pyqtSignal, QTimer, QThread

try:
    from .log_engine import LogEngine
    from .ui_components import BagConfigDialog, BagRecordDialog, TrajectoryMap
except ImportError:
    from log_engine import LogEngine
    from ui_components import BagConfigDialog, BagRecordDialog, TrajectoryMap

class TopicHzWorker(QThread):
    hz_updated = pyqtSignal(str, str) # topic_key, hz_str

    def __init__(self, topic_key, topic_name, window, ros_distro='humble'):
        super().__init__()
        self.topic_key = topic_key
        self.topic_name = topic_name
        self.window = window
        self.ros_distro = ros_distro

        self.process = None
        self.is_running = True

    def run(self):
        cmd = f"source /opt/ros/{self.ros_distro}/setup.bash && ros2 topic hz {self.topic_name} --window {self.window}"
        try:
            self.process = subprocess.Popen(
                ['bash', '-c', cmd],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1
            )
            for line in iter(self.process.stdout.readline, ''):
                if not self.is_running:
                    break
                if 'average rate:' in line:
                    parts = line.strip().split('average rate:')
                    if len(parts) > 1:
                        try:
                            rate = float(parts[1].strip().split()[0])
                            self.hz_updated.emit(self.topic_key, f"{rate:6.2f} Hz")
                        except ValueError:
                            pass
                elif 'no new messages' in line:
                    self.hz_updated.emit(self.topic_key, "    -- Hz")
        except Exception:
            pass

    def stop(self):
        self.is_running = False
        if self.process:
            self.process.terminate()
            try:
                self.process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                self.process.kill()
        self.quit()
        self.wait()


class RobotMainWindow(QMainWindow):
    sig_reset_tf = pyqtSignal()
    sig_clear_nav = pyqtSignal()   # 停止导航时通知 ROS 后台清除 /global_path 话题残影

    def __init__(self):
        super().__init__()
        self.setWindowTitle("🚀 机器狗 高精度 SLAM 控制台 Pro")
        self.resize(1200, 800) 

        # Keep every child process on the interpreter/virtualenv used to start the UI.
        self.python_executable = os.environ.get("DOG_PATROL_PYTHON", sys.executable)

        self.log_engine = LogEngine(self.append_log)
        self.is_localized = False
        self.log_batch_buffers = {
            "ALL": [],
            "SYS": [],
            "BUILD": [],
            "CAMERA": [],
            "LIDAR": [],
            "MAP": [],
            "LOC": [],
            "NAV": [],
            "BAG": [],
            "RVIZ": []
        }
        self.log_batch_timer = QTimer(self)
        self.log_batch_timer.setInterval(100)  # 100ms
        self.log_batch_timer.timeout.connect(self.flush_log_batch)
        self.log_batch_timer.start()

        self.ws_path = self.get_workspace_path()
        self.nav_root = os.path.join(self.ws_path, 'src', 'navigation', 'fast_livo_dog')
        self.assets_root = os.environ.get(
            'DOG_PATROL_ASSETS_ROOT',
            os.path.join(
                self.ws_path,
                'src',
                'perception',
                'dog_patrol_perception_assets_20260813',
            ),
        )
        self.default_profile_path = os.path.join(self.nav_root, 'config', 'device_profiles', 'stationary.yaml')
        self.target_parameters_path = os.path.join(self.nav_root, 'config', 'device_parameters.yaml')
        self.current_profile_path = self.default_profile_path
        
        # Overwrite default target parameters with mobile profile on startup
        if os.path.exists(self.default_profile_path):
            try:
                shutil.copy(self.default_profile_path, self.target_parameters_path)
            except Exception as e:
                print(f"Startup profile copy failed: {e}")

        ros_distro = os.environ.get('ROS_DISTRO')
        if not ros_distro:
            if os.path.exists('/opt/ros'):
                distros = [d for d in os.listdir('/opt/ros') if os.path.isdir(os.path.join('/opt/ros', d))]
                if distros:
                    ros_distro = distros[0]
        if not ros_distro:
            ros_distro = 'humble'
        self.ros_distro = ros_distro

        self.env_setup = (
            f"export DOG_PATROL_NAV_ROOT={self.nav_root} && "
            f"export DOG_PATROL_ASSETS_ROOT={self.assets_root} && "
            f"export LD_LIBRARY_PATH=/opt/MVS/lib/aarch64:${{LD_LIBRARY_PATH:-}} && "
            f"cd {self.ws_path} && "
            f"source /opt/ros/{self.ros_distro}/setup.bash && "
            f"[ -f {self.ws_path}/install/setup.bash ] && source {self.ws_path}/install/setup.bash || true"
        )

        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)

        left_panel = QWidget()
        left_panel.setFixedWidth(320)
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(10, 10, 10, 10)

        self.btn_compile = self.create_btn("🔨 一键编译", "#673AB7", self.start_compile)
        lbl_s1 = QLabel("<b>[ 工作空间编译 ]</b>")
        lbl_s1.setStyleSheet("font-size: 13px; color: #333;")
        left_layout.addWidget(lbl_s1)
        left_layout.addWidget(self.btn_compile)

        lbl_s2 = QLabel("<b>[ 启动硬件驱动 ]</b>")
        lbl_s2.setStyleSheet("font-size: 13px; color: #333;")
        left_layout.addWidget(lbl_s2)
        self.btn_sensor_driver = self.create_btn("开启传感器驱动", "#00BCD4", self.toggle_sensors)
        left_layout.addWidget(self.btn_sensor_driver)

        lbl_s3 = QLabel("<b>[ 录制传感器数据 ]</b>")
        lbl_s3.setStyleSheet("font-size: 13px; color: #333;")
        left_layout.addWidget(lbl_s3)
        self.btn_record_bag = self.create_btn("🔴 录制传感器数据", "#F44336", self.start_recording)
        self.btn_record_bag.setDisabled(True)
        left_layout.addWidget(self.btn_record_bag)

        lbl_s4 = QLabel("<b>[ 选择地图读写目录 (可选) ]</b>")
        lbl_s4.setStyleSheet("font-size: 13px; color: #333;")
        left_layout.addWidget(lbl_s4)
        default_dir = os.path.join(self.nav_root, 'data')
        if not os.path.exists(default_dir):
            default_dir = os.path.join(self.ws_path, 'src', 'data')
        if not os.path.exists(default_dir):
            default_dir = os.path.expanduser('~/fastlivo_data')
            os.makedirs(default_dir, exist_ok=True)
        self.map_save_dir = default_dir + '/'
            
        self.btn_browse_dir = self.create_btn("📂 设置地图存放/读取目录", "#009688", self.browse_map_dir)
        self.lbl_map_dir = QLabel(f"{self.map_save_dir}")
        self.lbl_map_dir.setStyleSheet("color: #AAA; font-size: 11px;")
        self.lbl_map_dir.setWordWrap(True)
        
        left_layout.addWidget(self.btn_browse_dir)
        left_layout.addWidget(self.lbl_map_dir)

        lbl_s5 = QLabel("<b>[ 设备参数选择 (可选) ]</b>")
        lbl_s5.setStyleSheet("font-size: 13px; color: #333;")
        left_layout.addWidget(lbl_s5)
        self.btn_select_profile = self.create_btn("⚙️ 选择设备参数配置文件", "#9C27B0", self.select_profile)
        self.lbl_profile_path = QLabel(f"{self.current_profile_path}")
        self.lbl_profile_path.setStyleSheet("color: #AAA; font-size: 11px;")
        self.lbl_profile_path.setWordWrap(True)
        left_layout.addWidget(self.btn_select_profile)
        left_layout.addWidget(self.lbl_profile_path)

        lbl_s6 = QLabel("<b>[ 启动核心节点 ]</b>")
        lbl_s6.setStyleSheet("font-size: 13px; color: #333;")
        left_layout.addWidget(lbl_s6)

        self.chk_livo_mode = QCheckBox("启用视觉融合 (LIVO模式 / 默认开启)")
        self.chk_livo_mode.setChecked(True)
        self.chk_livo_mode.setStyleSheet("font-size: 13px; font-weight: bold; color: #333333;")
        
        core_grid = QGridLayout()
        core_grid.setHorizontalSpacing(5)
        core_grid.setVerticalSpacing(10)
        core_grid.addWidget(self.chk_livo_mode, 0, 0, 1, 6)
        
        self.btn_mapping_mid = self.create_btn("🚀 单次建图", "#8BC34A", lambda: self.start_mapping("device_parameters.yaml"))
        self.btn_mapping_cum = self.create_btn("🚀 累积建图", "#9E9E9E", lambda: None)
        self.btn_mapping_cum.setEnabled(False)
        
        self.btn_loc_mid = self.create_btn("🎯 3D定位", "#03A9F4", lambda: self.start_localization("device_parameters.yaml"))
        self.btn_loc_2d = self.create_btn("🎯 2D定位", "#9E9E9E", lambda: self.start_localization_2d("device_parameters.yaml"))
        
        self.btn_nav_3d = self.create_btn("🗺️ 3D导航", "#9E9E9E", lambda: None)
        self.btn_nav_3d.setEnabled(False)
        
        self.btn_nav_planning = self.create_btn("🗺️ 2D导航", "#9E9E9E", self.toggle_navigation)
        self.btn_nav_planning.setEnabled(False)
        
        core_grid.addWidget(self.btn_mapping_mid, 1, 0, 1, 2)
        core_grid.addWidget(self.btn_loc_mid, 1, 2, 1, 2)
        core_grid.addWidget(self.btn_nav_3d, 1, 4, 1, 2)
        
        core_grid.addWidget(self.btn_mapping_cum, 2, 0, 1, 2)
        core_grid.addWidget(self.btn_loc_2d, 2, 2, 1, 2)
        core_grid.addWidget(self.btn_nav_planning, 2, 4, 1, 2)
        
        left_layout.addLayout(core_grid)

        lbl_perception = QLabel("<b>[ 感知任务 ]</b>")
        lbl_perception.setStyleSheet("font-size: 13px; color: #333;")
        left_layout.addWidget(lbl_perception)
        self.btn_perception = self.create_btn("启动感知任务", "#00897B", self.toggle_perception)
        left_layout.addWidget(self.btn_perception)

        lbl_s7 = QLabel("<b>[ 注入传感器数据 ]</b>")
        lbl_s7.setStyleSheet("font-size: 13px; color: #333;")
        left_layout.addWidget(lbl_s7)
        self.btn_play_bag = self.create_btn("▶️ 选择并播放数据包", "#FF9800", self.play_bag_dialog)
        left_layout.addWidget(self.btn_play_bag)

        self.btn_rviz = self.create_btn("🖥️ 开启Rviz", "#9C27B0", self.toggle_rviz)
        self.btn_stop = self.create_btn("⏯ 停止节点", "#f44336", lambda: self.stop_all(keep_sensors=True))
        self.btn_clear_log = self.create_btn("🧹 清空日志", "#607D8B", self.clear_log)
        self.btn_copy_log = self.create_btn("📋 复制日志", "#3F51B5", self.copy_log) 
        self.btn_save_log = self.create_btn("💾 保存日志", "#009688", self.save_log)
        self.btn_exit = self.create_btn("❌ 退出平台", "#333333", self.exit_app)

        grid_sys_control = QGridLayout()
        grid_sys_control.setHorizontalSpacing(5)
        grid_sys_control.setVerticalSpacing(10)
        grid_sys_control.setContentsMargins(0, 0, 0, 0)
        
        # Row 0: Clear, Copy, Save (each spans 2 columns of a 6-column grid)
        grid_sys_control.addWidget(self.btn_clear_log, 0, 0, 1, 2)
        grid_sys_control.addWidget(self.btn_copy_log, 0, 2, 1, 2)
        grid_sys_control.addWidget(self.btn_save_log, 0, 4, 1, 2)
        
        # Row 1: RViz, Stop, Exit (each spans 2 columns of a 6-column grid)
        grid_sys_control.addWidget(self.btn_rviz, 1, 0, 1, 2)
        grid_sys_control.addWidget(self.btn_stop, 1, 2, 1, 2)
        grid_sys_control.addWidget(self.btn_exit, 1, 4, 1, 2)

        lbl_s8 = QLabel("<b>[ 系统控制 ]</b>")
        lbl_s8.setStyleSheet("font-size: 13px; color: #333;")
        left_layout.addWidget(lbl_s8)
        left_layout.addLayout(grid_sys_control)
        left_layout.addStretch(1)

        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(10, 10, 10, 10)

        top_status_layout = QHBoxLayout()
        self.trajectory_map = TrajectoryMap()
        top_status_layout.addWidget(self.trajectory_map, stretch=5) 
        
        hud_container = QWidget()
        hud_container.setFixedWidth(180)
        hud_container_layout = QVBoxLayout(hud_container)
        hud_container_layout.setContentsMargins(0, 0, 0, 0)
        hud_container_layout.setSpacing(10)

        # Top box: Frequencies
        freq_frame = QFrame()
        freq_frame.setStyleSheet("background-color: #1E1E1E; border-radius: 10px; padding: 5px;")
        freq_layout = QVBoxLayout(freq_frame)
        freq_layout.setSpacing(5)
        
        self.lbl_freq_img = self.create_hud_label("IMG:    -- Hz", "#FF5722", font_size=15, padding=7)
        self.lbl_freq_lidar = self.create_hud_label("LDR:    -- Hz", "#00E5FF", font_size=15, padding=7)
        self.lbl_freq_imu = self.create_hud_label("IMU:    -- Hz", "#CDDC39", font_size=15, padding=7)
        
        freq_layout.addStretch(1)
        freq_layout.addWidget(self.lbl_freq_img)
        freq_layout.addWidget(self.lbl_freq_lidar)
        freq_layout.addWidget(self.lbl_freq_imu)
        freq_layout.addStretch(1)

        # Bottom box: Coordinates
        coord_frame = QFrame()
        coord_frame.setStyleSheet("background-color: #1E1E1E; border-radius: 10px; padding: 5px;")
        coord_layout = QVBoxLayout(coord_frame)
        coord_layout.setSpacing(5)
        
        self.lbl_x = self.create_hud_label("  X:   0.00 m", "#4CAF50", font_size=15, padding=7)
        self.lbl_y = self.create_hud_label("  Y:   0.00 m", "#2196F3", font_size=15, padding=7)
        self.lbl_z = self.create_hud_label("  Z:   0.00 m", "#9C27B0", font_size=15, padding=7)
        line = QFrame(); line.setFrameShape(QFrame.HLine); line.setStyleSheet("color: #444;")
        self.lbl_roll = self.create_hud_label("  R:   0.00 °", "#FFC107", font_size=15, padding=7)
        self.lbl_pitch = self.create_hud_label("  P:   0.00 °", "#E91E63", font_size=15, padding=7)
        self.lbl_yaw = self.create_hud_label("  Y:   0.00 °", "#FF9800", font_size=15, padding=7)
        line2 = QFrame(); line2.setFrameShape(QFrame.HLine); line2.setStyleSheet("color: #444;")
        self.lbl_dist = self.create_hud_label("Dst:   0.00 m", "#00BCD4", font_size=15, padding=7)
        
        coord_layout.addStretch(1)
        coord_layout.addWidget(self.lbl_x)
        coord_layout.addWidget(self.lbl_y)
        coord_layout.addWidget(self.lbl_z)
        coord_layout.addWidget(line)
        coord_layout.addWidget(self.lbl_roll)
        coord_layout.addWidget(self.lbl_pitch)
        coord_layout.addWidget(self.lbl_yaw)
        coord_layout.addWidget(line2)
        coord_layout.addWidget(self.lbl_dist)
        coord_layout.addStretch(1)
        
        # Add to container with different stretch proportions
        hud_container_layout.addWidget(freq_frame, stretch=0)
        hud_container_layout.addWidget(coord_frame, stretch=1)
        
        top_status_layout.addWidget(hud_container, stretch=1) 

        self.log_tabs = QTabWidget()
        self.log_tabs.setStyleSheet("""
            QTabWidget::pane { border: 1px solid #444; border-radius: 4px; }
            QTabBar::tab { background: #333; color: #CCC; padding: 4px 8px; border-top-left-radius: 4px; border-top-right-radius: 4px; margin-right: 2px; font-size: 11px; }
            QTabBar::tab:selected { background: #121212; color: #FFF; font-weight: bold; border-bottom: 2px solid #00BCD4; }
        """)

        self.log_consoles = {}
        tab_configs = [
            ("ALL", "ALL"),
            ("SYS", "SYS"),
            ("BUILD", "BUILD"),
            ("CAMERA", "CAMERA"),
            ("LIDAR", "LIDAR"),
            ("MAP", "MAP"),
            ("LOC", "LOC"),
            ("NAV", "NAV"),
            ("BAG", "BAG"),
            ("RVIZ", "RVIZ")
        ]

        for tab_name, key in tab_configs:
            console = QTextEdit()
            console.setReadOnly(True)
            console.setLineWrapMode(QTextEdit.WidgetWidth)
            console.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
            console.document().setMaximumBlockCount(2000)
            console.setStyleSheet("background-color: #121212; color: #E0E0E0; font-family: Consolas, monospace; font-size: 13px; border: none; padding: 10px;")
            self.log_tabs.addTab(console, tab_name)
            self.log_consoles[key] = console

        self.log_console = self.log_consoles["ALL"]

        lbl_traj = QLabel("<b>base_link实时轨迹与坐标</b> <font color='#757575' size='2'>(以建图起点时的body中心为原点)</font>")
        lbl_traj.setStyleSheet("font-size: 13px; color: #333;")
        right_layout.addWidget(lbl_traj)
        right_layout.addLayout(top_status_layout, stretch=3) 
        
        log_title = QLabel("<b>系统终端日志监控</b>")
        log_title.setStyleSheet("font-size: 13px; color: #333;")
        right_layout.addWidget(log_title)
        right_layout.addWidget(self.log_tabs, stretch=2) 

        main_layout.addWidget(left_panel)
        main_layout.addWidget(right_panel, stretch=1)

        self.process_compile = None 
        self.process_front = None
        self.process_back = None
        self.process_bag = None  
        self.camera_process = None
        self.lidar_process = None
        self.rviz_process = None
        self.bag_process = None
        self.is_handling_fault = False
        self.sensor_state = "DISCONNECTED"
        self.is_recording = False
        self.process_nav = None
        self.is_nav_running = False
        self.process_perception = None
        self.is_perception_running = False
        self.process_manager = None
        self._closing = False
        self._manager_should_run = True
        self.loc_mode = None
        self.running_processes = set()

        self.update_ui_state("IDLE")
        self.append_log(f"[SYS] 界面加载完毕。当前工作空间: {self.ws_path}")
        self.start_hz_workers()
        QTimer.singleShot(0, self.start_mission_manager)

    def start_hz_workers(self):
        self.hz_workers = {
            'img': TopicHzWorker('img', '/left_camera/image', 10, self.ros_distro),
            'lidar': TopicHzWorker('lidar', '/livox/lidar', 10, self.ros_distro),
            'imu': TopicHzWorker('imu', '/livox/imu', 200, self.ros_distro)
        }
        for worker in self.hz_workers.values():
            worker.hz_updated.connect(self.update_hz_display)
            worker.start()

    def update_hz_display(self, topic_key, hz_str):
        # 如果硬件驱动没开，且系统处于空闲状态（没在录包、没在播放包），强制忽略底层缓冲发来的旧频率
        if self.sensor_state != "RUNNING" and self.current_state == "IDLE":
            hz_str = "    -- Hz"
            
        if topic_key == 'img':
            self.lbl_freq_img.setText(f"IMG: {hz_str}")
        elif topic_key == 'lidar':
            self.lbl_freq_lidar.setText(f"LDR: {hz_str}")
        elif topic_key == 'imu':
            self.lbl_freq_imu.setText(f"IMU: {hz_str}")

    def get_workspace_path(self):
        path = os.path.abspath(os.path.dirname(__file__))
        while path != '/' and path != '':
            if os.path.basename(path) == 'src':
                return os.path.dirname(path)
            path = os.path.dirname(path)
        return os.getcwd()

    def create_tracked_process(self):
        proc = QProcess()
        self.running_processes.add(proc)
        proc.finished.connect(lambda exit_code, exit_status, p=proc: self.running_processes.discard(p) if p in self.running_processes else None)
        return proc

    def create_btn(self, text, color, slot=None):
        btn = QPushButton(text)
        btn.setFixedHeight(40)
        btn.setCursor(Qt.PointingHandCursor)
        btn.setStyleSheet(f"font-size: 13px; padding: 6px; background-color: {color}; color: white; border-radius: 6px; font-weight: bold;")
        if slot: btn.clicked.connect(slot)
        return btn

    def create_hud_label(self, text, color, font_size=18, padding=20):
        lbl = QLabel(text)
        lbl.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        lbl.setStyleSheet(f"font-size: {font_size}px; font-weight: bold; color: {color}; font-family: 'Consolas', 'Courier New', monospace; padding-left: {padding}px; white-space: pre;")
        return lbl

    def update_hud(self, x, y, z, roll, pitch, yaw):
        if self.current_state == "IDLE":
            return
            
        self.lbl_x.setText(f"  X: {x:6.2f} m")
        self.lbl_y.setText(f"  Y: {y:6.2f} m")
        self.lbl_z.setText(f"  Z: {z:6.2f} m")
        self.lbl_roll.setText(f"  R: {math.degrees(roll):6.2f} °")
        self.lbl_pitch.setText(f"  P: {math.degrees(pitch):6.2f} °")
        self.lbl_yaw.setText(f"  Y: {math.degrees(yaw):6.2f} °")

        self.trajectory_map.add_pose(x, y, yaw)

    def update_odom_distance(self, distance):
        if self.current_state == "IDLE":
            return
        self.lbl_dist.setText(f"Dst: {distance:5.2f} m")

    def handle_localization_status(self, is_localized):
        self.is_localized = is_localized
        if not is_localized:
            # 一旦底层撤销定位或未成功，轨迹和 HUD 立刻同时消失归零
            self.trajectory_map.clear_map()
            self.lbl_x.setText("  X:   0.00 m")
            self.lbl_y.setText("  Y:   0.00 m")
            self.lbl_z.setText("  Z:   0.00 m")
            self.lbl_roll.setText("  R:   0.00 °")
            self.lbl_pitch.setText("  P:   0.00 °")
            self.lbl_yaw.setText("  Y:   0.00 °")
            self.lbl_dist.setText("Dst:   0.00 m")

            # 定位丢失且导航正在运行时，自动中止并清空导航
            if self.is_nav_running:
                self.append_log("[SYS] 检测到定位跟踪失败！已自动安全中止并重置导航规划。")
                self.stop_navigation()

    def ui_clear_callback(self, data_bool):
        if data_bool:
            self.trajectory_map.clear_map()
            if hasattr(self, 'path_data'):
                self.path_data.clear()
            if hasattr(self, 'update_canvas'):
                self.update_canvas()

    def browse_map_dir(self):
        dir_path = QFileDialog.getExistingDirectory(self, "选择地图读写目录", self.map_save_dir)
        if dir_path:
            if not dir_path.endswith('/'):
                dir_path += '/'
            self.map_save_dir = dir_path
            self.lbl_map_dir.setText(f"{self.map_save_dir}")
            
            # 自动拷贝视觉词典文件到新目录，防止建图/定位失败
            target_vocab = os.path.join(self.map_save_dir, 'orb_vocabulary.dbow3')
            if not os.path.exists(target_vocab):
                source_vocab = os.path.join(self.nav_root, 'data', 'orb_vocabulary.dbow3')
                if os.path.exists(source_vocab):
                    try:
                        shutil.copy(source_vocab, target_vocab)
                        self.append_log(f"[SYS] 已自动将视觉词典文件拷贝至新地图目录。")
                    except Exception as e:
                        self.append_log(f"[SYS] 拷贝视觉词典文件失败: {e}")
                else:
                    self.append_log(f"[SYS] 视觉词典初始化失败：找不到默认文件 ({source_vocab})，请检查工作空间完整性！")

    def set_btn_style(self, btn, color, enabled):
        btn.setEnabled(enabled)
        if enabled:
            btn.setStyleSheet(f"font-size: 13px; padding: 6px; background-color: {color}; color: white; border-radius: 6px; font-weight: bold;")
        else:
            btn.setStyleSheet(f"font-size: 13px; padding: 6px; background-color: #333333; color: #888888; border-radius: 6px;")

    def setup_process_env(self, process):
        import platform
        from PyQt5.QtCore import QProcessEnvironment
        
        custom_env = os.environ.copy()
        custom_env["DOG_PATROL_NAV_ROOT"] = self.nav_root
        custom_env["DOG_PATROL_ASSETS_ROOT"] = self.assets_root
        mvs_lib = "/opt/MVS/lib/aarch64"
        current_ld_path = custom_env.get("LD_LIBRARY_PATH", "")
        if mvs_lib not in current_ld_path.split(":"):
            custom_env["LD_LIBRARY_PATH"] = (
                f"{mvs_lib}:{current_ld_path}" if current_ld_path else mvs_lib
            )
        # 为所有子进程保留/应用专属的 UDPv4 配置文件，避免频繁开关导航时发生 FastDDS 共享内存 (SHM) 端口锁冲突与卡顿
        udp_profile_path = os.path.join(self.nav_root, "config", "fastdds_udp_only.xml")
        if os.path.exists(udp_profile_path):
            custom_env["FASTRTPS_DEFAULT_PROFILES_FILE"] = udp_profile_path
        # 从环境变量中读取继承过来的 ROS_DOMAIN_ID，保持与主进程一致，避免与机器狗底层话题产生冲突
        custom_env["ROS_DOMAIN_ID"] = os.environ.get("ROS_DOMAIN_ID", "42")
        # 强制关闭 ROS 2 的日志缓冲（特别是 C++ 节点），保证实时吐出日志
        custom_env["RCUTILS_LOGGING_BUFFERED_STREAM"] = "0"
        # 强制关闭 Python 程序的标准输出缓冲
        custom_env["PYTHONUNBUFFERED"] = "1"
        custom_env["DOG_PATROL_PYTHON"] = self.python_executable
        python_dir = os.path.dirname(self.python_executable)
        if os.path.isabs(self.python_executable) and os.path.isdir(python_dir):
            custom_env["PATH"] = (
                f"{python_dir}:{custom_env.get('PATH', '')}"
            )
            custom_env["VIRTUAL_ENV"] = os.path.dirname(python_dir)
            
        cpu_arch = platform.machine()
        
        if cpu_arch == "aarch64":
            custom_env["LD_PRELOAD"] = "/lib/aarch64-linux-gnu/libusb-1.0.so.0"
        else:
            if "LD_PRELOAD" in custom_env:
                custom_env["LD_PRELOAD"] = "/lib/x86_64-linux-gnu/libusb-1.0.so.0"
                
        # 强制使 ROS 2 日志输出到 stdout 而非 stderr (避免 INFO 等正常日志被误判定为 ERR)
        custom_env["RCUTILS_LOGGING_USE_STDOUT"] = "1"
            
        q_env = QProcessEnvironment()
        for k, v in custom_env.items():
            q_env.insert(k, v)
        process.setProcessEnvironment(q_env)
        
        self.append_log(f"[SYS] 跨平台运行期环境就绪：检测到架构为 [{cpu_arch}]，已自动软适配 LD_PRELOAD 路径。")

    def update_ui_state(self, state):
        self.current_state = state
        
        self.btn_copy_log.setEnabled(True)
        self.btn_copy_log.setStyleSheet("font-size: 12px; padding: 5px; background-color: #3F51B5; color: white; border-radius: 6px; font-weight: bold;")
        self.btn_clear_log.setEnabled(True)
        self.btn_clear_log.setStyleSheet("font-size: 12px; padding: 5px; background-color: #607D8B; color: white; border-radius: 6px; font-weight: bold;")

        is_idle = (state == "IDLE")
        is_ready = (state == "CORE_READY")
        is_recording = self.is_recording
        
        self.set_btn_style(self.btn_compile, "#673AB7", is_idle and not is_recording)
        self.set_btn_style(self.btn_browse_dir, "#009688", is_idle and not is_recording)
        self.set_btn_style(self.btn_select_profile, "#9C27B0", is_idle and not is_recording)
        self.set_btn_style(self.btn_save_log, "#009688", is_idle)
        
        is_sensor_active = (self.camera_process is not None or self.lidar_process is not None)
        if self.sensor_state == "RUNNING":
            can_stop_sensors = is_idle and not is_recording
            self.btn_sensor_driver.setEnabled(can_stop_sensors)
            if can_stop_sensors:
                self.btn_sensor_driver.setText("关闭传感器驱动")
                self.btn_sensor_driver.setStyleSheet("font-size: 13px; padding: 6px; background-color: #E91E63; color: white; border-radius: 6px; font-weight: bold;")
            else:
                self.btn_sensor_driver.setText("传感器驱动已开启")
                self.btn_sensor_driver.setStyleSheet("font-size: 13px; padding: 6px; background-color: #4CAF50; color: white; border-radius: 6px; font-weight: bold;")
            
            if is_recording:
                self.btn_record_bag.setEnabled(False)
                self.btn_record_bag.setText("🔴 正在录制传感器数据...")
                self.btn_record_bag.setStyleSheet("font-size: 13px; padding: 6px; background-color: #E91E63; color: white; border-radius: 6px; font-weight: bold;")
            else:
                self.btn_record_bag.setEnabled(is_idle)
                self.btn_record_bag.setText("🔴 录制传感器数据")
                if is_idle:
                    self.btn_record_bag.setStyleSheet("font-size: 13px; padding: 6px; background-color: #F44336; color: white; border-radius: 6px; font-weight: bold;")
                else:
                    self.btn_record_bag.setStyleSheet("font-size: 13px; padding: 6px; background-color: #333333; color: #888888; border-radius: 6px;")
        else:
            self.btn_sensor_driver.setText("开启传感器驱动")
            self.set_btn_style(self.btn_sensor_driver, "#00BCD4", is_idle and not is_sensor_active and not is_recording)
            
            self.btn_record_bag.setEnabled(False)
            self.btn_record_bag.setText("🔴 录制传感器数据")
            self.btn_record_bag.setStyleSheet("font-size: 13px; padding: 6px; background-color: #333333; color: #888888; border-radius: 6px;")

        self.set_btn_style(self.btn_mapping_mid, "#8BC34A", is_idle and not is_recording)
        self.set_btn_style(self.btn_loc_mid, "#03A9F4", is_idle and not is_recording)
        self.set_btn_style(self.btn_loc_2d, "#4CAF50", is_idle and not is_recording)
        
        is_2d_loc_active = (self.loc_mode == "2D" and state in ["CORE_READY", "PLAYING"])
        if is_2d_loc_active:
            if self.is_nav_running:
                self.btn_nav_planning.setText("⏹ 停止2D导航")
                self.set_btn_style(self.btn_nav_planning, "#F44336", True)
            else:
                self.btn_nav_planning.setText("🗺️ 2D导航")
                self.set_btn_style(self.btn_nav_planning, "#FF5722", True)
        else:
            self.btn_nav_planning.setText("🗺️ 2D导航")
            self.btn_nav_planning.setEnabled(False)
            self.btn_nav_planning.setStyleSheet("font-size: 13px; padding: 6px; background-color: #333333; color: #888888; border-radius: 6px;")

        if self.is_perception_running:
            self.btn_perception.setText("停止感知任务")
            self.set_btn_style(self.btn_perception, "#F44336", True)
        else:
            self.btn_perception.setText("启动感知任务")
            self.set_btn_style(self.btn_perception, "#00897B", not is_recording)

        self.set_btn_style(self.btn_play_bag, "#FF9800", is_ready and not is_recording and self.sensor_state != "RUNNING")



    def _process_output_data(self, data, prefix, is_stderr):
        if not data: return
        lines = data.split('\n')
        merged_lines = []
        for line in lines:
            stripped = line.strip()
            # 只要是标准 ROS 日志起手（[INFO], [node-1] 等），就认为是新的一条日志
            if stripped.startswith('['):
                merged_lines.append(line)
            else:
                if merged_lines:
                    merged_lines[-1] += '\n' + line
                else:
                    merged_lines.append(line)
        for ml in merged_lines:
            if ml.strip():
                self.log_engine.parse_and_append_log(ml, prefix=prefix, is_stderr=is_stderr)

    def handle_process_out(self, proc, prefix=""):
        data = proc.readAllStandardOutput().data().decode('utf-8', errors='replace').strip('\r\n')
        self._process_output_data(data, prefix, is_stderr=False)

    def handle_process_err(self, proc, prefix=""):
        data = proc.readAllStandardError().data().decode('utf-8', errors='replace').strip('\r\n')
        self._process_output_data(data, prefix, is_stderr=True)

    def clean_log_text(self, text):
        # 1. Remove ANSI escape codes
        text = re.sub(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])', '', text)
        # 2. Keep only printable ASCII, common Chinese characters, degree symbol, and standard spacing
        allowed_pattern = re.compile(r'[^\x20-\x7E\u4e00-\u9fa5\u3000-\u303f\uff00-\uffef\u00b0\n\r\t]')
        text = allowed_pattern.sub('', text)
        return text

    def append_log(self, text, category="ALL"):
        # 将手工拼接的 [SYS] 内部日志路由到核心上色引擎，实现前缀和正文双色分离
        if "[SYS]" in text and not any(lvl in text for lvl in ["[INFO]", "[WARN]", "[ERROR]", "[FATAL]"]):
            # 强制统一 [SYS] 的前缀颜色为白色，最为醒目
            prefix_color = "#FFFFFF"
            
            # 剥离所有 HTML 标签并去掉 [SYS] 关键字，得到纯净正文
            clean_text = re.sub(r"<[^>]+>", "", text)
            clean_text = clean_text.replace("[SYS]", "").strip()
            
            # 根据语境推断日志等级
            level = "[INFO]"
            upper_text = clean_text.upper()
            if "FAIL" in upper_text or "ERR" in upper_text or "异常" in upper_text or "失败" in upper_text:
                level = "[ERROR]"
            elif "WARN" in upper_text or "NAN" in upper_text:
                level = "[WARN]"
                
            # 将干净的正文（附带推断的等级）重新扔给解析引擎，强行加上我们分离后的颜色前缀
            self.log_engine.parse_and_append_log(
                f"{level} {clean_text}", 
                prefix=f"<font color='{prefix_color}'>[SYS]</font> ", 
                is_stderr=(level == "[ERROR]")
            )
            return
        cleaned_text = self.clean_log_text(text)
        self.log_batch_buffers["ALL"].append(cleaned_text)
        if category != "ALL" and category in self.log_batch_buffers:
            self.log_batch_buffers[category].append(cleaned_text)

    def flush_log_batch(self):
        for key, buffer in self.log_batch_buffers.items():
            if buffer:
                merged_html = "<br>".join(buffer)
                buffer.clear()
                console = self.log_consoles[key]
                console.append(merged_html)
                scrollbar = console.verticalScrollBar()
                if scrollbar:
                    scrollbar.setValue(scrollbar.maximum())

    def copy_log(self):
        text = self.log_tabs.currentWidget().toPlainText()
        max_chars = 500000 
        if len(text) > max_chars:
            text = f"......\n[提示：日志过长，已为您截取最新的 {max_chars} 个字符]\n\n" + text[-max_chars:]
        
        QApplication.clipboard().setText(text)
        self.append_log("[SYS] 终端日志已成功复制到剪贴板！")

    def toggle_sensors(self):
        if self.sensor_state == "RUNNING":
            self.stop_sensor_drivers()
        else:
            self.start_sensors()

    def start_sensors(self):
        self.sensor_state = "RUNNING"
        self.update_ui_state(self.current_state)
        
        self.append_log("[SYS] 开始开启传感器驱动...")
        self.append_log("[SYS] 正在启动相机驱动...")
        
        cmd_camera = f"{self.env_setup} && exec ros2 run mvs_ros_driver grabImgWithTrigger {self.nav_root}/mvs_ros_driver/config/left_camera_trigger.yaml"
        self.camera_process = self.create_tracked_process()
        self.setup_process_env(self.camera_process)
        self.camera_process.readyReadStandardOutput.connect(lambda: self.handle_process_out(self.camera_process, "[CAMERA] "))
        self.camera_process.readyReadStandardError.connect(lambda: self.handle_process_err(self.camera_process, "[CAMERA] "))
        self.camera_process.start("bash", ["-c", cmd_camera])
        
        QTimer.singleShot(5000, self.start_lidar)

    def start_lidar(self):
        if self.sensor_state == "DISCONNECTED":
            return
        self.append_log("[SYS] 正在启动雷达驱动...")
        
        cmd_lidar = f"{self.env_setup} && exec ros2 launch livox_ros_driver2 msg_MID360_launch.py"
        self.lidar_process = self.create_tracked_process()
        self.setup_process_env(self.lidar_process)
        self.lidar_process.readyReadStandardOutput.connect(lambda: self.handle_process_out(self.lidar_process, "[LIDAR] "))
        self.lidar_process.readyReadStandardError.connect(lambda: self.handle_process_err(self.lidar_process, "[LIDAR] "))
        self.lidar_process.start("bash", ["-c", cmd_lidar])
        
        self.update_ui_state(self.current_state)

    def stop_sensor_drivers(self):
        self.sensor_state = "DISCONNECTED"
        cam_proc = self.camera_process
        lid_proc = self.lidar_process
        
        self.camera_process = None
        self.lidar_process = None
        
        if cam_proc and cam_proc.state() == QProcess.Running:
            self.append_log("[SYS] 正在停止相机驱动...")
            cam_proc.terminate()
            if not cam_proc.waitForFinished(2000):
                cam_proc.kill()
                cam_proc.waitForFinished(1000)
                
        if lid_proc and lid_proc.state() == QProcess.Running:
            self.append_log("[SYS] 正在停止雷达驱动...")
            lid_proc.terminate()
            if not lid_proc.waitForFinished(2000):
                lid_proc.kill()
                lid_proc.waitForFinished(1000)
        
        try:
            # 补刀残留的僵尸驱动进程（应对部分驱动 C++ 节点在收到信号后死锁不退出的情况）
            self.safe_kill_processes(["livox_ros_driver", "mvs_ros_driver"], 1000)
        except:
            pass
            
        # 延迟清理 FastDDS 残留共享内存，防止驱动重启时因幽灵端口导致 Segfault
        # 必须在进程退出后再清，故用 singleShot 延迟 500ms 执行
        QTimer.singleShot(500, self._cleanup_fastdds_shm)
                
        self.append_log("[SYS] 传感器驱动已安全停止并重置。")
        
        self.lbl_freq_img.setText("IMG:    -- Hz")
        self.lbl_freq_lidar.setText("LDR:    -- Hz")
        self.lbl_freq_imu.setText("IMU:    -- Hz")
        
        self.update_ui_state(self.current_state)

    def _cleanup_fastdds_shm(self):
        """
        智能安全清理残留的 FastDDS 共享内存和信号量锁文件。
        利用 fuser 检测共享内存占用，并根据活跃端口安全判定信号量文件的清理，避免误杀活着的节点（如定位、驱动）的信号量通信。
        """
        import glob
        import os
        import subprocess
        
        count = 0
        # 1. 扫描所有的 FastDDS 共享内存文件
        shm_files = glob.glob("/dev/shm/fastrtps_*")
        active_ports = set()
        
        # 第一遍：记录所有还在被进程占用的活跃端口
        for path in shm_files:
            try:
                res = subprocess.run(["fuser", path], capture_output=True, text=True)
                if res.returncode == 0:
                    # 正在被占用
                    match = re.search(r"fastrtps_port(\d+)", path)
                    if match:
                        active_ports.add(match.group(1))
            except Exception:
                pass

        # 第二遍：清理所有不再被占用，且不属于活跃端口的文件（包括孤立的 _el 文件）
        for path in shm_files:
            try:
                # 再次确认当前文件是否关联于活跃端口
                match = re.search(r"fastrtps_port(\d+)", path)
                if match and match.group(1) in active_ports:
                    continue
                
                # 双重保险：检查当前文件本身是否还在被占用（防止非 port 文件误删）
                res = subprocess.run(["fuser", path], capture_output=True, text=True)
                if res.returncode != 0:
                    if os.path.exists(path):
                        os.remove(path)
                        count += 1
            except Exception:
                pass
                 
        # 2. 扫描所有信号量文件，结合活跃端口状态判定是否可以删除
        sem_files = glob.glob("/dev/shm/sem.fastrtps_*")
        for path in sem_files:
            try:
                match = re.search(r"sem\.fastrtps_port(\d+)_mutex", path)
                if match:
                     port = match.group(1)
                     if port in active_ports:
                         # 对应的 port 处于活跃状态，绝对不能清理此信号量
                         continue
                
                # 如果没活跃关联，或本身就没 port 名字，再次检查是否在用
                res = subprocess.run(["fuser", path], capture_output=True, text=True)
                if res.returncode != 0:
                     if os.path.exists(path):
                         os.remove(path)
                         count += 1
            except Exception:
                pass
                
        if count > 0:
            self.append_log(f"[SYS] 自动扫描并安全清理了 {count} 个残留的 FastDDS 共享内存/信号量缓存。")


    def start_recording(self):
        dialog = BagRecordDialog(self.ws_path, self)
        if dialog.exec_() != QDialog.Accepted:
            self.append_log("[SYS] 用户取消录制配置，录制未启动。")
            return
            
        selected_dir = dialog.get_save_dir()
        if not selected_dir or not os.path.exists(selected_dir):
            self.append_log("[SYS] ❌ 选择的保存目录无效或不存在。")
            return
            
        lidar_imu_only = dialog.is_lidar_imu_only()
        self.bag_auto_compress = dialog.is_auto_compress()
        
        import datetime
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_dir_name = f"bag_{timestamp}"
        self.record_bag_path = os.path.join(selected_dir, bag_dir_name)
        
        # 立即更新状态锁，灰掉按钮，进入录制状态
        self.is_recording = True
        self.update_ui_state(self.current_state)
        
        self.append_log(f"[SYS] 开始录制传感器数据...")
        self.append_log(f"[SYS] 正在保存至: {self.record_bag_path}")

        self.start_bag_record_process(lidar_imu_only)

    def start_bag_record_process(self, lidar_imu_only=False):
        if not self.is_recording:
            return
        self.append_log("[SYS] 正在拉起 ros2 bag record 进行录制...")
        
        if lidar_imu_only:
            topics = "/livox/lidar /livox/imu"
            self.append_log("[SYS] 模式: 仅录制雷达和 IMU")
        else:
            topics = "/livox/lidar /livox/imu /left_camera/image"
            self.append_log("[SYS] 模式: 全量录制 (含图像)")
            
        cmd_bag = f"{self.env_setup} && exec ros2 bag record {topics} -o {self.record_bag_path}"
        self.bag_process = self.create_tracked_process()
        self.setup_process_env(self.bag_process)
        self.bag_process.readyReadStandardOutput.connect(lambda: self.handle_process_out(self.bag_process, "[BAG] "))
        self.bag_process.readyReadStandardError.connect(lambda: self.handle_process_err(self.bag_process, "[BAG] "))
        self.bag_process.start("bash", ["-c", cmd_bag])

    def start_compile(self):
        self.stop_all(keep_sensors=False) 
        self.update_ui_state("COMPILING")
        self.append_log("[SYS] 开始编译工作空间...")
        python_cmd = shlex.quote(self.python_executable)
        cmd = (
            f"cd {shlex.quote(self.ws_path)} && "
            f"source /opt/ros/{self.ros_distro}/setup.bash && "
            "rm -rf build/ install/ log/ && "
            f"{python_cmd} -m colcon build "
            "--cmake-args -DCMAKE_BUILD_TYPE=Release "
            "-DROS_EDITION=ROS2 --no-warn-unused-cli"
        )
        self.process_compile = self.create_tracked_process()
        self.setup_process_env(self.process_compile)
        self.process_compile.readyReadStandardOutput.connect(lambda: self.handle_process_out(self.process_compile, "[BUILD] "))
        self.process_compile.readyReadStandardError.connect(lambda: self.handle_process_err(self.process_compile, "[BUILD] "))
        self.process_compile.finished.connect(self.compile_finished)
        self.process_compile.start("bash", ["-c", cmd])

    def compile_finished(self, exitCode, exitStatus):
        now = datetime.now()
        ts_str = f"[{now.strftime('%H:%M:%S')}.{int(now.microsecond/1000):03d}]"
        if exitCode == 0:
            self.log_engine.parse_and_append_log("[INFO] Build completed successfully.", prefix="[BUILD] ", is_stderr=False)
        else:
            self.log_engine.parse_and_append_log(f"[ERROR] Build failed with exit code: {exitCode}", prefix="[BUILD] ", is_stderr=True)
        self.update_ui_state("IDLE")

    def start_dual_processes(self, front_launch, back_launch, mode=None):
        self.stop_all(keep_sensors=True) 
        self.loc_mode = mode
        self.log_current_parameters()
        
        # 自动判定是否为仿真时间 (若传感器未开启，则使用仿真时间)
        use_sim = "true" if self.sensor_state != "RUNNING" else "false"
        
        # 动态追加 use_sim_time 参数
        if "use_sim_time:=" not in front_launch:
            front_launch += f" use_sim_time:={use_sim}"
        if "use_sim_time:=" not in back_launch:
            back_launch += f" use_sim_time:={use_sim}"
            
        self.append_log(f"[SYS] 启动前端: {front_launch}")
        self.append_log(f"[SYS] 启动后端: {back_launch}")

        cmd_front = f"{self.env_setup} && exec {front_launch}"
        cmd_back = f"{self.env_setup} && exec stdbuf -oL -eL {back_launch}"

        self.process_front = self.create_tracked_process()
        self.setup_process_env(self.process_front)
        self.process_front.start("bash", ["-c", cmd_front])

        self.process_back = self.create_tracked_process()
        self.setup_process_env(self.process_back)
        if self.loc_mode == "MAPPING":
            pref = "[MAP] "     # 回归经典的护眼绿 (原INFO颜色)
            err_pref = "[MAP] "
        else:
            pref = "[LOC] "     # 回归经典的舒适蓝
            err_pref = "[LOC] "
        self.process_back.readyReadStandardOutput.connect(lambda p=pref: self.handle_process_out(self.process_back, p))
        self.process_back.readyReadStandardError.connect(lambda p=err_pref: self.handle_process_err(self.process_back, p))
        self.process_back.start("bash", ["-c", cmd_back])
        
        self.update_ui_state("CORE_READY")

    def apply_img_en(self, config_yaml):
        config_path = os.path.join(self.nav_root, 'config', config_yaml)
        if not os.path.exists(config_path):
            config_path = os.path.join(self.ws_path, 'src', 'config', config_yaml)
        if os.path.exists(config_path):
            try:
                import re
                with open(config_path, 'r', encoding='utf-8') as f:
                    content = f.read()
                val = "1" if self.chk_livo_mode.isChecked() else "0"
                # Replace img_en: 1 or img_en: 0
                new_content = re.sub(r'(img_en:\s*)[01]', rf'\g<1>{val}', content)
                with open(config_path, 'w', encoding='utf-8') as f:
                    f.write(new_content)
                self.append_log(f"[SYS] 已自动切换至 {'LIVO (视觉辅助)' if val=='1' else 'LIO (纯雷达)'} 模式 (img_en={val})")
            except Exception as e:
                self.append_log(f"[SYS] 更新 img_en 失败: {e}")

    def start_mapping(self, config_yaml):
        self.apply_img_en(config_yaml)
        config_path = os.path.join(self.nav_root, 'config', config_yaml)
        if not os.path.exists(config_path):
            config_path = os.path.join(self.ws_path, 'src', 'config', config_yaml)
        self.start_dual_processes(
            f"ros2 launch fast_livo frontend.launch.py rviz:=false params_file:={config_path}",
            f"ros2 launch mapping backend_mapping.launch.py map_save_dir:={self.map_save_dir}",
            mode="MAPPING"
        )

    def start_localization(self, config_yaml):
        self.apply_img_en(config_yaml)
        config_path = os.path.join(self.nav_root, 'config', config_yaml)
        if not os.path.exists(config_path):
            config_path = os.path.join(self.ws_path, 'src', 'config', config_yaml)
        self.start_dual_processes(
            f"ros2 launch fast_livo frontend.launch.py rviz:=false params_file:={config_path}",
            f"ros2 launch localization backend_localization_3d.launch.py map_save_dir:={self.map_save_dir}",
            mode="3D"
        )

    def start_localization_2d(self, config_yaml):
        self.apply_img_en(config_yaml)
        config_path = os.path.join(self.nav_root, 'config', config_yaml)
        if not os.path.exists(config_path):
            config_path = os.path.join(self.ws_path, 'src', 'config', config_yaml)
        self.start_dual_processes(
            f"ros2 launch fast_livo frontend.launch.py rviz:=false params_file:={config_path}",
            f"ros2 launch localization backend_localization_2d.launch.py map_save_dir:={self.map_save_dir}",
            mode="2D"
        )

    def play_bag_dialog(self):
        dialog = BagConfigDialog(self)
        if dialog.exec_() == QDialog.Accepted:
            bag_path, use_speed, speed_val, use_random = dialog.get_config()
            if bag_path:
                # 组装基础命令
                cmd_list = ["ros2", "bag", "play", bag_path, "--clock", "--remap", "/tf:=/tf_bag_garbage", "/tf_static:=/tf_static_bag_garbage"]
                
                # 处理倍速
                if use_speed:
                    cmd_list.extend(["-r", f"{speed_val:.2f}"])
                
                # 处理随机时刻
                if use_random:
                    # 获取持续时长
                    duration_sec = 0.0
                    metadata_path = os.path.join(bag_path, "metadata.yaml")
                    if os.path.exists(metadata_path):
                        try:
                            with open(metadata_path, 'r', encoding='utf-8') as f:
                                meta = yaml.safe_load(f)
                            nanos = meta.get('rosbag2_bagfile_information', {}).get('duration', {}).get('nanoseconds', 0)
                            if nanos > 0:
                                duration_sec = nanos / 1e9
                        except Exception as e:
                            print(f"Error parsing metadata.yaml: {e}")
                    
                    if duration_sec <= 0.0:
                        try:
                            res = subprocess.run(["ros2", "bag", "info", bag_path], capture_output=True, text=True, timeout=5)
                            match = re.search(r"Duration:\s*([\d\.]+)s", res.stdout)
                            if match:
                                duration_sec = float(match.group(1))
                        except Exception as e:
                            print(f"Error running ros2 bag info: {e}")
                    
                    t_start = 0.0
                    if duration_sec > 0.0:
                        import random
                        t_start = random.uniform(0.0, duration_sec * 0.75)
                    
                    cmd_list.extend(["--start-offset", f"{t_start:.2f}"])
                    self.append_log(f"[SYS] 检测到数据包时长为 {duration_sec:.2f} 秒，随机截取起始时刻：{t_start:.2f} 秒。")

                import shlex
                cmd_str = shlex.join(cmd_list)
                
                self.append_log(f"[SYS] 执行数据包回放命令: {cmd_str}")
                self.print_bag_info(bag_path)
                full_cmd = f"{self.env_setup} && exec stdbuf -oL -eL {cmd_str}"
                
                self.process_bag = self.create_tracked_process()
                self.setup_process_env(self.process_bag)
                self.process_bag.readyReadStandardOutput.connect(lambda: self.handle_process_out(self.process_bag, "[BAG] "))
                self.process_bag.readyReadStandardError.connect(lambda: self.handle_process_err(self.process_bag, "[BAG] "))
                self.process_bag.start("bash", ["-c", full_cmd])
                self.update_ui_state("PLAYING")

    def print_bag_info(self, bag_path):
        if not bag_path or not os.path.exists(bag_path):
            return
        self.log_engine.parse_and_append_log("[INFO] Analyzing bag information, please wait...", prefix="[BAG] ", is_stderr=False)
        try:
            res = subprocess.run(["ros2", "bag", "info", bag_path], capture_output=True, text=True, timeout=10)
            if res.returncode == 0 and res.stdout:
                self.log_engine.parse_and_append_log(f"[INFO] Bag Information ({os.path.basename(bag_path)}):", prefix="[BAG] ", is_stderr=False)
                for line in res.stdout.strip().split('\n'):
                    if line.strip():
                        self.log_engine.parse_and_append_log(f"[INFO]   {line}", prefix="[BAG] ", is_stderr=False)
            else:
                self.log_engine.parse_and_append_log(f"[ERROR] Failed to get bag information: {res.stderr.strip()}", prefix="[BAG] ", is_stderr=True)
        except Exception as e:
            self.log_engine.parse_and_append_log(f"[ERROR] Error reading bag information: {e}", prefix="[BAG] ", is_stderr=True)

    def get_dir_size_str(self, path):
        total_size = 0
        if not os.path.exists(path):
            return "0 B"
        for dirpath, _, filenames in os.walk(path):
            for f in filenames:
                fp = os.path.join(dirpath, f)
                if not os.path.islink(fp):
                    total_size += os.path.getsize(fp)
        for unit in ['B', 'KB', 'MB', 'GB', 'TB']:
            if total_size < 1024.0:
                return f"{total_size:.1f} {unit}"
            total_size /= 1024.0
        return f"{total_size:.1f} PB"

    def start_bag_compression(self, bag_path):
        self.log_engine.parse_and_append_log("[INFO] ==========================================", prefix="[BAG] ", is_stderr=False)
        self.log_engine.parse_and_append_log("[INFO] Starting lossless compression of ROS 2 bag...", prefix="[BAG] ", is_stderr=False)
        
        bag_path = bag_path.rstrip('/')
        output_bag = f"{bag_path}_compressed"
        tmp_yaml = os.path.join(os.path.dirname(bag_path), f"convert_tmp_{int(time.time())}.yaml")
        
        self.log_engine.parse_and_append_log(f"[INFO] Input path: {bag_path}", prefix="[BAG] ", is_stderr=False)
        self.log_engine.parse_and_append_log(f"[INFO] Output path: {output_bag}", prefix="[BAG] ", is_stderr=False)
        self.log_engine.parse_and_append_log("[INFO] ==========================================", prefix="[BAG] ", is_stderr=False)
        
        if os.path.exists(output_bag):
            shutil.rmtree(output_bag)
            
        yaml_content = f"output_bags:\n  - uri: \"{output_bag}\"\n    storage_id: \"mcap\"\n    all: true\n    storage_preset_profile: \"zstd_fast\"\n"
        try:
            with open(tmp_yaml, "w", encoding='utf-8') as f:
                f.write(yaml_content)
        except Exception as e:
            self.log_engine.parse_and_append_log(f"[ERROR] Failed to generate temporary YAML: {e}", prefix="[BAG] ", is_stderr=True)
            return
            
        cmd_compress = f"{self.env_setup} && exec ros2 bag convert -i {bag_path} -o {tmp_yaml}"
        self.compress_process = self.create_tracked_process()
        self.setup_process_env(self.compress_process)
        self.compress_process.readyReadStandardOutput.connect(lambda: self.handle_process_out(self.compress_process, "[BAG] "))
        self.compress_process.readyReadStandardError.connect(lambda: self.handle_process_err(self.compress_process, "[BAG] "))
        self.compress_process.finished.connect(
            lambda exitCode, exitStatus: self.compress_finished(exitCode, bag_path, output_bag, tmp_yaml)
        )
        self.compress_process.start("bash", ["-c", cmd_compress])

    def compress_finished(self, exitCode, bag_path, output_bag, tmp_yaml):
        if os.path.exists(tmp_yaml):
            os.remove(tmp_yaml)
            
        if exitCode == 0 and os.path.exists(output_bag):
            self.log_engine.parse_and_append_log("[INFO] Compression complete!", prefix="[BAG] ", is_stderr=False)
            self.log_engine.parse_and_append_log("[INFO] ------------------------------------------", prefix="[BAG] ", is_stderr=False)
            self.log_engine.parse_and_append_log("[INFO] Size comparison:", prefix="[BAG] ", is_stderr=False)
            
            orig_size = self.get_dir_size_str(bag_path)
            comp_size = self.get_dir_size_str(output_bag)
            
            self.log_engine.parse_and_append_log(f"[INFO] {orig_size} -> {bag_path}", prefix="[BAG] ", is_stderr=False)
            self.log_engine.parse_and_append_log(f"[INFO] {comp_size} -> {output_bag}", prefix="[BAG] ", is_stderr=False)
            self.log_engine.parse_and_append_log("[INFO] ------------------------------------------", prefix="[BAG] ", is_stderr=False)
            self.log_engine.parse_and_append_log("[INFO] Play command:", prefix="[BAG] ", is_stderr=False)
            self.log_engine.parse_and_append_log(f"[INFO] ros2 bag play {output_bag}", prefix="[BAG] ", is_stderr=False)
            self.log_engine.parse_and_append_log("[INFO] ==========================================", prefix="[BAG] ", is_stderr=False)
            self.log_engine.parse_and_append_log("[INFO] Deleting uncompressed original data...", prefix="[BAG] ", is_stderr=False)
            
            try:
                shutil.rmtree(bag_path)
            except Exception as e:
                self.log_engine.parse_and_append_log(f"[ERROR] Failed to delete original data: {e}", prefix="[BAG] ", is_stderr=True)
                
            self.print_bag_info(output_bag)
        else:
            self.log_engine.parse_and_append_log(f"[ERROR] Compression failed (Exit Code {exitCode}). Kept original bag data, please check logs above!", prefix="[BAG] ", is_stderr=True)
            self.print_bag_info(bag_path)

    def clear_log(self): 
        current_console = self.log_tabs.currentWidget()
        if current_console:
            current_console.clear()
            for key, console in self.log_consoles.items():
                if console == current_console:
                    self.log_batch_buffers[key].clear()
                    break
    
    def save_log(self):
        path, _ = QFileDialog.getSaveFileName(self, "保存日志", os.path.expanduser("~/robot_log.txt"), "Text (*.txt)")
        if path:
            current_console = self.log_tabs.currentWidget()
            if current_console:
                with open(path, 'w', encoding='utf-8') as f: 
                    f.write(current_console.toPlainText())

    def safe_kill_processes(self, patterns, delay_ms=500):
        """
        Safely shuts down processes matching the given patterns.
        First, it captures the PIDs of matching processes.
        It filters out the GUI's own PID and parent PID to prevent self-killing.
        Then, it sends SIGINT (2) to those PIDs to allow graceful shutdown.
        After delay_ms, it checks which of those pre-identified PIDs are still running,
        and sends SIGKILL (9) to force termination of ONLY those lingering processes.
        This call is synchronous (using processEvents) to guarantee clean process environments.
        """
        import time
        target_pids = set()
        own_pid = os.getpid()
        parent_pid = os.getppid()
        
        for pattern in patterns:
            try:
                res = subprocess.run(["pgrep", "-f", pattern], capture_output=True, text=True)
                for line in res.stdout.strip().split('\n'):
                    if line.strip():
                        pid = int(line.strip())
                        if pid != own_pid and pid != parent_pid:
                            target_pids.add(pid)
            except Exception as e:
                print(f"Error querying PIDs for pattern {pattern}: {e}")
                
        if not target_pids:
            return
            
        self.append_log(f"[SYS] 发送 SIGINT 信号优雅停止进程，目标 PIDs: {list(target_pids)}</font>")
        for pid in target_pids:
            try:
                os.kill(pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            except Exception as e:
                print(f"Error sending SIGINT to PID {pid}: {e}")
                
        # 阻塞等待进程退出，保持 UI 响应
        start_time = time.time()
        timeout = delay_ms / 1000.0
        while time.time() - start_time < timeout:
            lingering_pids = []
            for pid in target_pids:
                try:
                    os.kill(pid, 0)
                    lingering_pids.append(pid)
                except ProcessLookupError:
                    pass
            if not lingering_pids:
                break
            QApplication.processEvents()
            time.sleep(0.02)

        # 再次检查，如果仍有残留，则发送 SIGKILL 强杀
        lingering_pids = []
        for pid in target_pids:
            try:
                os.kill(pid, 0)
                lingering_pids.append(pid)
            except ProcessLookupError:
                pass
                
        if lingering_pids:
            self.append_log(f"[SYS] 检测到 {len(lingering_pids)} 个进程超时未响应 SIGINT，执行 SIGKILL 强制关闭，PIDs: {lingering_pids}</font>")
            for pid in lingering_pids:
                try:
                    os.kill(pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                except Exception as e:
                    print(f"Error sending SIGKILL to PID {pid}: {e}")
            
            # 给 SIGKILL 些许时间生效
            kill_start = time.time()
            while time.time() - kill_start < 0.2:
                still_alive = []
                for pid in lingering_pids:
                    try:
                        os.kill(pid, 0)
                        still_alive.append(pid)
                    except ProcessLookupError:
                        pass
                if not still_alive:
                    break
                QApplication.processEvents()
                time.sleep(0.01)

    def stop_qprocess(self, proc):
        if proc and proc.state() == QProcess.Running:
            pid = proc.processId()
            try: os.kill(pid, signal.SIGINT)
            except: pass
            if not proc.waitForFinished(3000): proc.kill(); proc.waitForFinished(1000)

    def start_mission_manager(self):
        """Start the single process that owns the global mission state."""
        if self._closing or not self._manager_should_run:
            return
        if self.process_manager and self.process_manager.state() == QProcess.Running:
            return
        self.append_log("[SYS] 正在启动常驻任务管理器...")
        process = QProcess(self)
        self.process_manager = process
        self.setup_process_env(process)
        process.readyReadStandardOutput.connect(
            lambda p=process: self.handle_process_out(p, "[SYS] ")
        )
        process.readyReadStandardError.connect(
            lambda p=process: self.handle_process_err(p, "[SYS] ")
        )
        process.finished.connect(self._mission_manager_finished)
        cmd = (
            f"{self.env_setup} && exec ros2 launch dog_patrol_manager "
            "mission_supervisor.launch.py use_sim_time:=false"
        )
        process.start("bash", ["-c", cmd])

    def _mission_manager_finished(self, exit_code, exit_status):
        del exit_status
        if not self._closing:
            self.append_log(
                f"[SYS] 任务管理器已退出，exit_code={exit_code}；业务状态协调已暂停。"
            )
        self.process_manager = None
        if not self._closing and self._manager_should_run:
            QTimer.singleShot(1000, self.start_mission_manager)

    def reset_mission_session(self, reason):
        """Invalidate readiness/target state while keeping supervisor alive."""
        if self._closing or not self.process_manager:
            return
        env = os.environ.copy()
        env["DOG_PATROL_NAV_ROOT"] = self.nav_root
        env["DOG_PATROL_ASSETS_ROOT"] = self.assets_root
        env["ROS_DOMAIN_ID"] = os.environ.get("ROS_DOMAIN_ID", "42")
        command = (
            f"{self.env_setup} && ros2 service call /mission/reset "
            "std_srvs/srv/Trigger '{}'"
        )
        try:
            subprocess.Popen(
                ["bash", "-c", command],
                env=env,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            self.append_log(f"[SYS] 已请求重置任务会话：{reason}")
        except OSError as exc:
            self.append_log(f"[SYS] 任务会话重置请求失败：{exc}")

    def toggle_perception(self):
        if self.is_perception_running:
            self.stop_perception()
        else:
            self.start_perception()

    def start_perception(self):
        if self.process_perception and self.process_perception.state() == QProcess.Running:
            return
        self.append_log("[SYS] 正在启动感知任务（tracking / face / voice）...")
        use_sim = "true" if self.sensor_state != "RUNNING" else "false"
        cmd = (
            f"{self.env_setup} && exec ros2 launch dog_patrol_perception_bringup "
            f"perception_stack.launch.py use_sim_time:={use_sim} "
            f"assets_root:={self.assets_root} image_topic:=/left_camera/image_raw "
            "preview:=true"
        )
        process = QProcess(self)
        self.process_perception = process
        self.setup_process_env(process)
        process.readyReadStandardOutput.connect(
            lambda p=process: self.handle_process_out(p, "[PERCEPTION] ")
        )
        process.readyReadStandardError.connect(
            lambda p=process: self.handle_process_err(p, "[PERCEPTION] ")
        )
        process.finished.connect(self._perception_finished)
        process.start("bash", ["-c", cmd])
        self.is_perception_running = True
        self.update_ui_state(self.current_state)

    def _perception_finished(self, exit_code, exit_status):
        del exit_status
        if not self._closing:
            self.append_log(
                f"[PERCEPTION] 感知任务已退出，exit_code={exit_code}。"
            )
            self.reset_mission_session("感知任务退出")
        self.process_perception = None
        self.is_perception_running = False
        self.update_ui_state(self.current_state)

    def stop_perception(self):
        if self.process_perception and self.process_perception.state() == QProcess.Running:
            self.append_log("[SYS] 正在停止感知任务...")
            self.stop_qprocess(self.process_perception)
        self.process_perception = None
        self.is_perception_running = False
        self.safe_kill_processes(
            [
                "dog_patrol_perception_tracking_node",
                "perception_face_readiness",
                "perception_face_provider",
                "perception_voice_readiness",
                "perception_voice_provider",
                "perception_readiness",
                "perception_authorization",
            ],
            2500,
        )
        self.reset_mission_session("感知任务关闭")
        self.update_ui_state(self.current_state)

    def stop_all(self, keep_sensors=None):
        self.is_recording = False
        if keep_sensors is None:
            keep_sensors = (self.sensor_state == "RUNNING")
            
        if not keep_sensors:
            self.stop_sensor_drivers()
            self.stop_perception()
        
        # 联动关闭录包 QProcess
        if self.bag_process and self.bag_process.state() == QProcess.Running:
            self.append_log("[SYS] 正在停止数据包录制，等待写入包尾索引...")
            self.bag_process.terminate()
            if not self.bag_process.waitForFinished(3000):
                self.bag_process.kill()
                self.bag_process.waitForFinished(1000)
            if hasattr(self, 'record_bag_path') and self.record_bag_path:
                bag_path_to_check = self.record_bag_path
                if getattr(self, 'bag_auto_compress', False):
                    QTimer.singleShot(1000, lambda: self.start_bag_compression(bag_path_to_check))
                else:
                    QTimer.singleShot(1000, lambda: self.print_bag_info(bag_path_to_check))
        self.bag_process = None

        # 联动关闭 RViz2 QProcess
        if self.rviz_process and self.rviz_process.state() == QProcess.Running:
            self.append_log("[SYS] 正在关闭原生 RViz2 窗口...")
            self.rviz_process.terminate()
            if not self.rviz_process.waitForFinished(2000):
                self.rviz_process.kill()
                self.rviz_process.waitForFinished(1000)
        self.rviz_process = None
        self.btn_rviz.setText("🖥️ 开启Rviz")

        if not keep_sensors:
            pass
        
        self.stop_qprocess(self.process_compile)
        self.stop_qprocess(self.process_front)
        self.stop_qprocess(self.process_back)
        self.stop_qprocess(self.process_bag)
        had_navigation = bool(
            self.process_nav and self.process_nav.state() == QProcess.Running
        )
        self.stop_qprocess(self.process_nav)
        self.process_nav = None
        self.is_nav_running = False
        self.loc_mode = None
        if had_navigation:
            self.reset_mission_session("导航链关闭")
        
        # 释放所有被追踪的 QProcess 资源
        for proc in list(self.running_processes):
            if proc.state() == QProcess.Running:
                proc.terminate()
                if not proc.waitForFinished(1000):
                    proc.kill()
        self.running_processes.clear()

        try:
            # 建图/定位/导航衍生进程 - 始终杀死，不影响硬件驱动
            kill_list = [
                "colcon",
                "ros2 bag play",
                "fastlivo_mapping",
                "global_localization_cpp",
                "lidar_loop_node",
                "lidar_global_locator",
                "visual_global_locator",
                "map_publisher_cpp",
                "odom_bridge_node",
                "transform_fusion_cpp",
                "navigation_launch",
                "pure_pursuit_controller",
                "priest_rl_controller",
                "frontend.launch.py",
                "backend_mapping.launch.py",
                "backend_localization_3d.launch.py",
                "backend_localization_2d.launch.py",
                "map_server",
                "lifecycle_manager",
                "controller_server",
                "planner_server",
                "bt_navigator",
                "recoveries_server",
                "behavior_server",
                "waypoint_follower",
            ]
            if not keep_sensors:
                # 只有在完全退出时才杀掉 RViz 和 ros2 launch 父进程
                kill_list.extend(["rviz2", "ros2 launch"])
            self.safe_kill_processes(kill_list, 10000)
        except: pass

        # 清理 FastRTPS 残留共享内存与信号量文件，避免 FastDDS 报错（无论是否保留传感器，都智能清理已死节点的残留）
        self._cleanup_fastdds_shm()
        QTimer.singleShot(1000, self._cleanup_fastdds_shm)

        self.trajectory_map.clear_map() 
        
        self.lbl_x.setText("  X:   0.00 m")
        self.lbl_y.setText("  Y:   0.00 m")
        self.lbl_z.setText("  Z:   0.00 m")
        self.lbl_roll.setText("  R:   0.00 °")
        self.lbl_pitch.setText("  P:   0.00 °")
        self.lbl_yaw.setText("  Y:   0.00 °")
        self.lbl_dist.setText("Dst:   0.00 m")
        if not keep_sensors or self.sensor_state != "RUNNING":
            self.lbl_freq_img.setText("IMG:    -- Hz")
            self.lbl_freq_lidar.setText("LDR:    -- Hz")
            self.lbl_freq_imu.setText("IMU:    -- Hz")

        self.sig_reset_tf.emit()
        
        self.append_log("[SYS] 系统已重置，历史数据清理完毕。")
        self.update_ui_state("IDLE")

    def exit_app(self):
        self.append_log("[SYS] 正在强制安全退出...")
        self.stop_all(keep_sensors=False)
        
        # 强制暂停 0.5 秒，等待系统回收刚刚被 SIGKILL 杀掉的僵尸进程
        import time
        time.sleep(0.5)
        
        # 退出前执行最后一次终极清理，防止遗漏
        self._cleanup_fastdds_shm()
        
        os._exit(0) 



    def select_profile(self):
        profiles_dir = os.path.abspath(os.path.join(self.nav_root, 'config', 'device_profiles'))
        if not os.path.exists(profiles_dir):
            QMessageBox.warning(self, "警告", f"参数配置模板目录不存在:\n{profiles_dir}")
            return

        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "选择设备参数配置文件",
            profiles_dir,
            "YAML 配置文件 (*.yaml *.yml)"
        )
        if not file_path:
            return

        # 限制只能在 device_profiles 目录下选择
        abs_file_path = os.path.abspath(file_path)
        if not abs_file_path.startswith(profiles_dir):
            QMessageBox.critical(self, "错误", f"只能在以下参数配置目录下选择文件:\n{profiles_dir}")
            return

        selected_file = os.path.basename(abs_file_path)
        try:
            with open(abs_file_path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
            
            if not isinstance(data, dict):
                raise ValueError("YAML 格式无效，必须是字典结构。")
            
            # 校验是否为合法的机器人参数配置文件 (检查核心外参键值)
            params = data.get('/**', {}).get('ros__parameters', {})
            extrin = params.get('extrin_calib', {})
            if 'T_imu_lidar' not in extrin or 'T_camera_lidar' not in extrin:
                raise ValueError("文件内容缺失核心标定参数 (T_imu_lidar 或 T_camera_lidar)。")

            shutil.copy(abs_file_path, self.target_parameters_path)
            self.current_profile_path = abs_file_path
            self.lbl_profile_path.setText(f"{self.current_profile_path}")
            self.append_log(f"[SYS] 已成功加载并覆盖参数配置文件: {selected_file}")
        except Exception as e:
            QMessageBox.critical(self, "错误", f"配置文件校验失败，未能加载参数:\n{str(e)}")
            self.append_log(f"[SYS] 应用参数文件 {selected_file} 失败: {str(e)}</font>")
    def log_current_parameters(self):
        try:
            if not os.path.exists(self.target_parameters_path):
                return
            with open(self.target_parameters_path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
            
            params = data.get('/**', {}).get('ros__parameters', {})
            extrin = params.get('extrin_calib', {})
            
            t_imu_lid = extrin.get('T_imu_lidar', [])
            t_cam_lid = extrin.get('T_camera_lidar', [])
            
            fusion_params = data.get('transform_fusion', {}).get('ros__parameters', {})
            t_lid_base = fusion_params.get('T_lidar_base', [])
            if not t_lid_base:
                t_lid_base = data.get('/**', {}).get('ros__parameters', {}).get('T_lidar_base', [])
            
            profile_name = os.path.basename(self.current_profile_path)
            self.append_log(f"<b>[SYS] 加载硬件参数模板: {profile_name}</b>")
            
            def parse_matrix(flat_list):
                if not flat_list or len(flat_list) != 16:
                    return None
                # translation
                tx, ty, tz = flat_list[3], flat_list[7], flat_list[11]
                # rotation matrix
                r00, r01, r02 = flat_list[0], flat_list[1], flat_list[2]
                r10, r11, r12 = flat_list[4], flat_list[5], flat_list[6]
                r20, r21, r22 = flat_list[8], flat_list[9], flat_list[10]
                
                # Euler angles (roll, pitch, yaw) in degrees
                sy = math.sqrt(r00 * r00 + r10 * r10)
                singular = sy < 1e-6
                if not singular:
                    roll = math.atan2(r21, r22)
                    pitch = math.atan2(-r20, sy)
                    yaw = math.atan2(r10, r00)
                else:
                    roll = math.atan2(-r12, r11)
                    pitch = math.atan2(-r20, sy)
                    yaw = 0.0
                
                return (tx, ty, tz), (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))

            # Print T_camera_lidar
            res_cam = parse_matrix(t_cam_lid)
            if res_cam:
                t, r = res_cam
                self.append_log(f"[SYS] 相机 -> 雷达 外参 T_camera_lidar:")
                self.append_log(f"[SYS]    - 平移 (Translation): X={t[0]:.6f} m, Y={t[1]:.6f} m, Z={t[2]:.6f} m")
                self.append_log(f"[SYS]    - 旋转 (Euler RPY): Roll={r[0]:.2f}°, Pitch={r[1]:.2f}°, Yaw={r[2]:.2f}°")
            
            # Print T_lidar_base
            res_lid = parse_matrix(t_lid_base)
            if res_lid:
                t, r = res_lid
                self.append_log(f"[SYS] 雷达 -> 车体 外参 T_lidar_base:")
                self.append_log(f"[SYS]    - 平移 (Translation): X={t[0]:.6f} m, Y={t[1]:.6f} m, Z={t[2]:.6f} m")
                self.append_log(f"[SYS]    - 旋转 (Euler RPY): Roll={r[0]:.2f}°, Pitch={r[1]:.2f}°, Yaw={r[2]:.2f}°")
            
            # Print T_imu_lidar
            res_imu = parse_matrix(t_imu_lid)
            if res_imu:
                t, r = res_imu
                self.append_log(f"[SYS] IMU -> 雷达 外参 T_imu_lidar:")
                self.append_log(f"[SYS]    - 平移 (Translation): X={t[0]:.6f} m, Y={t[1]:.6f} m, Z={t[2]:.6f} m")
                self.append_log(f"[SYS]    - 旋转 (Euler RPY): Roll={r[0]:.2f}°, Pitch={r[1]:.2f}°, Yaw={r[2]:.2f}°")

        except Exception as e:
            self.append_log(f"[SYS] 读取或解析参数失败: {e}")

    def toggle_navigation(self):
        if not self.is_nav_running:
            self.start_navigation()
        else:
            self.stop_navigation()

    def toggle_rviz(self):
        if self.rviz_process and self.rviz_process.state() == QProcess.Running:
            self.append_log("[SYS] 正在关闭 RViz2 窗口...")
            self.rviz_process.terminate()
            if not self.rviz_process.waitForFinished(1000):
                self.rviz_process.kill()
            self.rviz_process = None
            self.btn_rviz.setText("🖥️ 开启Rviz")
        else:
            if self.loc_mode == "MAPPING":
                rviz_cfg = "src/navigation/fast_livo_dog/mapping/rviz_cfg/mapping.rviz"
                self.append_log("[SYS] 正在启动 建图专属 RViz2...")
                cmd_rviz = f"{self.env_setup} && exec rviz2 -d {self.ws_path}/{rviz_cfg}"
            elif self.loc_mode == "3D":
                rviz_cfg = "src/navigation/fast_livo_dog/localization/rviz_cfg/localization_3d.rviz"
                self.append_log("[SYS] 正在启动 3D定位专属 RViz2...")
                cmd_rviz = f"{self.env_setup} && exec rviz2 -d {self.ws_path}/{rviz_cfg}"
            elif self.loc_mode == "2D" or self.is_nav_running:
                rviz_cfg = "src/navigation/fast_livo_dog/localization/rviz_cfg/localization_2d.rviz"
                self.append_log("[SYS] 正在启动 2D定位/导航专属 RViz2...")
                cmd_rviz = f"{self.env_setup} && exec rviz2 -d {self.ws_path}/{rviz_cfg}"
            else:
                self.append_log("[SYS] 正在启动 系统原生 RViz2...")
                cmd_rviz = f"{self.env_setup} && exec rviz2"

            self.rviz_process = self.create_tracked_process()
            self.setup_process_env(self.rviz_process)
            self.rviz_process.readyReadStandardOutput.connect(lambda: self.handle_process_out(self.rviz_process, "[RVIZ] "))
            self.rviz_process.readyReadStandardError.connect(lambda: self.handle_process_err(self.rviz_process, "[RVIZ] "))
            
            # 监听进程结束信号，不论是被 UI 杀掉还是用户点 X 手动关闭，都自动恢复按钮文字
            self.rviz_process.finished.connect(lambda: self.btn_rviz.setText("🖥️ 开启Rviz"))
            
            self.rviz_process.start("bash", ["-c", cmd_rviz])
            self.btn_rviz.setText("🚫 关闭Rviz")

    def start_navigation(self):
        if self.loc_mode != "2D":
            QMessageBox.warning(self, "警告", "请先启动并成功运行 2D 定位，再开启导航功能！")
            return
            
        self.append_log("[SYS] 开始启动 Nav2 规划与导航...")
        self.append_log("[SYS] 正在启动 Nav2 导航节点与控制器节点...")
        
        use_sim = "true" if self.sensor_state != "RUNNING" else "false"
        
        # move/navigation.launch.py 同时拉起 Nav2 控制链和导航任务协调器。
        cmd = (
            f"{self.env_setup} && exec ros2 launch move "
            f"navigation.launch.py use_sim_time:={use_sim} "
            f"params_file:={self.nav_root}/config/nav_parameters.yaml"
        )
        
        self.process_nav = self.create_tracked_process()
        self.setup_process_env(self.process_nav)
        self.process_nav.readyReadStandardOutput.connect(lambda: self.handle_process_out(self.process_nav, "[NAV] "))
        self.process_nav.readyReadStandardError.connect(lambda: self.handle_process_err(self.process_nav, "[NAV] "))
        self.process_nav.start("bash", ["-c", cmd])
        
        self.is_nav_running = True
        self.update_ui_state(self.current_state)
        self.append_log("[SYS] Nav2 导航与路线控制器已成功拉起。可在 RViz 使用 Publish Point (/clicked_point) 顺次下发路径点进行连续巡航。")

    def stop_navigation(self):
        if self.process_nav and self.process_nav.state() == QProcess.Running:
            self.append_log("[SYS] 正在关闭 Nav2 规划与导航...")
            self.process_nav.terminate()
            if not self.process_nav.waitForFinished(3000):
                self.process_nav.kill()
                self.process_nav.waitForFinished(1000)
        self.process_nav = None
        
        try:
            # 杀死 navigation.launch.py 拉起的所有 Nav2 子进程（包括 costmap 相关节点）
            nav_kill_list = [
                "navigation_launch",
                "controller_server",
                "planner_server",
                "bt_navigator",
                "lifecycle_manager_navigation",
                "smoother_server",
                "behavior_server",
                "velocity_smoother",
                "collision_monitor",
                "waypoint_follower",
                "global_path_seq_publisher",
                "move.pure_pursuit",
                "move.priest_rl_pub",
                "move.priest_mppi",
                "move.nav_cmd_domain_bridge",
                "navigation_mission_coordinator",
            ]
            self.safe_kill_processes(nav_kill_list, 2500)
        except:
            pass
            
        self.is_nav_running = False
        self.update_ui_state(self.current_state)
        # 清理停止导航后残留的死锁文件，保持共享内存环境干净
        self._cleanup_fastdds_shm()
        QTimer.singleShot(1000, self._cleanup_fastdds_shm)
        self.sig_clear_nav.emit()   # 通知 ROS 后台清除 /global_path 全局路径残影
        self.reset_mission_session("2D导航关闭")
        self.append_log("[SYS] 规划与导航已安全停止并重置。")


    def closeEvent(self, event):
        self._closing = True
        self.stop_perception()
        self._manager_should_run = False
        self.stop_qprocess(self.process_manager)
        self.process_manager = None
        self.stop_all(keep_sensors=False)
        
        # 强制暂停 0.5 秒，等待系统回收僵尸进程
        import time
        time.sleep(0.5)
        
        # 退出前执行最后一次终极清理
        self._cleanup_fastdds_shm()
        
        os._exit(0)
