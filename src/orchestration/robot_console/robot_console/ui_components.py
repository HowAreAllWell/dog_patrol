import os
import math
from PyQt5.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QLabel, 
                             QLineEdit, QPushButton, QCheckBox, QDialogButtonBox, 
                             QMessageBox, QFileDialog, QGraphicsView, QGraphicsScene)
from PyQt5.QtWidgets import QGraphicsPathItem, QGraphicsPolygonItem
from PyQt5.QtCore import Qt, QPointF
from PyQt5.QtGui import QPainter, QPen, QColor, QPainterPath, QPolygonF

class BagRecordDialog(QDialog):
    def __init__(self, default_dir="", parent=None):
        super().__init__(parent)
        self.setWindowTitle("🔴 录制传感器数据配置")
        self.resize(480, 200)
        layout = QVBoxLayout(self)

        path_layout = QHBoxLayout()
        path_layout.addWidget(QLabel("保存根目录:"))
        self.path_input = QLineEdit()
        self.path_input.setText(default_dir)
        self.path_input.setPlaceholderText("请输入或选择 Rosbag 保存路径...")
        self.path_input.setStyleSheet("font-size: 13px; padding: 4px; border: 1px solid #ccc; border-radius: 4px;")
        
        self.btn_browse = QPushButton("📂")
        self.btn_browse.clicked.connect(self.browse)
        
        path_layout.addWidget(self.path_input, stretch=1)
        path_layout.addWidget(self.btn_browse)
        layout.addLayout(path_layout)

        self.cb_lidar_imu_only = QCheckBox("🚫 仅录制雷达和 IMU (不包含图像话题，节省空间)")
        self.cb_lidar_imu_only.setChecked(False)
        self.cb_lidar_imu_only.setStyleSheet("font-size: 13px; margin: 10px 0;")
        layout.addWidget(self.cb_lidar_imu_only)

        self.cb_auto_compress = QCheckBox("🗜️ 录制结束后自动压缩 (无损转换为 MCAP/ZSTD)")
        self.cb_auto_compress.setChecked(False)
        self.cb_auto_compress.setStyleSheet("font-size: 13px; margin: 5px 0 10px 0;")
        layout.addWidget(self.cb_auto_compress)

        layout.addStretch()

        self.button_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        self.button_box.button(QDialogButtonBox.Ok).setText("开始录制")
        self.button_box.button(QDialogButtonBox.Cancel).setText("取消")
        self.button_box.accepted.connect(self.accept)
        self.button_box.rejected.connect(self.reject)
        layout.addWidget(self.button_box)

    def browse(self):
        dir_path = QFileDialog.getExistingDirectory(self, "选择数据包保存根路径", self.path_input.text())
        if dir_path:
            self.path_input.setText(dir_path)

    def get_save_dir(self):
        return self.path_input.text()

    def is_lidar_imu_only(self):
        return self.cb_lidar_imu_only.isChecked()
        
    def is_auto_compress(self):
        return self.cb_auto_compress.isChecked()

class BagConfigDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("⚙️ 配置回放参数")
        self.resize(480, 220)
        layout = QVBoxLayout(self)

        path_layout = QHBoxLayout()
        path_layout.addWidget(QLabel("数据包目录:"))
        self.path_input = QLineEdit()
        self.path_input.setPlaceholderText("请输入或选择 Rosbag2 路径...")
        self.path_input.setStyleSheet("font-size: 13px; padding: 4px; border: 1px solid #ccc; border-radius: 4px;")
        
        self.btn_browse = QPushButton("📂")
        self.btn_browse.clicked.connect(self.browse)
        
        path_layout.addWidget(self.path_input, stretch=1)
        path_layout.addWidget(self.btn_browse)
        layout.addLayout(path_layout)

        # 🚀 自定义倍速播放布局
        speed_layout = QHBoxLayout()
        self.cb_custom_speed = QCheckBox("🚀 开启自定义倍速播放")
        self.cb_custom_speed.setChecked(False)
        
        self.speed_input = QLineEdit("1.0")
        self.speed_input.setPlaceholderText("输入倍速 (如: 1.5, 2.0)...")
        self.speed_input.setEnabled(False)
        self.speed_input.setFixedWidth(150)
        self.speed_input.setStyleSheet("font-size: 13px; padding: 4px; border: 1px solid #ccc; border-radius: 4px;")
        
        self.cb_custom_speed.stateChanged.connect(self.on_speed_cb_changed)
        
        speed_layout.addWidget(self.cb_custom_speed)
        speed_layout.addWidget(self.speed_input)
        speed_layout.addStretch()
        layout.addLayout(speed_layout)

        speed_warning = QLabel("<font color='#FF5722'>⚠️ 提示：多倍速播放可能导致资源拥塞或丢帧，从而引发建图或定位失败。</font>")
        speed_warning.setWordWrap(True)
        speed_warning.setStyleSheet("font-size: 12px; margin-bottom: 5px;")
        layout.addWidget(speed_warning)

        # 🎲 开启随机时刻播放
        self.cb_random_start = QCheckBox("🎲 开启随机时刻播放 (随机断点抽检)")
        self.cb_random_start.setChecked(False)
        layout.addWidget(self.cb_random_start)
        
        random_warning = QLabel("<font color='#FF5722'>⚠️ 提示：随机播放可能导致重力初始化错误，从而引发前端崩溃。</font>")
        random_warning.setWordWrap(True)
        random_warning.setStyleSheet("font-size: 12px; margin-bottom: 10px;")
        layout.addWidget(random_warning)

        self.button_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        self.button_box.accepted.connect(self.accept)
        self.button_box.rejected.connect(self.reject)
        self.button_box.button(QDialogButtonBox.Ok).setText("确定播放")
        self.button_box.button(QDialogButtonBox.Cancel).setText("取消")
        layout.addWidget(self.button_box)

    def on_speed_cb_changed(self, state):
        if state == Qt.Checked:
            self.speed_input.setEnabled(True)
        else:
            self.speed_input.setEnabled(False)
            self.speed_input.setText("1.0")

    def browse(self):
        dir_path = QFileDialog.getExistingDirectory(self, "选择 Rosbag2 目录", os.path.expanduser("~"))
        if dir_path:
            self.path_input.setText(dir_path)

    def accept(self):
        path = self.path_input.text().strip()
        if not path:
            QMessageBox.warning(self, "警告", "请先选择数据包目录！")
            return
        if not os.path.exists(path):
            QMessageBox.warning(self, "警告", "所选数据包目录不存在，请重新选择！")
            return
            
        if self.cb_custom_speed.isChecked():
            speed_txt = self.speed_input.text().strip()
            try:
                val = float(speed_txt)
                if val <= 0:
                    raise ValueError
            except ValueError:
                QMessageBox.warning(self, "警告", "请输入合法的正数播放倍速！")
                return
        super().accept()

    def get_config(self):
        speed_val = 1.0
        if self.cb_custom_speed.isChecked():
            try:
                speed_val = float(self.speed_input.text().strip())
            except ValueError:
                speed_val = 1.0
        return (
            self.path_input.text().strip(),
            self.cb_custom_speed.isChecked(),
            speed_val,
            self.cb_random_start.isChecked()
        )

class TrajectoryMap(QGraphicsView):
    def __init__(self):
        super().__init__()
        self.scene = QGraphicsScene(self)
        self.setScene(self.scene)
        self.setRenderHint(QPainter.Antialiasing)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setStyleSheet("background-color: #121212; border-radius: 10px; border: 1px solid #333;")

        self.path_item = None
        self.arrow_item = None
        self.scale_factor = 40.0 

        self.clear_map()

    def clear_map(self):
        self.scene.clear()
        self.path = QPainterPath()
        
        self.path_item = QGraphicsPathItem()
        # 设置 Cosmetic Pen，确保视图缩放时轨迹线宽保持屏幕像素恒定
        pen = QPen(QColor("#00E676"), 2)
        pen.setCosmetic(True) 
        self.path_item.setPen(pen)
        self.scene.addItem(self.path_item)

        arrow_polygon = QPolygonF([
            QPointF(0, -12), 
            QPointF(-8, 10),  
            QPointF(0, 5),    
            QPointF(8, 10)     
        ])
        self.arrow_item = QGraphicsPolygonItem(arrow_polygon)
        self.arrow_item.setBrush(QColor("#F44336")) 
        
        # 箭头的边框线也设为 Cosmetic，防止缩放时边框消失
        arrow_pen = QPen(QColor("#FFFFFF"), 1)
        arrow_pen.setCosmetic(True)
        self.arrow_item.setPen(arrow_pen) 
        
        self.arrow_item.setZValue(10) 
        self.scene.addItem(self.arrow_item)

        self.is_first_point = True

    def add_pose(self, x, y, yaw):
        map_x = -y * self.scale_factor
        map_y = -x * self.scale_factor 

        if self.is_first_point:
            self.path.moveTo(map_x, map_y)
            self.is_first_point = False
            self.centerOn(map_x, map_y)
        else:
            self.path.lineTo(map_x, map_y)

        self.path_item.setPath(self.path)
        self.arrow_item.setPos(map_x, map_y)
        
        self.arrow_item.setRotation(math.degrees(-yaw))

        rect = self.path.boundingRect()
        self.scene.setSceneRect(rect.adjusted(-200, -200, 200, 200))

    def wheelEvent(self, event):
        zoom_in_factor = 1.15
        zoom_out_factor = 1.0 / zoom_in_factor
        
        # 动态调整箭头比例，确保视图放大或缩小时箭头的屏幕视觉尺寸保持恒定
        if event.angleDelta().y() > 0:
            self.scale(zoom_in_factor, zoom_in_factor)
            current_scale = self.arrow_item.scale()
            self.arrow_item.setScale(current_scale * zoom_out_factor)
        else:
            self.scale(zoom_out_factor, zoom_out_factor)
            current_scale = self.arrow_item.scale()
            self.arrow_item.setScale(current_scale * zoom_in_factor)
