"""
摄像头相关组件
"""
from PyQt5.QtWidgets import (QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout, 
                           QGroupBox, QFrame, QSizePolicy)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QPixmap, QImage
from .base_component import BaseComponent, default_font
import cv2
import numpy as np


class CameraFrame(BaseComponent):
    """摄像头控制框架"""
    
    # 定义信号
    connect_camera_requested = pyqtSignal()  # 连接摄像头请求
    disconnect_camera_requested = pyqtSignal()  # 断开摄像头请求
    color_display_requested = pyqtSignal()  # 显示彩色图请求
    depth_display_requested = pyqtSignal()  # 显示深度图请求
    stop_display_requested = pyqtSignal()   # 停止显示请求
    start_detection_requested = pyqtSignal()  # 开始检测请求
    stop_detection_requested = pyqtSignal()   # 停止检测请求
    start_pose_publish_requested = pyqtSignal()  # 开始位姿发布请求
    stop_pose_publish_requested = pyqtSignal()   # 停止位姿发布请求
    move_to_part_requested = pyqtSignal()  # 运动到零件位置请求
    
    def __init__(self, parent=None, view_model=None):
        self.is_connected = False
        super().__init__(parent, view_model)
    
    def setup_ui(self):
        """设置UI"""
        # 创建分组框
        group_box = QGroupBox("摄像头控制")
        main_layout = QVBoxLayout(self)
        main_layout.addWidget(group_box)
        
        layout = QVBoxLayout(group_box)
        
        # 创建控制按钮区域
        button_layout = QHBoxLayout()
        
        # 连接摄像头按钮
        self.connect_button = QPushButton("连接摄像头")
        self.connect_button.setFont(default_font)
        self.connect_button.clicked.connect(self._on_connect_button_clicked)
        button_layout.addWidget(self.connect_button)
        
        # 添加分隔线
        separator = QFrame()
        separator.setFrameShape(QFrame.VLine)
        separator.setFrameShadow(QFrame.Sunken)
        button_layout.addWidget(separator)
        
        # 显示彩色图按钮
        self.color_button = QPushButton("显示彩色图")
        self.color_button.setFont(default_font)
        self.color_button.setCheckable(True)  # 可切换状态
        self.color_button.setEnabled(False)  # 初始不可用
        self.color_button.clicked.connect(self._on_color_button_clicked)
        button_layout.addWidget(self.color_button)
        
        # 显示深度图按钮
        self.depth_button = QPushButton("显示深度图")
        self.depth_button.setFont(default_font)
        self.depth_button.setCheckable(True)  # 可切换状态
        self.depth_button.setEnabled(False)  # 初始不可用
        self.depth_button.clicked.connect(self._on_depth_button_clicked)
        button_layout.addWidget(self.depth_button)
        
        # 停止显示按钮
        self.stop_button = QPushButton("停止显示")
        self.stop_button.setFont(default_font)
        self.stop_button.setEnabled(False)  # 初始不可用
        self.stop_button.clicked.connect(self._on_stop_button_clicked)
        button_layout.addWidget(self.stop_button)
        
        # 添加分隔线
        separator2 = QFrame()
        separator2.setFrameShape(QFrame.VLine)
        separator2.setFrameShadow(QFrame.Sunken)
        button_layout.addWidget(separator2)
        
        # 开始检测按钮
        self.start_detection_button = QPushButton("开始检测")
        self.start_detection_button.setFont(default_font)
        self.start_detection_button.setCheckable(True)  # 可切换状态
        self.start_detection_button.setEnabled(True)  # 始终可用，不依赖摄像头连接
        self.start_detection_button.clicked.connect(self._on_detection_button_clicked)
        button_layout.addWidget(self.start_detection_button)
        
        # 运动按钮
        self.move_button = QPushButton("运动")
        self.move_button.setFont(default_font)
        self.move_button.setEnabled(False)  # 初始不可用
        self.move_button.clicked.connect(self._on_move_button_clicked)
        button_layout.addWidget(self.move_button)
        
        # 添加分隔线
        separator3 = QFrame()
        separator3.setFrameShape(QFrame.VLine)
        separator3.setFrameShadow(QFrame.Sunken)
        button_layout.addWidget(separator3)
        
        # 开启位姿发布按钮
        self.pose_publish_button = QPushButton("开启位姿发布")
        self.pose_publish_button.setFont(default_font)
        self.pose_publish_button.setCheckable(True)  # 可切换状态
        self.pose_publish_button.setEnabled(False)  # 初始不可用
        self.pose_publish_button.clicked.connect(self._on_pose_publish_button_clicked)
        button_layout.addWidget(self.pose_publish_button)
        
        # 添加伸缩项
        button_layout.addStretch()
        
        layout.addLayout(button_layout)
        
        # 添加分隔线
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        layout.addWidget(line)
        
        # 创建图像显示区域
        self.image_display_group = QGroupBox("图像显示")
        image_layout = QVBoxLayout(self.image_display_group)
        
        # 状态标签
        self.status_label = QLabel("未连接摄像头")
        self.status_label.setFont(default_font)
        self.status_label.setAlignment(Qt.AlignCenter)
        image_layout.addWidget(self.status_label)
        
        # 图像显示标签
        self.image_label = QLabel()
        self.image_label.setMinimumSize(640, 480)
        self.image_label.setStyleSheet("border: 1px solid gray; background-color: #f0f0f0;")
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setScaledContents(True)
        self.image_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.image_label.setText("点击按钮开始显示图像")
        image_layout.addWidget(self.image_label)
        
        # 图像信息标签
        self.info_label = QLabel("图像信息：无")
        self.info_label.setFont(default_font)
        image_layout.addWidget(self.info_label)
        
        layout.addWidget(self.image_display_group)
    
    def _on_connect_button_clicked(self):
        """连接摄像头按钮点击事件"""
        if not self.is_connected:
            self.connect_camera_requested.emit()
        else:
            self.disconnect_camera_requested.emit()
    
    def _on_color_button_clicked(self):
        """彩色图按钮点击事件"""
        if self.color_button.isChecked():
            # 取消深度图按钮状态
            self.depth_button.setChecked(False)
            # 发送显示彩色图请求
            self.color_display_requested.emit()
        else:
            # 停止显示
            self.stop_display_requested.emit()
    
    def _on_depth_button_clicked(self):
        """深度图按钮点击事件"""
        if self.depth_button.isChecked():
            # 取消彩色图按钮状态
            self.color_button.setChecked(False)
            # 发送显示深度图请求
            self.depth_display_requested.emit()
        else:
            # 停止显示
            self.stop_display_requested.emit()
    
    def _on_stop_button_clicked(self):
        """停止按钮点击事件"""
        self.stop_display_requested.emit()
        # 停止显示时也停止检测
        if self.start_detection_button.isChecked():
            self.start_detection_button.setChecked(False)
            self.start_detection_button.setText("开始检测")
            self.stop_detection_requested.emit()
    
    def _on_detection_button_clicked(self):
        """检测按钮点击事件"""
        if self.start_detection_button.isChecked():
            self.start_detection_button.setText("停止检测")
            self.start_detection_requested.emit()
            # 检测开启时，启用运动按钮
            self.move_button.setEnabled(True)
        else:
            self.start_detection_button.setText("开始检测")
            self.stop_detection_requested.emit()
            # 检测停止时，禁用运动按钮
            self.move_button.setEnabled(False)
    
    def _on_pose_publish_button_clicked(self):
        """位姿发布按钮点击事件"""
        if self.pose_publish_button.isChecked():
            self.pose_publish_button.setText("停止位姿发布")
            self.start_pose_publish_requested.emit()
        else:
            self.pose_publish_button.setText("开启位姿发布")
            self.stop_pose_publish_requested.emit()
    
    def _on_move_button_clicked(self):
        """运动按钮点击事件"""
        self.move_to_part_requested.emit()
    
    def update_connection_status(self, connected):
        """更新连接状态"""
        self.is_connected = connected
        if connected:
            self.connect_button.setText("断开摄像头")
            self.color_button.setEnabled(True)
            self.depth_button.setEnabled(True)
            self.stop_button.setEnabled(True)
            # 检测按钮始终保持启用状态，不依赖摄像头连接
            self.pose_publish_button.setEnabled(True)
        else:
            self.connect_button.setText("连接摄像头")
            self.color_button.setEnabled(False)
            self.depth_button.setEnabled(False)
            self.stop_button.setEnabled(False)
            # 检测按钮保持启用，但重置其状态
            self.move_button.setEnabled(False)
            self.pose_publish_button.setEnabled(False)
            # 清除选中状态
            self.color_button.setChecked(False)
            self.depth_button.setChecked(False)
            # 不重置检测按钮状态，让用户自己控制
            self.pose_publish_button.setChecked(False)
            self.pose_publish_button.setText("开启位姿发布")
    
    def update_button_states(self, color_active, depth_active):
        """更新按钮状态"""
        self.color_button.setChecked(color_active)
        self.depth_button.setChecked(depth_active)
    
    def update_detection_state(self, detection_active):
        """更新检测状态"""
        self.start_detection_button.setChecked(detection_active)
        if detection_active:
            self.start_detection_button.setText("停止检测")
            self.move_button.setEnabled(True)
        else:
            self.start_detection_button.setText("开始检测")
            self.move_button.setEnabled(False)
    
    def display_image(self, image, image_type):
        """显示图像"""
        try:
            if image is not None:
                # 转换OpenCV图像为Qt格式
                if len(image.shape) == 3:  # 彩色图像
                    height, width, channel = image.shape
                    bytes_per_line = 3 * width
                    # OpenCV使用BGR，Qt使用RGB，需要转换
                    rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                    q_image = QImage(rgb_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
                else:  # 灰度图像
                    height, width = image.shape
                    bytes_per_line = width
                    q_image = QImage(image.data, width, height, bytes_per_line, QImage.Format_Grayscale8)
                
                # 创建QPixmap并显示
                pixmap = QPixmap.fromImage(q_image)
                
                # 缩放图像以适应显示区域
                scaled_pixmap = pixmap.scaled(
                    self.image_label.width(), 
                    self.image_label.height(), 
                    Qt.KeepAspectRatio, 
                    Qt.SmoothTransformation
                )
                
                self.image_label.setPixmap(scaled_pixmap)
                
        except Exception as e:
            self.status_label.setText(f"图像显示错误: {str(e)}")
    
    def update_status(self, status):
        """更新状态"""
        self.status_label.setText(status)
    
    def update_image_info(self, info):
        """更新图像信息"""
        if info:
            info_text = f"图像信息：{info.get('width', 0)}x{info.get('height', 0)}, " \
                       f"{info.get('channels', 0)}通道, {info.get('dtype', 'unknown')}"
            self.info_label.setText(info_text)
        else:
            self.info_label.setText("图像信息：无")
    
    def clear_display(self):
        """清除显示"""
        self.image_label.clear()
        self.image_label.setText("点击按钮开始显示图像")
        self.update_button_states(False, False)
        self.update_status("已停止显示")
        self.update_image_info({})


class CameraDisplayWidget(BaseComponent):
    """摄像头显示小部件"""
    
    # 定义信号
    connect_camera_requested = pyqtSignal()
    disconnect_camera_requested = pyqtSignal()
    color_display_requested = pyqtSignal()
    depth_display_requested = pyqtSignal()
    stop_display_requested = pyqtSignal()
    start_detection_requested = pyqtSignal()
    stop_detection_requested = pyqtSignal()
    start_pose_publish_requested = pyqtSignal()
    stop_pose_publish_requested = pyqtSignal()
    move_to_part_requested = pyqtSignal()
    
    def __init__(self, parent=None, view_model=None):
        super().__init__(parent, view_model)
    
    def setup_ui(self):
        """设置UI"""
        layout = QVBoxLayout()
        layout.setContentsMargins(6, 6, 6, 6)
        
        # 添加摄像头控制框架
        self.camera_frame = CameraFrame(self, self.view_model)
        layout.addWidget(self.camera_frame)
        
        # 连接信号
        self.camera_frame.connect_camera_requested.connect(self.connect_camera_requested.emit)
        self.camera_frame.disconnect_camera_requested.connect(self.disconnect_camera_requested.emit)
        self.camera_frame.color_display_requested.connect(self.color_display_requested.emit)
        self.camera_frame.depth_display_requested.connect(self.depth_display_requested.emit)
        self.camera_frame.stop_display_requested.connect(self.stop_display_requested.emit)
        self.camera_frame.start_detection_requested.connect(self.start_detection_requested.emit)
        self.camera_frame.stop_detection_requested.connect(self.stop_detection_requested.emit)
        self.camera_frame.start_pose_publish_requested.connect(self.start_pose_publish_requested.emit)
        self.camera_frame.stop_pose_publish_requested.connect(self.stop_pose_publish_requested.emit)
        self.camera_frame.move_to_part_requested.connect(self.move_to_part_requested.emit)
        
        # 添加伸缩项
        layout.addStretch()
        
        self.setLayout(layout)
    
    def display_image(self, image, image_type):
        """显示图像"""
        self.camera_frame.display_image(image, image_type)
    
    def update_status(self, status):
        """更新状态"""
        self.camera_frame.update_status(status)
    
    def update_image_info(self, info):
        """更新图像信息"""
        self.camera_frame.update_image_info(info)
    
    def update_connection_status(self, connected):
        """更新连接状态"""
        self.camera_frame.update_connection_status(connected)
    
    def update_button_states(self, color_active, depth_active):
        """更新按钮状态"""
        self.camera_frame.update_button_states(color_active, depth_active)
    
    def update_detection_state(self, detection_active):
        """更新检测状态"""
        self.camera_frame.update_detection_state(detection_active)
    
    def clear_display(self):
        """清除显示"""
        self.camera_frame.clear_display() 