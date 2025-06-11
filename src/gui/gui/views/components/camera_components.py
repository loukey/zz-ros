"""
摄像头相关组件
"""
from PyQt5.QtWidgets import (QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout, 
                           QGroupBox, QFrame, QSizePolicy)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QPixmap, QImage
from .base_components import default_font
import cv2
import numpy as np


class CameraFrame(QGroupBox):
    """摄像头控制框架"""
    
    # 定义信号
    color_display_requested = pyqtSignal()  # 显示彩色图请求
    depth_display_requested = pyqtSignal()  # 显示深度图请求
    stop_display_requested = pyqtSignal()   # 停止显示请求
    
    def __init__(self, parent=None):
        super().__init__("摄像头控制", parent)
        self._init_ui()
    
    def _init_ui(self):
        """初始化UI"""
        layout = QVBoxLayout()
        
        # 创建控制按钮区域
        button_layout = QHBoxLayout()
        
        # 显示彩色图按钮
        self.color_button = QPushButton("显示彩色图")
        self.color_button.setFont(default_font)
        self.color_button.setCheckable(True)  # 可切换状态
        self.color_button.clicked.connect(self._on_color_button_clicked)
        button_layout.addWidget(self.color_button)
        
        # 显示深度图按钮
        self.depth_button = QPushButton("显示深度图")
        self.depth_button.setFont(default_font)
        self.depth_button.setCheckable(True)  # 可切换状态
        self.depth_button.clicked.connect(self._on_depth_button_clicked)
        button_layout.addWidget(self.depth_button)
        
        # 停止显示按钮
        self.stop_button = QPushButton("停止显示")
        self.stop_button.setFont(default_font)
        self.stop_button.clicked.connect(self._on_stop_button_clicked)
        button_layout.addWidget(self.stop_button)
        
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
        self.setLayout(layout)
    
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
    
    def update_button_states(self, color_active, depth_active):
        """更新按钮状态"""
        self.color_button.setChecked(color_active)
        self.depth_button.setChecked(depth_active)
    
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


class CameraDisplayWidget(QWidget):
    """摄像头显示小部件"""
    
    # 定义信号
    color_display_requested = pyqtSignal()
    depth_display_requested = pyqtSignal()
    stop_display_requested = pyqtSignal()
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self._init_ui()
    
    def _init_ui(self):
        """初始化UI"""
        layout = QVBoxLayout()
        layout.setContentsMargins(6, 6, 6, 6)
        
        # 添加摄像头控制框架
        self.camera_frame = CameraFrame(self)
        layout.addWidget(self.camera_frame)
        
        # 连接信号
        self.camera_frame.color_display_requested.connect(self.color_display_requested.emit)
        self.camera_frame.depth_display_requested.connect(self.depth_display_requested.emit)
        self.camera_frame.stop_display_requested.connect(self.stop_display_requested.emit)
        
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
    
    def update_button_states(self, color_active, depth_active):
        """更新按钮状态"""
        self.camera_frame.update_button_states(color_active, depth_active)
    
    def clear_display(self):
        """清除显示"""
        self.camera_frame.clear_display() 