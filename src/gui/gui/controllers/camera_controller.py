"""
摄像头控制器
"""
from PyQt5.QtCore import pyqtSignal, QTimer
from .base_controller import BaseController
import numpy as np


class CameraController(BaseController):
    """摄像头控制器类"""
    
    # 定义信号
    image_display_requested = pyqtSignal(np.ndarray, str)  # 图像显示请求信号 (图像, 类型)
    status_update_requested = pyqtSignal(str)  # 状态更新信号
    image_info_updated = pyqtSignal(dict)  # 图像信息更新信号
    
    def __init__(self, camera_model):
        super().__init__()
        self.camera_model = camera_model
        self.current_display_mode = None  # 'color', 'depth', or None
        self.display_timer = QTimer()
        self.display_timer.timeout.connect(self._update_display)
        
        # 连接模型信号
        self._connect_model_signals()
        
        # 初始化摄像头
        self._initialize_camera()
    
    def _connect_model_signals(self):
        """连接模型信号"""
        self.camera_model.color_image_received.connect(self._on_color_image_received)
        self.camera_model.depth_image_received.connect(self._on_depth_image_received)
        self.camera_model.error_occurred.connect(self._on_error_occurred)
        self.camera_model.connection_status_changed.connect(self._on_status_changed)
    
    def _initialize_camera(self):
        """初始化摄像头"""
        success = self.camera_model.initialize_ros()
        if success:
            self.display("摄像头系统初始化成功", "摄像头")
        else:
            self.display("摄像头系统初始化失败", "错误")
    
    def _on_color_image_received(self, image):
        """彩色图像接收处理"""
        if self.current_display_mode == 'color':
            self.image_display_requested.emit(image, 'color')
            self._update_image_info(image, 'color')
    
    def _on_depth_image_received(self, image):
        """深度图像接收处理"""
        if self.current_display_mode == 'depth':
            self.image_display_requested.emit(image, 'depth')
            self._update_image_info(image, 'depth')
    
    def _on_error_occurred(self, error_msg):
        """错误处理"""
        self.display(f"摄像头错误: {error_msg}", "错误")
        self.status_update_requested.emit(f"错误: {error_msg}")
    
    def _on_status_changed(self, status):
        """状态变化处理"""
        self.display(status, "摄像头")
        self.status_update_requested.emit(status)
    
    def _update_display(self):
        """更新显示"""
        if self.current_display_mode == 'color':
            if self.camera_model.is_color_available():
                image = self.camera_model.get_latest_color_image()
                if image is not None:
                    self.image_display_requested.emit(image, 'color')
                    self._update_image_info(image, 'color')
            else:
                self.status_update_requested.emit("等待彩色图像数据...")
                
        elif self.current_display_mode == 'depth':
            if self.camera_model.is_depth_available():
                image = self.camera_model.get_latest_depth_image()
                if image is not None:
                    self.image_display_requested.emit(image, 'depth')
                    self._update_image_info(image, 'depth')
            else:
                self.status_update_requested.emit("等待深度图像数据...")
    
    def _update_image_info(self, image, image_type):
        """更新图像信息"""
        try:
            if image is not None:
                height, width = image.shape[:2]
                channels = image.shape[2] if len(image.shape) == 3 else 1
                info = {
                    'type': image_type,
                    'width': width,
                    'height': height,
                    'channels': channels,
                    'dtype': str(image.dtype)
                }
                self.image_info_updated.emit(info)
        except Exception as e:
            self.display(f"更新图像信息失败: {str(e)}", "错误")
    
    def start_color_display(self):
        """开始显示彩色图像"""
        try:
            self.current_display_mode = 'color'
            self.display_timer.start(33)  # 约30FPS
            self.status_update_requested.emit("显示彩色图像")
            self.display("开始显示彩色图像", "摄像头")
        except Exception as e:
            self.display(f"启动彩色图像显示失败: {str(e)}", "错误")
    
    def start_depth_display(self):
        """开始显示深度图像"""
        try:
            self.current_display_mode = 'depth'
            self.display_timer.start(33)  # 约30FPS
            self.status_update_requested.emit("显示深度图像")
            self.display("开始显示深度图像", "摄像头")
        except Exception as e:
            self.display(f"启动深度图像显示失败: {str(e)}", "错误")
    
    def stop_display(self):
        """停止图像显示"""
        try:
            self.display_timer.stop()
            self.current_display_mode = None
            self.status_update_requested.emit("已停止显示")
            self.image_info_updated.emit({})
            self.display("停止图像显示", "摄像头")
        except Exception as e:
            self.display(f"停止图像显示失败: {str(e)}", "错误")
    
    def get_current_color_image(self):
        """获取当前彩色图像"""
        return self.camera_model.get_latest_color_image()
    
    def get_current_depth_image(self):
        """获取当前深度图像"""
        return self.camera_model.get_latest_depth_image()
    
    def is_color_available(self):
        """检查彩色图像是否可用"""
        return self.camera_model.is_color_available()
    
    def is_depth_available(self):
        """检查深度图像是否可用"""
        return self.camera_model.is_depth_available()
    
    def cleanup(self):
        """清理资源"""
        try:
            # 停止显示
            self.stop_display()
            
            # 清理模型
            self.camera_model.cleanup()
            
            self.display("摄像头系统已关闭", "摄像头")
            
        except Exception as e:
            self.display(f"摄像头清理失败: {str(e)}", "错误") 