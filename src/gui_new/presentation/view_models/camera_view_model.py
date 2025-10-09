"""
摄像头视图模型 - Presentation层
"""
import numpy as np
from PyQt5.QtCore import QTimer, pyqtSignal
from typing import Optional
from .base_view_model import BaseViewModel
from application.services.camera_application_service import CameraApplicationService
from domain.services.vision import CameraDomainService


class CameraViewModel(BaseViewModel):
    """
    摄像头视图模型
    
    职责：
    - 连接UI和Application Service
    - 管理图像显示状态（定时刷新）
    - 提供图像数据给UI
    """
    
    # ========== 信号定义 ==========
    image_display_requested = pyqtSignal(np.ndarray, str)  # (image, image_type: 'color'/'depth')
    status_updated = pyqtSignal(str)  # 状态文本更新
    image_info_updated = pyqtSignal(dict)  # 图像信息更新
    connection_status_changed = pyqtSignal(bool)  # 连接状态
    button_states_changed = pyqtSignal(bool, bool)  # (color_active, depth_active)
    clear_display_requested = pyqtSignal()  # 清除显示
    
    def __init__(
        self,
        app_service: CameraApplicationService,
        camera_service: CameraDomainService,
        parent=None
    ):
        super().__init__(parent)
        self.app_service = app_service
        self.camera_service = camera_service
        
        # 显示状态
        self.current_display_mode = None  # None, 'color', 'depth'
        
        # 定时器（30FPS刷新图像）
        self.display_timer = QTimer()
        self.display_timer.timeout.connect(self._update_display)
        
        # 连接信号
        self._connect_signals()
    
    # ========== 用户操作方法 ==========
    
    def connect_camera(self):
        """连接摄像头"""
        self.app_service.connect_camera()
    
    def disconnect_camera(self):
        """断开摄像头"""
        # 先停止显示
        if self.current_display_mode is not None:
            self.stop_display()
        
        # 断开摄像头
        self.app_service.disconnect_camera()
    
    def start_color_display(self):
        """开始显示彩色图"""
        # 检查摄像头是否连接
        if not self.camera_service.is_connected:
            self.status_updated.emit("请先连接摄像头")
            return
        
        # 设置显示模式
        self.current_display_mode = 'color'
        
        # 启动定时器（33ms = ~30FPS）
        self.display_timer.start(33)
        
        # 更新状态
        self.status_updated.emit("显示彩色图像")
        self.button_states_changed.emit(True, False)
    
    def start_depth_display(self):
        """开始显示深度图"""
        # 检查摄像头是否连接
        if not self.camera_service.is_connected:
            self.status_updated.emit("请先连接摄像头")
            return
        
        # 设置显示模式
        self.current_display_mode = 'depth'
        
        # 启动定时器
        self.display_timer.start(33)
        
        # 更新状态
        self.status_updated.emit("显示深度图像")
        self.button_states_changed.emit(False, True)
    
    def stop_display(self):
        """停止显示"""
        # 停止定时器
        self.display_timer.stop()
        
        # 清除显示模式
        self.current_display_mode = None
        
        # 清除显示
        self.clear_display_requested.emit()
        
        # 更新状态
        self.status_updated.emit("已停止显示")
        self.button_states_changed.emit(False, False)
        self.image_info_updated.emit({})
    
    # ========== 私有方法 ==========
    
    def _update_display(self):
        """定时器回调，刷新图像显示"""
        if self.current_display_mode == 'color':
            if self.camera_service.is_color_available():
                image = self.camera_service.get_latest_color_image()
                if image is not None:
                    self.image_display_requested.emit(image, 'color')
                    self._update_image_info(image, 'color')
            else:
                self.status_updated.emit("等待彩色图像数据...")
        
        elif self.current_display_mode == 'depth':
            if self.camera_service.is_depth_available():
                raw_depth = self.camera_service.get_latest_depth_image()
                if raw_depth is not None:
                    # 可视化深度图
                    vis_depth = self.camera_service.visualize_depth_image(raw_depth)
                    self.image_display_requested.emit(vis_depth, 'depth')
                    self._update_image_info(raw_depth, 'depth')
            else:
                self.status_updated.emit("等待深度图像数据...")
    
    def _update_image_info(self, image: np.ndarray, image_type: str):
        """更新图像信息"""
        try:
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
            pass
    
    def _connect_signals(self):
        """连接Application Service信号"""
        self.app_service.connection_status_changed.connect(
            self.connection_status_changed.emit
        )
        self.app_service.connection_status_changed.connect(
            self._on_connection_status_changed
        )
    
    def _on_connection_status_changed(self, connected: bool):
        """处理连接状态变化"""
        # 如果断开连接，自动停止显示
        if not connected and self.current_display_mode is not None:
            self.stop_display()
    
    def cleanup(self):
        """清理资源"""
        try:
            # 停止显示
            self.stop_display()
            
            # 断开摄像头
            if self.camera_service.is_connected:
                self.camera_service.disconnect()
            
        except Exception as e:
            print(f"清理摄像头资源失败: {e}")

