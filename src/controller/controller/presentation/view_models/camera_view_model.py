"""
摄像头视图模型 - Presentation层
只依赖Application层，不直接访问Domain层
"""
import numpy as np
from PyQt5.QtCore import QTimer, pyqtSignal
from typing import Optional
from .base_view_model import BaseViewModel
from controller.application import CameraApplicationService


class CameraViewModel(BaseViewModel):
    """摄像头视图模型。
    
    职责：
    - 连接 UI 和 Application Service
    - 管理图像显示状态（定时刷新）
    - 管理检测状态
    - 通过 Application Service 获取图像和检测结果
    
    架构原则：
    - 只依赖 Application 层，不直接访问 Domain 层
    - 所有 Domain 层调用通过 Application Service 转发
    
    Attributes:
        image_display_requested (pyqtSignal): 图像显示请求信号，携带 (image, image_type)。
        status_updated (pyqtSignal): 状态文本更新信号。
        image_info_updated (pyqtSignal): 图像信息更新信号。
        connection_status_changed (pyqtSignal): 摄像头连接状态信号。
        button_states_changed (pyqtSignal): 按钮状态变更信号，携带 (color_active, depth_active)。
        clear_display_requested (pyqtSignal): 清除显示请求信号。
        detection_status_changed (pyqtSignal): 检测状态变更信号。
        detection_result_updated (pyqtSignal): 检测结果更新信号。
        app_service (CameraApplicationService): 摄像头应用服务。
        display_timer (QTimer): 图像刷新定时器。
    """
    
    # ========== 信号定义 ==========
    image_display_requested = pyqtSignal(np.ndarray, str)  # (image, image_type: 'color'/'depth')
    status_updated = pyqtSignal(str)  # 状态文本更新
    image_info_updated = pyqtSignal(dict)  # 图像信息更新
    connection_status_changed = pyqtSignal(bool)  # 摄像头连接状态
    button_states_changed = pyqtSignal(bool, bool)  # (color_active, depth_active)
    clear_display_requested = pyqtSignal()  # 清除显示
    detection_status_changed = pyqtSignal(bool)  # 检测状态
    detection_result_updated = pyqtSignal(dict)  # 检测结果（用于UI更新，如启用运动按钮）
    
    def __init__(
        self,
        app_service: CameraApplicationService,
        parent=None
    ):
        """初始化摄像头视图模型。
        
        Args:
            app_service (CameraApplicationService): 摄像头应用服务。
            parent (QObject, optional): 父对象. Defaults to None.
        """
        super().__init__(parent)
        self.app_service = app_service
        
        # 显示状态
        self.current_display_mode = None  # None, 'color', 'depth'
        
        # 定时器（30FPS刷新图像）
        self.display_timer = QTimer()
        self.display_timer.timeout.connect(self._update_display)
        
        # 连接信号
        self._connect_signals()
    
    # ========== 用户操作方法 ==========
    
    def connect_camera(self):
        """连接摄像头。"""
        self.app_service.connect_camera()
    
    def disconnect_camera(self):
        """断开摄像头。"""
        # 先停止显示
        if self.current_display_mode is not None:
            self.stop_display()
        
        # 断开摄像头
        self.app_service.disconnect_camera()
    
    def start_color_display(self):
        """开始显示彩色图。"""
        # ✅ 通过Application层检查摄像头是否连接
        if not self.app_service.is_camera_connected():
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
        """开始显示深度图。"""
        # ✅ 通过Application层检查摄像头是否连接
        if not self.app_service.is_camera_connected():
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
        """停止显示。"""
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
    
    def start_detection(self):
        """开始检测。"""
        self.app_service.start_detection()
    
    def stop_detection(self):
        """停止检测。"""
        self.app_service.stop_detection()
    
    def move_to_detected_part(self):
        """运动到检测到的零件位置。
        
        职责：仅转发到 Application 层。
        """
        self.app_service.move_to_detected_part()
    
    # ========== 私有方法 ==========
    
    def _update_display(self):
        """定时器回调，刷新图像显示。"""
        if self.current_display_mode == 'color':
            # ✅ 通过Application层检查彩色图像是否可用
            if self.app_service.is_color_available():
                # ✅ 通过Application层获取最新彩色图像
                image = self.app_service.get_latest_color_image()
                if image is not None:
                    # ✅ 通过Application层的高级接口获取叠加了检测结果的图像
                    # 封装了检测状态检查和图像绘制逻辑
                    image = self.app_service.get_image_with_detection(image)
                    
                    self.image_display_requested.emit(image, 'color')
                    self._update_image_info(image, 'color')
            else:
                self.status_updated.emit("等待彩色图像数据...")
        
        elif self.current_display_mode == 'depth':
            # ✅ 通过Application层检查深度图像是否可用
            if self.app_service.is_depth_available():
                # ✅ 通过Application层获取最新深度图像
                raw_depth = self.app_service.get_latest_depth_image()
                if raw_depth is not None:
                    # ✅ 通过Application层进行深度图可视化
                    vis_depth = self.app_service.visualize_depth_image(raw_depth)
                    
                    # ✅ 通过Application层的高级接口叠加检测结果
                    vis_depth = self.app_service.get_image_with_detection(vis_depth)
                    
                    self.image_display_requested.emit(vis_depth, 'depth')
                    self._update_image_info(raw_depth, 'depth')
            else:
                self.status_updated.emit("等待深度图像数据...")
    
    def _update_image_info(self, image: np.ndarray, image_type: str):
        """更新图像信息。
        
        Args:
            image (np.ndarray): 图像数据。
            image_type (str): 图像类型。
        """
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
        """连接Application Service信号。"""
        # 摄像头连接状态
        self.app_service.connection_status_changed.connect(
            self.connection_status_changed.emit
        )
        self.app_service.connection_status_changed.connect(
            self._on_connection_status_changed
        )
        
        # 检测状态
        self.app_service.detection_status_changed.connect(
            self.detection_status_changed.emit
        )
        
        # ✅ 检测结果（现在不再需要单独订阅，因为在 _update_display 中统一处理）
        # 如果需要接收检测结果用于UI更新（如启用运动按钮），
        # 可以通过 Application Service 中转 Domain 层信号
    
    def _on_connection_status_changed(self, connected: bool):
        """处理连接状态变化。
        
        如果断开连接，自动停止显示。
        
        Args:
            connected (bool): 是否已连接。
        """
        if not connected and self.current_display_mode is not None:
            self.stop_display()
    
    def cleanup(self):
        """清理资源。"""
        try:
            # 停止检测（✅ 通过Application层）
            if self.app_service.is_detection_running():
                self.app_service.stop_detection()
            
            # 停止显示
            self.stop_display()
            
            # 断开摄像头（✅ 通过Application层）
            if self.app_service.is_camera_connected():
                self.app_service.disconnect_camera()
            
        except Exception as e:
            pass

