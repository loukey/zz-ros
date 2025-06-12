"""
摄像头控制器
"""
from PyQt5.QtCore import pyqtSignal, QTimer
from .base_controller import BaseController
import numpy as np
import cv2


class CameraController(BaseController):
    """摄像头控制器类"""
    
    # 定义信号
    image_display_requested = pyqtSignal(np.ndarray, str)  # 图像显示请求信号 (图像, 类型)
    status_update_requested = pyqtSignal(str)  # 状态更新信号
    image_info_updated = pyqtSignal(dict)  # 图像信息更新信号
    connection_status_changed = pyqtSignal(bool)  # 连接状态变化信号
    
    def __init__(self, camera_model, detection_model=None):
        super().__init__()
        self.camera_model = camera_model
        self.detection_model = detection_model
        self.current_display_mode = None  # 'color', 'depth', or None
        self.detection_enabled = False  # 检测开关状态
        self.display_timer = QTimer()
        self.display_timer.timeout.connect(self._update_display)
        
        # 连接模型信号
        self._connect_model_signals()
    
    def _connect_model_signals(self):
        """连接模型信号"""
        self.camera_model.color_image_received.connect(self._on_color_image_received)
        self.camera_model.depth_image_received.connect(self._on_depth_image_received)
        self.camera_model.error_occurred.connect(self._on_error_occurred)
        self.camera_model.connection_status_changed.connect(self._on_status_changed)
    
    def connect_camera(self):
        """连接摄像头"""
        try:
            success = self.camera_model.connect_camera()
            if success:
                self.display("摄像头连接成功", "摄像头")
                self.connection_status_changed.emit(True)
            else:
                self.display("摄像头连接失败", "错误")
                self.connection_status_changed.emit(False)
            return success
        except Exception as e:
            self.display(f"连接摄像头异常: {str(e)}", "错误")
            self.connection_status_changed.emit(False)
            return False
    
    def disconnect_camera(self):
        """断开摄像头连接"""
        try:
            # 先停止显示
            self.stop_display()
            
            success = self.camera_model.disconnect_camera()
            if success:
                self.display("摄像头已断开", "摄像头")
                self.connection_status_changed.emit(False)
            else:
                self.display("断开摄像头失败", "错误")
            return success
        except Exception as e:
            self.display(f"断开摄像头异常: {str(e)}", "错误")
            return False
    
    def _on_color_image_received(self, image):
        """彩色图像接收处理"""
        if self.current_display_mode == 'color':
            self.image_display_requested.emit(image, 'color')
            self._update_image_info(image, 'color')
    
    def _on_depth_image_received(self, image):
        """深度图像接收处理"""
        if self.current_display_mode == 'depth':
            vis_image = self._visualize_depth_image(image)
            self.image_display_requested.emit(vis_image, 'depth')
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
                    # 如果检测开启，在图像上绘制检测框
                    if self.detection_enabled and self.detection_model is not None:
                        display_image = self._draw_detection_boxes(image, 'color')
                    else:
                        display_image = image
                    
                    self.image_display_requested.emit(display_image, 'color')
                    self._update_image_info(image, 'color')
            else:
                self.status_update_requested.emit("等待彩色图像数据...")
                
        elif self.current_display_mode == 'depth':
            if self.camera_model.is_depth_available():
                image = self.camera_model.get_latest_depth_image()
                if image is not None:
                    vis_image = self._visualize_depth_image(image)
                    # 如果检测开启，在图像上绘制检测框
                    if self.detection_enabled and self.detection_model is not None:
                        display_image = self._draw_detection_boxes(vis_image, 'depth')
                    else:
                        display_image = vis_image
                    
                    self.image_display_requested.emit(display_image, 'depth')
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
            if not self.camera_model.is_camera_connected:
                self.status_update_requested.emit("请先连接摄像头")
                return
            
            self.current_display_mode = 'color'
            self.display_timer.start(33)  # 约30FPS
            self.status_update_requested.emit("显示彩色图像")
            self.display("开始显示彩色图像", "摄像头")
        except Exception as e:
            self.display(f"启动彩色图像显示失败: {str(e)}", "错误")
    
    def start_depth_display(self):
        """开始显示深度图像"""
        try:
            if not self.camera_model.is_camera_connected:
                self.status_update_requested.emit("请先连接摄像头")
                return
            
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
    
    def start_detection(self):
        """开始检测"""
        try:
            if not self.camera_model.is_camera_connected:
                self.status_update_requested.emit("请先连接摄像头")
                return
            
            if self.detection_model is None:
                self.status_update_requested.emit("检测模型未初始化")
                return
            
            self.detection_enabled = True
            self.display("开始目标检测", "检测")
            self.status_update_requested.emit("检测已开启")
        except Exception as e:
            self.display(f"启动检测失败: {str(e)}", "错误")
    
    def stop_detection(self):
        """停止检测"""
        try:
            self.detection_enabled = False
            self.display("停止目标检测", "检测")
            self.status_update_requested.emit("检测已停止")
        except Exception as e:
            self.display(f"停止检测失败: {str(e)}", "错误")
    
    def _draw_detection_boxes(self, image, image_type):
        """在图像上绘制检测框"""
        try:
            if self.detection_model is None or not self.detection_enabled:
                return image
            
            # 获取检测结果
            detections = self.detection_model.detect_objects(image, image_type)
            
            # 如果没有检测结果，直接返回原图像
            if not detections:
                return image
            
            # 在图像副本上绘制检测框
            result_image = image.copy()
            
            for detection in detections:
                # 假设检测结果格式为 {'bbox': [x, y, w, h], 'class': 'class_name', 'confidence': 0.85}
                bbox = detection.get('bbox')
                class_name = detection.get('class', 'object')
                confidence = detection.get('confidence', 0.0)
                
                if bbox is not None:
                    x, y, w, h = bbox
                    
                    # 绘制边界框
                    cv2.rectangle(result_image, (int(x), int(y)), (int(x + w), int(y + h)), (0, 255, 0), 2)
                    
                    # 绘制标签
                    label = f"{class_name}: {confidence:.2f}"
                    label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
                    cv2.rectangle(result_image, (int(x), int(y - label_size[1] - 10)), 
                                (int(x + label_size[0]), int(y)), (0, 255, 0), -1)
                    cv2.putText(result_image, label, (int(x), int(y - 5)), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
            
            return result_image
            
        except Exception as e:
            self.display(f"绘制检测框失败: {str(e)}", "错误")
            return image
    
    def cleanup(self):
        """清理资源"""
        try:
            # 停止检测
            self.stop_detection()
            
            # 停止显示
            self.stop_display()
            
            # 清理模型
            self.camera_model.cleanup()
            
            self.display("摄像头系统已关闭", "摄像头")
            
        except Exception as e:
            self.display(f"摄像头清理失败: {str(e)}", "错误")
    
    def _visualize_depth_image(self, image):
        valid_mask = (image > 0) & np.isfinite(image)
        if not np.any(valid_mask):
            depth_display = np.zeros_like(image, dtype=np.uint8)
        else:
            min_val = np.min(image[valid_mask])
            max_val = np.max(image[valid_mask])
            if max_val - min_val < 1e-6:
                depth_display = np.zeros_like(image, dtype=np.uint8)
            else:
                norm = (image - min_val) / (max_val - min_val)
                norm[~valid_mask] = 0
                depth_display = (norm * 255).astype(np.uint8)
        depth_colormap = cv2.applyColorMap(depth_display, cv2.COLORMAP_JET)
        return depth_colormap 