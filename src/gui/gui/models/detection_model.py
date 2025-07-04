"""
检测数据模型
"""
import numpy as np
from PyQt5.QtCore import QObject, QTimer, pyqtSignal
import cv2
import numpy as np
from ultralytics import YOLO
import time
import numpy as np
from typing import Optional, Dict, Tuple
import cv2
import math


class DetectionModel(QObject):
    """检测数据模型类"""
    
    error_occurred = pyqtSignal(str)  # 错误信号
    
    def __init__(self, camera_model=None):
        super().__init__()
        self.camera_model = camera_model
        self.detection_enabled = False
        self.latest_detection_result = {}
        self.yolo_segmentor = YOLOSegmentor("./detection_models/yolo_multi_seg_n.pt")
        self.detect_timer = QTimer()
        self.detect_timer.setInterval(200)
        self.detect_timer.timeout.connect(self.process_detection)
        
    def set_camera_model(self, camera_model):
        """设置摄像头模型"""
        self.camera_model = camera_model
        
    def get_current_color_image(self):
        """获取当前彩色图像"""
        if self.camera_model and self.camera_model.is_camera_connected:
            return self.camera_model.get_latest_color_image()
        return None
    
    def get_current_depth_image(self):
        """获取当前深度图像"""
        if self.camera_model and self.camera_model.is_camera_connected:
            return self.camera_model.get_latest_depth_image()
        return None
    
    def is_camera_available(self):
        """检查摄像头是否可用"""
        return (self.camera_model and 
                self.camera_model.is_camera_connected and
                self.camera_model.is_color_available() and
                self.camera_model.is_depth_available())
    
    def start_auto_detection(self):
        self.detect_timer.start()
    
    def stop_auto_detection(self):
        self.detect_timer.stop()

    def get_depth_at_position(self, depth_image, x, y):
        """获取指定位置的深度值"""
        try:
            if depth_image is None:
                return None
            height, width = depth_image.shape[:2]
            x = max(0, min(int(x), width - 1))
            y = max(0, min(int(y), height - 1))
            depth_value = depth_image[y, x]
            return float(depth_value)
            
        except Exception as e:
            return None
    
    def process_detection(self):
        try:            
            if not self.is_camera_available():
                self.error_occurred.emit("摄像头不可用")
                return
            
            color_image = self.get_current_color_image()
            depth_image = self.get_current_depth_image()
            
            if color_image is None:
                self.error_occurred.emit("无法获取彩色图像")
                return
            
            detections = self.yolo_segmentor.detect(color_image)
            depth = self.get_depth_at_position(depth_image, *detections['central_center'])
            detections['depth'] = depth

            # 更新检测结果
            self.latest_detection_result = detections
        except Exception as e:
            self.error_occurred.emit(f"检测处理失败: {str(e)}")
            return
    
    def get_latest_detection_result(self):
        """获取最新检测结果"""
        return self.latest_detection_result
    
    def clear_detection_result(self):
        """清除检测结果"""
        self.latest_detection_result = {}


class YOLOSegmentor:

    def __init__(self, model_path):
        self.model = YOLO(model_path)
        self.class_mapping = {
            "central": 0,
            "head": 1
        }

    def _calculate_mask_center(self, mask: np.ndarray) -> Tuple[float, float]:
        """计算掩码中心点"""
        binary_mask = (mask > 0.5).astype(np.uint8)
        moments = cv2.moments(binary_mask)
        
        if moments['m00'] == 0:
            h, w = mask.shape
            return (w / 2, h / 2)
        
        cx = moments['m10'] / moments['m00']
        cy = moments['m01'] / moments['m00']
        
        return (cx, cy)

    def _extract_target_info(self, masks: np.ndarray, scores: np.ndarray, 
                            classes: np.ndarray, target_class: str, 
                            scale_x: float, scale_y: float) -> Optional[Dict]:
        """提取目标信息，并将中心点坐标缩放到原图像尺寸"""
        if target_class not in self.class_mapping:
            return None
        
        target_class_id = self.class_mapping[target_class]
        class_indices = np.where(classes == target_class_id)[0]
        
        if len(class_indices) == 0:
            return None
        
        # 选择置信度最高的
        best_idx = class_indices[np.argmax(scores[class_indices])]
        
        mask = masks[best_idx]
        confidence = scores[best_idx]
        center = self._calculate_mask_center(mask)
        
        # 将中心点坐标缩放到原图像尺寸
        scaled_center = (center[0] * scale_x, center[1] * scale_y)
        
        return {
            'center': scaled_center,
            'confidence': confidence
        }

    def _calculate_angle(self, central_center: Tuple[float, float], 
                        head_center: Tuple[float, float]) -> float:
        """
        计算角度
        直着向上为0度，顺时针为正值，范围[-180, 180]
        """
        dx = head_center[0] - central_center[0]
        dy = head_center[1] - central_center[1]
        
        angle_rad = math.atan2(dx, -dy)
        angle_deg = math.degrees(angle_rad)
        
        # 确保角度在[-180, 180]范围内
        if angle_deg > 180:
            angle_deg -= 360
        elif angle_deg < -180:
            angle_deg += 360
        
        return angle_deg

    def detect(self, image):
        results = self.model.predict(image, conf=0.25, iou=0.5)
        detection_result = results[0]
        masks = detection_result.masks.data.cpu().numpy()
        scores = detection_result.boxes.conf.cpu().numpy()
        classes = detection_result.boxes.cls.cpu().numpy().astype(int)

        # 计算缩放比例
        origin_height, origin_width = image.shape[:2]
        mask_height, mask_width = masks.shape[1], masks.shape[2]
        
        scale_x = origin_width / mask_width
        scale_y = origin_height / mask_height

        head_info = self._extract_target_info(masks, scores, classes, "head", scale_x, scale_y)
        central_info = self._extract_target_info(masks, scores, classes, "central", scale_x, scale_y)

        if head_info and central_info:
            angle = self._calculate_angle(central_info["center"], head_info["center"])
            return {
                "head_center": head_info["center"],
                "central_center": central_info["center"],
                "angle": angle
            }
        return {}
