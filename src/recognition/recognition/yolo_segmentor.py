import numpy as np
import cv2
import math
from typing import Tuple, Optional, Dict
from ultralytics import YOLO


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
        dx = central_center[0] - head_center[0]
        dy = central_center[1] - head_center[1]
        
        angle_rad = math.atan2(dx, -dy)
        if angle_rad > math.pi:
            angle_rad -= 2 * math.pi
        elif angle_rad < -math.pi/2:
            angle_rad += 2 * math.pi
            
        return angle_rad

    def detect(self, image):
        results = self.model.predict(image, conf=0.25, iou=0.5)
        detection_result = results[0]
        
        # 检查是否有检测结果
        if detection_result.masks is None or detection_result.boxes is None:
            return {}
        
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
