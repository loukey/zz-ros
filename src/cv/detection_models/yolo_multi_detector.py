#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO多目标分割检测器 - 生产版本
用于实际环境的纯推理模型
"""

import cv2
import numpy as np
from ultralytics import YOLO
import math
import time
from typing import Dict, Tuple, Optional, Any
import os


class YOLOMultiDetector:
    """
    YOLO多目标分割检测器 - 生产版本
    专门用于实际环境的高效推理
    """
    
    def __init__(self, model_path: str, confidence_threshold: float = 0.5):
        """
        初始化检测器
        
        Args:
            model_path: YOLO模型文件路径
            confidence_threshold: 置信度阈值
        """
        self.model_path = model_path
        self.confidence_threshold = confidence_threshold
        self.model = None
        self.class_mapping = {}
        self.target_classes = ['head', 'central']
        
        self._load_model()
    
    def _load_model(self):
        """加载YOLO模型"""
        if not os.path.exists(self.model_path):
            raise FileNotFoundError(f"模型文件不存在: {self.model_path}")
        
        self.model = YOLO(self.model_path)
        
        # 获取类别映射
        if hasattr(self.model, 'names'):
            self.class_mapping = {name: idx for idx, name in enumerate(self.model.names.values())}
        
        # 验证目标类别是否存在
        missing_classes = [cls for cls in self.target_classes if cls not in self.class_mapping]
        if missing_classes:
            raise ValueError(f"模型中缺少必要类别: {missing_classes}")
    
    def detect(self, image: np.ndarray) -> Dict[str, Any]:
        """
        检测图像中的目标
        
        Args:
            image: 输入图像 (BGR格式)
            
        Returns:
            检测结果字典:
            {
                'head_center': (x, y) or None,
                'central_center': (x, y) or None,
                'angle': float or None,
                'head_confidence': float or None,
                'central_confidence': float or None,
                'success': bool,
                'timing': {
                    'inference': float,
                    'mask_processing': float,
                    'target_extraction': float,
                    'angle_calculation': float,
                    'total': float
                }
            }
        """
        start_total = time.perf_counter()
        
        # 执行推理
        if self.model is None:
            raise RuntimeError("模型未加载")
        
        start_inference = time.perf_counter()
        results = self.model(image, conf=self.confidence_threshold, verbose=False)
        inference_time = time.perf_counter() - start_inference
        
        # 初始化结果
        result = {
            'head_center': None,
            'central_center': None,
            'angle': None,
            'head_confidence': None,
            'central_confidence': None,
            'success': False,
            'timing': {
                'inference': inference_time,
                'mask_processing': 0.0,
                'target_extraction': 0.0,
                'angle_calculation': 0.0,
                'total': 0.0
            }
        }
        
        if len(results) == 0 or not hasattr(results[0], 'masks') or results[0].masks is None:
            result['timing']['total'] = time.perf_counter() - start_total
            return result
        
        detection_result = results[0]
        
        # 获取检测数据并恢复掩码尺寸
        start_mask = time.perf_counter()
        masks = detection_result.masks.data.cpu().numpy()
        scores = detection_result.boxes.conf.cpu().numpy()
        classes = detection_result.boxes.cls.cpu().numpy().astype(int)
        
        orig_height, orig_width = image.shape[:2]
        if masks.shape[1] != orig_height or masks.shape[2] != orig_width:
            resized_masks = []
            for mask in masks:
                resized_mask = cv2.resize(mask, (orig_width, orig_height), interpolation=cv2.INTER_LINEAR)
                resized_masks.append(resized_mask)
            masks = np.array(resized_masks)
        
        mask_time = time.perf_counter() - start_mask
        result['timing']['mask_processing'] = mask_time
        
        # 提取目标信息
        start_extraction = time.perf_counter()
        head_info = self._extract_target_info(masks, scores, classes, 'head')
        central_info = self._extract_target_info(masks, scores, classes, 'central')
        extraction_time = time.perf_counter() - start_extraction
        result['timing']['target_extraction'] = extraction_time
        
        # 填充结果
        if head_info:
            result['head_center'] = head_info['center']
            result['head_confidence'] = head_info['confidence']
        
        if central_info:
            result['central_center'] = central_info['center']
            result['central_confidence'] = central_info['confidence']
        
        # 计算角度
        start_angle = time.perf_counter()
        if head_info and central_info:
            result['angle'] = self._calculate_angle(central_info['center'], head_info['center'])
            result['success'] = True
        angle_time = time.perf_counter() - start_angle
        result['timing']['angle_calculation'] = angle_time
        
        # 总时间
        result['timing']['total'] = time.perf_counter() - start_total
        
        return result
    
    def _extract_target_info(self, masks: np.ndarray, scores: np.ndarray, 
                            classes: np.ndarray, target_class: str) -> Optional[Dict]:
        """提取目标信息"""
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
        
        return {
            'center': center,
            'confidence': confidence
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
    
    def detect_from_file(self, image_path: str) -> Dict[str, Any]:
        """从文件检测"""
        image = cv2.imread(image_path)
        if image is None:
            raise ValueError(f"无法读取图像: {image_path}")
        
        return self.detect(image)
    
    def is_ready(self) -> bool:
        """检查模型是否准备就绪"""
        return self.model is not None
    
    def print_timing(self, result: Dict[str, Any]):
        """打印时间统计信息"""
        if 'timing' not in result:
            print("结果中没有时间信息")
            return
        
        timing = result['timing']
        print("时间统计:")
        print(f"  推理时间: {timing['inference']*1000:.1f}ms")
        print(f"  掩码处理: {timing['mask_processing']*1000:.1f}ms")
        print(f"  目标提取: {timing['target_extraction']*1000:.1f}ms")
        print(f"  角度计算: {timing['angle_calculation']*1000:.1f}ms")
        print(f"  总时间: {timing['total']*1000:.1f}ms")
        
        # 显示各阶段占比
        total = timing['total']
        if total > 0:
            print("时间占比:")
            print(f"  推理: {timing['inference']/total*100:.1f}%")
            print(f"  掩码处理: {timing['mask_processing']/total*100:.1f}%")
            print(f"  目标提取: {timing['target_extraction']/total*100:.1f}%")
            print(f"  角度计算: {timing['angle_calculation']/total*100:.1f}%")


# 使用示例
if __name__ == "__main__":
    # 创建检测器
    detector = YOLOMultiDetector("../models_cache/yolo_multi_seg_n.pt")
    
    # 检测图像
    result = detector.detect_from_file("../data/origin_img/img1.jpg")
    
    # 输出结果
    if result['success']:
        print(f"检测成功！")
        print(f"Central中心: {result['central_center']}")
        print(f"Head中心: {result['head_center']}")
        print(f"角度: {result['angle']:.1f}°")
        print(f"置信度 - Central: {result['central_confidence']:.3f}, Head: {result['head_confidence']:.3f}")
    else:
        print("检测失败 - 未找到所需目标")
    
    # 显示时间统计
    print()
    detector.print_timing(result)
