#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO11 OBB (Oriented Bounding Box) 检测模块
使用ultralytics YOLO11进行旋转边界框检测
"""

import os
import cv2
import json
import numpy as np
from pathlib import Path
from ultralytics import YOLO
from datetime import datetime
import logging

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


class YOLO11OBBDetector:
    """YOLO11 OBB检测器"""
    
    def __init__(self, model_path=None, confidence=0.5, iou_threshold=0.45):
        """
        初始化YOLO11 OBB检测器
        
        Args:
            model_path (str): 模型路径，如果为None则使用预训练模型
            confidence (float): 置信度阈值
            iou_threshold (float): IoU阈值
        """
        self.confidence = confidence
        self.iou_threshold = iou_threshold
        
        # 加载模型
        if model_path and os.path.exists(model_path):
            logger.info(f"加载自定义模型: {model_path}")
            self.model = YOLO(model_path)
        else:
            logger.info("加载YOLO11n-obb预训练模型")
            # 使用YOLO11的OBB预训练模型
            self.model = YOLO('yolo11x-obb.pt')
        
        # 支持的图像格式
        self.supported_formats = {'.jpg', '.jpeg', '.png', '.bmp', '.tiff', '.tif', '.webp'}
        
    def detect_image(self, image_path):
        """
        对单张图像进行OBB检测
        
        Args:
            image_path (str): 图像路径
            
        Returns:
            results: YOLO检测结果
        """
        try:
            # 进行检测
            results = self.model(
                image_path,
                conf=self.confidence,
                iou=self.iou_threshold,
                verbose=False
            )
            return results[0] if results else None
            
        except Exception as e:
            logger.error(f"检测图像 {image_path} 时出错: {str(e)}")
            return None
    
    def save_results(self, results, image_path, output_dir):
        """
        保存检测结果
        
        Args:
            results: YOLO检测结果
            image_path (str): 原始图像路径
            output_dir (str): 输出目录
        """
        if not results or len(results.obb) == 0:
            logger.info(f"图像 {os.path.basename(image_path)} 中未检测到目标")
            return
            
        image_name = Path(image_path).stem
        
        # 保存带注释的图像
        annotated_image = results.plot()
        output_image_path = os.path.join(output_dir, f"{image_name}_detected.jpg")
        cv2.imwrite(output_image_path, annotated_image)
        
        # 保存检测结果为JSON
        detections = []
        if results.obb is not None:
            for i, (xyxyxyxy, conf, cls) in enumerate(zip(
                results.obb.xyxyxyxy.cpu().numpy(),
                results.obb.conf.cpu().numpy(),  
                results.obb.cls.cpu().numpy()
            )):
                detection = {
                    'id': i,
                    'class_id': int(cls),
                    'class_name': self.model.names[int(cls)],
                    'confidence': float(conf),
                    'obb_points': xyxyxyxy.tolist(),  # 8个坐标点 [x1,y1,x2,y2,x3,y3,x4,y4]
                }
                detections.append(detection)
        
        # 保存JSON结果
        json_output_path = os.path.join(output_dir, f"{image_name}_results.json")
        result_data = {
            'image_path': image_path,
            'image_name': image_name,
            'detection_time': datetime.now().isoformat(),
            'model_confidence': self.confidence,
            'total_detections': len(detections),
            'detections': detections
        }
        
        with open(json_output_path, 'w', encoding='utf-8') as f:
            json.dump(result_data, f, ensure_ascii=False, indent=2)
            
        logger.info(f"检测完成: {image_name}, 发现 {len(detections)} 个目标")
        logger.info(f"结果已保存: {output_image_path}, {json_output_path}")
    
    def detect_batch(self, input_dir, output_dir):
        """
        批量检测图像
        
        Args:
            input_dir (str): 输入图像目录
            output_dir (str): 输出结果目录
        """
        # 确保输出目录存在
        os.makedirs(output_dir, exist_ok=True)
        
        # 获取所有支持的图像文件
        image_files = []
        for ext in self.supported_formats:
            image_files.extend(Path(input_dir).glob(f"*{ext}"))
            image_files.extend(Path(input_dir).glob(f"*{ext.upper()}"))
        
        if not image_files:
            logger.warning(f"在目录 {input_dir} 中未找到支持的图像文件")
            return
        
        logger.info(f"找到 {len(image_files)} 张图像，开始检测...")
        
        # 批量检测
        for i, image_path in enumerate(image_files, 1):
            logger.info(f"正在处理 ({i}/{len(image_files)}): {image_path.name}")
            
            # 检测图像
            results = self.detect_image(str(image_path))
            
            if results:
                # 保存结果
                self.save_results(results, str(image_path), output_dir)
            else:
                logger.warning(f"跳过图像: {image_path.name}")
        
        logger.info(f"批量检测完成！结果保存在: {output_dir}")
    
    def detect_single(self, image_path, output_dir):
        """
        检测单张图像
        
        Args:
            image_path (str): 图像路径
            output_dir (str): 输出目录
        """
        # 确保输出目录存在
        os.makedirs(output_dir, exist_ok=True)
        
        if not os.path.exists(image_path):
            logger.error(f"图像文件不存在: {image_path}")
            return
        
        # 检查文件格式
        file_ext = Path(image_path).suffix.lower()
        if file_ext not in self.supported_formats:
            logger.error(f"不支持的图像格式: {file_ext}")
            return
        
        logger.info(f"开始检测图像: {os.path.basename(image_path)}")
        
        # 检测
        results = self.detect_image(image_path)
        
        if results:
            self.save_results(results, image_path, output_dir)
        else:
            logger.error(f"检测失败: {image_path}")


def main():
    """主函数"""
    # 设置路径
    current_dir = Path(__file__).parent.parent
    input_dir = current_dir / "data" / "images"
    output_dir = current_dir / "data" / "results"
    
    logger.info("=== YOLO11 OBB 检测开始 ===")
    logger.info(f"输入目录: {input_dir}")
    logger.info(f"输出目录: {output_dir}")
    
    # 创建检测器
    detector = YOLO11OBBDetector(
        confidence=0.5,  # 置信度阈值
        iou_threshold=0.45  # IoU阈值
    )
    
    # 批量检测
    detector.detect_batch(str(input_dir), str(output_dir))
    
    logger.info("=== YOLO11 OBB 检测结束 ===")


if __name__ == "__main__":
    main()
