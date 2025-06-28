#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO11 OBB检测使用示例
"""

from pathlib import Path
from obb import YOLO11OBBDetector
import logging

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


def example_batch_detection():
    """批量检测示例"""
    logger.info("=== 批量检测示例 ===")
    
    # 设置路径
    current_dir = Path(__file__).parent.parent
    input_dir = current_dir / "data" / "images"
    output_dir = current_dir / "data" / "results" / "batch_detection"
    
    # 创建检测器
    detector = YOLO11OBBDetector(
        confidence=0.5,      # 置信度阈值
        iou_threshold=0.45   # IoU阈值
    )
    
    # 执行批量检测
    detector.detect_batch(str(input_dir), str(output_dir))


def example_single_detection():
    """单图检测示例"""
    logger.info("=== 单图检测示例 ===")
    
    # 设置路径
    current_dir = Path(__file__).parent.parent
    input_image = current_dir / "data" / "images" / "test_image.jpg"  # 替换为实际图像文件名
    output_dir = current_dir / "data" / "results" / "single_detection"
    
    # 创建检测器
    detector = YOLO11OBBDetector(
        confidence=0.3,      # 较低的置信度阈值用于检测更多目标
        iou_threshold=0.45
    )
    
    # 执行单图检测
    detector.detect_single(str(input_image), str(output_dir))


def example_custom_model():
    """使用自定义模型示例"""
    logger.info("=== 自定义模型检测示例 ===")
    
    # 设置路径
    current_dir = Path(__file__).parent.parent
    input_dir = current_dir / "data" / "images"
    output_dir = current_dir / "data" / "results" / "custom_model"
    custom_model_path = current_dir / "models" / "my_custom_obb_model.pt"  # 替换为你的模型路径
    
    # 创建检测器（使用自定义模型）
    detector = YOLO11OBBDetector(
        model_path=str(custom_model_path),  # 指定自定义模型路径
        confidence=0.6,
        iou_threshold=0.4
    )
    
    # 执行检测
    detector.detect_batch(str(input_dir), str(output_dir))


def main():
    """主函数"""
    print("选择检测模式:")
    print("1. 批量检测 (默认)")
    print("2. 单图检测")
    print("3. 使用自定义模型")
    
    choice = input("请输入选择 (1-3, 默认为1): ").strip()
    
    if choice == "2":
        example_single_detection()
    elif choice == "3":
        example_custom_model()
    else:
        example_batch_detection()


if __name__ == "__main__":
    main() 