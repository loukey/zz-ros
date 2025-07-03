#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
计算机视觉主程序
使用YOLO检测器进行图像分割和方向识别
"""

import os
import sys

# 添加detection_models路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'detection_models'))

from detection_models.yolo_detector import YOLODetector


def main():
    """
    主函数 - 执行完整的识别和保存流程
    """
    print("=== 语义分割和方向识别工具 ===")
    
    # 创建检测器
    detector = YOLODetector(model_path='./models_cache/yolo_seg_n.pt')
    
    # 执行检测和保存
    results = detector.process_and_save(
        images_dir='./data/origin_img',
        results_dir='./data/result'
    )
    
    if results:
        print("\n🎉 处理成功完成！")
    else:
        print("\n❌ 处理失败或没有找到图像")


if __name__ == "__main__":
    main() 