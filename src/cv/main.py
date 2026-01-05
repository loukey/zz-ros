#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
计算机视觉主程序入口
使用重构后的 YOLO-OBB 检测器
"""

import os
import sys

# 添加 core 路径以便导入模块
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'core'))

from detectors.obb import YOLOOBBDetector

def main():
    print("=== YOLO OBB 机器人视觉系统 ===")
    
    # 1. 配置路径
    model_path = './weights/best.pt' # 确保这里有训练好的模型
    images_dir = './data/raw'
    output_dir = './data/outputs'
    
    if not os.path.exists(model_path):
        print(f"❌ 模型文件不存在: {model_path}")
        print("请先运行 generate_yolo_obb_data.py 生成数据并训练模型")
        return

    # 2. 初始化检测器
    detector = YOLOOBBDetector(
        model_path=model_path,
        conf_threshold=0.5
    )
    
    # 3. 批量处理图片
    if not os.path.exists(images_dir):
        print(f"❌ 图片目录不存在: {images_dir}")
        return
        
    image_files = [f for f in os.listdir(images_dir) if f.lower().endswith(('.png', '.jpg', '.jpeg'))]
    print(f"找到 {len(image_files)} 张图片")
    
    for img_file in image_files:
        img_path = os.path.join(images_dir, img_file)
        print(f"\n正在处理: {img_file}")
        
        try:
            result = detector.detect_image(
                img_path, 
                save_result=True, 
                output_dir=output_dir
            )
            
            # 打印结果
            pose = result['pose_result']
            if pose:
                print(f"  ✅ 姿态解算成功!")
                print(f"  角度: {pose.final_angle * 180 / 3.14159:.1f}°")
                print(f"  真实中心: ({pose.real_center[0]:.1f}, {pose.real_center[1]:.1f})")
            else:
                print(f"  ⚠️ 未检测到完整姿态 (Head+Central)")
                print(f"  检测到: {len(result['detections'])} 个目标")
                
        except Exception as e:
            print(f"  ❌ 处理失败: {e}")

if __name__ == "__main__":
    main()
