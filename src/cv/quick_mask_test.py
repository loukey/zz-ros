#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
快速掩码测试脚本
"""

import cv2
import numpy as np
import os
import sys

# 添加detection_models目录到路径
sys.path.append(os.path.join(os.path.dirname(__file__), 'detection_models'))

from yolo_multi_detector import YOLOMultiDetector


def quick_mask_test():
    """快速掩码测试"""
    
    # 配置文件路径
    model_path = "./best-20250710.pt"
    image_path = "data/images/test/test1.jpg"
    output_dir = "data/mask_output"
    
    # 创建输出目录
    os.makedirs(output_dir, exist_ok=True)
    
    print("🎯 快速掩码测试")
    print("=" * 50)
    
    # 检查文件
    if not os.path.exists(model_path):
        print(f"❌ 模型文件不存在: {model_path}")
        return
    
    if not os.path.exists(image_path):
        print(f"❌ 图像文件不存在: {image_path}")
        return
    
    print(f"📁 模型: {model_path}")
    print(f"📁 图像: {image_path}")
    print(f"📁 输出: {output_dir}")
    
    try:
        # 创建检测器
        print("\n🔧 初始化检测器...")
        detector = YOLOMultiDetector(model_path, confidence_threshold=0.5)
        print("✅ 检测器初始化成功")
        
        # 加载图像
        image = cv2.imread(image_path)
        if image is None:
            print("❌ 无法加载图像")
            return
        
        print(f"📸 图像尺寸: {image.shape}")
        
        # 执行检测
        print("\n🔍 执行检测...")
        result = detector.detect(image)
        
        if not result['success']:
            print("❌ 检测失败，未找到目标")
            return
        
        print("✅ 检测成功!")
        print(f"   Central置信度: {result['central_confidence']:.3f}")
        print(f"   Head置信度: {result['head_confidence']:.3f}")
        print(f"   角度: {result['angle']:.1f}°")
        
        # 生成不同风格的可视化
        print("\n🎨 生成可视化结果...")
        
        # 1. 完整的检测结果（包含掩码）
        vis_complete = detector.visualize_results(image, result, show_masks=True, show_info=True)
        complete_path = os.path.join(output_dir, "complete_result.jpg")
        cv2.imwrite(complete_path, vis_complete)
        print(f"📄 完整结果: {complete_path}")
        
        # 2. 不含掩码的检测结果
        vis_no_mask = detector.visualize_results(image, result, show_masks=False, show_info=True)
        no_mask_path = os.path.join(output_dir, "no_mask_result.jpg")
        cv2.imwrite(no_mask_path, vis_no_mask)
        print(f"📄 无掩码结果: {no_mask_path}")
        
        # 3. 不同风格的掩码
        mask_styles = {
            'overlay': '半透明叠加',
            'solid': '纯色填充',
            'contour': '仅显示轮廓',
            'both': '填充+轮廓'
        }
        
        for style, description in mask_styles.items():
            mask_vis = detector.visualize_masks_only(image, result, mask_style=style)
            mask_path = os.path.join(output_dir, f"mask_{style}.jpg")
            cv2.imwrite(mask_path, mask_vis)
            print(f"📄 掩码({description}): {mask_path}")
        
        # 4. 创建对比图
        print("\n🔄 创建对比图...")
        create_comparison_image(image, result, detector, output_dir)
        
        print("\n✅ 测试完成!")
        print(f"📁 所有结果已保存到: {output_dir}")
        
        # 显示统计信息
        print("\n📊 性能统计:")
        detector.print_timing(result)
        
    except Exception as e:
        print(f"❌ 测试失败: {e}")
        import traceback
        traceback.print_exc()


def create_comparison_image(image, result, detector, output_dir):
    """创建对比图"""
    try:
        # 创建2x3的对比图
        h, w = image.shape[:2]
        
        # 生成6种不同的可视化
        images = []
        
        # 1. 原图
        images.append(cv2.putText(image.copy(), "Original", (10, 30), 
                                 cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2))
        
        # 2. 完整结果
        complete = detector.visualize_results(image, result, show_masks=True, show_info=False)
        images.append(cv2.putText(complete, "Complete", (10, 30), 
                                 cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2))
        
        # 3-6. 不同风格的掩码
        styles = ['overlay', 'solid', 'contour', 'both']
        for style in styles:
            mask_vis = detector.visualize_masks_only(image, result, mask_style=style)
            images.append(cv2.putText(mask_vis, style.title(), (10, 30), 
                                     cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2))
        
        # 调整图像大小
        target_size = (w//2, h//2)
        images = [cv2.resize(img, target_size) for img in images]
        
        # 创建拼接图
        row1 = np.hstack(images[:3])
        row2 = np.hstack(images[3:])
        comparison = np.vstack([row1, row2])
        
        # 保存对比图
        comparison_path = os.path.join(output_dir, "comparison.jpg")
        cv2.imwrite(comparison_path, comparison)
        print(f"📄 对比图: {comparison_path}")
        
    except Exception as e:
        print(f"❌ 创建对比图失败: {e}")


if __name__ == "__main__":
    quick_mask_test() 