#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO多目标检测器可视化测试脚本
"""

import cv2
import numpy as np
import os
import sys

# 添加detection_models目录到路径
sys.path.append(os.path.join(os.path.dirname(__file__), 'detection_models'))

from yolo_multi_detector import YOLOMultiDetector


def test_visualization():
    """测试可视化功能"""
    
    # 模型和图像路径
    model_path = "./best-20250710.pt"
    image_path = "data/images/test/test1.jpg"
    
    # 检查文件是否存在
    if not os.path.exists(model_path):
        print(f"模型文件不存在: {model_path}")
        print("请检查模型文件路径")
        return
    
    if not os.path.exists(image_path):
        print(f"图像文件不存在: {image_path}")
        print("请检查图像文件路径")
        return
    
    print("正在初始化检测器...")
    
    try:
        # 创建检测器
        detector = YOLOMultiDetector(model_path, confidence_threshold=0.5)
        
        print("检测器初始化成功！")
        print(f"目标类别: {detector.target_classes}")
        print(f"置信度阈值: {detector.confidence_threshold}")
        print("=" * 60)
        
        # 测试1: 基本检测
        print("测试1: 基本检测")
        result = detector.detect_from_file(image_path)
        
        if result['success']:
            print("✓ 检测成功!")
            print(f"  Central中心: ({result['central_center'][0]:.1f}, {result['central_center'][1]:.1f})")
            print(f"  Head中心: ({result['head_center'][0]:.1f}, {result['head_center'][1]:.1f})")
            print(f"  角度: {result['angle']:.1f}°")
            print(f"  置信度 - Central: {result['central_confidence']:.3f}")
            print(f"  置信度 - Head: {result['head_confidence']:.3f}")
        else:
            print("✗ 检测失败")
            print("  未找到所需目标")
        
        # 显示时间统计
        print("\n时间统计:")
        detector.print_timing(result)
        
        print("\n" + "=" * 60)
        
        # 测试2: 可视化检测（保存图像）
        print("测试2: 可视化检测（保存图像）")
        
        # 确保输出目录存在
        output_dir = "data/output"
        os.makedirs(output_dir, exist_ok=True)
        
        save_path = os.path.join(output_dir, "detection_result.jpg")
        
        try:
            result, vis_image = detector.detect_and_visualize_from_file(
                image_path, 
                save_path=save_path,
                show_image=False  # 不显示窗口，仅保存
            )
            
            print(f"✓ 可视化结果已保存到: {save_path}")
            print(f"  图像尺寸: {vis_image.shape}")
            
        except Exception as e:
            print(f"✗ 可视化保存失败: {e}")
        
        print("\n" + "=" * 60)
        
        # 测试3: 交互式可视化（显示窗口）
        print("测试3: 交互式可视化")
        print("提示: 图像窗口将会打开，按任意键关闭")
        
        user_input = input("是否要显示可视化窗口？(y/n): ")
        
        if user_input.lower() == 'y':
            try:
                result, vis_image = detector.detect_and_visualize_from_file(
                    image_path, 
                    save_path=None,
                    show_image=True  # 显示窗口
                )
                print("✓ 可视化窗口已关闭")
                
            except Exception as e:
                print(f"✗ 可视化显示失败: {e}")
        else:
            print("跳过交互式可视化")
        
        print("\n" + "=" * 60)
        
        # 测试4: 掩码可视化
        print("测试4: 掩码可视化")
        
        try:
            # 不同的掩码样式
            mask_styles = ['overlay', 'solid', 'contour', 'both']
            
            for style in mask_styles:
                print(f"生成 {style} 风格的掩码...")
                
                # 执行检测
                result = detector.detect_from_file(image_path)
                
                if result['success']:
                    # 生成掩码可视化
                    mask_vis = detector.visualize_masks_only(
                        cv2.imread(image_path), 
                        result, 
                        mask_style=style
                    )
                    
                    # 保存不同风格的掩码
                    save_path = os.path.join(output_dir, f"mask_{style}.jpg")
                    cv2.imwrite(save_path, mask_vis)
                    print(f"  ✓ 保存到: {save_path}")
                else:
                    print(f"  ✗ 检测失败，无法生成掩码")
                    
        except Exception as e:
            print(f"✗ 掩码可视化失败: {e}")
        
        print("\n" + "=" * 60)
        print("测试完成!")
        
        # 显示使用说明
        print("\n可视化说明:")
        print("🎯 完整检测结果:")
        print("1. 🟢 绿色圆点和圆环 - Central目标中心")
        print("2. 🔴 红色圆点和圆环 - Head目标中心")
        print("3. 🔵 青色连接线 - 连接两个中心点")
        print("4. 🟣 紫色箭头 - 角度方向指示")
        print("5. 📝 左上角文本 - 角度、置信度、坐标信息")
        print("6. 🎨 半透明掩码 - 目标区域覆盖")
        print("")
        print("🎭 掩码样式:")
        print("• overlay - 半透明叠加（默认）")
        print("• solid - 纯色填充")
        print("• contour - 仅显示轮廓")
        print("• both - 填充+轮廓")
        
    except Exception as e:
        print(f"初始化检测器失败: {e}")
        print("请检查模型文件和依赖项")


def test_with_webcam():
    """测试摄像头实时检测"""
    
    model_path = "models_cache/yolo_multi_seg_n.pt"
    
    if not os.path.exists(model_path):
        print(f"模型文件不存在: {model_path}")
        return
    
    print("正在初始化摄像头检测...")
    
    try:
        # 创建检测器
        detector = YOLOMultiDetector(model_path, confidence_threshold=0.5)
        
        # 打开摄像头
        cap = cv2.VideoCapture(0)
        
        if not cap.isOpened():
            print("无法打开摄像头")
            return
        
        print("摄像头检测已启动，按 'q' 退出")
        
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            # 执行检测
            result = detector.detect(frame)
            
            # 可视化结果
            vis_frame = detector.visualize_results(frame, result)
            
            # 显示帧率
            fps_text = f"FPS: {1000/result['timing']['total']:.1f}"
            cv2.putText(vis_frame, fps_text, (10, vis_frame.shape[0] - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # 显示结果
            cv2.imshow('Real-time Detection', vis_frame)
            
            # 按 'q' 退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        cap.release()
        cv2.destroyAllWindows()
        print("摄像头检测结束")
        
    except Exception as e:
        print(f"摄像头检测失败: {e}")


if __name__ == "__main__":
    print("YOLO多目标检测器可视化测试")
    print("=" * 60)
    
    # 测试静态图像
    test_visualization()
    
    print("\n" + "=" * 60)
    
    # 询问是否测试摄像头
    user_input = input("是否要测试摄像头实时检测？(y/n): ")
    if user_input.lower() == 'y':
        test_with_webcam()
    
    print("\n测试程序结束") 