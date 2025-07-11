#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO11-OBB 检测器
用于加载训练好的YOLO11-OBB模型并进行旋转边界框检测
"""

import os
import cv2
import numpy as np
import torch
from ultralytics import YOLO
from typing import List, Dict, Any, Tuple, Optional, Union
import json
import yaml
from pathlib import Path
import argparse
from datetime import datetime
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Polygon
import math


class YOLO11OBBDetector:
    """YOLO11-OBB检测器"""
    
    def __init__(self, model_path: str, conf_threshold: float = 0.001, iou_threshold: float = 0.45):
        """
        初始化检测器
        
        Args:
            model_path: 模型文件路径
            conf_threshold: 置信度阈值 (默认0.001，与训练验证一致)
            iou_threshold: IoU阈值
        """
        self.model_path = model_path
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold
        self.model = None
        self.class_names = []
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        
        # 加载模型
        self.load_model()
        
        print(f"YOLO11-OBB检测器初始化完成")
        print(f"模型: {model_path}")
        print(f"设备: {self.device}")
        print(f"置信度阈值: {conf_threshold}")
        print(f"IoU阈值: {iou_threshold}")
    
    def load_model(self):
        """加载YOLO11-OBB模型"""
        try:
            self.model = YOLO(self.model_path)
            print(f"✅ 模型加载成功: {self.model_path}")
            
            # 获取类别名称
            if hasattr(self.model, 'names'):
                self.class_names = list(self.model.names.values())
                print(f"类别: {self.class_names}")
                print(f"类别数量: {len(self.class_names)}")
            else:
                print("⚠️ 模型没有类别信息")
                self.class_names = []
            
            # 打印模型信息
            print(f"模型类型: {type(self.model)}")
            if hasattr(self.model, 'model'):
                print(f"内部模型: {type(self.model.model)}")
            
        except Exception as e:
            print(f"❌ 模型加载失败: {e}")
            raise
    
    def detect_image(self, image: Union[str, np.ndarray], 
                    save_result: bool = True, 
                    output_dir: str = "./results") -> Dict[str, Any]:
        """
        检测单张图像
        
        Args:
            image: 图像路径或numpy数组
            save_result: 是否保存结果
            output_dir: 输出目录
            
        Returns:
            检测结果字典
        """
        # 检查模型是否已加载
        if self.model is None:
            raise RuntimeError("模型未加载，请先调用load_model()")
        
        # 加载图像
        if isinstance(image, str):
            if not os.path.exists(image):
                raise FileNotFoundError(f"图像文件不存在: {image}")
            img = cv2.imread(image)
            image_name = os.path.basename(image)
        else:
            img = image.copy()
            image_name = f"image_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        
        if img is None:
            raise ValueError("无法读取图像")
        
        # 执行检测
        results = self.model(img)
        
        # 解析结果
        detection_results = self.parse_results(results, tuple(img.shape))
        
        # 可视化结果
        if save_result:
            os.makedirs(output_dir, exist_ok=True)
            vis_img = self.visualize_detections(img, detection_results)
            
            # 保存可视化结果
            output_path = os.path.join(output_dir, f"obb_detection_{image_name}")
            cv2.imwrite(output_path, vis_img)
            print(f"检测结果已保存: {output_path}")
        
        return detection_results
    
    def parse_results(self, results, image_shape: Tuple[int, ...]) -> Dict[str, Any]:
        """
        解析YOLO11-OBB检测结果
        
        Args:
            results: YOLO检测结果
            image_shape: 图像形状 (height, width, channels)
            
        Returns:
            解析后的检测结果
        """
        height, width = image_shape[:2]
        detections = []
        
        for result in results:
            print(f"处理结果: boxes={result.boxes}, obb={result.obb}")
            
            # 检查OBB结果
            if result.obb is not None:
                obb = result.obb
                print(f"检测到 {len(obb)} 个OBB目标")
                
                for i in range(len(obb)):
                    # 获取OBB信息
                    coords = obb.xyxyxyxy[i].cpu().numpy()  # 四个角点坐标
                    conf = float(obb.conf[i].cpu().numpy())
                    cls_id = int(obb.cls[i].cpu().numpy())
                    cls_name = self.class_names[cls_id] if cls_id < len(self.class_names) else f"class_{cls_id}"
                    
                    print(f"  目标 {i+1}: 类别={cls_name}, 置信度={conf:.3f}")
                    
                    # 计算中心点 - 使用四个角点的几何中心
                    corners_array = coords.reshape(4, 2)
                    center_x = corners_array[:, 0].mean()
                    center_y = corners_array[:, 1].mean()
                    
                    # 计算旋转角度
                    angle = self.calculate_obb_angle(coords)
                    
                    detection = {
                        'class_id': cls_id,
                        'class_name': cls_name,
                        'confidence': conf,
                        'center': [float(center_x), float(center_y)],
                        'corners': corners_array.tolist(),
                        'angle': angle,
                        'bbox': [
                            float(corners_array[:, 0].min()),  # x_min
                            float(corners_array[:, 1].min()),  # y_min
                            float(corners_array[:, 0].max()),  # x_max
                            float(corners_array[:, 1].max())   # y_max
                        ]
                    }
                    detections.append(detection)
            
            # 如果没有OBB结果，检查普通边界框
            elif result.boxes is not None:
                boxes = result.boxes
                print(f"检测到 {len(boxes)} 个普通边界框")
                
                for i in range(len(boxes)):
                    # 获取置信度和类别
                    conf = float(boxes.conf[i].cpu().numpy())
                    cls_id = int(boxes.cls[i].cpu().numpy())
                    cls_name = self.class_names[cls_id] if cls_id < len(self.class_names) else f"class_{cls_id}"
                    
                    print(f"  目标 {i+1}: 类别={cls_name}, 置信度={conf:.3f}")
                    
                    # 处理普通边界框，转换为OBB格式
                    bbox = boxes.xyxy[i].cpu().numpy()  # [x1, y1, x2, y2]
                    x1, y1, x2, y2 = bbox
                    
                    # 创建四个角点（矩形）
                    corners = np.array([
                        [x1, y1],  # 左上
                        [x2, y1],  # 右上
                        [x2, y2],  # 右下
                        [x1, y2]   # 左下
                    ], dtype=np.float32)
                    
                    # 计算中心点
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2
                    
                    detection = {
                        'class_id': cls_id,
                        'class_name': cls_name,
                        'confidence': conf,
                        'center': [float(center_x), float(center_y)],
                        'corners': corners.tolist(),
                        'angle': 0.0,  # 矩形角度为0
                        'bbox': [float(x1), float(y1), float(x2), float(y2)]
                    }
                    detections.append(detection)
            else:
                print("没有检测到任何目标")
        
        # 只保留每个类别置信度最高的检测结果
        best_detections = {}
        for detection in detections:
            cls_name = detection['class_name']
            if cls_name not in best_detections or detection['confidence'] > best_detections[cls_name]['confidence']:
                best_detections[cls_name] = detection
        
        final_detections = list(best_detections.values())
        
        print(f"原始检测结果: {len(detections)} 个")
        print(f"筛选后检测结果: {len(final_detections)} 个")
        for detection in final_detections:
            print(f"  最佳 {detection['class_name']}: 置信度={detection['confidence']:.3f}")
        
        return {
            'image_shape': image_shape,
            'detections': final_detections,
            'total_detections': len(final_detections)
        }
    
    def calculate_obb_angle(self, coords: np.ndarray) -> float:
        """
        计算OBB的旋转角度
        
        Args:
            coords: 四个角点坐标 [x1,y1,x2,y2,x3,y3,x4,y4]
            
        Returns:
            旋转角度(度) - 直着向上为0度，顺时针为正值，范围[-180, 180]
        """
        # 重新排列为4x2的坐标矩阵
        points = coords.reshape(4, 2)
        
        # 计算长边的方向向量
        # 假设前两个点是长边的端点
        edge1 = points[1] - points[0]
        edge2 = points[2] - points[1]
        
        # 选择较长的边作为主方向
        if np.linalg.norm(edge1) > np.linalg.norm(edge2):
            main_edge = edge1
        else:
            main_edge = edge2
        
        # 计算角度 - 直着向上为0度，顺时针为正值
        # 向上的向量是 (0, -1)，因为图像坐标系Y轴向下
        # 使用 atan2 计算方向向量与向上方向的夹角
        angle_rad = math.atan2(main_edge[0], -main_edge[1])
        angle_deg = math.degrees(angle_rad)
        
        # 标准化到[-180, 180]度
        if angle_deg > 180:
            angle_deg -= 360
        elif angle_deg < -180:
            angle_deg += 360
        
        return angle_deg
    
    def visualize_detections(self, image: np.ndarray, detection_results: Dict[str, Any]) -> np.ndarray:
        """
        可视化检测结果
        
        Args:
            image: 原始图像
            detection_results: 检测结果
            
        Returns:
            可视化后的图像
        """
        vis_img = image.copy()
        detections = detection_results['detections']
        
        # 定义颜色
        colors = {
            'central': (0, 255, 0),   # 绿色
            'head': (0, 0, 255),      # 红色
        }
        
        # 分别存储central和head的检测结果
        central_detection = None
        head_detection = None
        
        # 首先绘制所有检测框和标签
        for detection in detections:
            cls_name = detection['class_name']
            conf = detection['confidence']
            corners = np.array(detection['corners'], dtype=np.int32)
            center = detection['center']
            
            # 选择颜色
            color = colors.get(cls_name, (255, 0, 0))
            
            # 绘制旋转边界框
            cv2.polylines(vis_img, [corners], True, color, 2)
            
            # 绘制中心点
            center_point = (int(center[0]), int(center[1]))
            cv2.circle(vis_img, center_point, 5, color, -1)
            
            # 绘制标签
            label = f"{cls_name}: {conf:.2f}"
            label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
            
            # 标签背景
            label_bg_pts = np.array([
                [corners[0][0], corners[0][1] - label_size[1] - 10],
                [corners[0][0] + label_size[0] + 10, corners[0][1] - label_size[1] - 10],
                [corners[0][0] + label_size[0] + 10, corners[0][1]],
                [corners[0][0], corners[0][1]]
            ], dtype=np.int32)
            
            cv2.fillPoly(vis_img, [label_bg_pts], color)
            cv2.putText(vis_img, label, 
                       (corners[0][0] + 5, corners[0][1] - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # 保存central和head的检测结果
            if cls_name == 'central':
                central_detection = detection
            elif cls_name == 'head':
                head_detection = detection
        
        # 绘制从central指向head的方向箭头
        if central_detection is not None and head_detection is not None:
            # 使用已经计算好的中心点（与绘制的中心点保持一致）
            central_center = np.array(central_detection['center'])
            head_center = np.array(head_detection['center'])
            
            # 获取central的旋转角度
            central_angle = central_detection['angle']
            
            # 计算从central到head的方向向量
            dx = head_center[0] - central_center[0]
            dy = head_center[1] - central_center[1]
            
            # 计算距离
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance > 0:
                # 计算旋转框的方向向量（沿着旋转角度）
                # 角度定义：直着向上为0度，顺时针为正值
                # 向上方向为 (0, -1)，所以需要相应调整
                box_dir_x = math.sin(math.radians(central_angle))
                box_dir_y = -math.cos(math.radians(central_angle))
                
                # 计算从central到head的单位向量
                head_dir_x = dx / distance
                head_dir_y = dy / distance
                
                # 计算旋转框方向与head方向的点积，判断应该指向哪个方向
                dot_product = box_dir_x * head_dir_x + box_dir_y * head_dir_y
                
                # 如果点积为负，说明应该反向
                if dot_product < 0:
                    box_dir_x = -box_dir_x
                    box_dir_y = -box_dir_y
                
                # 箭头长度
                arrow_length = 50
                
                # 计算箭头的起点和终点（沿着旋转框方向）
                arrow_start = (int(central_center[0]), int(central_center[1]))
                arrow_end_x = central_center[0] + arrow_length * box_dir_x
                arrow_end_y = central_center[1] + arrow_length * box_dir_y
                arrow_end = (int(arrow_end_x), int(arrow_end_y))
                
                # 绘制箭头（平行于旋转框，指向head方向）
                cv2.arrowedLine(vis_img, arrow_start, arrow_end, (255, 0, 0), 3, tipLength=0.3)
                
                # 在箭头旁边添加方向标签
                direction_label = f"Direction: {central_angle:.1f}"
                cv2.putText(vis_img, direction_label, 
                           (arrow_start[0] + 10, arrow_start[1] - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        
        return vis_img
    

    
    def batch_detect(self, image_dir: str, output_dir: str = "./batch_results") -> Dict[str, Any]:
        """
        批量检测图像
        
        Args:
            image_dir: 图像目录
            output_dir: 输出目录
            
        Returns:
            批量检测结果统计
        """
        os.makedirs(output_dir, exist_ok=True)
        
        # 支持的图像格式
        image_extensions = ['.jpg', '.jpeg', '.png', '.bmp', '.tiff', '.tif']
        
        # 获取所有图像文件
        image_files = []
        for ext in image_extensions:
            image_files.extend(Path(image_dir).glob(f"*{ext}"))
            image_files.extend(Path(image_dir).glob(f"*{ext.upper()}"))
        
        if not image_files:
            print(f"在 {image_dir} 中未找到图像文件")
            return {}
        
        print(f"找到 {len(image_files)} 个图像文件")
        
        # 批量检测
        results_summary = {
            'total_images': len(image_files),
            'processed_images': 0,
            'total_detections': 0,
            'class_statistics': {},
            'processing_time': 0
        }
        
        start_time = datetime.now()
        
        for i, image_path in enumerate(image_files):
            try:
                print(f"\n处理图像 {i+1}/{len(image_files)}: {image_path.name}")
                
                # 检测图像
                detection_results = self.detect_image(
                    str(image_path), 
                    save_result=True, 
                    output_dir=output_dir
                )
                
                print(f"  检测到 {detection_results['total_detections']} 个目标")
                
                # 更新统计信息
                results_summary['processed_images'] += 1
                results_summary['total_detections'] += detection_results['total_detections']
                
                # 更新类别统计
                for detection in detection_results['detections']:
                    cls_name = detection['class_name']
                    if cls_name not in results_summary['class_statistics']:
                        results_summary['class_statistics'][cls_name] = 0
                    results_summary['class_statistics'][cls_name] += 1
                
            except Exception as e:
                print(f"处理图像 {image_path.name} 时出错: {e}")
                import traceback
                traceback.print_exc()
        
        # 计算处理时间
        end_time = datetime.now()
        results_summary['processing_time'] = (end_time - start_time).total_seconds()
        
        # 保存批量处理结果
        summary_path = os.path.join(output_dir, "batch_detection_summary.json")
        with open(summary_path, 'w', encoding='utf-8') as f:
            json.dump(results_summary, f, indent=2, ensure_ascii=False)
        
        # 打印统计信息
        print(f"\n批量检测完成!")
        print(f"处理图像数: {results_summary['processed_images']}/{results_summary['total_images']}")
        print(f"总检测数: {results_summary['total_detections']}")
        print(f"处理时间: {results_summary['processing_time']:.2f}秒")
        print(f"平均每张图像: {results_summary['processing_time']/results_summary['processed_images']:.2f}秒")
        
        print(f"\n类别统计:")
        for cls_name, count in results_summary['class_statistics'].items():
            print(f"  {cls_name}: {count}")
        
        return results_summary
    
    def export_model(self, format: str = 'onnx') -> str:
        """
        导出模型
        
        Args:
            format: 导出格式 ('onnx', 'torchscript', 'tflite', etc.)
            
        Returns:
            导出文件路径
        """
        if self.model is None:
            raise RuntimeError("模型未加载，请先调用load_model()")
        
        try:
            export_path = self.model.export(format=format)
            print(f"✅ 模型导出成功: {export_path}")
            return export_path
        except Exception as e:
            print(f"❌ 模型导出失败: {e}")
            raise


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='YOLO11-OBB检测器')
    parser.add_argument('--model', type=str, default='../detection_models/runs/obb_train/exp2/weights/best.pt', help='模型文件路径')
    parser.add_argument('--image', type=str, help='单张图像路径')
    parser.add_argument('--image_dir', type=str, help='图像目录路径(批量检测)')
    parser.add_argument('--output', type=str, default='./results', help='输出目录')
    parser.add_argument('--conf', type=float, default=0.5, help='置信度阈值 (默认0.001，与训练验证一致)')
    parser.add_argument('--iou', type=float, default=0.25, help='IoU阈值')
    parser.add_argument('--export', type=str, help='导出模型格式')
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("YOLO11-OBB 检测器")
    print("=" * 60)
    
    # 创建检测器
    detector = YOLO11OBBDetector(
        model_path=args.model,
        conf_threshold=args.conf,
        iou_threshold=args.iou
    )
    
    try:
        # 导出模型
        if args.export:
            detector.export_model(args.export)
        
        # 单张图像检测
        if args.image:
            print(f"\n检测单张图像: {args.image}")
            results = detector.detect_image(args.image, save_result=True, output_dir=args.output)
            print(f"检测到 {results['total_detections']} 个目标")
        
        # 批量检测
        elif args.image_dir:
            print(f"\n批量检测图像目录: {args.image_dir}")
            summary = detector.batch_detect(args.image_dir, args.output)
        
        else:
            print("请指定 --image 或 --image_dir 参数")
        
        print(f"\n✅ 检测完成!")
        
    except Exception as e:
        print(f"\n❌ 检测失败: {e}")
        return 1
    
    return 0

if __name__ == "__main__":
    main()

