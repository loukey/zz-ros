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
                    
                    detection = {
                        'class_id': cls_id,
                        'class_name': cls_name,
                        'confidence': conf,
                        'center': [float(center_x), float(center_y)],
                        'corners': corners_array.tolist(),
                        'coords': coords,  # 保存原始坐标用于后续计算
                        'angle': 0.0,  # 初始角度
                        'real_center': [float(center_x), float(center_y)],  # 初始real_center
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
                    
                    # 转换为OBB格式的坐标
                    coords = corners.flatten()
                    
                    detection = {
                        'class_id': cls_id,
                        'class_name': cls_name,
                        'confidence': conf,
                        'center': [float(center_x), float(center_y)],
                        'corners': corners.tolist(),
                        'coords': coords,  # 保存原始坐标用于后续计算
                        'angle': 0.0,  # 初始角度
                        'real_center': [float(center_x), float(center_y)],  # 初始real_center
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
        
        # 查找central和head检测结果
        central_detection = None
        head_detection = None
        
        for detection in final_detections:
            if detection['class_name'] == 'central':
                central_detection = detection
            elif detection['class_name'] == 'head':
                head_detection = detection
        
        # 如果同时检测到central和head，重新计算角度和real_center
        if central_detection is not None and head_detection is not None:
            central_center = tuple(central_detection['center'])
            head_center = tuple(head_detection['center'])
            
            # 重新计算central的角度
            central_detection['angle'] = self.calculate_obb_angle(
                central_detection['coords'], central_center, head_center
            )
            
            # 计算real_center
            central_detection['real_center'] = list(self.calculate_real_center(
                central_detection['coords'], central_center, head_center
            ))
            
            print(f"重新计算结果: 角度={math.degrees(central_detection['angle']):.1f}°, real_center={central_detection['real_center']}")
        
        print(f"原始检测结果: {len(detections)} 个")
        print(f"筛选后检测结果: {len(final_detections)} 个")
        for detection in final_detections:
            print(f"  最佳 {detection['class_name']}: 置信度={detection['confidence']:.3f}")
        
        return {
            'image_shape': image_shape,
            'detections': final_detections,
            'total_detections': len(final_detections),
            'central_detection': central_detection,
            'head_detection': head_detection
        }
    
    def calculate_obb_angle(self, coords: np.ndarray, central_center: Tuple[float, float], head_center: Tuple[float, float]) -> float:
        """
        计算OBB的旋转角度 - 长边指向head方向与垂直向下方向的夹角，左侧为正，右侧为负
        
        Args:
            coords: 四个角点坐标 [x1,y1,x2,y2,x3,y3,x4,y4]
            central_center: central目标的中心点坐标
            head_center: head目标的中心点坐标
            
        Returns:
            旋转角度(弧度) - 与垂直向下方向的夹角，左侧为正，右侧为负
        """
        # 重新排列为4x2的坐标矩阵
        points = coords.reshape(4, 2)
        
        # 计算长边的方向向量
        edge1 = points[1] - points[0]
        edge2 = points[2] - points[1]
        
        # 选择较长的边作为主方向
        if np.linalg.norm(edge1) > np.linalg.norm(edge2):
            main_edge = edge1
        else:
            main_edge = edge2
        
        # 计算从central中心到head中心的方向向量
        central_to_head = np.array([head_center[0] - central_center[0], 
                                   head_center[1] - central_center[1]])
        
        # 计算长边方向与central到head方向的点积
        dot_product = np.dot(main_edge, central_to_head)
        
        # 如果点积为负，说明长边方向与head方向相反，需要取反
        if dot_product < 0:
            main_edge = -main_edge
        
        # 计算角度 - 与垂直向下方向的夹角，左侧为正，右侧为负
        # 垂直向下的向量是 [0, 1]（图像坐标系中y向下为正）
        # 使用 -atan2(main_edge[0], main_edge[1]) 来实现左侧为正，右侧为负
        angle_rad = -math.atan2(main_edge[0], main_edge[1])
        
        # 标准化到[-180, 180]度
        if angle_rad > math.pi:
            angle_rad -= 2 * math.pi
        elif angle_rad < -math.pi:
            angle_rad += 2 * math.pi
        
        return angle_rad
    
    def calculate_real_center(self, coords: np.ndarray, central_center: Tuple[float, float], head_center: Tuple[float, float]) -> Tuple[float, float]:
        """
        计算real_center点：从central_center出发，沿着正方向前进长边长度的1/3距离的点
        
        Args:
            coords: 四个角点坐标 [x1,y1,x2,y2,x3,y3,x4,y4]
            central_center: central目标的中心点坐标
            head_center: head目标的中心点坐标
            
        Returns:
            real_center点的坐标
        """
        # 重新排列为4x2的坐标矩阵
        points = coords.reshape(4, 2)
        
        # 计算长边的方向向量
        edge1 = points[1] - points[0]
        edge2 = points[2] - points[1]
        
        # 选择较长的边作为主方向
        if np.linalg.norm(edge1) > np.linalg.norm(edge2):
            main_edge = edge1
        else:
            main_edge = edge2
        
        # 计算从central中心到head中心的方向向量
        central_to_head = np.array([head_center[0] - central_center[0], 
                                   head_center[1] - central_center[1]])
        
        # 计算长边方向与central到head方向的点积
        dot_product = np.dot(main_edge, central_to_head)
        
        # 如果点积为负，说明长边方向与head方向相反，需要取反
        if dot_product < 0:
            main_edge = -main_edge
        
        # 计算长边长度
        edge_length = np.linalg.norm(main_edge)
        
        # 计算单位方向向量
        if edge_length > 0:
            unit_direction = main_edge / edge_length
            # 计算real_center：从central_center沿正方向前进长边长度的1/3
            real_center = (
                float(central_center[0] + unit_direction[0] * edge_length / 5),
                float(central_center[1] + unit_direction[1] * edge_length / 5)
            )
        else:
            real_center = central_center
        
        return real_center
    
    def visualize_detections(self, image: np.ndarray, detection_results: Dict[str, Any]) -> np.ndarray:
        """
        可视化检测结果 - 显示3个中心点、箭头和角度
        
        Args:
            image: 原始图像
            detection_results: 检测结果
            
        Returns:
            可视化后的图像
        """
        vis_img = image.copy()
        detections = detection_results['detections']
        central_detection = detection_results.get('central_detection')
        head_detection = detection_results.get('head_detection')
        
        # 定义颜色
        colors = {
            'central': (0, 255, 0),   # 绿色
            'head': (0, 0, 255),      # 红色
        }
        
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
            cv2.circle(vis_img, center_point, 6, color, -1)
            cv2.circle(vis_img, center_point, 6, (255, 255, 255), 2)  # 白色边框
            
            # 绘制标签
            label = f"{cls_name}: {conf:.2f}"
            cv2.putText(vis_img, label, 
                       (center_point[0] + 10, center_point[1] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        # 如果同时检测到central和head，绘制real_center点、箭头和角度
        if central_detection is not None and head_detection is not None:
            central_center = np.array(central_detection['center'])
            head_center = np.array(head_detection['center'])
            real_center = np.array(central_detection['real_center'])
            angle_rad = central_detection['angle']
            angle_deg = math.degrees(angle_rad)
            
            # 绘制real_center点
            real_center_point = (int(real_center[0]), int(real_center[1]))
            cv2.circle(vis_img, real_center_point, 6, (255, 255, 0), -1)  # 青色圆点
            cv2.circle(vis_img, real_center_point, 6, (255, 255, 255), 2)  # 白色边框
            
            # 绘制real_center标签
            cv2.putText(vis_img, f"real_center", 
                       (real_center_point[0] + 10, real_center_point[1] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
            
            # 计算正方向（指向head的长边方向）
            points = np.array(central_detection['corners'])
            edge1 = points[1] - points[0]
            edge2 = points[2] - points[1]
            
            # 选择较长的边作为主方向
            if np.linalg.norm(edge1) > np.linalg.norm(edge2):
                main_edge = edge1
            else:
                main_edge = edge2
            
            # 计算从central中心到head中心的方向向量
            central_to_head = head_center - central_center
            
            # 调整方向指向head
            if np.dot(main_edge, central_to_head) < 0:
                main_edge = -main_edge
            
            # 绘制正方向箭头 - 起点在central中心点下方
            arrow_length = 50
            edge_length = np.linalg.norm(main_edge)
            if edge_length > 0:
                unit_direction = main_edge / edge_length
                
                # 箭头起点在central中心点下方20像素处
                arrow_start = central_center + np.array([0, 20])
                arrow_end = arrow_start + arrow_length * unit_direction
                
                cv2.arrowedLine(vis_img, 
                               (int(arrow_start[0]), int(arrow_start[1])),
                               (int(arrow_end[0]), int(arrow_end[1])),
                               (0, 255, 255), 3, tipLength=0.3)  # 黄色箭头
                
                # 绘制垂直参考线（在箭头起点处）- 向下方向
                vertical_start = arrow_start + np.array([0, -30])
                vertical_end = arrow_start + np.array([0, 30])
                cv2.line(vis_img, 
                        (int(vertical_start[0]), int(vertical_start[1])),
                        (int(vertical_end[0]), int(vertical_end[1])),
                        (128, 128, 128), 1)  # 灰色垂直线
                
                # 绘制向下的小箭头表示参考方向
                cv2.arrowedLine(vis_img,
                               (int(arrow_start[0]), int(arrow_start[1])),
                               (int(arrow_start[0]), int(arrow_start[1] + 20)),
                               (128, 128, 128), 1, tipLength=0.5)  # 灰色向下箭头
                
                # 绘制角度弧线（在箭头起点处）
                arc_radius = 25
                start_angle = 90  # 垂直向下
                end_angle = 90 + angle_deg
                
                # 确保角度在正确范围内
                if end_angle < start_angle:
                    start_angle, end_angle = end_angle, start_angle
                
                # 绘制角度弧线
                cv2.ellipse(vis_img, 
                           (int(arrow_start[0]), int(arrow_start[1])),
                           (arc_radius, arc_radius),
                           0, start_angle, end_angle,
                           (255, 0, 255), 2)  # 紫色弧线
                
                # 在箭头附近显示角度值
                angle_text_pos = arrow_start + np.array([30, -10])
                angle_color = (0, 255, 0) if angle_deg > 0 else (0, 0, 255) if angle_deg < 0 else (255, 255, 255)
                angle_sign = "+" if angle_deg > 0 else ""
                cv2.putText(vis_img, f"{angle_sign}{angle_deg:.1f}°", 
                           (int(angle_text_pos[0]), int(angle_text_pos[1])),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, angle_color, 2)
                
                # 在箭头末端显示方向标识
                if angle_deg > 0:
                    direction_text = f'L'
                    direction_color = (0, 255, 0)  # 绿色表示正数
                elif angle_deg < 0:
                    direction_text = f'R'
                    direction_color = (0, 0, 255)  # 红色表示负数
                else:
                    direction_text = 'S'
                    direction_color = (255, 255, 255)  # 白色表示零
                
                direction_pos = arrow_end + np.array([10, 5])
                cv2.putText(vis_img, direction_text, 
                           (int(direction_pos[0]), int(direction_pos[1])),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, direction_color, 2)
        
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
    parser.add_argument('--model', type=str, default='../detection_models/runs/obb_train/exp4/weights/best.pt', help='模型文件路径')
    parser.add_argument('--image', type=str, help='单张图像路径')
    parser.add_argument('--image_dir', default='../data/origin_img', type=str, help='图像目录路径(批量检测)')
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

