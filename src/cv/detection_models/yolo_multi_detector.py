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
import glob


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
            # 新增：head的旋转边界框
            head_obb = self._calculate_obb_from_mask(head_info.get('mask', None), masks, classes, scores, 'head')
            result['head_obb'] = head_obb
        if central_info:
            result['central_center'] = central_info['center']
            result['central_confidence'] = central_info['confidence']
            # 新增：central的旋转边界框
            central_obb = self._calculate_obb_from_mask(central_info.get('mask', None), masks, classes, scores, 'central')
            result['central_obb'] = central_obb

        # 新增：计算connect点（central的mask中距离head中心点最近的点）
        connect_point = None
        if head_info and central_info:
            head_center = np.array(head_info['center'])
            central_mask = central_info.get('mask', None)
            if central_mask is None:
                # 重新获取central的mask
                central_class_id = self.class_mapping['central']
                central_class_indices = np.where(classes == central_class_id)[0]
                if len(central_class_indices) > 0:
                    best_idx = central_class_indices[np.argmax(scores[central_class_indices])]
                    central_mask = masks[best_idx]
            if central_mask is not None:
                # 找到mask中所有点
                ys, xs = np.where(central_mask > 0.5)
                if len(xs) > 0:
                    points = np.stack([xs, ys], axis=1)
                    dists = np.linalg.norm(points - head_center, axis=1)
                    min_idx = np.argmin(dists)
                    connect_point = tuple(points[min_idx])
        result['connect_point'] = connect_point

        # 角度和方向箭头的计算方式更改为central中心点到connect点
        start_angle = time.perf_counter()
        if central_info and head_info:
            # 获取central的OBB角度
            central_obb = self._calculate_obb_from_mask(central_info.get('mask', None), masks, classes, scores, 'central')
            if central_obb is not None:
                # 计算OBB的方向角度
                obb_angle = self._calculate_obb_angle(central_obb)
                
                # 计算从central中心点到head中心点的方向
                central_center = np.array(central_info['center'])
                head_center = np.array(head_info['center'])
                to_head_vector = head_center - central_center
                to_head_vector = to_head_vector / np.linalg.norm(to_head_vector)  # 归一化
                
                # 确定OBB的两个主方向（相差90度）
                obb_angle1 = obb_angle
                obb_angle2 = obb_angle + 90
                
                # 计算两个OBB方向的单位向量
                obb_vector1 = np.array([math.cos(math.radians(obb_angle1)), math.sin(math.radians(obb_angle1))])
                obb_vector2 = np.array([math.cos(math.radians(obb_angle2)), math.sin(math.radians(obb_angle2))])
                
                # 使用点积判断哪个方向更接近head方向
                dot1 = np.dot(obb_vector1, to_head_vector)
                dot2 = np.dot(obb_vector2, to_head_vector)
                
                # 调试输出
                print(f"OBB angle1: {obb_angle1:.1f}°, dot1: {dot1:.3f}")
                print(f"OBB angle2: {obb_angle2:.1f}°, dot2: {dot2:.3f}")
                print(f"To head vector: {to_head_vector}")
                print(f"OBB vector1: {obb_vector1}")
                print(f"OBB vector2: {obb_vector2}")
                
                # 选择点积绝对值更大的方向（更接近平行，无论正负）
                if abs(dot1) > abs(dot2):
                    final_angle = obb_angle1
                    final_vector = obb_vector1
                    print(f"选择方向1: {obb_angle1:.1f}°, dot: {dot1:.3f}, abs_dot: {abs(dot1):.3f}")
                else:
                    final_angle = obb_angle2
                    final_vector = obb_vector2
                    print(f"选择方向2: {obb_angle2:.1f}°, dot: {dot2:.3f}, abs_dot: {abs(dot2):.3f}")
                
                # 如果选择的方向与head方向夹角大于90度，取反方向
                final_dot = np.dot(final_vector, to_head_vector)
                print(f"Final dot product: {final_dot:.3f}")
                if final_dot < 0:
                    final_angle += 180
                    if final_angle > 180:
                        final_angle -= 360
                    print(f"反向调整后角度: {final_angle:.1f}°")
                
                print(f"最终选择角度: {final_angle:.1f}°")
                
                # 计算方向箭头的终点（沿着OBB方向）
                central_center = central_info['center']
                arrow_length = 50  # 箭头长度
                end_x = central_center[0] + arrow_length * math.cos(math.radians(final_angle))
                end_y = central_center[1] + arrow_length * math.sin(math.radians(final_angle))
                arrow_end = (end_x, end_y)
                
                result['angle'] = final_angle
                result['arrow_end'] = arrow_end
                result['success'] = True
            else:
                # 如果无法获取OBB，回退到原来的计算方式
                if connect_point is not None:
                    result['angle'] = self._calculate_angle(central_info['center'], connect_point)
                    result['arrow_end'] = connect_point
                    result['success'] = True
        angle_time = time.perf_counter() - start_angle
        result['timing']['angle_calculation'] = angle_time

        # 总时间
        result['timing']['total'] = time.perf_counter() - start_total

        # 新增：返回所有mask、类别、置信度，便于可视化
        result['masks'] = masks
        result['classes'] = classes
        result['scores'] = scores

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

    def _calculate_obb_from_mask(self, mask: Optional[np.ndarray], masks: np.ndarray, 
                                classes: np.ndarray, scores: np.ndarray, target_class: str) -> Optional[np.ndarray]:
        """计算mask的最小外接旋转矩形"""
        if mask is None:
            # 重新获取mask
            target_class_id = self.class_mapping[target_class]
            class_indices = np.where(classes == target_class_id)[0]
            if len(class_indices) == 0:
                return None
            best_idx = class_indices[np.argmax(scores[class_indices])]
            mask = masks[best_idx]
        
        # 二值化mask
        binary_mask = (mask > 0.5).astype(np.uint8)
        
        # 找到轮廓
        contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        
        # 找到最大的轮廓
        largest_contour = max(contours, key=cv2.contourArea)
        
        # 计算最小外接旋转矩形
        rect = cv2.minAreaRect(largest_contour)
        box = cv2.boxPoints(rect)
        box = box.astype(np.int32)
        
        return box

    def _draw_rotated_bbox(self, image: np.ndarray, box: np.ndarray, color: Tuple[int, int, int], thickness: int = 2):
        """绘制旋转边界框"""
        cv2.drawContours(image, [box], 0, color, thickness)
    
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

    def visualize_result(self, image: np.ndarray, result: dict, alpha: float = 0.5, show: bool = True, save_path: Optional[str] = None) -> np.ndarray:
        """
        可视化检测结果，叠加mask、中心点、类别和置信度，并画出方向线
        """
        vis_img = image.copy()
        overlay = vis_img.copy()
        color_map = {
            'head': (0, 0, 255),      # 红色
            'central': (0, 255, 0),   # 绿色
        }
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.7
        thickness = 2

        # 始终可视化所有mask
        masks = result.get('masks')
        classes = result.get('classes')
        scores = result.get('scores')
        if masks is not None and classes is not None and scores is not None:
            for i in range(len(masks)):
                mask = masks[i]
                cls_id = classes[i]
                score = scores[i]
                # 获取类别名
                class_name = None
                for k, v in self.class_mapping.items():
                    if v == cls_id:
                        class_name = k
                        break
                if class_name is None:
                    color = (255, 255, 255)
                else:
                    color = color_map.get(class_name, (255, 255, 255))
                # 生成彩色mask (已注释)
                colored_mask = np.zeros_like(vis_img, dtype=np.uint8)
                binary_mask = (mask > 0.5).astype(np.uint8)
                for c in range(3):
                    colored_mask[:, :, c] = binary_mask * color[c]
                # 叠加mask (已注释)
                overlay = cv2.addWeighted(overlay, 1, colored_mask, alpha, 0)
                # 计算中心点
                center = self._calculate_mask_center(mask)
                cx, cy = int(center[0]), int(center[1])
                cv2.circle(overlay, (cx, cy), 6, color, -1)
                # 标注类别和置信度
                label = f"{class_name}: {score:.2f}" if class_name is not None else f"cls{cls_id}: {score:.2f}"
                cv2.putText(overlay, label, (cx + 8, cy - 8), font, font_scale, color, thickness, cv2.LINE_AA)

        # 叠加所有mask和标注
        vis_img = cv2.addWeighted(vis_img, 1 - alpha, overlay, alpha, 0)

        # 可视化方向线（central_center->arrow_end）
        if result.get('success') and result.get('central_center') and result.get('arrow_end'):
            c1 = tuple(map(int, result['central_center']))
            c2 = tuple(map(int, result['arrow_end']))
            cv2.arrowedLine(vis_img, c1, c2, (255, 255, 0), 3, tipLength=0.15)
            # 箭头终点画圈
            cv2.circle(vis_img, c2, 4, (0, 255, 255), -1)

        # 可视化旋转边界框 (已注释)
        if result.get('head_obb') is not None:
            self._draw_rotated_bbox(vis_img, result['head_obb'], (0, 0, 255), 2)  # 红色
        if result.get('central_obb') is not None:
            self._draw_rotated_bbox(vis_img, result['central_obb'], (0, 255, 0), 2)  # 绿色

        # 显示角度
        if result.get('success') and result.get('angle') is not None:
            angle = result['angle']
            text = f"Angle: {angle:.1f} deg"
            cv2.putText(vis_img, text, (20, 40), font, 1.0, (255, 255, 0), 2, cv2.LINE_AA)

        if show:
            cv2.imshow('YOLO Detection Visualization', vis_img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        if save_path:
            cv2.imwrite(save_path, vis_img)
        return vis_img

    def _calculate_obb_angle(self, box: np.ndarray) -> float:
        """计算OBB的方向角度（度）"""
        # box是4个顶点，计算长边的方向
        # 计算各边的长度
        edges = []
        for i in range(4):
            p1 = box[i]
            p2 = box[(i + 1) % 4]
            edge_vector = p2 - p1
            edge_length = np.linalg.norm(edge_vector)
            edge_angle = math.degrees(math.atan2(edge_vector[1], edge_vector[0]))
            edges.append((edge_length, edge_angle))
        
        # 找到最长的边
        longest_edge = max(edges, key=lambda x: x[0])
        return longest_edge[1]
    
    def _angle_difference(self, angle1: float, angle2: float) -> float:
        """计算两个角度之间的最小差值"""
        diff = angle1 - angle2
        while diff > 180:
            diff -= 360
        while diff < -180:
            diff += 360
        return diff

    def detect_from_folder(self, folder_path: str, output_folder: Optional[str] = None, 
                          image_extensions: Optional[list] = None, visualize: bool = True) -> Dict[str, Any]:
        """
        批量检测文件夹中的图像文件
        
        Args:
            folder_path: 图像文件夹路径
            output_folder: 可视化结果保存文件夹，如果为None则不保存
            image_extensions: 支持的图像文件扩展名列表
            visualize: 是否生成可视化结果
            
        Returns:
            包含所有图像检测结果的字典
        """
        if image_extensions is None:
            image_extensions = ['.jpg', '.jpeg', '.png', '.bmp', '.tiff', '.tif']
        
        if not os.path.exists(folder_path):
            raise ValueError(f"文件夹不存在: {folder_path}")
        
        # 创建输出文件夹
        if output_folder and not os.path.exists(output_folder):
            os.makedirs(output_folder)
        
        # 获取所有图像文件
        image_files = []
        for ext in image_extensions:
            pattern = os.path.join(folder_path, f"*{ext}")
            image_files.extend(glob.glob(pattern))
            pattern = os.path.join(folder_path, f"*{ext.upper()}")
            image_files.extend(glob.glob(pattern))
        
        if not image_files:
            print(f"在文件夹 {folder_path} 中未找到图像文件")
            return {}
        
        print(f"找到 {len(image_files)} 个图像文件")
        
        # 批量处理结果
        batch_results = {}
        successful_detections = 0
        
        for i, image_path in enumerate(image_files):
            try:
                # 获取文件名（不含扩展名）
                filename = os.path.splitext(os.path.basename(image_path))[0]
                
                print(f"处理 [{i+1}/{len(image_files)}]: {filename}")
                
                # 检测图像
                result = self.detect_from_file(image_path)
                batch_results[filename] = result
                
                if result['success']:
                    successful_detections += 1
                    print(f"  ✓ 检测成功 - 角度: {result['angle']:.1f}°")
                else:
                    print(f"  ✗ 检测失败 - 未找到所需目标")
                
                # 生成可视化结果
                if visualize and output_folder:
                    image = cv2.imread(image_path)
                    if image is not None:
                        output_path = os.path.join(output_folder, f"{filename}_result.png")
                        self.visualize_result(image, result, alpha=0.5, show=False, save_path=output_path)
                
            except Exception as e:
                print(f"  ✗ 处理失败: {str(e)}")
                batch_results[filename] = {'success': False, 'error': str(e)}
        
        # 打印统计信息
        print(f"\n批量处理完成:")
        print(f"  总文件数: {len(image_files)}")
        print(f"  成功检测: {successful_detections}")
        print(f"  失败检测: {len(image_files) - successful_detections}")
        if output_folder:
            print(f"  结果保存至: {output_folder}")
        
        return batch_results
    
    def save_batch_results_to_csv(self, batch_results: Dict[str, Any], csv_path: str):
        """
        将批量检测结果保存为CSV文件
        
        Args:
            batch_results: detect_from_folder返回的结果
            csv_path: CSV文件保存路径
        """
        import csv
        
        with open(csv_path, 'w', newline='', encoding='utf-8') as csvfile:
            fieldnames = ['filename', 'success', 'angle', 'head_confidence', 'central_confidence', 
                         'head_center_x', 'head_center_y', 'central_center_x', 'central_center_y',
                         'total_time_ms', 'error']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            
            for filename, result in batch_results.items():
                if result.get('success', False):
                    head_center = result.get('head_center', [0, 0])
                    central_center = result.get('central_center', [0, 0])
                    timing = result.get('timing', {})
                    
                    row = {
                        'filename': filename,
                        'success': 'True',
                        'angle': str(result.get('angle', 0)),
                        'head_confidence': str(result.get('head_confidence', 0)),
                        'central_confidence': str(result.get('central_confidence', 0)),
                        'head_center_x': str(head_center[0]),
                        'head_center_y': str(head_center[1]),
                        'central_center_x': str(central_center[0]),
                        'central_center_y': str(central_center[1]),
                        'total_time_ms': str(timing.get('total', 0) * 1000),
                        'error': ''
                    }
                else:
                    row = {
                        'filename': filename,
                        'success': 'False',
                        'angle': '0',
                        'head_confidence': '0',
                        'central_confidence': '0',
                        'head_center_x': '0',
                        'head_center_y': '0',
                        'central_center_x': '0',
                        'central_center_y': '0',
                        'total_time_ms': '0',
                        'error': result.get('error', '检测失败')
                    }
                writer.writerow(row)
        
        print(f"批量结果已保存至: {csv_path}")


# 使用示例
if __name__ == "__main__":
    # 创建检测器
    detector = YOLOMultiDetector("./runs/multi-segment/train_20250711_174626/weights/best.pt")
    
    # 单张图片检测示例
    print("=== 单张图片检测 ===")
    img_path = "../data/origin_img/22_Color.png"
    result = detector.detect_from_file(img_path)
    
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
    # 可视化显示
    image = cv2.imread(img_path)
    detector.visualize_result(image, result, alpha=0.5, show=False, save_path="./visualization_result.png")
    print("可视化结果已保存为 visualization_result.png")
    
    print("\n" + "="*50 + "\n")
    
    # 批量处理示例
    print("=== 批量处理示例 ===")
    # 批量处理文件夹中的图像
    batch_results = detector.detect_from_folder(
        folder_path="../data/origin_img",  # 图像文件夹路径
        output_folder="../data/batch_results",   # 可视化结果保存文件夹
        visualize=True                     # 生成可视化结果
    )
    
    # 保存批量结果为CSV
    detector.save_batch_results_to_csv(batch_results, "./batch_results.csv")
    
    print("批量处理完成！")
