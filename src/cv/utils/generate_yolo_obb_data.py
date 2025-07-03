#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO OBB 训练数据生成器
从分割模型结果生成旋转边界框(OBB)训练数据
基于PCA方法计算零件的中心点和方向
"""

import os
import sys
import cv2
import numpy as np
import yaml
import random
from pathlib import Path
import torch
from sklearn.decomposition import PCA
from ultralytics import YOLO
import time
from datetime import datetime

# 直接实现所需的函数，避免导入问题
def calculate_extension_distance(mask, start_point, direction_vector, width, height):
    """
    计算从起始点沿指定方向能在掩码内延伸的最大距离
    """
    # 标准化方向向量
    direction_vector = direction_vector / np.linalg.norm(direction_vector)
    
    max_distance = 0
    step_size = 0.5
    
    # 逐步延伸直到离开掩码边界
    for step in range(1, int(max(width, height) * 2)):
        # 计算当前点
        current_point = start_point + step * step_size * direction_vector
        
        # 检查边界
        x, y = int(round(current_point[0])), int(round(current_point[1]))
        if x < 0 or x >= width or y < 0 or y >= height:
            break
        
        # 检查是否仍在掩码内
        if mask[y, x] > 0.5:
            max_distance = step * step_size
        else:
            break
    
    return max_distance


def calculate_part_orientation(mask):
    """
    检测零件尖端方向
    角度系统：尖端向上为0度，顺时针为正角度
    返回: (角度, 质心, 尖端向量)
    """
    if torch.is_tensor(mask):
        mask_np = mask.cpu().numpy()
    else:
        mask_np = mask
    
    # 获取掩码的像素坐标
    y_coords, x_coords = np.where(mask_np > 0.5)
    
    if len(x_coords) < 10:
        return None, None, None
    
    # 构造坐标点矩阵
    points = np.column_stack((x_coords, y_coords))
    
    # 计算质心
    centroid = np.mean(points, axis=0)
    
    # 使用PCA计算主轴方向
    pca = PCA(n_components=2)
    pca.fit(points)
    principal_vector = pca.components_[0]
    
    # 计算沿主轴正负方向的延伸距离
    mask_height, mask_width = mask_np.shape
    
    # 正方向延伸距离
    positive_distance = calculate_extension_distance(
        mask_np, centroid, principal_vector, mask_width, mask_height
    )
    
    # 负方向延伸距离
    negative_distance = calculate_extension_distance(
        mask_np, centroid, -principal_vector, mask_width, mask_height
    )
    
    # 尖端在延伸更远的方向
    if positive_distance > negative_distance:
        tip_vector = principal_vector
    else:
        tip_vector = -principal_vector
    
    # 计算尖端方向角度（从正y轴向上开始，顺时针为正）
    # tip_vector[0] = x分量, tip_vector[1] = y分量
    # 注意：图像坐标系中y轴向下，所以需要取负号
    angle_to_up = np.degrees(np.arctan2(tip_vector[0], -tip_vector[1]))
    
    # 标准化角度到[-180, 180]范围
    while angle_to_up > 180:
        angle_to_up -= 360
    while angle_to_up <= -180:
        angle_to_up += 360
    
    return angle_to_up, centroid, tip_vector


class YOLOOBBDataGenerator:
    """YOLO OBB数据生成器"""
    
    def __init__(self, 
                 seg_model_path='../models_cache/yolo_seg_n.pt',
                 images_dir='../data/yolo_dataset/images',
                 output_dir='../data/yolo_obb_dataset',
                 device=None):
        """
        初始化OBB数据生成器
        
        Args:
            seg_model_path: 分割模型路径
            images_dir: 输入图像目录
            output_dir: 输出数据集目录
            device: 设备选择
        """
        self.seg_model_path = seg_model_path
        self.images_dir = images_dir
        self.output_dir = output_dir
        
        # 设备选择
        if device is None:
            self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        else:
            self.device = device
            
        print(f"使用设备: {self.device}")
        
        # 创建输出目录
        self.images_output_dir = os.path.join(output_dir, 'images')
        self.labels_output_dir = os.path.join(output_dir, 'labels')
        os.makedirs(self.images_output_dir, exist_ok=True)
        os.makedirs(self.labels_output_dir, exist_ok=True)
        
        # 加载分割模型
        self.load_segmentation_model()
        
        # 统计信息
        self.total_images = 0
        self.total_objects = 0
        self.successful_obb = 0
    
    def load_segmentation_model(self):
        """加载分割模型"""
        try:
            self.seg_model = YOLO(self.seg_model_path)
            self.seg_model.to(self.device)
            print(f"✓ 分割模型加载成功: {self.seg_model_path}")
        except Exception as e:
            raise RuntimeError(f"分割模型加载失败: {e}")
    
    def get_mask_from_segmentation(self, image_path):
        """
        从分割模型获取mask
        
        Args:
            image_path: 图像路径
            
        Returns:
            tuple: (原图像, mask列表)
        """
        # 读取原始图像
        image = cv2.imread(image_path)
        if image is None:
            print(f"无法读取图像: {image_path}")
            return None, []
        
        # 使用分割模型进行预测
        try:
            if self.device == 'cuda':
                with torch.autocast(device_type='cuda'):
                    results = self.seg_model(image_path, device=self.device)
            else:
                results = self.seg_model(image_path, device=self.device)
        except Exception as e:
            print(f"分割预测失败: {e}")
            return image, []
        
        masks = []
        img_height, img_width = image.shape[:2]
        
        # 处理分割结果
        for r in results:
            if r.masks is not None:
                # 获取掩码数据
                masks_data = r.masks.data
                
                # 批量处理掩码
                if self.device == 'cuda':
                    target_size = (img_height, img_width)
                    masks_resized = torch.nn.functional.interpolate(
                        masks_data.unsqueeze(1).float(), 
                        size=target_size, 
                        mode='bilinear', 
                        align_corners=False
                    ).squeeze(1)
                    masks_cpu = masks_resized.cpu().numpy()
                else:
                    masks_cpu = masks_data.cpu().numpy()
                
                # 处理每个mask
                for mask in masks_cpu:
                    if self.device != 'cuda':
                        mask_resized = cv2.resize(mask, (img_width, img_height))
                    else:
                        mask_resized = mask
                    
                    # 转换为布尔mask
                    mask_bool = mask_resized > 0.5
                    
                    # 确保mask尺寸正确
                    if mask_bool.shape != (img_height, img_width):
                        mask_bool = cv2.resize(mask_bool.astype(np.uint8), 
                                             (img_width, img_height)) > 0.5
                    
                    masks.append(mask_bool)
        
        # 清理GPU内存
        if self.device == 'cuda':
            torch.cuda.empty_cache()
        
        return image, masks
    
    def calculate_obb_from_mask(self, mask):
        """
        从mask计算OBB参数
        基于PCA尖端方向和mask的精确投影计算
        
        Args:
            mask: 布尔mask数组
            
        Returns:
            dict: OBB参数 {'center_x', 'center_y', 'width', 'height', 'angle'}
        """
        # 使用PCA计算方向和质心
        angle, centroid, tip_vector = calculate_part_orientation(mask)
        
        if angle is None or centroid is None or tip_vector is None:
            return None
        
        # 获取mask的像素坐标
        y_coords, x_coords = np.where(mask > 0.5)
        
        if len(x_coords) < 3:
            return None
        
        # 构造坐标点矩阵
        points = np.column_stack((x_coords, y_coords))
        
        # 使用已计算好的tip_vector作为主轴方向（尖端方向）
        principal_axis = tip_vector / np.linalg.norm(tip_vector)  # 标准化
        
        # 计算垂直轴（次轴）- 垂直于尖端方向
        secondary_axis = np.array([-principal_axis[1], principal_axis[0]])  # 逆时针旋转90度
        
        # 将坐标转换到局部坐标系（相对于质心）
        centered_points = points - centroid
        
        # 投影到主轴和次轴
        proj_principal = np.dot(centered_points, principal_axis)  # 沿尖端方向的投影
        proj_secondary = np.dot(centered_points, secondary_axis)  # 沿垂直方向的投影
        
        # 按照用户要求的方式计算OBB尺寸：
        # 1. 沿着PCA尖端方向，找到mask的最远投影点
        max_proj_principal = np.max(proj_principal)
        min_proj_principal = np.min(proj_principal)
        
        # 2. 沿着PCA尖端方向的垂直方向，找到mask的最远投影点
        max_proj_secondary = np.max(proj_secondary)
        min_proj_secondary = np.min(proj_secondary)
        
        # 计算OBB的宽度和高度
        # 主轴方向（尖端方向）上的最大投影范围
        obb_length = max_proj_principal - min_proj_principal
        # 次轴方向（垂直方向）上的最大投影范围
        obb_width = max_proj_secondary - min_proj_secondary
        
        # 计算OBB的真实中心点
        # 沿主轴方向得到两端最远投影的中点作为中心点
        center_proj_principal = (max_proj_principal + min_proj_principal) / 2
        center_proj_secondary = (max_proj_secondary + min_proj_secondary) / 2
        
        # 将投影中心转换回图像坐标系
        obb_center = centroid + center_proj_principal * principal_axis + center_proj_secondary * secondary_axis
        
        # 直接使用PCA计算的真实角度（包含方向信息）
        # angle已经包含了正确的方向信息：朝上是0°，逆时针是负，顺时针是正
        obb_angle_deg = angle
        obb_angle_rad = np.radians(angle)
        
        return {
            'center_x': float(obb_center[0]),
            'center_y': float(obb_center[1]),
            'width': float(obb_length),    # 主轴方向（尖端方向）的长度
            'height': float(obb_width),    # 次轴方向（垂直方向）的宽度
            'angle': float(obb_angle_rad),
            'angle_deg': float(obb_angle_deg),
            'principal_axis': principal_axis,  # 保存主轴方向用于可视化
            'secondary_axis': secondary_axis,  # 保存次轴方向用于可视化
            'projection_info': {  # 调试信息
                'max_principal': float(max_proj_principal),
                'min_principal': float(min_proj_principal),
                'max_secondary': float(max_proj_secondary),
                'min_secondary': float(min_proj_secondary),
                'centroid': centroid.tolist(),
                'obb_center': obb_center.tolist()
            }
        }
    
    def normalize_obb(self, obb_params, img_width, img_height):
        """
        将OBB参数归一化到[0,1]范围
        
        Args:
            obb_params: OBB参数字典
            img_width: 图像宽度
            img_height: 图像高度
            
        Returns:
            dict: 归一化后的OBB参数
        """
        normalized = {
            'center_x': obb_params['center_x'] / img_width,
            'center_y': obb_params['center_y'] / img_height,
            'width': obb_params['width'] / img_width,
            'height': obb_params['height'] / img_height,
            'angle': obb_params['angle'],  # 角度不需要归一化
            'angle_deg': obb_params['angle_deg']
        }
        
        # 确保坐标在有效范围内
        normalized['center_x'] = np.clip(normalized['center_x'], 0.0, 1.0)
        normalized['center_y'] = np.clip(normalized['center_y'], 0.0, 1.0)
        normalized['width'] = np.clip(normalized['width'], 0.0, 1.0)
        normalized['height'] = np.clip(normalized['height'], 0.0, 1.0)
        
        return normalized
    
    def save_obb_data(self, image, obb_list, base_name):
        """
        保存OBB数据
        
        Args:
            image: 图像数组
            obb_list: OBB参数列表
            base_name: 基础文件名
            
        Returns:
            tuple: (图像文件名, 标签文件名)
        """
        # 生成文件名
        img_filename = f"{base_name}.jpg"
        label_filename = f"{base_name}.txt"
        
        img_path = os.path.join(self.images_output_dir, img_filename)
        label_path = os.path.join(self.labels_output_dir, label_filename)
        
        # 保存图像
        cv2.imwrite(img_path, image)
        
        # 保存标签
        img_height, img_width = image.shape[:2]
        
        with open(label_path, 'w') as f:
            for obb in obb_list:
                # 归一化OBB参数
                normalized_obb = self.normalize_obb(obb, img_width, img_height)
                
                # 先在像素坐标系中计算角点，然后再归一化
                center_x_px, center_y_px = obb['center_x'], obb['center_y']  # 像素坐标
                width_px, height_px = obb['width'], obb['height']  # 像素尺寸
                
                # 直接使用主轴和次轴方向来计算角点（避免角度转换的误差）
                principal_axis = obb['principal_axis']  # 主轴方向（尖端方向）
                secondary_axis = obb['secondary_axis']  # 次轴方向（垂直方向）
                
                # 计算四个角点相对于中心点的位置
                # 使用主轴和次轴方向直接计算，确保OBB与轴方向完全对齐
                half_width = width_px / 2   # 主轴方向的一半长度
                half_height = height_px / 2  # 次轴方向的一半长度
                
                # 四个角点在主轴/次轴坐标系中的位置
                corners_in_local = [
                    (-half_width, -half_height),  # 左下
                    (half_width, -half_height),   # 右下
                    (half_width, half_height),    # 右上
                    (-half_width, half_height)    # 左上
                ]
                
                # 转换到图像坐标系
                rotated_corners_px = []
                for local_x, local_y in corners_in_local:
                    # 使用主轴和次轴方向计算实际坐标
                    # local_x沿主轴方向，local_y沿次轴方向
                    point_vector = local_x * principal_axis + local_y * secondary_axis
                    
                    # 转换到图像坐标系（加上中心点坐标）
                    point_x = center_x_px + point_vector[0]
                    point_y = center_y_px + point_vector[1]
                    rotated_corners_px.append([point_x, point_y])
                
                rotated_corners_px = np.array(rotated_corners_px)
                
                # 归一化角点坐标
                rotated_corners = rotated_corners_px.copy()
                rotated_corners[:, 0] /= img_width   # 归一化x坐标
                rotated_corners[:, 1] /= img_height  # 归一化y坐标
                
                # 确保坐标在有效范围内
                rotated_corners = np.clip(rotated_corners, 0.0, 1.0)
                
                # 扩展YOLO OBB格式: class_id x1 y1 x2 y2 x3 y3 x4 y4 angle
                # 展平角点坐标
                coords = rotated_corners.flatten()
                coords_str = ' '.join([f"{coord:.6f}" for coord in coords])
                
                # 添加角度信息（转换为度数）
                angle_deg = obb['angle_deg']
                
                line = f"0 {coords_str} {angle_deg:.6f}\n"
                f.write(line)
        
        return img_filename, label_filename
    
    def process_single_image(self, image_path):
        """
        处理单张图像生成OBB数据
        
        Args:
            image_path: 图像路径
            
        Returns:
            bool: 是否成功处理
        """
        print(f"正在处理: {os.path.basename(image_path)}")
        
        # 从分割模型获取mask
        image, masks = self.get_mask_from_segmentation(image_path)
        
        if image is None or not masks:
            print(f"  - 跳过: 无法获取有效的分割结果")
            return False
        
        print(f"  - 检测到 {len(masks)} 个对象")
        
        # 计算每个mask的OBB参数
        obb_list = []
        for i, mask in enumerate(masks):
            obb_params = self.calculate_obb_from_mask(mask)
            if obb_params is not None:
                obb_list.append(obb_params)
                print(f"    对象 {i+1}: 中心({obb_params['center_x']:.1f}, {obb_params['center_y']:.1f}), "
                      f"尺寸({obb_params['width']:.1f}x{obb_params['height']:.1f}), "
                      f"角度{obb_params['angle_deg']:.1f}°")
                self.successful_obb += 1
            else:
                print(f"    对象 {i+1}: OBB计算失败")
        
        if not obb_list:
            print(f"  - 跳过: 没有有效的OBB数据")
            return False
        
        # 保存OBB数据
        base_name = os.path.splitext(os.path.basename(image_path))[0]
        img_filename, label_filename = self.save_obb_data(image, obb_list, base_name)
        
        print(f"  - 保存成功: {img_filename}, {label_filename}")
        self.total_objects += len(obb_list)
        
        return True
    
    def generate_dataset(self):
        """
        生成完整的OBB数据集
        
        Returns:
            tuple: (处理的图像数量, 生成的OBB数量)
        """
        print("开始生成YOLO OBB训练数据集...")
        print("=" * 60)
        
        # 检查输入目录
        if not os.path.exists(self.images_dir):
            print(f"错误: 输入目录不存在 {self.images_dir}")
            return 0, 0
        
        # 获取所有图像文件
        image_files = []
        for ext in ['*.jpg', '*.jpeg', '*.png', '*.bmp']:
            image_files.extend(Path(self.images_dir).glob(ext))
            image_files.extend(Path(self.images_dir).glob(ext.upper()))
        
        if not image_files:
            print(f"错误: 在 {self.images_dir} 中没有找到图像文件")
            return 0, 0
        
        print(f"找到 {len(image_files)} 个图像文件")
        
        # 处理每个图像
        start_time = time.time()
        
        for i, image_path in enumerate(image_files, 1):
            print(f"\n[{i}/{len(image_files)}] ", end="")
            
            if self.process_single_image(str(image_path)):
                self.total_images += 1
        
        # 处理完成统计
        total_time = time.time() - start_time
        
        print(f"\n" + "=" * 60)
        print(f"OBB数据集生成完成!")
        print(f"✅ 处理图像: {self.total_images}/{len(image_files)}")
        print(f"✅ 生成对象: {self.total_objects}")
        print(f"✅ 成功OBB: {self.successful_obb}")
        print(f"✅ 总用时: {total_time:.2f}秒")
        print(f"✅ 平均每张: {total_time/len(image_files):.2f}秒")
        
        # 生成数据集配置文件
        self.create_dataset_yaml()
        
        return self.total_images, self.total_objects
    
    def create_dataset_yaml(self):
        """创建YOLO OBB数据集配置文件"""
        yaml_content = f"""# YOLO OBB Dataset Configuration
# Generated from segmentation model results using PCA-based orientation
# Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}

path: {os.path.abspath(self.output_dir)}
train: images
val: images  # 使用相同目录，实际使用时应该分割train/val

# Classes
nc: 1  # number of classes
names: ['part']  # class names

# Dataset info
total_images: {self.total_images}
total_objects: {self.total_objects}
successful_obb: {self.successful_obb}
source_model: {self.seg_model_path}

# Extended OBB format: class_id x1 y1 x2 y2 x3 y3 x4 y4 angle
# 8 coordinates representing 4 corners of oriented bounding box + angle in degrees
# Coordinates are normalized to [0,1]
# Corner order: left-bottom, right-bottom, right-top, left-top
# Angle in degrees: 0° = tip pointing up, positive = clockwise, negative = counterclockwise
# Range: [-180°, 180°]

# Angle Information:
# Explicit angle value is saved as the 10th parameter
# Generated using PCA-based tip direction detection
# This enables training models that predict both OBB corners and explicit angle

# Training Command:
# yolo obb train data={os.path.abspath(self.output_dir)}/dataset.yaml model=yolo11n-obb.pt epochs=100
# Or use custom config: model=detection_models/configs/yolo_obb_model.yaml
"""
        
        yaml_path = os.path.join(self.output_dir, "dataset.yaml")
        with open(yaml_path, 'w', encoding='utf-8') as f:
            f.write(yaml_content)
        
        print(f"✅ 数据集配置文件已保存: {yaml_path}")
    
    def visualize_obb(self, image_path, save_path=None):
        """
        可视化OBB结果
        
        Args:
            image_path: 图像路径
            save_path: 保存路径
        """
        image, masks = self.get_mask_from_segmentation(image_path)
        
        if image is None or not masks:
            print("无法获取有效的分割结果")
            return
        
        vis_image = image.copy()
        
        for i, mask in enumerate(masks):
            obb_params = self.calculate_obb_from_mask(mask)
            if obb_params is None:
                continue
            
            # 绘制OBB
            center_x, center_y = obb_params['center_x'], obb_params['center_y']
            width, height = obb_params['width'], obb_params['height']
            principal_axis = obb_params['principal_axis']
            secondary_axis = obb_params['secondary_axis']
            
            # 直接使用主轴和次轴方向来计算角点（与保存标签逻辑一致）
            # 计算四个角点相对于中心点的位置
            # 使用主轴和次轴方向直接计算，确保OBB与轴方向完全对齐
            half_width = width / 2   # 主轴方向的一半长度
            half_height = height / 2  # 次轴方向的一半长度
            
            # 四个角点在主轴/次轴坐标系中的位置
            corners_in_local = [
                (-half_width, -half_height),  # 左下
                (half_width, -half_height),   # 右下
                (half_width, half_height),    # 右上
                (-half_width, half_height)    # 左上
            ]
            
            # 转换到图像坐标系
            rotated_corners = []
            for local_x, local_y in corners_in_local:
                # 使用主轴和次轴方向计算实际坐标
                # local_x沿主轴方向，local_y沿次轴方向
                point_vector = local_x * principal_axis + local_y * secondary_axis
                
                # 转换到图像坐标系（加上中心点坐标）
                point = (center_x + point_vector[0], center_y + point_vector[1])
                rotated_corners.append(point)
            
            rotated_corners = np.array(rotated_corners).astype(np.int32)
            
            # 绘制OBB
            cv2.polylines(vis_image, [rotated_corners], True, (0, 255, 0), 2)
            
            # 绘制中心点
            cv2.circle(vis_image, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)
            
            # 绘制尖端方向箭头（沿主轴方向）
            arrow_length = 50
            # 直接使用主轴方向绘制箭头，确保与OBB框方向一致
            arrow_vector = arrow_length * principal_axis
            arrow_end_x = center_x + arrow_vector[0]
            arrow_end_y = center_y + arrow_vector[1]
            cv2.arrowedLine(vis_image, 
                          (int(center_x), int(center_y)), 
                          (int(arrow_end_x), int(arrow_end_y)), 
                          (255, 0, 0), 2, tipLength=0.3)
            
            # 添加角度文本
            angle_text = f"{obb_params['angle_deg']:.1f}°"
            cv2.putText(vis_image, angle_text, 
                       (int(center_x) + 10, int(center_y) - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        if save_path:
            cv2.imwrite(save_path, vis_image)
            print(f"可视化结果已保存到: {save_path}")
        
        return vis_image


def main():
    """主函数"""
    print("YOLO OBB 训练数据生成器")
    print("基于PCA方法计算零件方向和边界框")
    print("扩展格式：OBB角点 + 显式角度信息（用于训练OBB+角度预测模型）")
    print("=" * 60)
    
    # 创建生成器
    generator = YOLOOBBDataGenerator(
        seg_model_path='../models_cache/yolo_seg_n.pt',
        images_dir='../data/yolo_dataset/images',
        output_dir='../data/yolo_obb_dataset'
    )
    
    # 生成数据集
    total_images, total_objects = generator.generate_dataset()
    
    if total_images > 0:
        print(f"\n🎉 数据集生成成功!")
        print(f"✅ 扩展OBB格式：8个角点坐标 + 1个角度值")
        print(f"✅ 角度信息：0°=向上，顺时针为正，范围[-180°,180°]")
        print(f"✅ 支持训练OBB+角度预测模型")
        
        print(f"\n📋 训练命令:")
        print(f"标准训练:")
        print(f"  yolo obb train data=../data/yolo_obb_dataset/dataset.yaml model=yolo11n-obb.pt epochs=100")
        print(f"使用自定义配置:")
        print(f"  yolo obb train data=../data/yolo_obb_dataset/dataset.yaml model=detection_models/configs/yolo_obb_model.yaml epochs=100")
        
        # 可视化一个样例
        if total_objects > 0:
            import glob
            sample_images = glob.glob('../data/yolo_dataset/images/*.jpg')[:1]
            if sample_images:
                print(f"\n生成可视化样例...")
                vis_path = '../data/yolo_obb_dataset/visualization_sample.jpg'
                generator.visualize_obb(sample_images[0], vis_path)
    else:
        print(f"\n❌ 数据集生成失败，请检查输入目录和模型路径")


if __name__ == "__main__":
    main()
