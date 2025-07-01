#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO分割训练数据生成器
从LabelMe标注文件生成增强的YOLO segmentation训练数据
"""

import os
import json
import cv2
import numpy as np
import random
from pathlib import Path
import math


class YOLODataGenerator:
    def __init__(self, annotations_dir="./data/annotations", 
                 output_dir="./data/yolo_dataset", 
                 target_count=500):
        self.annotations_dir = annotations_dir
        self.output_dir = output_dir
        self.target_count = target_count
        
        # 创建输出目录结构
        self.images_dir = os.path.join(output_dir, "images")
        self.labels_dir = os.path.join(output_dir, "labels")
        os.makedirs(self.images_dir, exist_ok=True)
        os.makedirs(self.labels_dir, exist_ok=True)

    def load_labelme_data(self, json_path):
        """加载LabelMe标注数据"""
        with open(json_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
        
        # 获取图像路径
        json_dir = os.path.dirname(json_path)
        image_path = data['imagePath']
        if image_path.startswith('..'):
            full_image_path = os.path.normpath(os.path.join(json_dir, image_path))
        else:
            full_image_path = os.path.join(json_dir, os.path.basename(image_path))
        
        # 读取图像
        if not os.path.exists(full_image_path):
            print(f"警告: 图像文件不存在 {full_image_path}")
            return None, None
        
        image = cv2.imread(full_image_path)
        if image is None:
            print(f"警告: 无法读取图像 {full_image_path}")
            return None, None
        
        # 处理标注
        shapes = data['shapes']
        annotations = []
        
        for shape in shapes:
            if shape['shape_type'] == 'polygon':
                points = np.array(shape['points'], dtype=np.float32)
                label = shape.get('label', 'part')
                annotations.append({
                    'label': label,
                    'points': points,
                    'class_id': 0  # 假设只有一个类别
                })
        
        return image, annotations

    def apply_random_augmentation(self, image, annotations):
        """应用随机数据增强"""
        img_height, img_width = image.shape[:2]
        aug_image = image.copy()
        aug_annotations = []
        
        for ann in annotations:
            aug_points = ann['points'].copy()
            aug_annotations.append({
                'label': ann['label'],
                'points': aug_points,
                'class_id': ann['class_id']
            })
        
        # 随机选择增强操作
        operations = []
        
        # 翻转操作
        if random.random() < 0.4:
            operations.append('horizontal_flip')
        if random.random() < 0.2:
            operations.append('vertical_flip')
        
        # 旋转操作
        if random.random() < 0.6:
            angle = random.uniform(-45, 45)
            operations.append(('rotate', angle))
        
        # 亮度对比度调整
        if random.random() < 0.5:
            brightness = random.uniform(-30, 30)
            contrast = random.uniform(0.8, 1.2)
            operations.append(('brightness_contrast', brightness, contrast))
        
        # 噪声
        if random.random() < 0.3:
            operations.append('noise')
        
        # 模糊
        if random.random() < 0.2:
            operations.append('blur')
        
        # 应用增强操作
        for op in operations:
            if op == 'horizontal_flip':
                aug_image, aug_annotations = self.horizontal_flip(aug_image, aug_annotations)
            elif op == 'vertical_flip':
                aug_image, aug_annotations = self.vertical_flip(aug_image, aug_annotations)
            elif isinstance(op, tuple) and op[0] == 'rotate':
                aug_image, aug_annotations = self.rotate_image(aug_image, aug_annotations, op[1])
            elif isinstance(op, tuple) and op[0] == 'brightness_contrast':
                aug_image = self.adjust_brightness_contrast(aug_image, op[1], op[2])
            elif op == 'noise':
                aug_image = self.add_noise(aug_image)
            elif op == 'blur':
                aug_image = self.add_blur(aug_image)
        
        return aug_image, aug_annotations

    def horizontal_flip(self, image, annotations):
        """水平翻转"""
        img_height, img_width = image.shape[:2]
        flipped_image = cv2.flip(image, 1)
        
        flipped_annotations = []
        for ann in annotations:
            flipped_points = ann['points'].copy()
            flipped_points[:, 0] = img_width - flipped_points[:, 0]
            flipped_annotations.append({
                'label': ann['label'],
                'points': flipped_points,
                'class_id': ann['class_id']
            })
        
        return flipped_image, flipped_annotations

    def vertical_flip(self, image, annotations):
        """垂直翻转"""
        img_height, img_width = image.shape[:2]
        flipped_image = cv2.flip(image, 0)
        
        flipped_annotations = []
        for ann in annotations:
            flipped_points = ann['points'].copy()
            flipped_points[:, 1] = img_height - flipped_points[:, 1]
            flipped_annotations.append({
                'label': ann['label'],
                'points': flipped_points,
                'class_id': ann['class_id']
            })
        
        return flipped_image, flipped_annotations

    def rotate_image(self, image, annotations, angle):
        """旋转图像和标注"""
        img_height, img_width = image.shape[:2]
        center = (img_width // 2, img_height // 2)
        
        # 计算旋转矩阵
        rotation_matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
        
        # 旋转图像
        rotated_image = cv2.warpAffine(image, rotation_matrix, (img_width, img_height))
        
        # 旋转标注点
        rotated_annotations = []
        for ann in annotations:
            points = ann['points']
            # 转换点坐标
            ones = np.ones(shape=(len(points), 1))
            points_homogeneous = np.hstack([points, ones])
            rotated_points = rotation_matrix.dot(points_homogeneous.T).T
            
            rotated_annotations.append({
                'label': ann['label'],
                'points': rotated_points,
                'class_id': ann['class_id']
            })
        
        return rotated_image, rotated_annotations

    def adjust_brightness_contrast(self, image, brightness, contrast):
        """调整亮度和对比度"""
        adjusted = cv2.convertScaleAbs(image, alpha=contrast, beta=brightness)
        return adjusted

    def add_noise(self, image):
        """添加高斯噪声"""
        noise = np.random.normal(0, 25, image.shape).astype(np.uint8)
        noisy_image = cv2.add(image, noise)
        return noisy_image

    def add_blur(self, image):
        """添加模糊"""
        kernel_size = random.choice([3, 5])
        blurred = cv2.GaussianBlur(image, (kernel_size, kernel_size), 0)
        return blurred

    def normalize_polygon(self, points, img_width, img_height):
        """将多边形坐标标准化到[0,1]范围"""
        normalized = points.copy().astype(np.float32)
        normalized[:, 0] /= img_width
        normalized[:, 1] /= img_height
        # 确保坐标在有效范围内
        normalized = np.clip(normalized, 0.0, 1.0)
        return normalized

    def points_to_yolo_format(self, points):
        """将多边形点转换为YOLO格式字符串"""
        # 展平坐标点 [x1, y1, x2, y2, ...]
        flattened = points.flatten()
        # 转换为字符串
        coords_str = ' '.join([f"{coord:.6f}" for coord in flattened])
        return coords_str

    def save_yolo_data(self, image, annotations, base_name, index):
        """保存YOLO格式的数据"""
        # 生成文件名
        img_filename = f"{base_name}_{index:04d}.jpg"
        label_filename = f"{base_name}_{index:04d}.txt"
        
        img_path = os.path.join(self.images_dir, img_filename)
        label_path = os.path.join(self.labels_dir, label_filename)
        
        # 保存图像
        cv2.imwrite(img_path, image)
        
        # 保存标签
        img_height, img_width = image.shape[:2]
        
        with open(label_path, 'w') as f:
            for ann in annotations:
                # 过滤掉超出边界的点
                valid_points = []
                for point in ann['points']:
                    if 0 <= point[0] < img_width and 0 <= point[1] < img_height:
                        valid_points.append(point)
                
                if len(valid_points) >= 3:  # 至少需要3个点构成多边形
                    valid_points = np.array(valid_points)
                    # 标准化坐标
                    normalized_points = self.normalize_polygon(valid_points, img_width, img_height)
                    # 转换为YOLO格式
                    coords_str = self.points_to_yolo_format(normalized_points)
                    # 写入文件: class_id x1 y1 x2 y2 x3 y3 ...
                    f.write(f"{ann['class_id']} {coords_str}\n")
        
        return img_filename, label_filename

    def generate_dataset(self):
        """生成完整数据集"""
        # 获取所有json文件
        json_files = []
        for file in os.listdir(self.annotations_dir):
            if file.lower().endswith('.json'):
                json_files.append(os.path.join(self.annotations_dir, file))
        
        if not json_files:
            print(f"错误: 在 {self.annotations_dir} 中没有找到json文件")
            return
        
        print(f"找到 {len(json_files)} 个标注文件")
        print(f"目标生成 {self.target_count} 个训练样本")
        
        # 计算每个文件需要生成的样本数
        samples_per_file = self.target_count // len(json_files)
        remaining_samples = self.target_count % len(json_files)
        
        print(f"每个文件生成 {samples_per_file} 个样本")
        if remaining_samples > 0:
            print(f"前 {remaining_samples} 个文件额外生成 1 个样本")
        
        total_generated = 0
        
        for file_idx, json_path in enumerate(json_files):
            print(f"\n处理文件 [{file_idx+1}/{len(json_files)}]: {os.path.basename(json_path)}")
            
            # 加载原始数据
            image, annotations = self.load_labelme_data(json_path)
            if image is None or not annotations:
                print(f"跳过文件: {json_path}")
                continue
            
            # 确定此文件要生成的样本数
            current_samples = samples_per_file
            if file_idx < remaining_samples:
                current_samples += 1
            
            base_name = os.path.splitext(os.path.basename(json_path))[0]
            
            # 先保存原始样本
            self.save_yolo_data(image, annotations, base_name, 0)
            generated_count = 1
            total_generated += 1
            
            # 生成增强样本
            for i in range(1, current_samples):
                try:
                    # 应用随机增强
                    aug_image, aug_annotations = self.apply_random_augmentation(image, annotations)
                    
                    # 过滤掉无效的标注
                    valid_annotations = []
                    for ann in aug_annotations:
                        if len(ann['points']) >= 3:  # 至少3个点才能构成多边形
                            valid_annotations.append(ann)
                    
                    if valid_annotations:
                        self.save_yolo_data(aug_image, valid_annotations, base_name, i)
                        generated_count += 1
                        total_generated += 1
                    
                except Exception as e:
                    print(f"  警告: 生成第 {i} 个样本时出错: {e}")
                    continue
            
            print(f"  完成: 生成了 {generated_count} 个样本")
        
        print(f"\n数据生成完成!")
        print(f"总共生成: {total_generated} 个训练样本")
        print(f"图像保存至: {self.images_dir}")
        print(f"标签保存至: {self.labels_dir}")
        
        # 生成数据集配置文件
        self.create_dataset_yaml()

    def create_dataset_yaml(self):
        """创建YOLO数据集配置文件"""
        yaml_content = f"""# YOLO Segmentation Dataset Configuration
# Generated from LabelMe annotations

path: {os.path.abspath(self.output_dir)}
train: images
val: images  # 使用相同目录，实际使用时应该分割train/val

# Classes
nc: 1  # number of classes
names: ['part']  # class names

# Dataset info
total_samples: {self.target_count}
source_annotations: 5 LabelMe JSON files
"""
        
        yaml_path = os.path.join(self.output_dir, "dataset.yaml")
        with open(yaml_path, 'w', encoding='utf-8') as f:
            f.write(yaml_content)
        
        print(f"数据集配置文件已保存: {yaml_path}")


def main():
    """主函数"""
    print("YOLO分割训练数据生成器")
    print("=" * 50)
    
    # 创建生成器
    generator = YOLODataGenerator(
        annotations_dir="./data/annotations",
        output_dir="./data/yolo_dataset", 
        target_count=500
    )
    
    # 生成数据集
    generator.generate_dataset()
    
    print("\n生成完成! 可以使用以下命令训练YOLO模型:")
    print("yolo segment train data=./data/yolo_dataset/dataset.yaml model=yolov8n-seg.pt epochs=100")


if __name__ == "__main__":
    main() 