#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO分割训练数据生成器
从LabelMe标注文件生成增强的YOLO segmentation训练数据
支持多类别目标检测
"""

import os
import json
import cv2
import numpy as np
import random
from pathlib import Path
import math
from collections import defaultdict


class YOLODataGenerator:
    def __init__(self, annotations_dir="./data/annotations", 
                 output_dir="./data/yolo_dataset", 
                 target_count=500,
                 class_mapping=None,
                 auto_discover_classes=True):
        self.annotations_dir = annotations_dir
        self.output_dir = output_dir
        self.target_count = target_count
        self.auto_discover_classes = auto_discover_classes
        
        # 类别映射相关
        self.class_mapping = class_mapping or {}  # label_name -> class_id
        self.reverse_mapping = {}  # class_id -> label_name
        self.class_names = []  # 按class_id顺序排列的类别名称
        self.unknown_labels = set()  # 记录未知的标签
        
        # 创建输出目录结构
        self.images_dir = os.path.join(output_dir, "images")
        self.labels_dir = os.path.join(output_dir, "labels")
        os.makedirs(self.images_dir, exist_ok=True)
        os.makedirs(self.labels_dir, exist_ok=True)
        
        # 如果需要自动发现类别，先扫描所有文件
        if self.auto_discover_classes:
            self.discover_classes()
        else:
            self.setup_class_mapping()
    
    def discover_classes(self):
        """自动发现数据集中的所有类别"""
        print("正在自动发现类别...")
        all_labels = set()
        
        # 扫描所有JSON文件，收集所有标签
        for file in os.listdir(self.annotations_dir):
            if file.lower().endswith('.json'):
                json_path = os.path.join(self.annotations_dir, file)
                try:
                    with open(json_path, 'r', encoding='utf-8') as f:
                        data = json.load(f)
                    
                    for shape in data.get('shapes', []):
                        if shape['shape_type'] == 'polygon':
                            label = shape.get('label', 'unknown')
                            all_labels.add(label)
                            
                except Exception as e:
                    print(f"警告: 读取文件 {file} 时出错: {e}")
        
        # 如果有预定义的类别映射，保留它们
        if self.class_mapping:
            print(f"使用预定义的类别映射: {self.class_mapping}")
            existing_labels = set(self.class_mapping.keys())
            new_labels = all_labels - existing_labels
            
            if new_labels:
                print(f"发现新类别: {new_labels}")
                # 为新类别分配ID
                max_id = max(self.class_mapping.values()) if self.class_mapping else -1
                for label in sorted(new_labels):
                    max_id += 1
                    self.class_mapping[label] = max_id
        else:
            # 创建新的类别映射
            print(f"发现的所有类别: {sorted(all_labels)}")
            self.class_mapping = {}
            for i, label in enumerate(sorted(all_labels)):
                self.class_mapping[label] = i
        
        self.setup_class_mapping()
        print(f"最终类别映射: {self.class_mapping}")
    
    def setup_class_mapping(self):
        """设置类别映射关系"""
        # 创建反向映射
        self.reverse_mapping = {v: k for k, v in self.class_mapping.items()}
        
        # 创建按ID排序的类别名称列表
        max_id = max(self.class_mapping.values()) if self.class_mapping else -1
        self.class_names = ['unknown'] * (max_id + 1)
        
        for label, class_id in self.class_mapping.items():
            self.class_names[class_id] = label
        
        print(f"类别设置完成: {len(self.class_names)} 个类别")
        for i, name in enumerate(self.class_names):
            print(f"  {i}: {name}")
    
    def get_class_id(self, label):
        """根据标签名获取类别ID"""
        if label in self.class_mapping:
            return self.class_mapping[label]
        else:
            # 记录未知标签
            self.unknown_labels.add(label)
            # 返回一个默认值或跳过
            return -1  # 用-1表示未知类别，后续会过滤掉

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
                label = shape.get('label', 'unknown')
                class_id = self.get_class_id(label)
                
                # 只保留有效的类别
                if class_id >= 0:
                    annotations.append({
                        'label': label,
                        'points': points,
                        'class_id': class_id
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

    def print_class_statistics(self):
        """打印类别统计信息"""
        if self.unknown_labels:
            print(f"\n警告: 发现 {len(self.unknown_labels)} 个未知标签:")
            for label in sorted(self.unknown_labels):
                print(f"  - {label}")
            print("这些标签将被跳过。如需包含，请更新类别映射。")

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
        class_counts = defaultdict(int)  # 统计每个类别的样本数
        
        for file_idx, json_path in enumerate(json_files):
            print(f"\n处理文件 [{file_idx+1}/{len(json_files)}]: {os.path.basename(json_path)}")
            
            # 加载原始数据
            image, annotations = self.load_labelme_data(json_path)
            if image is None or not annotations:
                print(f"跳过文件: {json_path}")
                continue
            
            # 统计类别
            for ann in annotations:
                class_counts[ann['label']] += 1
            
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
        
        # 打印类别统计
        print(f"\n类别统计:")
        for label, count in sorted(class_counts.items()):
            class_id = self.get_class_id(label)
            print(f"  {label} (ID: {class_id}): {count} 个原始样本")
        
        # 打印未知标签警告
        self.print_class_statistics()
        
        # 生成数据集配置文件
        self.create_dataset_yaml()

    def create_dataset_yaml(self):
        """创建YOLO数据集配置文件"""
        yaml_content = f"""# YOLO Segmentation Dataset Configuration
# Generated from LabelMe annotations
# Multi-class support enabled

path: {os.path.abspath(self.output_dir)}
train: images
val: images  # 使用相同目录，实际使用时应该分割train/val

# Classes
nc: {len(self.class_names)}  # number of classes
names: {self.class_names}  # class names

# Class mapping
class_mapping: {dict(self.class_mapping)}

# Dataset info
total_samples: {self.target_count}
source_annotations: {len(os.listdir(self.annotations_dir))} LabelMe JSON files
auto_discovered_classes: {self.auto_discover_classes}
"""
        
        yaml_path = os.path.join(self.output_dir, "dataset.yaml")
        with open(yaml_path, 'w', encoding='utf-8') as f:
            f.write(yaml_content)
        
        print(f"数据集配置文件已保存: {yaml_path}")


def main():
    """主函数"""
    print("YOLO分割训练数据生成器 (多类别支持)")
    print("=" * 50)
    
    # 示例1: 自动发现类别
    print("\n示例1: 自动发现类别")
    generator1 = YOLODataGenerator(
        annotations_dir="../data/annotations",
        output_dir="../data/yolo_dataset_auto", 
        target_count=2200,
        auto_discover_classes=True
    )
    
    # 示例2: 手动指定类别映射
    print("\n示例2: 手动指定类别映射")
    custom_mapping = {
        'part': 0,
        'defect': 1,
        'background': 2
    }
    generator2 = YOLODataGenerator(
        annotations_dir="../data/annotations",
        output_dir="../data/yolo_dataset_custom", 
        target_count=2200,
        class_mapping=custom_mapping,
        auto_discover_classes=False
    )
    
    # 选择使用哪个生成器
    print("\n选择生成器:")
    print("1. 自动发现类别")
    print("2. 使用自定义类别映射")
    choice = input("请选择 (1 或 2): ").strip()
    
    if choice == "2":
        generator = generator2
        print("使用自定义类别映射")
    else:
        generator = generator1
        print("使用自动发现类别")
    
    # 生成数据集
    generator.generate_dataset()
    
    print("\n生成完成! 可以使用以下命令训练YOLO模型:")
    print(f"yolo segment train data={generator.output_dir}/dataset.yaml model=yolo11n-seg.pt epochs=100")


if __name__ == "__main__":
    main() 