#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO11-OBB 训练数据生成器
从LabelMe标注文件生成增强的YOLO11-OBB训练数据
支持旋转边界框(Oriented Bounding Box)检测
"""

import os
import json
import cv2
import numpy as np
import random
from pathlib import Path
import math
from collections import defaultdict
import yaml
from typing import List, Tuple, Dict, Any


class YOLO11OBBDataGenerator:
    def __init__(self, annotations_dir="./data/annotations", 
                 output_dir="./data/yolo11_obb_dataset", 
                 target_count=500,
                 class_mapping=None,
                 auto_discover_classes=True,
                 train_ratio=0.7,
                 val_ratio=0.2,
                 test_ratio=0.1):
        self.annotations_dir = annotations_dir
        self.output_dir = output_dir
        self.target_count = target_count
        self.auto_discover_classes = auto_discover_classes
        self.train_ratio = train_ratio
        self.val_ratio = val_ratio
        self.test_ratio = test_ratio
        
        # 验证比例总和
        total_ratio = train_ratio + val_ratio + test_ratio
        if abs(total_ratio - 1.0) > 1e-6:
            raise ValueError(f"数据集分割比例总和必须为1.0，当前为{total_ratio}")
        
        # 类别映射相关
        self.class_mapping = class_mapping or {}  # label_name -> class_id
        self.reverse_mapping = {}  # class_id -> label_name
        self.class_names = []  # 按class_id顺序排列的类别名称
        self.unknown_labels = set()  # 记录未知的标签
        
        # 创建输出目录结构
        self.create_output_directories()
        
        # 如果需要自动发现类别，先扫描所有文件
        if self.auto_discover_classes:
            self.discover_classes()
        else:
            self.setup_class_mapping()
    
    def create_output_directories(self):
        """创建输出目录结构"""
        # 主目录
        os.makedirs(self.output_dir, exist_ok=True)
        
        # 图像目录
        self.images_dir = os.path.join(self.output_dir, "images")
        self.train_images_dir = os.path.join(self.images_dir, "train")
        self.val_images_dir = os.path.join(self.images_dir, "val")
        self.test_images_dir = os.path.join(self.images_dir, "test")
        
        # 标签目录
        self.labels_dir = os.path.join(self.output_dir, "labels")
        self.train_labels_dir = os.path.join(self.labels_dir, "train")
        self.val_labels_dir = os.path.join(self.labels_dir, "val")
        self.test_labels_dir = os.path.join(self.labels_dir, "test")
        
        # 创建所有目录
        for dir_path in [self.train_images_dir, self.val_images_dir, self.test_images_dir,
                        self.train_labels_dir, self.val_labels_dir, self.test_labels_dir]:
            os.makedirs(dir_path, exist_ok=True)
        
        print(f"输出目录结构创建完成: {self.output_dir}")
    
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

    def compute_obb_from_polygon(self, points: np.ndarray) -> Tuple[np.ndarray, float]:
        """
        从多边形计算最小面积旋转边界框(OBB)
        
        Args:
            points: 多边形顶点坐标 (N, 2)
            
        Returns:
            obb_points: 旋转边界框的四个顶点坐标 (4, 2)
            angle: 旋转角度(弧度)
        """
        # 确保多边形是凸的
        hull = cv2.convexHull(points)
        hull = hull.reshape(-1, 2)
        
        if len(hull) < 3:
            # 如果点太少，返回原始边界框
            x_min, y_min = np.min(hull, axis=0)
            x_max, y_max = np.max(hull, axis=0)
            obb_points = np.array([
                [x_min, y_min],
                [x_max, y_min],
                [x_max, y_max],
                [x_min, y_max]
            ], dtype=np.float32)
            return obb_points, 0.0
        
        # 计算最小面积旋转边界框
        # 使用OpenCV的minAreaRect
        rect = cv2.minAreaRect(hull)
        box = cv2.boxPoints(rect)
        box = np.array(box, dtype=np.float32)
        
        # 获取旋转角度
        angle = rect[2]
        if angle < -45:
            angle += 90
        
        return box, np.radians(angle)

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
            angle = random.uniform(-30, 30)  # OBB旋转角度限制更小
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
        
        # 计算旋转中心
        center = (img_width / 2, img_height / 2)
        
        # 计算旋转矩阵
        rotation_matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
        
        # 计算新图像尺寸
        cos_val = abs(rotation_matrix[0, 0])
        sin_val = abs(rotation_matrix[0, 1])
        new_width = int((img_height * sin_val) + (img_width * cos_val))
        new_height = int((img_height * cos_val) + (img_width * sin_val))
        
        # 调整旋转矩阵
        rotation_matrix[0, 2] += (new_width / 2) - center[0]
        rotation_matrix[1, 2] += (new_height / 2) - center[1]
        
        # 旋转图像
        rotated_image = cv2.warpAffine(image, rotation_matrix, (new_width, new_height))
        
        # 旋转标注点
        rotated_annotations = []
        for ann in annotations:
            rotated_points = ann['points'].copy()
            
            # 转换点坐标
            for i in range(len(rotated_points)):
                x, y = rotated_points[i]
                new_x = rotation_matrix[0, 0] * x + rotation_matrix[0, 1] * y + rotation_matrix[0, 2]
                new_y = rotation_matrix[1, 0] * x + rotation_matrix[1, 1] * y + rotation_matrix[1, 2]
                rotated_points[i] = [new_x, new_y]
            
            rotated_annotations.append({
                'label': ann['label'],
                'points': rotated_points,
                'class_id': ann['class_id']
            })
        
        return rotated_image, rotated_annotations

    def adjust_brightness_contrast(self, image, brightness, contrast):
        """调整亮度和对比度"""
        return cv2.convertScaleAbs(image, alpha=contrast, beta=brightness)

    def add_noise(self, image):
        """添加噪声"""
        noise = np.random.normal(0, 25, image.shape).astype(np.uint8)
        noisy_image = cv2.add(image, noise)
        return np.clip(noisy_image, 0, 255)

    def add_blur(self, image):
        """添加模糊"""
        kernel_size = random.choice([3, 5])
        return cv2.GaussianBlur(image, (kernel_size, kernel_size), 0)

    def normalize_obb_coordinates(self, obb_points, img_width, img_height):
        """归一化OBB坐标到[0,1]范围"""
        normalized_points = obb_points.copy()
        normalized_points[:, 0] /= img_width
        normalized_points[:, 1] /= img_height
        return normalized_points

    def obb_points_to_yolo_format(self, obb_points):
        """将OBB点转换为YOLO11-OBB格式"""
        # YOLO11-OBB格式: class_id x1 y1 x2 y2 x3 y3 x4 y4
        # 其中(x1,y1), (x2,y2), (x3,y3), (x4,y4)是四个角点的归一化坐标
        flattened = obb_points.flatten()
        return ' '.join([f'{coord:.6f}' for coord in flattened])

    def save_obb_data(self, image, annotations, base_name, index, split='train'):
        """保存OBB数据"""
        img_height, img_width = image.shape[:2]
        
        # 选择保存目录
        if split == 'train':
            images_dir = self.train_images_dir
            labels_dir = self.train_labels_dir
        elif split == 'val':
            images_dir = self.val_images_dir
            labels_dir = self.val_labels_dir
        else:  # test
            images_dir = self.test_images_dir
            labels_dir = self.test_labels_dir
        
        # 保存图像
        image_filename = f"{base_name}_{index:03d}.jpg"
        image_path = os.path.join(images_dir, image_filename)
        cv2.imwrite(image_path, image)
        
        # 保存标签
        label_filename = f"{base_name}_{index:03d}.txt"
        label_path = os.path.join(labels_dir, label_filename)
        
        with open(label_path, 'w', encoding='utf-8') as f:
            for ann in annotations:
                # 计算OBB
                obb_points, angle = self.compute_obb_from_polygon(ann['points'])
                
                # 归一化坐标
                normalized_obb = self.normalize_obb_coordinates(obb_points, img_width, img_height)
                
                # 转换为YOLO格式
                yolo_line = f"{ann['class_id']} {self.obb_points_to_yolo_format(normalized_obb)}"
                f.write(yolo_line + '\n')
        
        return image_filename, label_filename

    def print_class_statistics(self):
        """打印类别统计信息"""
        if self.unknown_labels:
            print(f"警告: 发现未知标签: {self.unknown_labels}")
        
        print(f"\n类别统计:")
        for class_id, class_name in enumerate(self.class_names):
            print(f"  {class_id}: {class_name}")

    def generate_dataset(self):
        """生成完整的数据集"""
        print(f"\n开始生成YOLO11-OBB数据集...")
        print(f"目标数量: {self.target_count}")
        print(f"输出目录: {self.output_dir}")
        
        # 获取所有JSON文件
        json_files = [f for f in os.listdir(self.annotations_dir) if f.lower().endswith('.json')]
        if not json_files:
            print(f"❌ 在 {self.annotations_dir} 中没有找到JSON文件")
            return
        
        print(f"找到 {len(json_files)} 个标注文件")
        
        # 统计数据
        total_generated = 0
        class_counts = defaultdict(int)
        split_counts = {'train': 0, 'val': 0, 'test': 0}
        
        # 处理每个原始文件
        for json_file in json_files:
            json_path = os.path.join(self.annotations_dir, json_file)
            base_name = os.path.splitext(json_file)[0]
            
            print(f"\n处理文件: {json_file}")
            
            # 加载数据
            image, annotations = self.load_labelme_data(json_path)
            if image is None or not annotations:
                print(f"跳过文件 {json_file} (无有效数据)")
                continue
            
            # 计算每个文件需要生成的增强样本数
            samples_per_file = max(1, self.target_count // len(json_files))
            
            # 生成原始样本
            split = self.determine_split(split_counts)
            image_filename, label_filename = self.save_obb_data(
                image, annotations, base_name, 0, split
            )
            split_counts[split] += 1
            total_generated += 1
            
            # 统计类别
            for ann in annotations:
                class_counts[ann['class_id']] += 1
            
            print(f"  原始样本: {image_filename}")
            
            # 生成增强样本
            for i in range(1, samples_per_file):
                if total_generated >= self.target_count:
                    break
                
                # 应用数据增强
                aug_image, aug_annotations = self.apply_random_augmentation(image, annotations)
                
                # 确定分割
                split = self.determine_split(split_counts)
                
                # 保存增强样本
                image_filename, label_filename = self.save_obb_data(
                    aug_image, aug_annotations, base_name, i, split
                )
                split_counts[split] += 1
                total_generated += 1
                
                # 统计类别
                for ann in aug_annotations:
                    class_counts[ann['class_id']] += 1
                
                print(f"  增强样本 {i}: {image_filename}")
        
        # 打印统计信息
        print(f"\n数据集生成完成!")
        print(f"总样本数: {total_generated}")
        print(f"训练集: {split_counts['train']}")
        print(f"验证集: {split_counts['val']}")
        print(f"测试集: {split_counts['test']}")
        
        print(f"\n类别分布:")
        for class_id, count in sorted(class_counts.items()):
            class_name = self.class_names[class_id]
            print(f"  {class_id} ({class_name}): {count}")
        
        # 创建数据集配置文件
        self.create_dataset_yaml()
        
        print(f"\n数据集已保存到: {self.output_dir}")
        print(f"配置文件: {os.path.join(self.output_dir, 'dataset.yaml')}")

    def determine_split(self, split_counts):
        """根据当前分布确定数据分割"""
        total = sum(split_counts.values())
        if total == 0:
            return 'train'
        
        current_train_ratio = split_counts['train'] / total
        current_val_ratio = split_counts['val'] / total
        
        if current_train_ratio < self.train_ratio:
            return 'train'
        elif current_val_ratio < self.val_ratio:
            return 'val'
        else:
            return 'test'

    def create_dataset_yaml(self):
        """创建数据集配置文件"""
        config = {
            'path': os.path.abspath(self.output_dir),
            'train': 'images/train',
            'val': 'images/val',
            'test': 'images/test',
            'nc': len(self.class_names),
            'names': self.class_names,
            'dataset_info': {
                'total_images': sum([
                    len([f for f in os.listdir(self.train_images_dir) if f.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp'))]),
                    len([f for f in os.listdir(self.val_images_dir) if f.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp'))]),
                    len([f for f in os.listdir(self.test_images_dir) if f.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp'))])
                ]),
                'train_images': len([f for f in os.listdir(self.train_images_dir) if f.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp'))]),
                'val_images': len([f for f in os.listdir(self.val_images_dir) if f.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp'))]),
                'test_images': len([f for f in os.listdir(self.test_images_dir) if f.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp'))])
            },
            'class_mapping': self.class_mapping,
            'task': 'obb'  # Oriented Bounding Box detection
        }
        
        yaml_path = os.path.join(self.output_dir, 'dataset.yaml')
        with open(yaml_path, 'w', encoding='utf-8') as f:
            yaml.dump(config, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
        
        print(f"数据集配置文件已创建: {yaml_path}")


def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description='YOLO11-OBB训练数据生成器')
    parser.add_argument('--annotations', type=str, default='../data/annotations',
                       help='LabelMe标注文件目录')
    parser.add_argument('--output', type=str, default='../data/yolo11_obb_dataset',
                       help='输出目录')
    parser.add_argument('--target_count', type=int, default=500,
                       help='目标生成样本数量')
    parser.add_argument('--train_ratio', type=float, default=0.7,
                       help='训练集比例')
    parser.add_argument('--val_ratio', type=float, default=0.2,
                       help='验证集比例')
    parser.add_argument('--test_ratio', type=float, default=0.1,
                       help='测试集比例')
    parser.add_argument('--class_mapping', type=str, default=None,
                       help='类别映射文件路径(JSON格式)')
    
    args = parser.parse_args()
    
    # 加载类别映射
    class_mapping = None
    if args.class_mapping and os.path.exists(args.class_mapping):
        with open(args.class_mapping, 'r', encoding='utf-8') as f:
            class_mapping = json.load(f)
        print(f"加载类别映射: {class_mapping}")
    
    # 创建数据生成器
    generator = YOLO11OBBDataGenerator(
        annotations_dir=args.annotations,
        output_dir=args.output,
        target_count=args.target_count,
        class_mapping=class_mapping,
        auto_discover_classes=class_mapping is None,
        train_ratio=args.train_ratio,
        val_ratio=args.val_ratio,
        test_ratio=args.test_ratio
    )
    
    # 生成数据集
    generator.generate_dataset()


if __name__ == "__main__":
    main()
