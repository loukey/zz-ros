#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
分割标注转OBB标注转换器 - YOLO11-OBB版本
将JSON格式的分割标注转换为ultralytics YOLO11-OBB格式
"""

import json
import os
import cv2
import numpy as np
from typing import List, Tuple, Dict, Any, Optional
import glob
import shutil
import random
from pathlib import Path


class SegmentationToYOLOOBBConverter:
    """分割标注转YOLO11-OBB标注转换器"""
    
    def __init__(self, class_mapping: Optional[Dict[str, int]] = None):
        """
        初始化转换器
        
        Args:
            class_mapping: 类别名称到ID的映射，如果为None则自动生成
        """
        if class_mapping is None:
            class_mapping = {'central': 0, 'head': 1}
        self.class_mapping = class_mapping
        self.class_names = {v: k for k, v in self.class_mapping.items()}
    
    def polygon_to_obb_points(self, points: List[List[float]]) -> np.ndarray:
        """
        将多边形点转换为旋转边界框的四个角点
        
        Args:
            points: 多边形顶点列表 [[x1, y1], [x2, y2], ...]
            
        Returns:
            四个角点的坐标数组 [[x1, y1], [x2, y2], [x3, y3], [x4, y4]]
        """
        # 转换为numpy数组
        points_array = np.array(points, dtype=np.float32)
        
        # 计算最小外接旋转矩形
        rect = cv2.minAreaRect(points_array)
        
        # 获取四个角点
        box_points = cv2.boxPoints(rect)
        
        # 确保角点按顺序排列（通常是逆时针）
        box_points = np.array(box_points, dtype=np.float32)
        
        return box_points
    
    def normalize_obb_points(self, box_points: np.ndarray, img_width: int, img_height: int) -> np.ndarray:
        """
        将OBB角点坐标归一化到[0, 1]范围
        
        Args:
            box_points: 四个角点坐标
            img_width, img_height: 图像尺寸
            
        Returns:
            归一化后的角点坐标
        """
        normalized_points = box_points.copy()
        normalized_points[:, 0] /= img_width
        normalized_points[:, 1] /= img_height
        
        # 确保坐标在有效范围内
        normalized_points = np.clip(normalized_points, 0.0, 1.0)
        
        return normalized_points
    
    def convert_json_to_obb(self, json_path: str, img_width: int, img_height: int) -> List[str]:
        """
        将单个JSON标注文件转换为YOLO11-OBB格式
        
        Args:
            json_path: JSON标注文件路径
            img_width, img_height: 对应图像的尺寸
            
        Returns:
            YOLO11-OBB标注行列表
        """
        with open(json_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
        
        obb_lines = []
        
        for shape in data.get('shapes', []):
            if shape['shape_type'] != 'polygon':
                continue
            
            label = shape['label']
            if label not in self.class_mapping:
                print(f"警告: 未知类别 '{label}' 在文件 {json_path}")
                continue
            
            class_id = self.class_mapping[label]
            points = shape['points']
            
            # 转换为OBB四个角点
            box_points = self.polygon_to_obb_points(points)
            
            # 归一化坐标
            normalized_points = self.normalize_obb_points(box_points, img_width, img_height)
            
            # 格式化为YOLO11-OBB格式: class_id x1 y1 x2 y2 x3 y3 x4 y4
            coords = normalized_points.flatten()
            coords_str = ' '.join([f"{coord:.6f}" for coord in coords])
            obb_line = f"{class_id} {coords_str}"
            obb_lines.append(obb_line)
        
        return obb_lines
    
    def get_image_size(self, img_path: str) -> Tuple[int, int]:
        """
        获取图像尺寸
        
        Args:
            img_path: 图像文件路径
            
        Returns:
            (width, height) 图像尺寸
        """
        img = cv2.imread(img_path)
        if img is None:
            raise ValueError(f"无法读取图像: {img_path}")
        
        height, width = img.shape[:2]
        return width, height
    
    def split_dataset(self, file_list: List[str], train_ratio: float = 0.8, val_ratio: float = 0.1, test_ratio: float = 0.1) -> Dict[str, List[str]]:
        """
        划分数据集
        
        Args:
            file_list: 文件列表
            train_ratio: 训练集比例
            val_ratio: 验证集比例
            test_ratio: 测试集比例
            
        Returns:
            划分后的数据集字典
        """
        # 确保比例之和为1
        total_ratio = train_ratio + val_ratio + test_ratio
        if abs(total_ratio - 1.0) > 1e-6:
            print(f"警告: 数据集划分比例之和不为1 ({total_ratio}), 自动调整")
            train_ratio /= total_ratio
            val_ratio /= total_ratio
            test_ratio /= total_ratio
        
        # 随机打乱文件列表
        random.shuffle(file_list)
        
        total_files = len(file_list)
        train_count = int(total_files * train_ratio)
        val_count = int(total_files * val_ratio)
        
        train_files = file_list[:train_count]
        val_files = file_list[train_count:train_count + val_count]
        test_files = file_list[train_count + val_count:]
        
        return {
            'train': train_files,
            'val': val_files,
            'test': test_files
        }
    
    def create_yolo_dataset(self, annotations_dir: str, images_dir: str, output_dir: str, 
                           train_ratio: float = 0.8, val_ratio: float = 0.1, test_ratio: float = 0.1):
        """
        创建完整的YOLO11-OBB数据集
        
        Args:
            annotations_dir: 标注文件夹路径
            images_dir: 图像文件夹路径
            output_dir: 输出文件夹路径
            train_ratio: 训练集比例
            val_ratio: 验证集比例
            test_ratio: 测试集比例
        """
        # 创建输出目录结构
        os.makedirs(output_dir, exist_ok=True)
        
        # 创建子目录
        for subset in ['train', 'val', 'test']:
            os.makedirs(os.path.join(output_dir, 'images', subset), exist_ok=True)
            os.makedirs(os.path.join(output_dir, 'labels', subset), exist_ok=True)
        
        # 获取所有JSON文件
        json_files = glob.glob(os.path.join(annotations_dir, "*.json"))
        
        if not json_files:
            print(f"在 {annotations_dir} 中未找到JSON文件")
            return
        
        print(f"找到 {len(json_files)} 个标注文件")
        
        # 验证对应的图像文件
        valid_files = []
        for json_path in json_files:
            base_name = os.path.splitext(os.path.basename(json_path))[0]
            
            # 查找对应的图像文件
            img_path = None
            for ext in ['.png', '.jpg', '.jpeg', '.bmp', '.tiff']:
                potential_path = os.path.join(images_dir, base_name + ext)
                if os.path.exists(potential_path):
                    img_path = potential_path
                    break
            
            if img_path is None:
                print(f"警告: 未找到对应的图像文件: {base_name}")
                continue
            
            valid_files.append((json_path, img_path, base_name))
        
        print(f"有效文件对: {len(valid_files)}")
        
        # 划分数据集
        dataset_split = self.split_dataset(valid_files, train_ratio, val_ratio, test_ratio)
        
        converted_count = 0
        failed_count = 0
        subset_stats = {}
        
        for subset, files in dataset_split.items():
            if not files:
                continue
                
            print(f"\n处理 {subset} 数据集 ({len(files)} 个文件)")
            subset_converted = 0
            
            for json_path, img_path, base_name in files:
                try:
                    # 获取图像尺寸
                    img_width, img_height = self.get_image_size(img_path)
                    
                    # 转换标注
                    obb_lines = self.convert_json_to_obb(json_path, img_width, img_height)
                    
                    if not obb_lines:
                        print(f"  警告: {base_name} 没有有效的标注")
                        failed_count += 1
                        continue
                    
                    # 复制图像文件
                    img_dest = os.path.join(output_dir, 'images', subset, base_name + os.path.splitext(img_path)[1])
                    shutil.copy2(img_path, img_dest)
                    
                    # 保存标注文件
                    label_dest = os.path.join(output_dir, 'labels', subset, base_name + '.txt')
                    with open(label_dest, 'w', encoding='utf-8') as f:
                        f.write('\n'.join(obb_lines))
                    
                    subset_converted += 1
                    converted_count += 1
                    
                except Exception as e:
                    print(f"  ✗ 转换失败: {base_name} - {str(e)}")
                    failed_count += 1
            
            subset_stats[subset] = subset_converted
            print(f"  完成: {subset_converted} 个文件")
        
        print(f"\n数据集创建完成:")
        print(f"  成功转换: {converted_count} 个文件")
        print(f"  失败: {failed_count} 个文件")
        for subset, count in subset_stats.items():
            print(f"  {subset}: {count} 个文件")
        print(f"  输出目录: {output_dir}")
        
        # 创建dataset.yaml配置文件
        self.create_dataset_yaml(output_dir, subset_stats)
    
    def create_dataset_yaml(self, output_dir: str, subset_stats: Dict[str, int]):
        """
        创建YOLO11-OBB数据集配置文件
        
        Args:
            output_dir: 输出目录
            subset_stats: 各子集统计信息
        """
        # 创建类别名称列表
        class_names_list = [''] * len(self.class_mapping)
        for name, idx in self.class_mapping.items():
            class_names_list[idx] = name
        
        yaml_content = f"""# Ultralytics YOLO11-OBB Dataset Configuration
# Generated from LabelMe JSON annotations

# Dataset path (relative to this file)
path: {os.path.abspath(output_dir)}

# Dataset splits
train: images/train
val: images/val
test: images/test

# Number of classes
nc: {len(self.class_mapping)}

# Class names
names: {class_names_list}

# Dataset statistics
dataset_info:
  total_images: {sum(subset_stats.values())}
  train_images: {subset_stats.get('train', 0)}
  val_images: {subset_stats.get('val', 0)}
  test_images: {subset_stats.get('test', 0)}
  
# Class mapping
class_mapping: {dict(self.class_mapping)}

# Task type
task: obb  # Oriented Bounding Box detection
"""
        
        yaml_path = os.path.join(output_dir, "dataset.yaml")
        with open(yaml_path, 'w', encoding='utf-8') as f:
            f.write(yaml_content)
        
        print(f"数据集配置文件已保存: {yaml_path}")
    
    def visualize_obb_dataset(self, dataset_dir: str, subset: str = 'train', num_samples: int = 5):
        """
        可视化YOLO11-OBB数据集
        
        Args:
            dataset_dir: 数据集目录
            subset: 子集名称 ('train', 'val', 'test')
            num_samples: 可视化样本数量
        """
        images_dir = os.path.join(dataset_dir, 'images', subset)
        labels_dir = os.path.join(dataset_dir, 'labels', subset)
        
        if not os.path.exists(images_dir) or not os.path.exists(labels_dir):
            print(f"数据集目录不存在: {images_dir} 或 {labels_dir}")
            return
        
        # 获取图像文件
        image_files = []
        for ext in ['.jpg', '.jpeg', '.png', '.bmp']:
            image_files.extend(glob.glob(os.path.join(images_dir, f"*{ext}")))
        
        if not image_files:
            print(f"在 {images_dir} 中未找到图像文件")
            return
        
        # 随机选择样本
        sample_files = random.sample(image_files, min(num_samples, len(image_files)))
        
        colors = [(0, 255, 0), (0, 0, 255), (255, 0, 0), (255, 255, 0), (255, 0, 255), (0, 255, 255)]
        
        for img_path in sample_files:
            img = cv2.imread(img_path)
            if img is None:
                continue
            
            height, width = img.shape[:2]
            base_name = os.path.splitext(os.path.basename(img_path))[0]
            label_path = os.path.join(labels_dir, base_name + '.txt')
            
            if not os.path.exists(label_path):
                continue
            
            # 读取标注
            with open(label_path, 'r', encoding='utf-8') as f:
                lines = f.readlines()
            
            for line in lines:
                parts = line.strip().split()
                if len(parts) != 9:  # class_id + 8 coordinates
                    continue
                
                class_id = int(parts[0])
                coords = list(map(float, parts[1:]))
                
                # 反归一化坐标
                points = []
                for i in range(0, 8, 2):
                    x = coords[i] * width
                    y = coords[i + 1] * height
                    points.append([x, y])
                
                points = np.array(points, dtype=np.int32)
                
                # 绘制旋转矩形
                color = colors[class_id % len(colors)]
                cv2.drawContours(img, [points], 0, color, 2)
                
                # 绘制中心点
                center = points.mean(axis=0).astype(int)
                cv2.circle(img, tuple(center), 5, color, -1)
                
                # 标注类别
                class_name = self.class_names.get(class_id, f"class_{class_id}")
                cv2.putText(img, class_name, tuple(center + [10, -10]),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            
            # 保存可视化结果
            output_path = os.path.join(dataset_dir, f"visualization_{base_name}.jpg")
            cv2.imwrite(output_path, img)
            print(f"可视化结果已保存: {output_path}")


def main():
    """主函数"""
    # 创建转换器
    converter = SegmentationToYOLOOBBConverter()
    
    # 设置路径
    annotations_dir = "../data/annotations"
    images_dir = "../data/origin_img"
    output_dir = "../data/yolo11_obb_dataset"
    
    print("=== 分割标注转YOLO11-OBB数据集 ===")
    print(f"标注目录: {annotations_dir}")
    print(f"图像目录: {images_dir}")
    print(f"输出目录: {output_dir}")
    print()
    
    # 创建YOLO11-OBB数据集
    converter.create_yolo_dataset(
        annotations_dir=annotations_dir,
        images_dir=images_dir,
        output_dir=output_dir,
        train_ratio=0.7,  # 70% 训练集
        val_ratio=0.2,    # 20% 验证集
        test_ratio=0.1    # 10% 测试集
    )
    
    # 可视化示例
    print("\n=== 可视化示例 ===")
    converter.visualize_obb_dataset(output_dir, 'train', 3)
    
    print(f"\n完成！可以使用以下命令训练YOLO11-OBB模型:")
    print(f"yolo obb train data={output_dir}/dataset.yaml model=yolo11n-obb.pt epochs=100 imgsz=640")


if __name__ == "__main__":
    main() 