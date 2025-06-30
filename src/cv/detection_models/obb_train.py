#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO11 OBB (Oriented Bounding Box) 训练脚本
用于训练旋转边界框检测模型
"""

import os
import yaml
import argparse
from pathlib import Path
from ultralytics import YOLO
import logging
from datetime import datetime
import torch
import shutil

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


class YOLO11OBBTrainer:
    """YOLO11 OBB训练器"""
    
    def __init__(self, data_yaml_path, model_size='n', pretrained=True):
        """
        初始化训练器
        
        Args:
            data_yaml_path (str): 数据集配置文件路径
            model_size (str): 模型大小 ('n', 's', 'm', 'l', 'x')
            pretrained (bool): 是否使用预训练模型
        """
        self.data_yaml_path = data_yaml_path
        self.model_size = model_size
        self.pretrained = pretrained
        
        # 验证数据集配置
        self.validate_dataset()
        
        # 初始化模型
        self.init_model()
        
        # 设置训练参数
        self.set_default_params()
    
    def validate_dataset(self):
        """验证数据集配置和文件"""
        if not os.path.exists(self.data_yaml_path):
            raise FileNotFoundError(f"数据集配置文件不存在: {self.data_yaml_path}")
        
        # 读取配置文件
        with open(self.data_yaml_path, 'r', encoding='utf-8') as f:
            self.dataset_config = yaml.safe_load(f)
        
        logger.info(f"数据集配置: {self.dataset_config}")
        
        # 验证必要的字段
        required_fields = ['path', 'train', 'nc', 'names']
        for field in required_fields:
            if field not in self.dataset_config:
                raise ValueError(f"数据集配置文件缺少必要字段: {field}")
        
        # 验证路径
        dataset_path = Path(self.dataset_config['path'])
        if not dataset_path.exists():
            raise FileNotFoundError(f"数据集路径不存在: {dataset_path}")
        
        # 验证训练数据
        train_images_path = dataset_path / self.dataset_config['train']
        train_labels_path = dataset_path / 'labels'
        
        if not train_images_path.exists():
            raise FileNotFoundError(f"训练图像目录不存在: {train_images_path}")
        
        if not train_labels_path.exists():
            raise FileNotFoundError(f"训练标签目录不存在: {train_labels_path}")
        
        # 统计数据
        image_files = list(train_images_path.glob('*.jpg')) + list(train_images_path.glob('*.png'))
        label_files = list(train_labels_path.glob('*.txt'))
        
        logger.info(f"找到 {len(image_files)} 张训练图像")
        logger.info(f"找到 {len(label_files)} 个标签文件")
        
        if len(image_files) == 0:
            raise ValueError("训练图像目录为空")
        
        if len(label_files) == 0:
            raise ValueError("标签目录为空")
        
        self.num_train_images = len(image_files)
        self.num_classes = self.dataset_config['nc']
        
    def init_model(self):
        """初始化模型"""
        if self.pretrained:
            model_name = f'yolo11{self.model_size}-obb.pt'
            logger.info(f"加载预训练模型: {model_name}")
        else:
            model_name = f'yolo11{self.model_size}-obb.yaml'
            logger.info(f"从配置文件创建模型: {model_name}")
        
        try:
            self.model = YOLO(model_name)
            logger.info(f"模型初始化成功: {model_name}")
        except Exception as e:
            logger.error(f"模型初始化失败: {str(e)}")
            raise
    
    def set_default_params(self):
        """设置默认训练参数"""
        self.train_params = {
            'data': self.data_yaml_path,
            'epochs': 100,
            'batch': 16,
            'imgsz': 640,
            'lr0': 0.01,
            'lrf': 0.01,
            'momentum': 0.937,
            'weight_decay': 0.0005,
            'warmup_epochs': 3.0,
            'warmup_momentum': 0.8,
            'warmup_bias_lr': 0.1,
            'box': 7.5,
            'cls': 0.5,
            'dfl': 1.5,
            'pose': 12.0,
            'kobj': 2.0,
            'label_smoothing': 0.0,
            'nbs': 64,
            'hsv_h': 0.015,
            'hsv_s': 0.7,
            'hsv_v': 0.4,
            'degrees': 0.0,
            'translate': 0.1,
            'scale': 0.5,
            'shear': 0.0,
            'perspective': 0.0,
            'flipud': 0.0,
            'fliplr': 0.5,
            'mosaic': 1.0,
            'mixup': 0.0,
            'copy_paste': 0.0,
            'auto_augment': 'randaugment',
            'erasing': 0.4,
            'crop_fraction': 1.0,
            'save': True,
            'save_period': -1,
            'cache': False,
            'device': '',
            'workers': 8,
            'project': './runs/train',
            'name': f'yolo11{self.model_size}_obb_{datetime.now().strftime("%Y%m%d_%H%M%S")}',
            'exist_ok': False,
            'pretrained': self.pretrained,
            'optimizer': 'auto',
            'verbose': True,
            'seed': 0,
            'deterministic': True,
            'single_cls': False,
            'rect': False,
            'cos_lr': False,
            'close_mosaic': 10,
            'resume': False,
            'amp': True,
            'fraction': 1.0,
            'profile': False,
            'freeze': None,
            'multi_scale': False,
            'overlap_mask': True,
            'mask_ratio': 4,
            'dropout': 0.0,
            'val': True,
        }
        
        # 根据数据集大小调整参数
        if self.num_train_images < 100:
            self.train_params['epochs'] = 50
            self.train_params['batch'] = 8
            logger.info("检测到小数据集，调整训练参数")
        elif self.num_train_images < 500:
            self.train_params['epochs'] = 80
            self.train_params['batch'] = 12
        
        # 检查GPU可用性并调整batch size
        if torch.cuda.is_available():
            gpu_memory = torch.cuda.get_device_properties(0).total_memory / 1024**3
            logger.info(f"检测到GPU: {torch.cuda.get_device_name(0)}, 内存: {gpu_memory:.1f}GB")
            
            if gpu_memory < 8:
                self.train_params['batch'] = max(4, self.train_params['batch'] // 2)
                logger.info("GPU内存较小，减少batch size")
        else:
            logger.warning("未检测到GPU，将使用CPU训练（速度较慢）")
            self.train_params['batch'] = max(2, self.train_params['batch'] // 4)
    
    def update_params(self, **kwargs):
        """更新训练参数"""
        self.train_params.update(kwargs)
        logger.info(f"更新训练参数: {kwargs}")
    
    def create_train_val_split(self, val_ratio=0.2):
        """创建训练验证集分割"""
        dataset_path = Path(self.dataset_config['path'])
        images_path = dataset_path / self.dataset_config['train']
        labels_path = dataset_path / 'labels'
        
        # 获取所有图像文件
        image_files = []
        for ext in ['*.jpg', '*.jpeg', '*.png', '*.bmp']:
            image_files.extend(images_path.glob(ext))
        
        # 随机分割
        import random
        random.seed(42)
        random.shuffle(image_files)
        
        split_idx = int(len(image_files) * (1 - val_ratio))
        train_files = image_files[:split_idx]
        val_files = image_files[split_idx:]
        
        # 创建验证集目录
        val_images_path = dataset_path / 'val_images'
        val_labels_path = dataset_path / 'val_labels'
        val_images_path.mkdir(exist_ok=True)
        val_labels_path.mkdir(exist_ok=True)
        
        # 移动验证集文件
        for img_file in val_files:
            # 移动图像
            shutil.move(str(img_file), str(val_images_path / img_file.name))
            
            # 移动对应的标签
            label_file = labels_path / f"{img_file.stem}.txt"
            if label_file.exists():
                shutil.move(str(label_file), str(val_labels_path / label_file.name))
        
        # 更新数据集配置
        self.dataset_config['val'] = 'val_images'
        updated_yaml_path = dataset_path / 'dataset_with_val.yaml'
        
        with open(updated_yaml_path, 'w', encoding='utf-8') as f:
            yaml.dump(self.dataset_config, f, default_flow_style=False, allow_unicode=True)
        
        self.data_yaml_path = str(updated_yaml_path)
        self.train_params['data'] = self.data_yaml_path
        
        logger.info(f"创建训练验证集分割: 训练集 {len(train_files)} 张, 验证集 {len(val_files)} 张")
        
    def train(self, create_val_split=True):
        """开始训练"""
        logger.info("=== 开始YOLO11 OBB训练 ===")
        logger.info(f"数据集: {self.data_yaml_path}")
        logger.info(f"模型: yolo11{self.model_size}-obb")
        logger.info(f"类别数: {self.num_classes}")
        logger.info(f"训练图像数: {self.num_train_images}")
        
        # 创建训练验证集分割
        if create_val_split and 'val' not in self.dataset_config:
            self.create_train_val_split()
        
        # 打印主要训练参数
        logger.info("主要训练参数:")
        important_params = ['epochs', 'batch', 'imgsz', 'lr0', 'device']
        for param in important_params:
            if param in self.train_params:
                logger.info(f"  {param}: {self.train_params[param]}")
        
        try:
            # 开始训练
            results = self.model.train(**self.train_params)
            logger.info("训练完成！")
            
            # 保存训练结果信息
            self.save_training_info(results)
            
            return results
            
        except Exception as e:
            logger.error(f"训练过程中出现错误: {str(e)}")
            raise
    
    def save_training_info(self, results):
        """保存训练信息"""
        info = {
            'model_size': self.model_size,
            'num_classes': self.num_classes,
            'num_train_images': self.num_train_images,
            'train_params': self.train_params,
            'dataset_config': self.dataset_config,
            'training_time': datetime.now().isoformat(),
        }
        
        # 保存到训练结果目录
        save_dir = Path(self.train_params['project']) / self.train_params['name']
        info_file = save_dir / 'training_info.yaml'
        
        with open(info_file, 'w', encoding='utf-8') as f:
            yaml.dump(info, f, default_flow_style=False, allow_unicode=True)
        
        logger.info(f"训练信息已保存: {info_file}")
    
    def validate(self, model_path=None):
        """验证模型"""
        if model_path:
            model = YOLO(model_path)
        else:
            model = self.model
            
        logger.info("开始模型验证...")
        results = model.val(data=self.data_yaml_path)
        logger.info("验证完成！")
        
        return results


def parse_args():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(description='YOLO11 OBB 训练脚本')
    
    parser.add_argument('--data', type=str, required=True,
                       help='数据集配置文件路径 (dataset.yaml)')
    parser.add_argument('--model', type=str, default='n',
                       choices=['n', 's', 'm', 'l', 'x'],
                       help='模型大小')
    parser.add_argument('--epochs', type=int, default=100,
                       help='训练轮数')
    parser.add_argument('--batch', type=int, default=16,
                       help='批次大小')
    parser.add_argument('--imgsz', type=int, default=640,
                       help='图像尺寸')
    parser.add_argument('--lr0', type=float, default=0.01,
                       help='初始学习率')
    parser.add_argument('--device', type=str, default='',
                       help='训练设备 (cpu, 0, 1, 2, 3 或 0,1,2,3)')
    parser.add_argument('--project', type=str, default='./runs/train',
                       help='项目保存目录')
    parser.add_argument('--name', type=str, default='',
                       help='实验名称')
    parser.add_argument('--pretrained', action='store_true',
                       help='使用预训练模型')
    parser.add_argument('--no-val-split', action='store_true',
                       help='不创建训练验证集分割')
    
    return parser.parse_args()


def main():
    """主函数"""
    args = parse_args()
    
    try:
        # 创建训练器
        trainer = YOLO11OBBTrainer(
            data_yaml_path=args.data,
            model_size=args.model,
            pretrained=args.pretrained
        )
        
        # 更新训练参数
        update_params = {}
        if args.epochs != 100:
            update_params['epochs'] = args.epochs
        if args.batch != 16:
            update_params['batch'] = args.batch
        if args.imgsz != 640:
            update_params['imgsz'] = args.imgsz
        if args.lr0 != 0.01:
            update_params['lr0'] = args.lr0
        if args.device:
            update_params['device'] = args.device
        if args.project != './runs/train':
            update_params['project'] = args.project
        if args.name:
            update_params['name'] = args.name
        
        if update_params:
            trainer.update_params(**update_params)
        
        # 开始训练
        results = trainer.train(create_val_split=not args.no_val_split)
        
        logger.info("=== 训练完成 ===")
        
    except Exception as e:
        logger.error(f"训练失败: {str(e)}")
        return 1
    
    return 0


if __name__ == "__main__":
    exit(main())
