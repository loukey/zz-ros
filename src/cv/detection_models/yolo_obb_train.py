#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO11-OBB 训练脚本
用于训练旋转边界框检测模型
"""

import os
import sys
import yaml
import argparse
from pathlib import Path
from typing import Dict, Any, Optional
import torch
from ultralytics import YOLO
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime


class YOLO11OBBTrainer:
    """YOLO11-OBB训练器"""
    
    def __init__(self, config_path: Optional[str] = None):
        """
        初始化训练器
        
        Args:
            config_path: 配置文件路径
        """
        self.config = self.load_config(config_path) if config_path else self.get_default_config()
        self.model = None
        self.results = None
        
        # 设置训练输出目录
        self.output_dir = self.config.get('output_dir', 'runs/obb_train')
        os.makedirs(self.output_dir, exist_ok=True)
        
        print(f"YOLO11-OBB训练器初始化完成")
        print(f"输出目录: {self.output_dir}")
    
    def load_config(self, config_path: str) -> Dict[str, Any]:
        """加载配置文件"""
        if not os.path.exists(config_path):
            raise FileNotFoundError(f"配置文件不存在: {config_path}")
        
        with open(config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
        
        print(f"配置文件加载成功: {config_path}")
        return config
    
    def get_default_config(self) -> Dict[str, Any]:
        """获取默认配置"""
        return {
            'data': '../data/yolo11_obb_dataset/dataset.yaml',
            'model': 'yolo11n-obb.pt',
            'epochs': 100,
            'imgsz': 640,
            'batch': 16,
            'lr0': 0.01,
            'lrf': 0.01,
            'momentum': 0.937,
            'weight_decay': 0.0005,
            'warmup_epochs': 3,
            'warmup_momentum': 0.8,
            'warmup_bias_lr': 0.1,
            'box': 7.5,
            'cls': 0.5,
            'dfl': 1.5,
            'pose': 12.0,
            'kobj': 1.0,
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
            'save_period': 10,
            'cache': False,
            'device': '',
            'workers': 8,
            'project': 'runs/obb_train',
            'name': 'exp',
            'exist_ok': False,
            'pretrained': True,
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
            'split': 'val',
            'save_json': False,
            'save_hybrid': False,
            'conf': None,
            'iou': 0.7,
            'max_det': 300,
            'half': False,
            'dnn': False,
            'plots': True,
            'output_dir': 'runs/obb_train'
        }
    
    def validate_dataset(self, data_path: str) -> bool:
        """验证数据集"""
        print(f"验证数据集: {data_path}")
        
        if not os.path.exists(data_path):
            print(f"❌ 数据集配置文件不存在: {data_path}")
            return False
        
        # 读取数据集配置
        with open(data_path, 'r', encoding='utf-8') as f:
            data_config = yaml.safe_load(f)
        
        # 检查必要字段
        required_fields = ['path', 'train', 'val', 'nc', 'names']
        for field in required_fields:
            if field not in data_config:
                print(f"❌ 数据集配置缺少字段: {field}")
                return False
        
        # 检查数据集路径
        dataset_path = data_config['path']
        if not os.path.exists(dataset_path):
            print(f"❌ 数据集路径不存在: {dataset_path}")
            return False
        
        # 检查训练和验证集
        train_path = os.path.join(dataset_path, data_config['train'])
        val_path = os.path.join(dataset_path, data_config['val'])
        
        if not os.path.exists(train_path):
            print(f"❌ 训练集路径不存在: {train_path}")
            return False
        
        if not os.path.exists(val_path):
            print(f"❌ 验证集路径不存在: {val_path}")
            return False
        
        # 统计数据集信息
        train_images = len([f for f in os.listdir(train_path) if f.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp'))])
        val_images = len([f for f in os.listdir(val_path) if f.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp'))])
        
        print(f"✅ 数据集验证通过")
        print(f"   类别数: {data_config['nc']}")
        print(f"   类别名: {data_config['names']}")
        print(f"   训练集: {train_images} 张图像")
        print(f"   验证集: {val_images} 张图像")
        
        return True
    
    def setup_model(self, model_name: str) -> YOLO:
        """设置模型"""
        print(f"设置模型: {model_name}")
        
        try:
            # 加载预训练模型
            model = YOLO(model_name)
            print(f"✅ 模型加载成功: {model_name}")
            
            # 打印模型信息
            if hasattr(model, 'info'):
                model.info()
            
            return model
        
        except Exception as e:
            print(f"❌ 模型加载失败: {e}")
            raise
    
    def train(self) -> Any:
        """训练模型"""
        print("开始训练YOLO11-OBB模型...")
        
        # 验证数据集
        if not self.validate_dataset(self.config['data']):
            raise ValueError("数据集验证失败")
        
        # 设置模型
        self.model = self.setup_model(self.config['model'])
        
        # 设置训练参数
        train_args = {
            'data': self.config['data'],
            'epochs': self.config['epochs'],
            'imgsz': self.config['imgsz'],
            'batch': self.config['batch'],
            'lr0': self.config['lr0'],
            'lrf': self.config['lrf'],
            'momentum': self.config['momentum'],
            'weight_decay': self.config['weight_decay'],
            'warmup_epochs': self.config['warmup_epochs'],
            'warmup_momentum': self.config['warmup_momentum'],
            'warmup_bias_lr': self.config['warmup_bias_lr'],
            'box': self.config['box'],
            'cls': self.config['cls'],
            'dfl': self.config['dfl'],
            'hsv_h': self.config['hsv_h'],
            'hsv_s': self.config['hsv_s'],
            'hsv_v': self.config['hsv_v'],
            'degrees': self.config['degrees'],
            'translate': self.config['translate'],
            'scale': self.config['scale'],
            'shear': self.config['shear'],
            'perspective': self.config['perspective'],
            'flipud': self.config['flipud'],
            'fliplr': self.config['fliplr'],
            'mosaic': self.config['mosaic'],
            'mixup': self.config['mixup'],
            'copy_paste': self.config['copy_paste'],
            'save_period': self.config['save_period'],
            'cache': self.config['cache'],
            'device': self.config['device'],
            'workers': self.config['workers'],
            'project': self.config['project'],
            'name': self.config['name'],
            'exist_ok': self.config['exist_ok'],
            'pretrained': self.config['pretrained'],
            'optimizer': self.config['optimizer'],
            'verbose': self.config['verbose'],
            'seed': self.config['seed'],
            'deterministic': self.config['deterministic'],
            'single_cls': self.config['single_cls'],
            'rect': self.config['rect'],
            'cos_lr': self.config['cos_lr'],
            'close_mosaic': self.config['close_mosaic'],
            'resume': self.config['resume'],
            'amp': self.config['amp'],
            'fraction': self.config['fraction'],
            'profile': self.config['profile'],
            'freeze': self.config['freeze'],
            'multi_scale': self.config['multi_scale'],
            'overlap_mask': self.config['overlap_mask'],
            'mask_ratio': self.config['mask_ratio'],
            'dropout': self.config['dropout'],
            'val': self.config['val'],
            'split': self.config['split'],
            'save_json': self.config['save_json'],
            'save_hybrid': self.config['save_hybrid'],
            'conf': self.config['conf'],
            'iou': self.config['iou'],
            'max_det': self.config['max_det'],
            'half': self.config['half'],
            'dnn': self.config['dnn'],
            'plots': self.config['plots']
        }
        
        # 过滤None值
        train_args = {k: v for k, v in train_args.items() if v is not None}
        
        print("训练参数:")
        for key, value in train_args.items():
            print(f"  {key}: {value}")
        
        # 开始训练
        try:
            self.results = self.model.train(**train_args)
            print("✅ 训练完成")
            return self.results
        
        except Exception as e:
            print(f"❌ 训练失败: {e}")
            raise
    
    def validate(self, model_path: Optional[str] = None) -> Any:
        """验证模型"""
        if model_path:
            model = YOLO(model_path)
        else:
            model = self.model
        
        if model is None:
            raise ValueError("模型未加载")
        
        print("开始验证模型...")
        
        val_results = model.val(
            data=self.config['data'],
            imgsz=self.config['imgsz'],
            batch=self.config['batch'],
            conf=self.config.get('conf', 0.001),
            iou=self.config['iou'],
            max_det=self.config['max_det'],
            half=self.config['half'],
            device=self.config['device'],
            dnn=self.config['dnn'],
            plots=self.config['plots'],
            split=self.config['split'],
            save_json=self.config['save_json'],
            save_hybrid=self.config['save_hybrid']
        )
        
        print("✅ 验证完成")
        return val_results
    
    def export_model(self, model_path: str, format: str = 'onnx') -> str:
        """导出模型"""
        print(f"导出模型: {model_path} -> {format}")
        
        model = YOLO(model_path)
        
        export_path = model.export(
            format=format,
            imgsz=self.config['imgsz'],
            half=self.config['half'],
            device=self.config['device'],
            dynamic=False,
            simplify=True,
            opset=11
        )
        
        print(f"✅ 模型导出成功: {export_path}")
        return export_path
    
    def plot_results(self, results_dir: str):
        """绘制训练结果"""
        print(f"绘制训练结果: {results_dir}")
        
        results_file = os.path.join(results_dir, 'results.csv')
        if not os.path.exists(results_file):
            print(f"❌ 结果文件不存在: {results_file}")
            return
        
        # 读取结果数据
        import pandas as pd
        df = pd.read_csv(results_file)
        
        # 创建图表
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        fig.suptitle('YOLO11-OBB Training Results', fontsize=16)
        
        # 损失曲线
        if 'train/box_loss' in df.columns:
            axes[0, 0].plot(df['epoch'], df['train/box_loss'], label='Train Box Loss')
            axes[0, 0].plot(df['epoch'], df['val/box_loss'], label='Val Box Loss')
            axes[0, 0].set_title('Box Loss')
            axes[0, 0].set_xlabel('Epoch')
            axes[0, 0].set_ylabel('Loss')
            axes[0, 0].legend()
            axes[0, 0].grid(True)
        
        # 分类损失
        if 'train/cls_loss' in df.columns:
            axes[0, 1].plot(df['epoch'], df['train/cls_loss'], label='Train Cls Loss')
            axes[0, 1].plot(df['epoch'], df['val/cls_loss'], label='Val Cls Loss')
            axes[0, 1].set_title('Classification Loss')
            axes[0, 1].set_xlabel('Epoch')
            axes[0, 1].set_ylabel('Loss')
            axes[0, 1].legend()
            axes[0, 1].grid(True)
        
        # mAP
        if 'metrics/mAP50(B)' in df.columns:
            axes[1, 0].plot(df['epoch'], df['metrics/mAP50(B)'], label='mAP@0.5')
            axes[1, 0].plot(df['epoch'], df['metrics/mAP50-95(B)'], label='mAP@0.5:0.95')
            axes[1, 0].set_title('mAP')
            axes[1, 0].set_xlabel('Epoch')
            axes[1, 0].set_ylabel('mAP')
            axes[1, 0].legend()
            axes[1, 0].grid(True)
        
        # 学习率
        if 'lr/pg0' in df.columns:
            axes[1, 1].plot(df['epoch'], df['lr/pg0'], label='Learning Rate')
            axes[1, 1].set_title('Learning Rate')
            axes[1, 1].set_xlabel('Epoch')
            axes[1, 1].set_ylabel('LR')
            axes[1, 1].legend()
            axes[1, 1].grid(True)
        
        plt.tight_layout()
        
        # 保存图表
        plot_path = os.path.join(self.output_dir, 'training_results.png')
        plt.savefig(plot_path, dpi=300, bbox_inches='tight')
        plt.close()
        
        print(f"✅ 训练结果图表已保存: {plot_path}")
    
    def save_config(self):
        """保存配置文件"""
        config_path = os.path.join(self.output_dir, 'train_config.yaml')
        with open(config_path, 'w', encoding='utf-8') as f:
            yaml.dump(self.config, f, default_flow_style=False, allow_unicode=True)
        
        print(f"✅ 配置文件已保存: {config_path}")


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='YOLO11-OBB训练脚本')
    parser.add_argument('--config', type=str, help='配置文件路径')
    parser.add_argument('--data', type=str, default='../data/yolo11_obb_dataset/dataset.yaml', help='数据集配置文件')
    parser.add_argument('--model', type=str, default='../models_cache/yolo11n-obb.pt', help='模型文件')
    parser.add_argument('--epochs', type=int, default=100, help='训练轮数')
    parser.add_argument('--imgsz', type=int, default=640, help='图像尺寸')
    parser.add_argument('--batch', type=int, default=16, help='批次大小')
    parser.add_argument('--lr0', type=float, default=0.01, help='初始学习率')
    parser.add_argument('--device', type=str, default='cuda', help='设备 (cpu, 0, 1, ...)')
    parser.add_argument('--project', type=str, default='runs/obb_train', help='项目目录')
    parser.add_argument('--name', type=str, default='exp', help='实验名称')
    parser.add_argument('--resume', action='store_true', help='恢复训练')
    parser.add_argument('--validate', action='store_true', help='只验证不训练')
    parser.add_argument('--export', type=str, help='导出模型格式 (onnx, torchscript, etc.)')
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("YOLO11-OBB 训练脚本")
    print("=" * 60)
    
    # 创建训练器
    if args.config:
        trainer = YOLO11OBBTrainer(args.config)
    else:
        trainer = YOLO11OBBTrainer()
        
        # 更新配置
        trainer.config.update({
            'data': args.data,
            'model': args.model,
            'epochs': args.epochs,
            'imgsz': args.imgsz,
            'batch': args.batch,
            'lr0': args.lr0,
            'device': args.device,
            'project': args.project,
            'name': args.name,
            'resume': args.resume
        })
    
    # 保存配置
    trainer.save_config()
    
    try:
        if args.validate:
            # 只验证
            trainer.validate(args.model)
        else:
            # 训练
            results = trainer.train()
            
            # 验证最佳模型
            best_model = os.path.join(trainer.config['project'], trainer.config['name'], 'weights', 'best.pt')
            if os.path.exists(best_model):
                print(f"\n验证最佳模型: {best_model}")
                trainer.validate(best_model)
            
            # 绘制结果
            results_dir = os.path.join(trainer.config['project'], trainer.config['name'])
            if os.path.exists(results_dir):
                trainer.plot_results(results_dir)
        
        # 导出模型
        if args.export:
            best_model = os.path.join(trainer.config['project'], trainer.config['name'], 'weights', 'best.pt')
            if os.path.exists(best_model):
                trainer.export_model(best_model, args.export)
        
        print("\n✅ 所有任务完成！")
        
    except Exception as e:
        print(f"\n❌ 训练失败: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
