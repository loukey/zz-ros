#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO Segmentation 训练脚本
基于Ultralytics YOLO框架进行分割模型训练
"""

import os
import sys
import yaml
import argparse
from pathlib import Path
import ultralytics
from ultralytics import YOLO
import torch
from datetime import datetime


class YOLOSegmentationTrainer:
    def __init__(self, 
                 data_config="../data/yolo_dataset/dataset.yaml",
                 model="../models_cache/yolo11n-seg.pt",
                 project="./runs/segment",
                 name="train"):
        """
        初始化YOLO分割训练器
        
        Args:
            data_config: 数据集配置文件路径
            model: 预训练模型名称或路径
            project: 训练结果保存项目目录
            name: 训练实验名称
        """
        self.data_config = data_config
        self.model_name = model
        self.project = project
        self.name = name
        
        # 检查数据集配置
        self.validate_dataset()
        
        # 初始化模型
        self.model = None
        self.load_model()
        
        print(f"Ultralytics YOLO version: {ultralytics.__version__}")
        print(f"PyTorch version: {torch.__version__}")
        print(f"CUDA available: {torch.cuda.is_available()}")
        if torch.cuda.is_available():
            print(f"GPU device: {torch.cuda.get_device_name()}")
    
    def validate_dataset(self):
        """验证数据集配置"""
        if not os.path.exists(self.data_config):
            raise FileNotFoundError(f"数据集配置文件不存在: {self.data_config}")
        
        # 读取配置文件
        with open(self.data_config, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
        
        # 检查必要字段
        required_keys = ['path', 'train', 'nc', 'names']
        for key in required_keys:
            if key not in config:
                raise ValueError(f"数据集配置缺少必要字段: {key}")
        
        # 检查数据集路径
        dataset_path = config['path']
        if not os.path.exists(dataset_path):
            raise FileNotFoundError(f"数据集路径不存在: {dataset_path}")
        
        train_path = os.path.join(dataset_path, config['train'])
        if not os.path.exists(train_path):
            raise FileNotFoundError(f"训练图像目录不存在: {train_path}")
        
        print(f"✓ 数据集验证通过")
        print(f"  数据集路径: {dataset_path}")
        print(f"  类别数量: {config['nc']}")
        print(f"  类别名称: {config['names']}")
    
    def load_model(self):
        """加载YOLO模型"""
        try:
            # 如果是预训练模型名称，会自动下载
            self.model = YOLO(self.model_name)
            print(f"✓ 模型加载成功: {self.model_name}")
            
            # 打印模型信息
            if hasattr(self.model, 'info'):
                self.model.info()
                
        except Exception as e:
            raise RuntimeError(f"模型加载失败: {e}")
    
    def train(self,
              epochs=100,
              imgsz=640,
              batch=8,
              lr0=0.01,
              lrf=0.1,
              momentum=0.937,
              weight_decay=0.0005,
              warmup_epochs=3,
              warmup_momentum=0.8,
              warmup_bias_lr=0.1,
              box=7.5,
              cls=0.5,
              mask_ratio=4,
              degrees=0.0,
              translate=0.1,
              scale=0.5,
              shear=0.0,
              perspective=0.0,
              flipud=0.0,
              fliplr=0.5,
              mosaic=1.0,
              mixup=0.0,
              copy_paste=0.0,
              auto_augment="randaugment",
              patience=50,
              save_period=10,
              val=True,
              save=True,
              device=""):
        """
        开始训练
        
        Args:
            epochs: 训练轮数
            imgsz: 输入图像尺寸
            batch: 批次大小
            lr0: 初始学习率
            lrf: 最终学习率(lr0 * lrf)
            momentum: 动量
            weight_decay: 权重衰减
            warmup_epochs: 预热轮数
            warmup_momentum: 预热动量
            warmup_bias_lr: 预热偏置学习率
            box: box损失增益
            cls: cls损失增益  
            mask_ratio: mask损失比率
            degrees: 图像旋转角度
            translate: 图像平移
            scale: 图像缩放
            shear: 图像剪切
            perspective: 图像透视变换
            flipud: 图像上下翻转概率
            fliplr: 图像左右翻转概率
            mosaic: mosaic增强概率
            mixup: mixup增强概率
            copy_paste: copy paste增强概率
            auto_augment: 自动增强策略
            patience: 早停耐心值
            save_period: 模型保存间隔
            val: 是否进行验证
            save: 是否保存模型
            device: 设备选择
        """
        
        print("开始训练...")
        print("=" * 60)
        
        # 创建时间戳
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        experiment_name = f"{self.name}_{timestamp}"
        
        try:
            # 训练参数
            train_args = {
                'data': self.data_config,
                'epochs': epochs,
                'imgsz': imgsz,
                'batch': batch,
                'lr0': lr0,
                'lrf': lrf,
                'momentum': momentum,
                'weight_decay': weight_decay,
                'warmup_epochs': warmup_epochs,
                'warmup_momentum': warmup_momentum,
                'warmup_bias_lr': warmup_bias_lr,
                'box': box,
                'cls': cls,
                'mask_ratio': mask_ratio,
                'degrees': degrees,
                'translate': translate,
                'scale': scale,
                'shear': shear,
                'perspective': perspective,
                'flipud': flipud,
                'fliplr': fliplr,
                'mosaic': mosaic,
                'mixup': mixup,
                'copy_paste': copy_paste,
                'auto_augment': auto_augment,
                'patience': patience,
                'save_period': save_period,
                'val': val,
                'save': save,
                'project': self.project,
                'name': experiment_name,
                'exist_ok': True,
            }
            
            # 如果指定了设备
            if device:
                train_args['device'] = device
            
            # 打印训练配置
            print("训练配置:")
            for key, value in train_args.items():
                print(f"  {key}: {value}")
            print()
            
            # 开始训练
            if self.model is None:
                raise RuntimeError("模型未正确加载")
            results = self.model.train(**train_args)
            
            print("训练完成!")
            print(f"训练结果保存在: {os.path.join(self.project, experiment_name)}")
            
            return results
            
        except Exception as e:
            print(f"训练过程中出现错误: {e}")
            raise
    
    def validate(self, weights=None, data=None, imgsz=640, batch=16, device=""):
        """
        验证模型
        
        Args:
            weights: 模型权重路径
            data: 数据集配置
            imgsz: 验证图像尺寸
            batch: 验证批次大小
            device: 设备选择
        """
        print("开始验证...")
        
        try:
            # 如果指定了权重文件，加载新模型
            if weights:
                model = YOLO(weights)
            else:
                model = self.model
            
            # 验证参数
            val_args = {
                'data': data or self.data_config,
                'imgsz': imgsz,
                'batch': batch,
                'project': self.project,
                'name': 'val',
            }
            
            if device:
                val_args['device'] = device
            
            # 开始验证
            if model is None:
                raise RuntimeError("模型未正确加载")
            results = model.val(**val_args)
            
            print("验证完成!")
            return results
            
        except Exception as e:
            print(f"验证过程中出现错误: {e}")
            raise
    
    def predict_samples(self, weights=None, source="../data/yolo_dataset/images", 
                       imgsz=640, conf=0.25, iou=0.7, max_det=1000,
                       save=True, save_txt=True, save_conf=True):
        """
        对样本进行预测
        
        Args:
            weights: 模型权重路径
            source: 预测源(图像目录或单张图像)
            imgsz: 预测图像尺寸
            conf: 置信度阈值
            iou: NMS IoU阈值
            max_det: 最大检测数量
            save: 是否保存预测结果图像
            save_txt: 是否保存预测结果文本
            save_conf: 是否保存置信度
        """
        print("开始预测...")
        
        try:
            # 如果指定了权重文件，加载新模型
            if weights:
                model = YOLO(weights)
            else:
                model = self.model
            
            # 预测参数
            predict_args = {
                'source': source,
                'imgsz': imgsz,
                'conf': conf,
                'iou': iou,
                'max_det': max_det,
                'save': save,
                'save_txt': save_txt,
                'save_conf': save_conf,
                'project': self.project,
                'name': 'predict',
            }
            
            # 开始预测
            if model is None:
                raise RuntimeError("模型未正确加载")
            results = model.predict(**predict_args)
            
            print("预测完成!")
            return results
            
        except Exception as e:
            print(f"预测过程中出现错误: {e}")
            raise


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description="YOLO Segmentation 训练脚本")
    
    # 基本参数
    parser.add_argument("--data", type=str, default="../data/yolo_dataset_auto/dataset.yaml",
                       help="数据集配置文件路径")
    parser.add_argument("--model", type=str, default="../models_cache/yolo11n-seg.pt",
                       help="预训练模型名称或路径")
    parser.add_argument("--project", type=str, default="./runs/multi-segment",
                       help="训练结果保存项目目录")
    parser.add_argument("--name", type=str, default="train",
                       help="训练实验名称")
    
    # 训练参数
    parser.add_argument("--epochs", type=int, default=20, help="训练轮数")
    parser.add_argument("--imgsz", type=int, default=640, help="输入图像尺寸")
    parser.add_argument("--batch", type=int, default=4, help="批次大小")
    parser.add_argument("--lr0", type=float, default=0.01, help="初始学习率")
    parser.add_argument("--device", type=str, default="cuda", help="设备选择")
    
    # 操作模式
    parser.add_argument("--mode", choices=["train", "val", "predict"], 
                       default="train", help="运行模式")
    parser.add_argument("--weights", type=str, help="模型权重路径(用于验证和预测)")
    parser.add_argument("--source", type=str, help="预测源路径")
    
    args = parser.parse_args()
    
    print("YOLO Segmentation 训练系统")
    print("=" * 60)
    
    # 创建训练器
    trainer = YOLOSegmentationTrainer(
        data_config=args.data,
        model=args.model,
        project=args.project,
        name=args.name
    )
    
    # 根据模式执行操作
    if args.mode == "train":
        trainer.train(
            epochs=args.epochs,
            imgsz=args.imgsz,
            batch=args.batch,
            lr0=args.lr0,
            device=args.device
        )
    elif args.mode == "val":
        trainer.validate(
            weights=args.weights,
            device=args.device
        )
    elif args.mode == "predict":
        trainer.predict_samples(
            weights=args.weights,
            source=args.source or "../data/yolo_dataset/images"
        )
    
    print("任务完成!")


if __name__ == "__main__":
    main()









