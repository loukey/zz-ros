#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO OBB + Direction 训练脚本
专门用于训练旋转边界框+角度检测模型
支持零件方向检测和显式角度预测
扩展格式：class + 8个OBB角点坐标 + 1个角度值
"""

import os
import sys
import yaml
import argparse
import cv2
import numpy as np
import torch
from pathlib import Path
import ultralytics
from ultralytics import YOLO
from datetime import datetime
import time
import json
from typing import Dict, List, Optional, Tuple


class YOLOOBBDirectionTrainer:
    """YOLO OBB + Direction训练器，支持角度预测"""
    
    def __init__(self, 
                 data_config: str = "../data/yolo_obb_dataset/dataset.yaml",
                 model: str = "configs/custom_obb_direction_model.yaml",
                 project: str = "./runs/obb_direction",
                 name: str = "train"):
        """
        初始化YOLO OBB + Direction训练器
        
        Args:
            data_config: 数据集配置文件路径（支持扩展格式：class + 8个OBB坐标 + 1个角度）
            model: 自定义模型配置文件或预训练模型路径
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
            print(f"GPU memory: {torch.cuda.get_device_properties(0).total_memory / 1024**3:.1f} GB")
    
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
        
        train_images_path = os.path.join(dataset_path, config['train'])
        train_labels_path = os.path.join(dataset_path, 'labels')
        
        if not os.path.exists(train_images_path):
            raise FileNotFoundError(f"训练图像目录不存在: {train_images_path}")
        
        if not os.path.exists(train_labels_path):
            raise FileNotFoundError(f"训练标签目录不存在: {train_labels_path}")
        
        # 检查是否有标签文件
        label_files = list(Path(train_labels_path).glob('*.txt'))
        if not label_files:
            raise FileNotFoundError(f"标签目录中没有找到标签文件: {train_labels_path}")
        
        # 验证扩展标签格式（class + 8个OBB坐标 + 1个角度）
        sample_label = label_files[0]
        with open(sample_label, 'r') as f:
            first_line = f.readline().strip()
            if first_line:
                parts = first_line.split()
                if len(parts) != 10:  # class_id + 8个坐标 + 1个角度
                    print(f"⚠️  警告: 标签格式可能不是扩展OBB格式")
                    print(f"    期望：class_id x1 y1 x2 y2 x3 y3 x4 y4 angle (10个值)")
                    print(f"    实际：{len(parts)}个值")
                else:
                    print(f"✓ 扩展OBB标签格式验证通过")
        
        print(f"✓ 数据集验证通过")
        print(f"  数据集路径: {dataset_path}")
        print(f"  图像目录: {train_images_path}")
        print(f"  标签目录: {train_labels_path}")
        print(f"  标签文件数量: {len(label_files)}")
        print(f"  类别数量: {config['nc']}")
        print(f"  类别名称: {config['names']}")
        print(f"  标签格式: 扩展OBB (class + 8坐标 + 1角度)")
    
    def load_model(self):
        """加载YOLO OBB + Direction模型"""
        try:
            # 加载自定义OBB+Direction模型
            self.model = YOLO(self.model_name)
            print(f"✓ YOLO OBB + Direction模型加载成功: {self.model_name}")
            
            # 检查是否为自定义配置文件
            if self.model_name.endswith('.yaml'):
                print(f"  使用自定义模型配置")
                print(f"  支持OBB角点 + 角度预测")
            
            # 打印模型信息
            if hasattr(self.model, 'info'):
                self.model.info()
            
            # 确认模型任务类型
            if hasattr(self.model, 'task'):
                print(f"  模型任务类型: {self.model.task}")
            
        except Exception as e:
            raise RuntimeError(f"模型加载失败: {e}")
    
    def train(self,
              epochs: int = 100,
              imgsz: int = 640,
              batch: int = 8,
              lr0: float = 0.01,
              lrf: float = 0.1,
              momentum: float = 0.937,
              weight_decay: float = 0.0005,
              warmup_epochs: int = 3,
              warmup_momentum: float = 0.8,
              warmup_bias_lr: float = 0.1,
              box: float = 7.5,
              cls: float = 0.5,
              direction: float = 2.0,
              degrees: float = 0.0,
              translate: float = 0.1,
              scale: float = 0.5,
              shear: float = 0.0,
              perspective: float = 0.0,
              flipud: float = 0.0,
              fliplr: float = 0.5,
              mosaic: float = 1.0,
              mixup: float = 0.0,
              copy_paste: float = 0.0,
              patience: int = 50,
              save_period: int = 10,
              val: bool = True,
              save: bool = True,
              device: str = ""):
        """
        开始YOLO OBB训练
        
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
            direction: 角度损失增益
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
            patience: 早停耐心值
            save_period: 模型保存间隔
            val: 是否进行验证
            save: 是否保存模型
            device: 设备选择
        """
        
        print("开始YOLO OBB + Direction训练...")
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
                'direction': direction,
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
                'patience': patience,
                'save_period': save_period,
                'val': val,
                'save': save,
                'project': self.project,
                'name': experiment_name,
                'exist_ok': True,
                'task': 'obb',  # 指定OBB任务
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
                
            print("🚀 开始训练YOLO OBB + Direction模型...")
            results = self.model.train(**train_args)
            
            print("✅ 训练完成!")
            
            # 保存训练结果路径
            self.last_train_dir = os.path.join(self.project, experiment_name)
            print(f"训练结果保存在: {self.last_train_dir}")
            
            # 打印最佳权重路径
            best_weights = os.path.join(self.last_train_dir, 'weights', 'best.pt')
            if os.path.exists(best_weights):
                print(f"最佳权重: {best_weights}")
            
            return results
            
        except Exception as e:
            print(f"训练过程中出现错误: {e}")
            raise
    
    def validate(self, 
                 weights: Optional[str] = None, 
                 data: Optional[str] = None, 
                 imgsz: int = 640, 
                 batch: int = 16, 
                 device: str = ""):
        """
        验证YOLO OBB + Direction模型
        
        Args:
            weights: 模型权重路径
            data: 数据集配置
            imgsz: 验证图像尺寸
            batch: 验证批次大小
            device: 设备选择
        """
        print("开始验证YOLO OBB模型...")
        
        try:
            # 如果指定了权重文件，加载新模型
            if weights:
                model = YOLO(weights)
                print(f"加载权重: {weights}")
            else:
                model = self.model
            
            # 验证参数
            val_args = {
                'data': data or self.data_config,
                'imgsz': imgsz,
                'batch': batch,
                'project': self.project,
                'name': 'val',
                'task': 'obb',  # 指定OBB任务
            }
            
            if device:
                val_args['device'] = device
            
            # 开始验证
            if model is None:
                raise RuntimeError("模型未正确加载")
                
            print("🔍 开始验证...")
            results = model.val(**val_args)
            
            print("✅ 验证完成!")
            
            # 打印验证结果
            if hasattr(results, 'results_dict'):
                metrics = results.results_dict
                print("\n📊 验证结果:")
                for key, value in metrics.items():
                    if isinstance(value, (int, float)):
                        print(f"  {key}: {value:.4f}")
            
            return results
            
        except Exception as e:
            print(f"验证过程中出现错误: {e}")
            raise
    
    def predict(self, 
                source: str,
                weights: Optional[str] = None,
                imgsz: int = 640,
                conf: float = 0.25,
                iou: float = 0.7,
                max_det: int = 1000,
                save: bool = True,
                save_txt: bool = True,
                save_conf: bool = True,
                visualize: bool = False):
        """
        使用YOLO OBB + Direction模型进行预测
        
        Args:
            source: 预测源(图像路径或目录)
            weights: 模型权重路径
            imgsz: 预测图像尺寸
            conf: 置信度阈值
            iou: NMS IoU阈值
            max_det: 最大检测数量
            save: 是否保存预测结果图像
            save_txt: 是否保存预测结果文本
            save_conf: 是否保存置信度
            visualize: 是否可视化预测过程
        """
        print(f"开始YOLO OBB预测: {source}")
        
        try:
            # 如果指定了权重文件，加载新模型
            if weights:
                model = YOLO(weights)
                print(f"加载权重: {weights}")
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
                'task': 'obb',  # 指定OBB任务
            }
            
            if visualize:
                predict_args['visualize'] = True
            
            # 开始预测
            if model is None:
                raise RuntimeError("模型未正确加载")
                
            print("🔮 开始预测...")
            results = model.predict(**predict_args)
            
            print("✅ 预测完成!")
            
            # 处理预测结果
            predictions = self.process_predictions(results)
            
            return results, predictions
            
        except Exception as e:
            print(f"预测过程中出现错误: {e}")
            raise
    
    def process_predictions(self, results) -> List[Dict]:
        """
        处理YOLO OBB + Direction预测结果，提取OBB信息和显式角度
        
        Args:
            results: YOLO预测结果
            
        Returns:
            List[Dict]: 处理后的预测结果
        """
        predictions = []
        
        for i, result in enumerate(results):
            pred_info = {
                'image_index': i,
                'objects': []
            }
            
            # 处理OBB结果
            if hasattr(result, 'obb') and result.obb is not None:
                obb_data = result.obb
                
                # 获取边界框信息
                if hasattr(obb_data, 'xywhr'):
                    xywhr = obb_data.xywhr.cpu().numpy()
                    
                    for j, box in enumerate(xywhr):
                        cx, cy, w, h, rotation = box
                        
                        # 获取置信度和类别
                        conf = float(obb_data.conf[j]) if hasattr(obb_data, 'conf') else 0.0
                        cls = int(obb_data.cls[j]) if hasattr(obb_data, 'cls') else 0
                        
                        # 角度转换
                        angle_degrees = np.degrees(rotation)
                        
                        # 检查是否有显式角度预测（扩展格式）
                        predicted_angle = None
                        if hasattr(obb_data, 'angle') and obb_data.angle is not None:
                            # 如果模型输出了显式角度，使用它
                            predicted_angle = float(obb_data.angle[j])
                        elif hasattr(obb_data, 'data') and obb_data.data.shape[1] > 10:
                            # 从原始数据中提取角度（扩展格式：8角点+置信度+类别+角度）
                            predicted_angle = float(obb_data.data[j, 10])
                        
                        obj_info = {
                            'center_x': float(cx),
                            'center_y': float(cy),
                            'width': float(w),
                            'height': float(h),
                            'rotation_rad': float(rotation),
                            'rotation_deg': float(angle_degrees),
                            'confidence': conf,
                            'class': cls
                        }
                        
                        # 添加显式角度信息
                        if predicted_angle is not None:
                            obj_info['predicted_angle'] = predicted_angle
                            obj_info['main_angle'] = predicted_angle  # 主要使用预测角度
                        else:
                            obj_info['main_angle'] = angle_degrees  # 回退到几何角度
                        
                        pred_info['objects'].append(obj_info)
                        
                        # 打印信息
                        angle_info = f"预测角度{predicted_angle:.1f}°" if predicted_angle is not None else f"几何角度{angle_degrees:.1f}°"
                        print(f"    检测到对象: 中心({cx:.1f}, {cy:.1f}), "
                              f"尺寸({w:.1f}x{h:.1f}), {angle_info}, "
                              f"置信度{conf:.3f}")
            
            predictions.append(pred_info)
            
            if pred_info['objects']:
                print(f"  图像 {i+1}: 检测到 {len(pred_info['objects'])} 个对象")
            else:
                print(f"  图像 {i+1}: 未检测到对象")
        
        return predictions
    
    def export_model(self, 
                     weights: Optional[str] = None,
                     format: str = 'onnx',
                     imgsz: int = 640,
                     half: bool = False,
                     int8: bool = False,
                     optimize: bool = False):
        """
        导出模型到不同格式
        
        Args:
            weights: 模型权重路径
            format: 导出格式 ('onnx', 'torchscript', 'tflite', 'coreml', 'tensorrt')
            imgsz: 导出图像尺寸
            half: 是否使用半精度
            int8: 是否使用INT8量化
            optimize: 是否优化模型
        """
        print(f"开始导出模型到 {format} 格式...")
        
        try:
            # 如果指定了权重文件，加载新模型
            if weights:
                model = YOLO(weights)
                print(f"加载权重: {weights}")
            else:
                model = self.model
            
            # 导出参数
            export_args = {
                'format': format,
                'imgsz': imgsz,
                'half': half,
                'int8': int8,
                'optimize': optimize,
            }
            
            # 开始导出
            if model is None:
                raise RuntimeError("模型未正确加载")
                
            exported_model = model.export(**export_args)
            
            print(f"✅ 模型导出完成: {exported_model}")
            
            return exported_model
            
        except Exception as e:
            print(f"模型导出过程中出现错误: {e}")
            raise
    
    def create_training_summary(self, results_dir: str):
        """
        创建训练总结报告
        
        Args:
            results_dir: 训练结果目录
        """
        summary_path = os.path.join(results_dir, 'training_summary.json')
        
        # 收集训练信息
        summary = {
            'timestamp': datetime.now().isoformat(),
            'model': self.model_name,
            'data_config': self.data_config,
            'project': self.project,
            'name': self.name,
            'task': 'obb',
            'ultralytics_version': ultralytics.__version__,
            'pytorch_version': torch.__version__,
            'cuda_available': torch.cuda.is_available(),
        }
        
        # 检查训练结果文件
        results_csv = os.path.join(results_dir, 'results.csv')
        if os.path.exists(results_csv):
            summary['results_file'] = results_csv
        
        # 检查权重文件
        weights_dir = os.path.join(results_dir, 'weights')
        if os.path.exists(weights_dir):
            best_weights = os.path.join(weights_dir, 'best.pt')
            last_weights = os.path.join(weights_dir, 'last.pt')
            
            if os.path.exists(best_weights):
                summary['best_weights'] = best_weights
            if os.path.exists(last_weights):
                summary['last_weights'] = last_weights
        
        # 保存总结
        with open(summary_path, 'w', encoding='utf-8') as f:
            json.dump(summary, f, indent=2, ensure_ascii=False)
        
        print(f"训练总结已保存: {summary_path}")


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description="YOLO OBB + Direction 训练脚本")
    
    # 基本参数
    parser.add_argument("--data", type=str, default="../data/yolo_obb_dataset/dataset.yaml",
                       help="数据集配置文件路径（支持扩展格式：class+8坐标+1角度）")
    parser.add_argument("--model", type=str, default="configs/custom_obb_direction_model.yaml",
                       help="自定义模型配置文件或预训练模型路径")
    parser.add_argument("--project", type=str, default="./runs/obb_direction",
                       help="训练结果保存项目目录")
    parser.add_argument("--name", type=str, default="train",
                       help="训练实验名称")
    
    # 训练参数
    parser.add_argument("--epochs", type=int, default=100, help="训练轮数")
    parser.add_argument("--imgsz", type=int, default=640, help="输入图像尺寸")
    parser.add_argument("--batch", type=int, default=8, help="批次大小")
    parser.add_argument("--lr0", type=float, default=0.01, help="初始学习率")
    parser.add_argument("--device", type=str, default="", help="设备选择")
    
    # 损失参数
    parser.add_argument("--box", type=float, default=7.5, help="box损失增益")
    parser.add_argument("--cls", type=float, default=0.5, help="cls损失增益")
    parser.add_argument("--direction", type=float, default=2.0, help="角度损失增益")
    
    # 操作模式
    parser.add_argument("--mode", choices=["train", "val", "predict", "export"], 
                       default="train", help="运行模式")
    parser.add_argument("--weights", type=str, help="模型权重路径(用于验证、预测和导出)")
    parser.add_argument("--source", type=str, help="预测源路径")
    parser.add_argument("--format", type=str, default="onnx", help="导出格式")
    
    # 预测参数
    parser.add_argument("--conf", type=float, default=0.25, help="置信度阈值")
    parser.add_argument("--iou", type=float, default=0.7, help="NMS IoU阈值")
    
    args = parser.parse_args()
    
    print("YOLO OBB + Direction 训练系统")
    print("支持角度预测的旋转边界框检测")
    print("=" * 60)
    
    # 创建训练器
    trainer = YOLOOBBDirectionTrainer(
        data_config=args.data,
        model=args.model,
        project=args.project,
        name=args.name
    )
    
    # 根据模式执行操作
    if args.mode == "train":
        print("🚀 开始训练模式...")
        results = trainer.train(
            epochs=args.epochs,
            imgsz=args.imgsz,
            batch=args.batch,
            lr0=args.lr0,
            box=args.box,
            cls=args.cls,
            direction=args.direction,
            device=args.device
        )
        
        # 创建训练总结
        if hasattr(trainer, 'last_train_dir'):
            trainer.create_training_summary(trainer.last_train_dir)
        
    elif args.mode == "val":
        print("🔍 开始验证模式...")
        trainer.validate(
            weights=args.weights,
            device=args.device
        )
        
    elif args.mode == "predict":
        if not args.source:
            print("❌ 预测模式需要指定 --source 参数")
            return
        
        print("🔮 开始预测模式...")
        results, predictions = trainer.predict(
            source=args.source,
            weights=args.weights,
            conf=args.conf,
            iou=args.iou
        )
        
    elif args.mode == "export":
        print("📦 开始导出模式...")
        trainer.export_model(
            weights=args.weights,
            format=args.format
        )
    
    print("\n🎉 任务完成!")


if __name__ == "__main__":
    main()
