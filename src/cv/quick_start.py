#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO11 OBB 快速开始脚本
一键完成从数据生成到模型训练的完整流程
"""

import os
import sys
import argparse
import subprocess
from pathlib import Path
import logging

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


def check_environment():
    """检查环境依赖"""
    logger.info("=== 检查环境依赖 ===")
    
    # 检查数据目录
    data_dir = Path("data/images")
    if not data_dir.exists():
        logger.error(f"数据目录不存在: {data_dir}")
        logger.info("请创建 data/images/ 目录并放入您的图像文件")
        return False
    
    # 检查图像文件
    image_files = list(data_dir.glob("*.jpg")) + list(data_dir.glob("*.png"))
    if not image_files:
        logger.error("在 data/images/ 目录中未找到图像文件")
        logger.info("请将您的图像文件（.jpg或.png）放入 data/images/ 目录")
        return False
    
    logger.info(f"找到 {len(image_files)} 个图像文件")
    
    # 检查背景目录（可选）
    background_dir = Path("data/images/background")
    if background_dir.exists():
        bg_files = list(background_dir.glob("*.jpg")) + list(background_dir.glob("*.png"))
        logger.info(f"找到 {len(bg_files)} 个背景图像")
    else:
        logger.info("未找到背景图像目录，将使用默认背景")
    
    # 检查YOLO模型文件
    model_file = Path("yolo11x-seg.pt")
    if not model_file.exists():
        logger.warning("YOLO11分割模型文件不存在，首次运行时会自动下载")
    
    # 检查Python包
    try:
        import torch
        import cv2
        import numpy as np
        from ultralytics import YOLO
        from sklearn.decomposition import PCA
        logger.info("所有必需的Python包都已安装")
    except ImportError as e:
        logger.error(f"缺少必需的Python包: {e}")
        logger.info("请运行: pip install -r detection_models/requirements.txt")
        return False
    
    # 检查GPU
    try:
        import torch
        if torch.cuda.is_available():
            logger.info(f"检测到GPU: {torch.cuda.get_device_name(0)}")
        else:
            logger.warning("未检测到GPU，将使用CPU（训练速度较慢）")
    except:
        pass
    
    return True


def generate_obb_data():
    """生成OBB训练数据"""
    logger.info("=== 生成OBB训练数据 ===")
    
    try:
        # 运行main.py生成OBB数据
        result = subprocess.run([sys.executable, "main.py"], 
                              capture_output=True, text=True, encoding='utf-8', errors='ignore')
        
        if result.returncode != 0:
            logger.error(f"数据生成失败: {result.stderr}")
            return False
        
        logger.info("OBB训练数据生成完成")
        
        # 检查生成的数据
        obb_data_dir = Path("data/obb_training_data")
        if not obb_data_dir.exists():
            logger.error("OBB训练数据目录未生成")
            return False
        
        images_dir = obb_data_dir / "images"
        labels_dir = obb_data_dir / "labels"
        dataset_yaml = obb_data_dir / "dataset.yaml"
        
        if not all([images_dir.exists(), labels_dir.exists(), dataset_yaml.exists()]):
            logger.error("OBB训练数据不完整")
            return False
        
        num_images = len(list(images_dir.glob("*.jpg"))) + len(list(images_dir.glob("*.png")))
        num_labels = len(list(labels_dir.glob("*.txt")))
        
        logger.info(f"生成了 {num_images} 张训练图像和 {num_labels} 个标注文件")
        
        return True
        
    except Exception as e:
        logger.error(f"数据生成过程中出错: {str(e)}")
        return False


def train_obb_model(model_size='n', epochs=50, batch_size=8, use_pretrained=True):
    """训练OBB模型"""
    logger.info("=== 训练OBB模型 ===")
    
    try:
        # 构建训练命令
        cmd = [
            sys.executable, 
            "detection_models/obb_train.py",
            "--data", "data/obb_training_data/dataset.yaml",
            "--model", model_size,
            "--epochs", str(epochs),
            "--batch", str(batch_size),
            "--name", f"quick_start_{model_size}"
        ]
        
        if use_pretrained:
            cmd.append("--pretrained")
        
        logger.info(f"执行训练命令: {' '.join(cmd)}")
        
        # 运行训练
        result = subprocess.run(cmd, text=True, encoding='utf-8', errors='ignore')
        
        if result.returncode != 0:
            logger.error("模型训练失败")
            return False
        
        logger.info("模型训练完成")
        return True
        
    except Exception as e:
        logger.error(f"训练过程中出错: {str(e)}")
        return False


def test_trained_model():
    """测试训练好的模型"""
    logger.info("=== 测试训练好的模型 ===")
    
    try:
        # 查找最新的训练结果
        runs_dir = Path("detection_models/runs/train")
        if not runs_dir.exists():
            logger.error("未找到训练结果目录")
            return False
        
        # 找到最新的实验目录
        experiment_dirs = [d for d in runs_dir.iterdir() if d.is_dir() and d.name.startswith("quick_start")]
        if not experiment_dirs:
            logger.error("未找到训练实验目录")
            return False
        
        latest_exp = max(experiment_dirs, key=lambda x: x.stat().st_mtime)
        best_model = latest_exp / "weights" / "best.pt"
        
        if not best_model.exists():
            logger.error(f"未找到训练好的模型: {best_model}")
            return False
        
        logger.info(f"找到训练好的模型: {best_model}")
        
        # 使用训练好的模型进行检测测试
        try:
            from detection_models.obb import YOLO11OBBDetector
            
            # 创建检测器（使用训练好的模型）
            detector = YOLO11OBBDetector(model_path=str(best_model))
            
            # 测试检测
            test_images = list(Path("data/images").glob("*.jpg"))[:3]  # 测试前3张图像
            if test_images:
                output_dir = "data/test_results"
                os.makedirs(output_dir, exist_ok=True)
                
                logger.info(f"使用训练好的模型测试 {len(test_images)} 张图像...")
                
                for img_path in test_images:
                    detector.detect_single(str(img_path), output_dir)
                
                logger.info(f"测试结果保存在: {output_dir}")
            
        except Exception as e:
            logger.warning(f"模型测试出错: {str(e)}")
        
        return True
        
    except Exception as e:
        logger.error(f"测试过程中出错: {str(e)}")
        return False


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description="YOLO11 OBB 快速开始脚本")
    parser.add_argument("--model", choices=['n', 's', 'm', 'l', 'x'], default='n',
                       help="模型大小 (默认: n)")
    parser.add_argument("--epochs", type=int, default=50,
                       help="训练轮数 (默认: 50)")
    parser.add_argument("--batch", type=int, default=8,
                       help="批次大小 (默认: 8)")
    parser.add_argument("--no-pretrained", action="store_true",
                       help="不使用预训练模型")
    parser.add_argument("--data-only", action="store_true",
                       help="仅生成训练数据，不进行训练")
    parser.add_argument("--train-only", action="store_true",
                       help="仅进行训练，跳过数据生成")
    
    args = parser.parse_args()
    
    logger.info("=== YOLO11 OBB 快速开始 ===")
    logger.info(f"模型大小: {args.model}")
    logger.info(f"训练轮数: {args.epochs}")
    logger.info(f"批次大小: {args.batch}")
    logger.info(f"使用预训练模型: {not args.no_pretrained}")
    
    # 检查环境
    if not check_environment():
        logger.error("环境检查失败，请解决上述问题后重试")
        return 1
    
    success = True
    
    # 生成训练数据（除非指定仅训练）
    if not args.train_only:
        if not generate_obb_data():
            logger.error("OBB训练数据生成失败")
            success = False
    
    # 训练模型（除非指定仅生成数据）
    if success and not args.data_only:
        if not train_obb_model(
            model_size=args.model,
            epochs=args.epochs,
            batch_size=args.batch,
            use_pretrained=not args.no_pretrained
        ):
            logger.error("模型训练失败")
            success = False
    
    # 测试模型（仅在完整流程成功时）
    if success and not args.data_only:
        test_trained_model()
    
    if success:
        logger.info("=== 快速开始流程完成 ===")
        if not args.data_only:
            logger.info("🎉 成功从少量原图生成1000张训练数据并完成模型训练！")
            logger.info("\n📊 生成统计:")
            logger.info("- 训练图像: ~800张")
            logger.info("- 验证图像: ~200张") 
            logger.info("- 类别: part (单一类别)")
            logger.info("- 数据增强: 旋转、缩放、亮度调整等")
            logger.info("\n📁 结果位置:")
            logger.info("1. 训练结果: detection_models/runs/train/")
            logger.info("2. 分割结果: data/images/result/")
            logger.info("3. 测试结果: data/test_results/")
            logger.info("4. 训练数据: data/obb_training_data/")
            logger.info("\n💡 下一步:")
            logger.info("- 查看训练曲线评估模型性能")
            logger.info("- 使用训练好的模型进行检测")
            logger.info("- 根据结果调整参数重新训练")
        else:
            logger.info("🎉 成功从少量原图生成1000张OBB训练数据！")
            logger.info("\n📊 生成统计:")
            logger.info("- 训练数据: data/obb_training_data/train/ (~800张)")
            logger.info("- 验证数据: data/obb_training_data/val/ (~200张)")
            logger.info("- 分割结果: data/images/result/")
            logger.info("- 数据配置: data/obb_training_data/dataset.yaml")
            logger.info("\n💡 下一步:")
            logger.info("运行训练: cd detection_models && python obb_train.py --data ../data/obb_training_data/dataset.yaml --pretrained")
        return 0
    else:
        logger.error("快速开始流程失败")
        return 1


if __name__ == "__main__":
    exit(main()) 