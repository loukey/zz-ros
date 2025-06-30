# YOLO11 OBB检测模块

基于ultralytics的YOLO11 OBB (Oriented Bounding Box) 检测模块，用于检测带有旋转角度的边界框。

## 功能特点

- 🔍 **OBB检测**: 支持旋转边界框检测，适用于倾斜目标
- 📸 **批量处理**: 支持批量检测多张图像
- 💾 **结果保存**: 自动保存带注释的图像和JSON格式的检测结果
- 🎯 **高精度**: 基于YOLO11最新模型架构
- 📊 **详细日志**: 完整的检测过程日志记录
- 🚀 **完整训练流程**: 从语义分割到OBB模型训练的完整解决方案

## 🚀 快速开始

### 一键运行完整流程

```bash
# 1. 将图像文件放入 ../data/images/ 目录
# 2. 运行快速开始脚本
cd ..
python quick_start.py --model n --epochs 50 --pretrained
```

这将自动完成：
- 语义分割和OBB数据生成
- 模型训练 
- 模型测试

### 快速开始选项

```bash
# 仅生成训练数据
python quick_start.py --data-only

# 使用更大的模型训练更多轮次
python quick_start.py --model m --epochs 100 --batch 16

# 不使用预训练模型
python quick_start.py --no-pretrained
```

## 安装依赖

```bash
# 安装必要的依赖包
pip install -r requirements.txt
```

## 使用方法

### 1. 基本使用

将要检测的图像放在 `../data/images/` 目录下，然后运行：

```bash
python obb.py
```

检测结果将保存在 `../data/results/` 目录下。

### 2. 使用示例脚本

```bash
python example_usage.py
```

### 3. 编程调用

```python
from obb import YOLO11OBBDetector

# 创建检测器
detector = YOLO11OBBDetector(
    confidence=0.5,      # 置信度阈值
    iou_threshold=0.45   # IoU阈值
)

# 批量检测
detector.detect_batch("input_dir", "output_dir")

# 单图检测
detector.detect_single("image_path", "output_dir")
```

## 输出格式

每张图像会生成两个文件：

### 1. 带注释的图像
- 文件名: `{原图名}_detected.jpg`
- 内容: 在原图上绘制OBB边界框

### 2. JSON结果文件
- 文件名: `{原图名}_results.json`
- 内容: 检测结果的详细信息

```json
{
  "image_path": "图像路径",
  "image_name": "图像名称",
  "detection_time": "检测时间",
  "model_confidence": 0.5,
  "total_detections": 2,
  "detections": [
    {
      "id": 0,
      "class_id": 0,
      "class_name": "person",
      "confidence": 0.85,
      "obb_points": [x1, y1, x2, y2, x3, y3, x4, y4]
    }
  ]
}
```

## 参数说明

- `confidence`: 置信度阈值 (0-1)，默认0.5
- `iou_threshold`: IoU阈值 (0-1)，默认0.45
- `model_path`: 自定义模型路径，默认使用预训练模型

## 支持的图像格式

- JPEG (.jpg, .jpeg)
- PNG (.png)
- BMP (.bmp)
- TIFF (.tiff, .tif)
- WebP (.webp)

## 模型信息

- 默认使用: `yolo11n-obb.pt` (YOLO11 Nano OBB模型)
- 支持自定义模型: 可以加载自己训练的OBB模型

## 目录结构

```
detection_models/
├── obb.py              # 主检测模块
├── example_usage.py    # 使用示例
├── requirements.txt    # 依赖包列表
├── README.md          # 说明文档
└── ../data/           # 数据目录
    ├── images/        # 输入图像
    └── results/       # 检测结果
```

## 注意事项

1. 首次运行时会自动下载YOLO11模型文件
2. 确保有足够的磁盘空间存储检测结果
3. 建议使用GPU加速以提高检测速度
4. 大批量图像处理时注意内存使用情况

## 错误处理

脚本包含完整的错误处理机制：
- 自动跳过无法读取的图像
- 检测失败时记录错误日志
- 自动创建输出目录

## 性能优化

- 使用批量处理提高效率
- 支持GPU加速
- 自动内存管理 