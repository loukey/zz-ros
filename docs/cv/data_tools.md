# 数据工具链

高质量的数据是模型精度的基石。我们提供了一套完整的工具链，用于从 LabelMe 标注生成 YOLO 训练数据。

## 数据生成器

**脚本**: `src/cv/data_tools/generate_yolo_obb_data.py`

该工具负责将少量的原始标注数据扩充为大规模的训练集。

### 主要功能

1.  **格式转换**:
    - 输入：LabelMe JSON（多边形标注）。
    - 输出：YOLO OBB 格式 (`class_id x1 y1 x2 y2 x3 y3 x4 y4`)。
    - 算法：自动计算多边形的最小外接旋转矩形。

2.  **数据增强 Pipeline**:
    - **几何变换**: 随机旋转（+/- 30度）、水平/垂直翻转。关键在于不仅旋转图像，还要精确旋转标注坐标。
    - **像素增强**: 随机亮度/对比度调整、高斯噪声、模糊处理。

3.  **自动数据集构建**:
    - 依据设定的 `target_count`（如 500 张），循环生成增强数据。
    - 自动按比例（默认 7:2:1）划分 `train` / `val` / `test` 集。
    - 自动生成 `dataset.yaml` 配置文件。

### 使用方法

```bash
python src/cv/data_tools/generate_yolo_obb_data.py \
    --annotations ./data/annotations \
    --output ./data/datasets/yolo_obb \
    --target_count 1000
```

