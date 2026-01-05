# 快速开始

本文档介绍如何快速运行 CV 模块进行推理测试。

## 环境准备

1. 确保已安装依赖：
   ```bash
   pip install ultralytics opencv-python numpy
   ```
2. 确保模型权重文件 `best.pt` 已放置在 `src/cv/weights/` 目录下。

## 运行推理

我们提供了一个统一的入口脚本 `main.py`。

1. **准备数据**:
   将待测试的图片放入 `src/cv/data/raw/` 目录。

2. **执行命令**:
   ```bash
   python src/cv/main.py
   ```

3. **查看结果**:
   处理完成后的图片将保存在 `src/cv/data/outputs/` 目录中。
   - 包含绘制好的 OBB 框。
   - 包含方向箭头和角度标注。

## 代码示例

如果你想在自己的代码中调用检测器：

```python
from src.cv.core.detectors.obb import YOLOOBBDetector

# 初始化
detector = YOLOOBBDetector(model_path='path/to/model.pt')

# 推理
result = detector.detect_image('path/to/image.jpg')

# 获取姿态信息
if result['pose_result']:
    angle = result['pose_result'].final_angle
    print(f"Robot Angle: {angle}")
```

