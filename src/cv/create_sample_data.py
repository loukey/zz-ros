#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
创建示例数据脚本
帮助用户快速准备测试数据来体验OBB标注工具
"""

import os
import cv2
import numpy as np
from pathlib import Path
import random

def create_sample_images(output_dir="data/sample_images", num_images=5):
    """创建示例图像用于测试标注工具"""
    
    # 创建输出目录
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    
    print(f"📁 创建示例图像目录: {output_path}")
    
    # 图像尺寸
    img_width, img_height = 800, 600
    
    # 创建示例图像
    for i in range(num_images):
        # 创建背景
        img = np.ones((img_height, img_width, 3), dtype=np.uint8) * 240
        
        # 添加一些噪声
        noise = np.random.randint(0, 30, (img_height, img_width, 3), dtype=np.uint8)
        img = cv2.add(img, noise)
        
        # 添加随机几何形状作为标注目标
        num_objects = random.randint(2, 5)
        
        for j in range(num_objects):
            # 随机颜色
            color = (random.randint(50, 200), random.randint(50, 200), random.randint(50, 200))
            
            # 随机位置和大小
            center_x = random.randint(100, img_width - 100)
            center_y = random.randint(100, img_height - 100)
            width = random.randint(50, 150)
            height = random.randint(30, 100)
            angle = random.randint(0, 180)
            
            # 创建旋转矩形
            rect = ((center_x, center_y), (width, height), angle)
            box = cv2.boxPoints(rect)
            box = np.array(box, dtype=np.int32)
            
            # 绘制填充的旋转矩形
            cv2.fillPoly(img, [box], color)
            
            # 添加边框
            cv2.polylines(img, [box], True, (0, 0, 0), 2)
            
            # 添加一些内部细节
            if random.random() > 0.5:
                # 添加圆形
                circle_radius = random.randint(5, 15)
                cv2.circle(img, (center_x, center_y), circle_radius, (255, 255, 255), -1)
            
            if random.random() > 0.5:
                # 添加线条
                pt1 = (center_x - width//4, center_y)
                pt2 = (center_x + width//4, center_y)
                cv2.line(img, pt1, pt2, (0, 0, 0), 2)
        
        # 添加网格背景
        grid_size = 50
        for x in range(0, img_width, grid_size):
            cv2.line(img, (x, 0), (x, img_height), (220, 220, 220), 1)
        for y in range(0, img_height, grid_size):
            cv2.line(img, (0, y), (img_width, y), (220, 220, 220), 1)
        
        # 添加图像标题
        title = f"Sample Image {i+1}"
        cv2.putText(img, title, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
        
        # 保存图像
        img_path = output_path / f"sample_{i+1:03d}.jpg"
        cv2.imwrite(str(img_path), img)
        print(f"✅ 创建示例图像: {img_path.name}")
    
    print(f"\n🎉 成功创建 {num_images} 张示例图像！")
    print(f"📂 图像保存在: {output_path}")
    return output_path

def create_readme(output_dir):
    """创建README文件"""
    readme_path = Path(output_dir) / "README.md"
    
    readme_content = """# 示例图像说明

## 📝 关于这些图像

这些是自动生成的示例图像，用于测试OBB标注工具的功能。

每张图像包含：
- 2-5个随机生成的旋转矩形对象
- 不同的颜色、大小和角度
- 网格背景便于定位
- 一些装饰性元素（圆形、线条等）

## 🎯 如何使用

1. 启动OBB标注工具：
   ```bash
   python start_annotation.py
   ```

2. 点击"加载图像目录"按钮

3. 选择这个示例图像目录

4. 开始标注：
   - 在每个旋转矩形上按顺序点击4个顶点
   - 使用键盘方向键切换图像
   - 标注会自动保存到 `labels/` 目录

## 📁 输出结构

标注完成后，目录结构如下：
```
sample_images/
├── sample_001.jpg
├── sample_002.jpg
├── ...
└── labels/
    ├── sample_001.txt
    ├── sample_002.txt
    ├── ...
    └── classes.txt
```

## 💡 标注建议

- 按顺时针或逆时针顺序点击4个顶点
- 尽量贴合对象边界
- 保持标注的一致性
- 使用缩放功能提高精度

祝您标注愉快！🎉
"""
    
    with open(readme_path, 'w', encoding='utf-8') as f:
        f.write(readme_content)
    
    print(f"📄 创建说明文件: {readme_path}")

def main():
    """主函数"""
    print("🎨 OBB标注工具 - 示例数据生成器")
    print("=" * 50)
    
    try:
        # 询问用户
        num_images = input("请输入要生成的示例图像数量 (默认5张): ").strip()
        if not num_images:
            num_images = 5
        else:
            num_images = int(num_images)
        
        output_dir = input("请输入输出目录 (默认: data/sample_images): ").strip()
        if not output_dir:
            output_dir = "data/sample_images"
        
        print(f"\n📊 配置信息:")
        print(f"   图像数量: {num_images}")
        print(f"   输出目录: {output_dir}")
        print()
        
        # 创建示例图像
        output_path = create_sample_images(output_dir, num_images)
        
        # 创建README
        create_readme(output_path)
        
        print("\n🎉 示例数据创建完成！")
        print(f"📂 请在OBB标注工具中加载目录: {output_path}")
        print("\n📝 使用步骤:")
        print("1. 运行: python start_annotation.py")
        print("2. 点击'加载图像目录'")
        print(f"3. 选择目录: {output_path}")
        print("4. 开始标注测试")
        
    except ValueError:
        print("❌ 输入的图像数量无效，请输入数字")
    except KeyboardInterrupt:
        print("\n\n⚠️  操作被用户取消")
    except Exception as e:
        print(f"❌ 创建示例数据失败: {e}")

if __name__ == "__main__":
    main() 