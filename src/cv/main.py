import os
import cv2
import numpy as np
import torch
from ultralytics import YOLO
from sklearn.decomposition import PCA
import time
import shutil
from pathlib import Path
import random
import json
from typing import List, Tuple, Optional

# 可选依赖：数据增强
try:
    import albumentations as A
    ALBUMENTATIONS_AVAILABLE = True
except ImportError:
    ALBUMENTATIONS_AVAILABLE = False
    print("警告: albumentations未安装，将使用基础数据增强。要使用高级数据增强，请运行: pip install albumentations")


def calculate_part_orientation(mask):
    """
    计算零件的方向角度
    使用PCA方法计算主轴方向
    """
    # 如果mask在GPU上，先转移到CPU进行PCA计算
    if torch.is_tensor(mask):
        mask_np = mask.cpu().numpy()
    else:
        mask_np = mask
    
    # 获取掩码的像素坐标
    y_coords, x_coords = np.where(mask_np > 0.5)
    
    if len(x_coords) < 10:  # 需要足够的点进行PCA
        return None, None, None
    
    # 构造坐标点矩阵
    points = np.column_stack((x_coords, y_coords))
    
    # 计算质心
    centroid = np.mean(points, axis=0)
    
    # 使用PCA计算主成分
    pca = PCA(n_components=2)
    pca.fit(points)
    
    # 获取第一主成分(主轴方向)
    principal_vector = pca.components_[0]
    
    # 计算角度(相对于水平轴)
    angle = np.arctan2(principal_vector[1], principal_vector[0])
    angle_degrees = np.degrees(angle)
    
    # 标准化角度到[-90, 90]范围
    if angle_degrees > 90:
        angle_degrees -= 180
    elif angle_degrees < -90:
        angle_degrees += 180
    
    return angle_degrees, centroid, principal_vector


def extract_rotated_bbox_from_mask(mask, image_shape):
    """
    从分割mask中提取旋转边界框
    
    Args:
        mask: 二值化掩码
        image_shape: 图像尺寸 (height, width)
        
    Returns:
        obb_points: 旋转边界框的4个角点坐标 (归一化)
        angle: 旋转角度
        center: 中心点坐标 (归一化)
    """
    # 如果mask在GPU上，先转移到CPU
    if torch.is_tensor(mask):
        mask_np = mask.cpu().numpy()
    else:
        mask_np = mask
    
    # 获取掩码的像素坐标
    y_coords, x_coords = np.where(mask_np > 0.5)
    
    if len(x_coords) < 10:
        return None, None, None
    
    # 构造坐标点矩阵
    points = np.column_stack((x_coords, y_coords))
    
    # 计算最小旋转矩形
    rect = cv2.minAreaRect(points)
    box_points = cv2.boxPoints(rect)
    
    # 获取图像尺寸
    img_height, img_width = image_shape
    
    # 归一化坐标
    normalized_points = []
    for point in box_points:
        x_norm = point[0] / img_width
        y_norm = point[1] / img_height
        normalized_points.extend([x_norm, y_norm])
    
    # 计算中心点和角度
    center_x = rect[0][0] / img_width
    center_y = rect[0][1] / img_height
    angle = rect[2]  # 旋转角度
    
    return normalized_points, angle, (center_x, center_y)


def load_background_images(background_dir: str, target_size: Optional[Tuple[int, int]] = None) -> List[np.ndarray]:
    """
    加载背景图像，智能处理宽高比差异
    
    Args:
        background_dir: 背景图像目录
        target_size: 目标图像尺寸 (height, width)
        
    Returns:
        背景图像列表
    """
    backgrounds = []
    background_path = Path(background_dir)
    
    if target_size:
        target_h, target_w = target_size
        target_ratio = target_w / target_h
    else:
        target_h, target_w = 640, 640
        target_ratio = 1.0
    
    print(f"目标背景尺寸: {target_w}x{target_h} (宽高比: {target_ratio:.3f})")
    
    if not background_path.exists():
        print(f"背景目录不存在: {background_dir}")
        print("将创建默认背景")
        return create_default_backgrounds(target_size=(target_h, target_w))
    
    # 支持的图像格式
    image_extensions = ['*.jpg', '*.jpeg', '*.png', '*.bmp']
    
    for ext in image_extensions:
        for img_path in background_path.glob(ext):
            img = cv2.imread(str(img_path))
            if img is not None:
                orig_h, orig_w = img.shape[:2]
                orig_ratio = orig_w / orig_h
                
                print(f"处理背景: {img_path.name} ({orig_w}x{orig_h}, 比例: {orig_ratio:.3f})")
                
                # 智能调整背景图像以适应目标宽高比
                if abs(orig_ratio - target_ratio) < 0.1:
                    # 宽高比接近，直接缩放
                    bg_resized = cv2.resize(img, (target_w, target_h))
                    print(f"  -> 直接缩放到 {target_w}x{target_h}")
                    
                elif orig_ratio > target_ratio:
                    # 原图更宽，需要裁剪宽度
                    new_w = int(orig_h * target_ratio)
                    if new_w <= orig_w:
                        # 裁剪宽度
                        start_x = (orig_w - new_w) // 2
                        img_cropped = img[:, start_x:start_x+new_w]
                        bg_resized = cv2.resize(img_cropped, (target_w, target_h))
                        print(f"  -> 裁剪宽度后缩放到 {target_w}x{target_h}")
                    else:
                        bg_resized = cv2.resize(img, (target_w, target_h))
                        print(f"  -> 强制缩放到 {target_w}x{target_h}")
                        
                else:
                    # 原图更高，需要裁剪高度
                    new_h = int(orig_w / target_ratio)
                    if new_h <= orig_h:
                        # 裁剪高度
                        start_y = (orig_h - new_h) // 2
                        img_cropped = img[start_y:start_y+new_h, :]
                        bg_resized = cv2.resize(img_cropped, (target_w, target_h))
                        print(f"  -> 裁剪高度后缩放到 {target_w}x{target_h}")
                    else:
                        bg_resized = cv2.resize(img, (target_w, target_h))
                        print(f"  -> 强制缩放到 {target_w}x{target_h}")
                
                backgrounds.append(bg_resized)
    
    if not backgrounds:
        print("未找到背景图像，创建默认背景")
        backgrounds = create_default_backgrounds(target_size=(target_h, target_w))
    else:
        print(f"成功处理 {len(backgrounds)} 张背景图像")
    
    return backgrounds


def create_default_backgrounds(num_backgrounds: int = 5, target_size: Tuple[int, int] = (640, 640)) -> List[np.ndarray]:
    """
    创建默认背景图像
    
    Args:
        num_backgrounds: 创建的背景数量
        target_size: 背景图像尺寸 (height, width)
        
    Returns:
        背景图像列表
    """
    backgrounds = []
    target_h, target_w = target_size
    
    # 创建不同颜色和纹理的背景
    for i in range(num_backgrounds):
        # 创建目标尺寸的背景
        bg = np.zeros((target_h, target_w, 3), dtype=np.uint8)
        
        if i == 0:  # 纯色背景
            color = (random.randint(50, 200), random.randint(50, 200), random.randint(50, 200))
            bg[:] = color
        elif i == 1:  # 渐变背景
            for y in range(target_h):
                color_factor = y / target_h
                color = (int(100 + color_factor * 100), int(100 + color_factor * 100), int(100 + color_factor * 100))
                bg[y, :] = color
        elif i == 2:  # 网格背景
            bg.fill(200)
            cv2.line(bg, (0, target_h//2), (target_w, target_h//2), (150, 150, 150), max(2, target_h//300))
            cv2.line(bg, (target_w//2, 0), (target_w//2, target_h), (150, 150, 150), max(2, target_w//300))
        elif i == 3:  # 噪声背景
            noise = np.random.randint(0, 50, (target_h, target_w, 3))
            bg = np.clip(150 + noise, 0, 255).astype(np.uint8)
        else:  # 圆形图案背景
            bg.fill(180)
            center_x, center_y = target_w//2, target_h//2
            radius1 = min(target_w, target_h) // 4
            radius2 = min(target_w, target_h) // 8
            cv2.circle(bg, (center_x, center_y), radius1, (160, 160, 160), -1)
            cv2.circle(bg, (center_x, center_y), radius2, (140, 140, 140), -1)
        
        backgrounds.append(bg)
    
    return backgrounds


def extract_object_from_mask(image: np.ndarray, mask: np.ndarray) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[Tuple[int, int, int, int]]]:
    """
    从图像中提取目标对象，确保提取完整的目标
    
    Args:
        image: 原始图像
        mask: 分割掩码
        
    Returns:
        object_img: 提取的目标图像
        object_mask: 目标掩码  
        bbox: 边界框 (x, y, w, h)
    """
    # 确保mask是二值的
    if mask.max() > 1:
        mask = (mask > 0.5).astype(np.uint8)
    else:
        mask = mask.astype(np.uint8)
    
    # 找到所有非零像素的边界
    rows = np.any(mask, axis=1)
    cols = np.any(mask, axis=0)
    
    if not np.any(rows) or not np.any(cols):
        return None, None, None
    
    # 获取实际的边界框
    y_min, y_max = np.where(rows)[0][[0, -1]]
    x_min, x_max = np.where(cols)[0][[0, -1]]
    
    # 添加适当的padding，确保不超出图像边界
    padding = 30
    img_h, img_w = image.shape[:2]
    
    x_start = max(0, x_min - padding)
    y_start = max(0, y_min - padding)
    x_end = min(img_w, x_max + padding + 1)
    y_end = min(img_h, y_max + padding + 1)
    
    # 提取目标区域
    object_img = image[y_start:y_end, x_start:x_end]
    object_mask = mask[y_start:y_end, x_start:x_end]
    
    # 确保提取的区域有效
    if object_img.size == 0 or object_mask.size == 0:
        return None, None, None
    
    return object_img, object_mask, (x_start, y_start, x_end - x_start, y_end - y_start)


def place_object_on_background(object_img: np.ndarray, object_mask: np.ndarray, 
                              background: np.ndarray, position: Tuple[int, int],
                              angle: float = 0, scale: float = 1.0) -> Tuple[np.ndarray, np.ndarray]:
    """
    将目标对象放置到背景图像上
    
    Args:
        object_img: 目标图像
        object_mask: 目标掩码
        background: 背景图像
        position: 放置位置 (x, y)
        angle: 旋转角度
        scale: 缩放比例
        
    Returns:
        合成图像, 合成掩码
    """
    if object_img is None or background is None:
        return background, np.zeros(background.shape[:2], dtype=np.uint8)
    
    # 确保object_img和object_mask尺寸匹配
    if object_img.shape[:2] != object_mask.shape[:2]:
        object_mask = cv2.resize(object_mask, (object_img.shape[1], object_img.shape[0]))
    
    # 缩放和旋转目标
    if scale != 1.0 or angle != 0:
        h, w = object_img.shape[:2]
        center = (w // 2, h // 2)
        
        # 计算变换矩阵
        M = cv2.getRotationMatrix2D(center, angle, scale)
        
        # 应用变换
        object_img = cv2.warpAffine(object_img, M, (w, h), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        object_mask = cv2.warpAffine(object_mask, M, (w, h), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        
        # 重新二值化掩码
        object_mask = (object_mask > 0.5).astype(np.uint8)
    
    # 确保背景是正确的大小
    bg_h, bg_w = background.shape[:2]
    obj_h, obj_w = object_img.shape[:2]
    
    # 计算放置位置，确保不超出边界
    x, y = position
    x = max(0, min(x, bg_w - obj_w))
    y = max(0, min(y, bg_h - obj_h))
    
    # 重新计算实际可放置的区域大小
    actual_obj_w = min(obj_w, bg_w - x)
    actual_obj_h = min(obj_h, bg_h - y)
    
    # 创建合成图像和掩码
    composite_img = background.copy()
    composite_mask = np.zeros((bg_h, bg_w), dtype=np.uint8)
    
    # 确保我们操作的区域尺寸匹配
    if actual_obj_w > 0 and actual_obj_h > 0:
        # 获取要操作的目标区域
        obj_region = object_img[:actual_obj_h, :actual_obj_w]
        mask_region = object_mask[:actual_obj_h, :actual_obj_w]
        
        # 创建布尔掩码
        mask_area = mask_region > 0
        
        # 只在有效掩码区域进行操作
        if np.any(mask_area):
            # 获取背景区域
            bg_region = composite_img[y:y+actual_obj_h, x:x+actual_obj_w]
            
            # 应用掩码进行合成
            bg_region[mask_area] = obj_region[mask_area]
            composite_img[y:y+actual_obj_h, x:x+actual_obj_w] = bg_region
            
            # 更新合成掩码
            composite_mask[y:y+actual_obj_h, x:x+actual_obj_w] = mask_region
    
    return composite_img, composite_mask


def generate_synthetic_data(original_objects: List[dict], backgrounds: List[np.ndarray], 
                          num_images: int = 1000, target_size: Optional[Tuple[int, int]] = None) -> List[dict]:
    """
    生成合成训练数据
    
    Args:
        original_objects: 原始提取的目标对象列表
        backgrounds: 背景图像列表
        num_images: 要生成的图像数量
        target_size: 目标图像尺寸 (height, width)，如果为None则自动确定
        
    Returns:
        合成数据列表
    """
    synthetic_data = []
    
    if target_size is None:
        raise ValueError("必须指定target_size参数")
    
    target_h, target_w = target_size
    print(f"开始生成 {num_images} 张合成图像，目标尺寸: {target_w}x{target_h}")
    
    # 数据增强配置
    if ALBUMENTATIONS_AVAILABLE:
        transform = A.Compose([
            A.RandomBrightnessContrast(brightness_limit=0.3, contrast_limit=0.3, p=0.7),
            A.HueSaturationValue(hue_shift_limit=20, sat_shift_limit=30, val_shift_limit=20, p=0.7),
            A.GaussNoise(noise_scale_factor=0.1, p=0.5),
            A.Blur(blur_limit=3, p=0.3),
            A.MotionBlur(blur_limit=7, p=0.3),
        ])
    else:
        transform = None
    
    for i in range(num_images):
        if i % 100 == 0:
            print(f"生成进度: {i}/{num_images}")
        
        # 随机选择背景
        background = random.choice(backgrounds).copy()
        
        # 调整背景到目标尺寸
        background = cv2.resize(background, (target_w, target_h))
        
        # 应用背景增强
        if random.random() < 0.5 and transform is not None:
            background = transform(image=background)['image']
        elif random.random() < 0.5:
            # 基础数据增强 fallback
            brightness = random.uniform(0.7, 1.3)
            background = np.clip(background * brightness, 0, 255).astype(np.uint8)
        
        # 根据图像尺寸智能选择对象数量 - 605x807可以容纳更多对象
        max_objects = min(4, len(original_objects))  # 中等尺寸图像可以放更多对象
        # 增加单个对象的概率，保证训练多样性
        if random.random() < 0.4:  # 40% 概率单个对象
            num_objects = 1
        elif random.random() < 0.3:  # 30% 概率两个对象
            num_objects = min(2, max_objects)
        elif random.random() < 0.2:  # 20% 概率三个对象
            num_objects = min(3, max_objects)
        else:  # 10% 概率四个或更多对象
            num_objects = max_objects
            
        selected_objects = random.sample(original_objects, num_objects)
        
        composite_img = background.copy()
        all_masks = []
        all_bboxes = []
        
        for obj_idx, obj_data in enumerate(selected_objects):
            object_img = obj_data['image']
            object_mask = obj_data['mask']
            
            if i < 5:  # 只在前5张图像中打印调试信息
                print(f"    目标 {obj_idx+1}: 图像尺寸={object_img.shape}, 掩码尺寸={object_mask.shape}")
                print(f"    掩码有效像素数: {np.sum(object_mask > 0)}")
            
            # 随机变换参数 - 为中等尺寸图像优化
            scale = random.uniform(0.7, 1.1)  # 适中的缩放范围，适合605x807尺寸
            angle = random.uniform(-180, 180)
            
            # 计算变换后的大小（预估）
            obj_h, obj_w = object_img.shape[:2]
            # 旋转后的边界框会变大，用对角线长度作为安全估计
            diagonal = int(np.sqrt(obj_h**2 + obj_w**2) * scale)
            
            # 根据目标尺寸调整边距 - 为605x807优化
            margin = max(20, min(target_w // 15, target_h // 15))  # 边距适中，大约40-54像素
            max_x = max(margin, target_w - diagonal - margin)
            max_y = max(margin, target_h - diagonal - margin)
            
            # 如果目标太大，调整缩放
            if max_x <= margin or max_y <= margin:
                scale_x = (target_w - 2 * margin) / diagonal if diagonal > 0 else scale
                scale_y = (target_h - 2 * margin) / diagonal if diagonal > 0 else scale
                scale = min(scale, scale_x, scale_y)
                diagonal = int(np.sqrt(obj_h**2 + obj_w**2) * scale)
                max_x = max(margin, target_w - diagonal - margin)
                max_y = max(margin, target_h - diagonal - margin)
            
            # 生成随机位置，确保有足够空间
            if max_x > margin and max_y > margin:
                pos_x = random.randint(margin, max_x)
                pos_y = random.randint(margin, max_y)
            else:
                pos_x = margin
                pos_y = margin
            
            if i < 5:  # 调试信息
                print(f"    变换参数: scale={scale:.2f}, angle={angle:.1f}°, pos=({pos_x}, {pos_y})")
            
            # 放置目标
            composite_img, obj_mask = place_object_on_background(
                object_img, object_mask, composite_img, 
                (pos_x, pos_y), angle, scale
            )
            
            if i < 5:  # 调试信息
                print(f"    合成后掩码有效像素数: {np.sum(obj_mask > 0)}")
            
            if np.any(obj_mask > 0):
                all_masks.append(obj_mask)
                
                # 计算OBB
                obb_points, _, _ = extract_rotated_bbox_from_mask(obj_mask, (target_h, target_w))
                if obb_points is not None:
                    all_bboxes.append(obb_points)
                    if i < 5:  # 调试信息
                        print(f"    成功生成OBB标注")
                else:
                    if i < 5:  # 调试信息
                        print(f"    OBB标注生成失败")
            else:
                if i < 5:  # 调试信息
                    print(f"    合成后掩码无有效像素")
        
        # 保存合成数据
        if all_bboxes:  # 只有当有有效目标时才保存
            synthetic_data.append({
                'image': composite_img,
                'masks': all_masks,
                'obb_boxes': all_bboxes,
                'image_id': f"synthetic_{i:04d}"
            })
    
    print(f"成功生成 {len(synthetic_data)} 张有效的合成图像")
    return synthetic_data


def save_obb_annotation(image_path, detections, output_dir):
    """
    保存OBB格式的标注文件
    
    Args:
        image_path: 图像路径或ID
        detections: 检测结果列表
        output_dir: 输出目录
    """
    if isinstance(image_path, str):
        image_name = Path(image_path).stem
    else:
        image_name = image_path  # 直接使用传入的ID
        
    label_path = os.path.join(output_dir, "labels", f"{image_name}.txt")
    
    # 确保标签目录存在
    os.makedirs(os.path.dirname(label_path), exist_ok=True)
    
    with open(label_path, 'w') as f:
        for detection in detections:
            if detection.get('obb_points') is not None:
                # 所有类别都标记为part (class_id = 0)
                class_id = 0  # 固定为part类别
                points = detection['obb_points']
                
                # 写入标注文件
                line = f"{class_id} " + " ".join([f"{p:.6f}" for p in points]) + "\n"
                f.write(line)


def copy_image_for_training(image, image_id, output_dir):
    """
    保存图像到训练数据目录
    
    Args:
        image: 图像数组
        image_id: 图像ID
        output_dir: 输出目录
    """
    images_dir = os.path.join(output_dir, "images")
    os.makedirs(images_dir, exist_ok=True)
    
    target_path = os.path.join(images_dir, f"{image_id}.jpg")
    cv2.imwrite(target_path, image)
    
    return target_path


def create_dataset_yaml(output_dir, train_ratio=0.8, target_size=None):
    """
    创建数据集配置文件
    
    Args:
        output_dir: 输出目录
        train_ratio: 训练集比例
        target_size: 图像尺寸 (height, width)
    """
    image_info = ""
    if target_size:
        target_h, target_w = target_size
        image_info = f"""
# 图像信息
image_width: {target_w}
image_height: {target_h}
"""
    
    yaml_content = f"""# OBB训练数据集配置文件
# 数据集路径
path: {os.path.abspath(output_dir)}

# 训练和验证数据
train: train/images
val: val/images

# 类别数量
nc: 1

# 类别名称
names:
  0: part{image_info}
"""
    
    yaml_path = os.path.join(output_dir, "dataset.yaml")
    with open(yaml_path, 'w', encoding='utf-8') as f:
        f.write(yaml_content)
    
    print(f"数据集配置文件已保存: {yaml_path}")


def split_and_save_dataset(synthetic_data: List[dict], output_dir: str, train_ratio: float = 0.8):
    """
    分割数据集并保存
    
    Args:
        synthetic_data: 合成数据列表
        output_dir: 输出目录
        train_ratio: 训练集比例
    """
    # 随机打乱数据
    random.shuffle(synthetic_data)
    
    # 分割数据
    split_idx = int(len(synthetic_data) * train_ratio)
    train_data = synthetic_data[:split_idx]
    val_data = synthetic_data[split_idx:]
    
    print(f"数据集分割: 训练集 {len(train_data)} 张，验证集 {len(val_data)} 张")
    
    # 保存训练集
    train_dir = os.path.join(output_dir, "train")
    os.makedirs(os.path.join(train_dir, "images"), exist_ok=True)
    os.makedirs(os.path.join(train_dir, "labels"), exist_ok=True)
    
    for i, data in enumerate(train_data):
        image_id = f"train_{i:04d}"
        
        # 保存图像
        copy_image_for_training(data['image'], image_id, train_dir)
        
        # 保存标注
        detections = [{'obb_points': bbox} for bbox in data['obb_boxes']]
        save_obb_annotation(image_id, detections, train_dir)
    
    # 保存验证集
    val_dir = os.path.join(output_dir, "val")
    os.makedirs(os.path.join(val_dir, "images"), exist_ok=True)
    os.makedirs(os.path.join(val_dir, "labels"), exist_ok=True)
    
    for i, data in enumerate(val_data):
        image_id = f"val_{i:04d}"
        
        # 保存图像
        copy_image_for_training(data['image'], image_id, val_dir)
        
        # 保存标注
        detections = [{'obb_points': bbox} for bbox in data['obb_boxes']]
        save_obb_annotation(image_id, detections, val_dir)
    
    print(f"训练数据已保存到: {train_dir}")
    print(f"验证数据已保存到: {val_dir}")


def draw_orientation_arrow(image, centroid, direction_vector, angle, length=100):
    """
    在图像上绘制方向箭头
    """
    if centroid is None:
        return image
    
    # 计算箭头的终点
    end_point = centroid + direction_vector * length
    
    # 绘制主轴线
    cv2.arrowedLine(image, 
                    (int(centroid[0]), int(centroid[1])), 
                    (int(end_point[0]), int(end_point[1])), 
                    (0, 255, 255), 3, tipLength=0.3)
    
    # 添加角度文字
    text_pos = (int(centroid[0] + 10), int(centroid[1] - 10))
    cv2.putText(image, f'{angle:.1f}°', text_pos, 
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    
    return image


def main():
    start_time = time.time()
    # 检查GPU可用性
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    print(f"使用设备: {device}")
    
    if device == 'cuda':
        print(f"GPU型号: {torch.cuda.get_device_name(0)}")
        print(f"GPU内存: {torch.cuda.get_device_properties(0).total_memory / 1024**3:.1f} GB")
    
    # 加载YOLO11分割模型并指定GPU设备
    model = YOLO('yolo11x-seg.pt')
    model.to(device)  # 将模型移动到GPU
    print(f"模型加载时间: {time.time() - start_time:.2f}秒")
    start_time = time.time()
    
    # 图像目录路径
    images_dir = './data/images'
    background_dir = './data/images/background'
    # OBB训练数据输出目录
    obb_output_dir = './data/obb_training_data'
    # 结果保存目录
    results_dir = './data/images/result'
    
    # 检查图像目录是否存在
    if not os.path.exists(images_dir):
        print(f"图像目录 {images_dir} 不存在")
        return
    
    # 创建必要的目录
    os.makedirs(obb_output_dir, exist_ok=True)
    os.makedirs(results_dir, exist_ok=True)
    
    # 获取所有jpg图像文件
    image_files = [f for f in os.listdir(images_dir) if f.lower().endswith('.jpg')]
    
    if not image_files:
        print("在images目录中未找到jpg图像文件")
        return
    
    print(f"找到 {len(image_files)} 个图像文件，开始进行语义分割检测...")
    print(f"获取图像文件时间: {time.time() - start_time:.2f}秒")
    
    # 存储提取的目标对象
    extracted_objects = []
    
    # 批量处理图像
    for img_file in image_files:
        img_path = os.path.join(images_dir, img_file)
        print(f"正在处理: {img_file}")
        start_time = time.time()
        
        # 使用YOLO进行语义分割，修复autocast问题
        if device == 'cuda':
            with torch.autocast(device_type='cuda'):
                results = model(img_path, device=device)
        else:
            results = model(img_path, device=device)
            
        print(f"语义分割时间: {time.time() - start_time:.2f}秒")
        start_time = time.time()
        
        # 显示分割结果
        for r in results:
            # 获取分割掩码
            if r.masks is not None:
                print(f"  - 检测到 {len(r.masks)} 个分割区域")
                
                # 读取原始图像获取尺寸
                original_img = cv2.imread(img_path)
                img_height, img_width = original_img.shape[:2]
                
                # 获取检测框信息
                boxes = r.boxes
                
                if boxes is not None:
                    for i, box in enumerate(boxes):
                        cls = int(box.cls)
                        conf = float(box.conf)
                        original_class_name = model.names[cls]
                        
                        print(f"    区域 {i+1}: {original_class_name} -> part, 置信度: {conf:.2f}")
                        
                        # 从对应的mask中提取目标对象
                        if i < len(r.masks.data):
                            mask = r.masks.data[i]
                            
                            # 如果在GPU上需要resize
                            if device == 'cuda':
                                # interpolate的size参数需要是(height, width)格式
                                target_size = (img_height, img_width)
                                mask_resized = torch.nn.functional.interpolate(
                                    mask.unsqueeze(0).unsqueeze(0).float(), 
                                    size=target_size, 
                                    mode='bilinear', 
                                    align_corners=False
                                ).squeeze()
                            else:
                                # cv2.resize的参数是(width, height)格式
                                mask_resized = cv2.resize(mask.cpu().numpy(), 
                                                        (img_width, img_height))
                            
                            # 转换为numpy数组
                            if torch.is_tensor(mask_resized):
                                mask_np = mask_resized.cpu().numpy()
                            else:
                                mask_np = mask_resized
                            
                            # 提取目标对象
                            object_img, object_mask, bbox = extract_object_from_mask(
                                original_img, (mask_np > 0.5).astype(np.uint8))
                            
                            if object_img is not None and object_mask is not None:
                                print(f"      提取目标对象成功: {bbox}")
                                print(f"      目标图像尺寸: {object_img.shape}")
                                print(f"      目标掩码尺寸: {object_mask.shape}")
                                print(f"      掩码有效像素数: {np.sum(object_mask > 0)}")
                                
                                extracted_objects.append({
                                    'image': object_img,  # 现在直接使用BGR图像
                                    'mask': object_mask,
                                    'bbox': bbox,
                                    'original_class': original_class_name,
                                    'source_image': img_file
                                })
                
                # 保存原始分割结果用于验证
                masks = r.masks.data
                colored_mask = np.zeros_like(original_img)
                orientation_img = original_img.copy()
                colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), 
                         (255, 0, 255), (0, 255, 255), (128, 0, 128), (255, 165, 0)]
                
                # 批量处理掩码，减少GPU-CPU数据传输
                if device == 'cuda':
                    # interpolate的size参数需要是(height, width)格式
                    target_size = (original_img.shape[0], original_img.shape[1])
                    masks_resized = torch.nn.functional.interpolate(
                        masks.unsqueeze(1).float(), 
                        size=target_size, 
                        mode='bilinear', 
                        align_corners=False
                    ).squeeze(1)
                    masks_cpu = masks_resized.cpu().numpy()
                else:
                    masks_cpu = masks.cpu().numpy()
                
                for i, mask in enumerate(masks_cpu):
                    if device != 'cuda':
                        # cv2.resize的参数是(width, height)格式
                        mask_resized = cv2.resize(mask, 
                                                (original_img.shape[1], original_img.shape[0]))
                    else:
                        mask_resized = mask
                        
                    mask_bool = mask_resized > 0.5
                    
                    # 确保mask_bool的形状与colored_mask兼容
                    if mask_bool.shape != original_img.shape[:2]:
                        print(f"警告：掩码尺寸不匹配，重新调整 - mask: {mask_bool.shape}, img: {original_img.shape[:2]}")
                        mask_bool = cv2.resize(mask_bool.astype(np.uint8), 
                                             (original_img.shape[1], original_img.shape[0])) > 0.5
                    
                    # 计算零件方向
                    orientation, centroid, direction_vector = calculate_part_orientation(mask_resized)
                    print(f"计算方向时间: {time.time() - start_time:.2f}秒")
                    start_time = time.time()
                    
                    if orientation is not None:
                        print(f"      零件方向: {orientation:.1f}°")
                        orientation_img = draw_orientation_arrow(orientation_img, centroid, 
                                                               direction_vector, orientation)
                    
                    # 应用颜色
                    color = colors[i % len(colors)]
                    colored_mask[mask_bool] = color
                
                # 将掩码与原图融合
                overlay = cv2.addWeighted(original_img, 0.7, colored_mask, 0.3, 0)
                
                # 保存分割结果到result目录
                mask_output_path = os.path.join(results_dir, f'segmented_{img_file}')
                cv2.imwrite(mask_output_path, overlay)
                print(f"  - 分割结果已保存到: {mask_output_path}")
                
                # 保存方向可视化结果到result目录
                orientation_output_path = os.path.join(results_dir, f'orientation_{img_file}')
                cv2.imwrite(orientation_output_path, orientation_img)
                print(f"  - 方向结果已保存到: {orientation_output_path}")
                
                # 清理GPU内存
                if device == 'cuda':
                    torch.cuda.empty_cache()
                
            else:
                print("  - 未检测到分割区域")
        
        # 保存带有检测框和掩码的完整结果到result目录
        annotated_frame = results[0].plot()
        output_path = os.path.join(results_dir, f'detected_{img_file}')
        cv2.imwrite(output_path, annotated_frame)
        print(f"  - 完整结果已保存到: {output_path}")
        
        # 处理完一张图片后清理GPU内存
        if device == 'cuda':
            torch.cuda.empty_cache()
    
    # 生成合成训练数据
    if extracted_objects:
        print(f"\n开始生成合成训练数据...")
        print(f"提取到 {len(extracted_objects)} 个目标对象")
        
        # 从目标对象推断原图尺寸，但使用合理的训练尺寸
        max_h = max_w = 0
        for obj in extracted_objects:
            bbox = obj['bbox']
            x, y, w, h = bbox
            max_h = max(max_h, y + h + 100)  # 添加一些边距
            max_w = max(max_w, x + w + 100)
        
        original_size = (max_h, max_w)
        print(f"原图推断尺寸: {max_w} x {max_h}")
        
        # 根据原图尺寸选择合适的训练尺寸
        # 中等尺寸图像，调整到32的倍数以优化YOLO性能
        if max_w <= 1280 and max_h <= 1280:
            # 调整到32的倍数（YOLO友好）
            target_w = ((max_w + 31) // 32) * 32
            target_h = ((max_h + 31) // 32) * 32
            
            target_size = (target_h, target_w)
            if target_w != max_w or target_h != max_h:
                print(f"优化训练尺寸: {target_w} x {target_h} (32倍数对齐，原尺寸: {max_w}x{max_h})")
            else:
                print(f"使用原图尺寸: {max_w} x {max_h}")
        elif max_w > 1536 or max_h > 1536:
            # 高分辨率图像，缩放到合理尺寸
            scale_factor = min(1536 / max_w, 1536 / max_h)
            target_w = int(max_w * scale_factor)
            target_h = int(max_h * scale_factor)
            
            # 调整到32的倍数（YOLO友好）
            target_w = ((target_w + 31) // 32) * 32
            target_h = ((target_h + 31) // 32) * 32
            
            target_size = (target_h, target_w)
            print(f"缩放训练尺寸: {target_w} x {target_h} (缩放比例: {scale_factor:.2f})")
        else:
            target_size = original_size
            print(f"使用原图尺寸: {max_w} x {max_h}")
        
        # 如果目标尺寸与原图不同，需要缩放提取的对象
        if target_size != original_size:
            scale_factor = min(target_size[1] / original_size[1], target_size[0] / original_size[0])
            print(f"缩放提取的目标对象，缩放比例: {scale_factor:.2f}")
            
            scaled_objects = []
            for obj in extracted_objects:
                scaled_obj = obj.copy()
                
                # 缩放图像和掩码
                original_img = obj['image']
                original_mask = obj['mask']
                
                new_h = int(original_img.shape[0] * scale_factor)
                new_w = int(original_img.shape[1] * scale_factor)
                
                scaled_img = cv2.resize(original_img, (new_w, new_h))
                scaled_mask = cv2.resize(original_mask, (new_w, new_h))
                scaled_mask = (scaled_mask > 0.5).astype(np.uint8)  # 重新二值化
                
                # 缩放bbox
                x, y, w, h = obj['bbox']
                scaled_bbox = (
                    int(x * scale_factor),
                    int(y * scale_factor), 
                    int(w * scale_factor),
                    int(h * scale_factor)
                )
                
                scaled_obj.update({
                    'image': scaled_img,
                    'mask': scaled_mask,
                    'bbox': scaled_bbox
                })
                
                scaled_objects.append(scaled_obj)
                
            extracted_objects = scaled_objects
            print(f"目标对象已缩放到合适尺寸")
        
        # 加载背景图像
        backgrounds = load_background_images(background_dir, target_size)
        print(f"加载了 {len(backgrounds)} 张背景图像")
        
        # 设置随机种子以获得可重复的结果
        random.seed(42)
        np.random.seed(42)
        
        # 生成1000张合成图像
        synthetic_data = generate_synthetic_data(extracted_objects, backgrounds, num_images=1000, target_size=target_size)
        
        if synthetic_data:
            # 分割并保存数据集
            split_and_save_dataset(synthetic_data, obb_output_dir, train_ratio=0.8)
            
            # 创建数据集配置文件
            create_dataset_yaml(obb_output_dir, target_size=target_size)
            
            print(f"\n=== OBB训练数据生成完成 ===")
            print(f"训练数据目录: {obb_output_dir}")
            print(f"训练集: {len(synthetic_data) * 0.8:.0f} 张")
            print(f"验证集: {len(synthetic_data) * 0.2:.0f} 张")
            print(f"类别: part (单一类别)")
            print(f"结果保存目录: {results_dir}")
            
            # 计算内存使用估算
            target_h, target_w = target_size
            memory_per_image = (target_h * target_w * 3) / (1024 * 1024)  # MB
            total_memory = memory_per_image * len(synthetic_data)
            
            # 保存合成数据的统计信息（确保所有数据都是JSON可序列化的）
            stats = {
                'total_synthetic_images': int(len(synthetic_data)),
                'train_images': int(len(synthetic_data) * 0.8),
                'val_images': int(len(synthetic_data) * 0.2),
                'original_objects': int(len(extracted_objects)),
                'background_images': int(len(backgrounds)),
                'class_name': 'part',
                'target_size': [int(target_size[0]), int(target_size[1])],  # 转换为Python int列表
                'image_dimensions': f"{int(target_w)}x{int(target_h)}",
                'memory_per_image_mb': float(round(memory_per_image, 2)),
                'total_memory_mb': float(round(total_memory, 2)),
                'total_memory_gb': float(round(total_memory / 1024, 2)),
                'generation_time': float(time.time())
            }
            
            print(f"内存使用: 单张图像 {memory_per_image:.1f}MB, 总计 {total_memory/1024:.1f}GB")
            
            with open(os.path.join(obb_output_dir, 'generation_stats.json'), 'w') as f:
                json.dump(stats, f, indent=2)
        else:
            print("未能生成有效的合成数据")
    else:
        print("未提取到任何目标对象，无法生成训练数据")


if __name__ == "__main__":
    main()
