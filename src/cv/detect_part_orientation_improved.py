#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
零件尖端方向检测
角度系统：尖端向上为0度，顺时针为正角度
"""

import os
import json
import cv2
import numpy as np
from sklearn.decomposition import PCA


def detect_part_tip_direction(mask):
    """
    检测零件尖端方向
    返回: (角度, 质心, 尖端向量)
    """
    # 获取掩码的像素坐标
    y_coords, x_coords = np.where(mask > 0.5)
    
    if len(x_coords) < 10:
        return None, None, None
    
    # 构造坐标点矩阵
    points = np.column_stack((x_coords, y_coords))
    
    # 计算质心
    centroid = np.mean(points, axis=0)
    
    # 使用PCA计算主轴方向
    pca = PCA(n_components=2)
    pca.fit(points)
    principal_vector = pca.components_[0]
    
    # 计算沿主轴正负方向的延伸距离
    mask_height, mask_width = mask.shape
    
    # 正方向延伸距离
    positive_distance = calculate_extension_distance(
        mask, centroid, principal_vector, mask_width, mask_height
    )
    
    # 负方向延伸距离
    negative_distance = calculate_extension_distance(
        mask, centroid, -principal_vector, mask_width, mask_height
    )
    
    # 尖端在延伸更远的方向
    if positive_distance > negative_distance:
        # 尖端在正方向
        tip_vector = principal_vector
    else:
        # 尖端在负方向
        tip_vector = -principal_vector
    
    # 计算尖端方向角度(相对于水平轴)
    angle_to_horizontal = np.degrees(np.arctan2(tip_vector[1], tip_vector[0]))
    
    # 转换为以向上为0度的角度系统
    # 在图像坐标系中，向上是[0, -1]，对应atan2(-1, 0) = -90度
    # 我们希望向上是0度，所以转换公式是：(atan2_angle + 90)
    angle_to_up = angle_to_horizontal + 90
    
    # 标准化角度到[-180, 180]范围
    while angle_to_up > 180:
        angle_to_up -= 360
    while angle_to_up <= -180:
        angle_to_up += 360
    
    return angle_to_up, centroid, tip_vector


def calculate_extension_distance(mask, start_point, direction_vector, width, height):
    """
    计算从起始点沿指定方向能在掩码内延伸的最大距离
    """
    # 标准化方向向量
    direction_vector = direction_vector / np.linalg.norm(direction_vector)
    
    max_distance = 0
    step_size = 0.5
    
    # 逐步延伸直到离开掩码边界
    for step in range(1, int(max(width, height) * 2)):
        # 计算当前点
        current_point = start_point + step * step_size * direction_vector
        
        # 检查边界
        x, y = int(round(current_point[0])), int(round(current_point[1]))
        if x < 0 or x >= width or y < 0 or y >= height:
            break
        
        # 检查是否仍在掩码内
        if mask[y, x] > 0.5:
            max_distance = step * step_size
        else:
            break
    
    return max_distance


def draw_tip_direction_visualization(image, polygon_points, centroid, tip_vector, angle):
    """
    绘制尖端方向可视化
    """
    if centroid is None or tip_vector is None:
        return image
    
    result_image = image.copy()
    center_x, center_y = int(centroid[0]), int(centroid[1])
    
    # 根据图像尺寸调整参数
    img_h, img_w = result_image.shape[:2]
    arrow_length = min(img_w, img_h) // 6
    line_thickness = max(2, min(img_w, img_h) // 200)
    font_scale = max(0.6, min(img_w, img_h) / 1000)
    font_thickness = max(1, int(font_scale * 2))
    
    # 绘制零件轮廓和填充
    cv2.polylines(result_image, [polygon_points], True, (0, 255, 255), line_thickness)  # 青色轮廓
    overlay = result_image.copy()
    cv2.fillPoly(overlay, [polygon_points], (255, 255, 0))  # 黄色填充
    result_image = cv2.addWeighted(result_image, 0.8, overlay, 0.2, 0)
    
    # 绘制尖端方向箭头（绿色）
    end_x = int(center_x + arrow_length * tip_vector[0])
    end_y = int(center_y + arrow_length * tip_vector[1])
    cv2.arrowedLine(result_image, (center_x, center_y), (end_x, end_y), 
                   (0, 255, 0), line_thickness, tipLength=0.3)
    
    # 绘制质心（红色圆点）
    cv2.circle(result_image, (center_x, center_y), max(5, line_thickness), (0, 0, 255), -1)
    cv2.circle(result_image, (center_x, center_y), max(8, line_thickness + 3), (255, 255, 255), 2)
    
    # 添加角度文本
    angle_text = f"Tip Angle: {angle:.1f}deg"
    text_x = center_x + max(20, line_thickness * 4)
    text_y = center_y - max(15, line_thickness * 3)
    
    # 黑色背景文字
    cv2.putText(result_image, angle_text, (text_x, text_y), 
               cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 0), font_thickness + 2)
    # 白色前景文字
    cv2.putText(result_image, angle_text, (text_x, text_y), 
               cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), font_thickness)
    
    return result_image


def process_part_detection(json_path, output_dir="./data/images/tip_result"):
    """
    处理零件检测的主函数
    """
    # 读取标注文件
    with open(json_path, 'r', encoding='utf-8') as f:
        data = json.load(f)
    
    # 获取图像和标注信息
    img_width, img_height = data['imageWidth'], data['imageHeight']
    image_path = data['imagePath']
    shapes = data['shapes']
    
    if not shapes:
        print("错误: 没有找到标注形状")
        return
    
    # 读取图像获取实际尺寸
    json_dir = os.path.dirname(json_path)
    if image_path.startswith('..'):
        full_image_path = os.path.normpath(os.path.join(json_dir, image_path))
    else:
        full_image_path = os.path.join(json_dir, os.path.basename(image_path))
    
    if os.path.exists(full_image_path):
        original_image = cv2.imread(full_image_path)
        if original_image is None:
            print(f"错误: 无法读取图像文件 {full_image_path}")
            return
        actual_height, actual_width = original_image.shape[:2]
        scale_x = actual_width / img_width
        scale_y = actual_height / img_height
        img_width, img_height = actual_width, actual_height
    else:
        print(f"错误: 图像文件不存在: {full_image_path}")
        return
    
    # 创建输出目录
    os.makedirs(output_dir, exist_ok=True)
    
    # 获取json文件名（不含扩展名）用于区分不同文件
    json_filename = os.path.splitext(os.path.basename(json_path))[0]
    
    # 处理标注形状
    for i, shape in enumerate(shapes):
        label = shape.get('label', 'part')
        points = shape['points']
        
        # 缩放坐标点
        scaled_points = [[point[0] * scale_x, point[1] * scale_y] for point in points]
        
        # 创建掩码
        mask = np.zeros((img_height, img_width), dtype=np.uint8)
        polygon_points = np.array(scaled_points, dtype=np.int32)
        cv2.fillPoly(mask, [polygon_points], (255,))
        
        # 检测尖端方向
        angle, centroid, tip_vector = detect_part_tip_direction(mask)
        
        if angle is not None:
            print(f"{angle:.1f}")
            
            # 生成可视化图像
            result_image = draw_tip_direction_visualization(
                original_image, polygon_points, centroid, tip_vector, angle
            )
            
            # 保存可视化结果 - 包含源文件名以避免冲突
            output_filename = f"{json_filename}_tip_direction_{label}_{i}.jpg"
            output_path = os.path.join(output_dir, output_filename)
            cv2.imwrite(output_path, result_image)
            print(f"可视化结果已保存: {output_path}")
            
        else:
            print("检测失败")


def process_directory(annotations_dir="./data/annotations", output_dir="./data/images/tip_result"):
    """
    处理目录中的所有json标注文件
    """
    if not os.path.exists(annotations_dir):
        print(f"标注目录不存在: {annotations_dir}")
        return
    
    # 查找所有json文件
    json_files = []
    for file in os.listdir(annotations_dir):
        if file.lower().endswith('.json'):
            json_files.append(os.path.join(annotations_dir, file))
    
    if not json_files:
        print(f"在目录 {annotations_dir} 中没有找到json文件")
        return
    
    print(f"找到 {len(json_files)} 个json文件，开始批量处理...")
    
    success_count = 0
    for i, json_file in enumerate(json_files, 1):
        print(f"\n[{i}/{len(json_files)}] 处理文件: {os.path.basename(json_file)}")
        try:
            process_part_detection(json_file, output_dir)
            success_count += 1
        except Exception as e:
            print(f"处理文件 {os.path.basename(json_file)} 时出错: {e}")
    
    print(f"\n批量处理完成！成功处理 {success_count}/{len(json_files)} 个文件")


def main():
    """主函数"""
    annotations_dir = "./data/annotations"
    
    print("开始零件尖端方向检测...")
    process_directory(annotations_dir)
    print("检测完成！")


if __name__ == "__main__":
    main() 