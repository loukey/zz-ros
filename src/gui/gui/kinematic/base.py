from math import cos, sin
import numpy as np
import cv2
import torch
from sklearn.decomposition import PCA


def dh_to_rotation_matrix(a, alpha, d, theta):
    c_theta = cos(theta)
    s_theta = sin(theta)
    c_alpha = cos(alpha)
    s_alpha = sin(alpha)
    
    return np.array([
        [c_theta, -s_theta, 0, a],
        [s_theta * c_alpha, c_theta * c_alpha, -s_alpha, -s_alpha * d],
        [s_theta * s_alpha, c_theta * s_alpha, c_alpha, c_alpha * d],
        [0, 0, 0, 1]
    ], dtype=float)


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


def calculate_part_orientation(mask):
    """
    检测零件尖端方向
    使用改进的PCA+延伸距离算法
    角度系统：尖端向上为0度，顺时针为正角度
    
    Args:
        mask: 掩码数组，可以是torch.Tensor或numpy.ndarray
        
    Returns:
        tuple: (角度, 质心, 尖端向量)
        - angle_degrees: 尖端方向角度（度）
        - centroid: 质心坐标 [x, y]
        - tip_vector: 尖端方向向量
    """
    # 如果mask在GPU上，先转移到CPU进行PCA计算
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
    
    # 计算质心
    centroid = np.mean(points, axis=0)
    
    # 使用PCA计算主轴方向
    pca = PCA(n_components=2)
    pca.fit(points)
    principal_vector = pca.components_[0]
    
    # 计算沿主轴正负方向的延伸距离
    mask_height, mask_width = mask_np.shape
    
    # 正方向延伸距离
    positive_distance = calculate_extension_distance(
        mask_np, centroid, principal_vector, mask_width, mask_height
    )
    
    # 负方向延伸距离
    negative_distance = calculate_extension_distance(
        mask_np, centroid, -principal_vector, mask_width, mask_height
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


def draw_orientation_arrow(image, centroid, tip_vector, angle, length=100):
    """
    在图像上绘制尖端方向箭头
    
    Args:
        image: 输入图像
        centroid: 质心坐标 [x, y]
        tip_vector: 尖端方向向量
        angle: 角度值
        length: 箭头长度
    
    Returns:
        绘制了尖端方向箭头的图像
    """
    if centroid is None or tip_vector is None:
        return image
    
    result_image = image.copy()
    center_x, center_y = int(centroid[0]), int(centroid[1])
    
    # 根据图像尺寸调整参数
    img_h, img_w = result_image.shape[:2]
    arrow_length = min(img_w, img_h) // 8
    line_thickness = max(2, min(img_w, img_h) // 200)
    font_scale = max(0.6, min(img_w, img_h) / 1000)
    font_thickness = max(1, int(font_scale * 2))
    
    # 绘制尖端方向箭头（绿色）
    end_x = int(center_x + arrow_length * tip_vector[0])
    end_y = int(center_y + arrow_length * tip_vector[1])
    cv2.arrowedLine(result_image, (center_x, center_y), (end_x, end_y), 
                   (0, 255, 0), line_thickness + 1, tipLength=0.3)
    
    # 绘制质心（红色圆点）
    cv2.circle(result_image, (center_x, center_y), max(5, line_thickness), (0, 0, 255), -1)
    cv2.circle(result_image, (center_x, center_y), max(8, line_thickness + 3), (255, 255, 255), 2)
    
    # 添加角度文本
    angle_text = f"Tip: {angle:.1f}deg"
    text_x = center_x + max(20, line_thickness * 4)
    text_y = center_y - max(15, line_thickness * 3)
    
    # 黑色背景文字
    cv2.putText(result_image, angle_text, (text_x, text_y), 
               cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 0), font_thickness + 2)
    # 白色前景文字
    cv2.putText(result_image, angle_text, (text_x, text_y), 
               cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), font_thickness)
    
    return result_image


def normalize_angle(angle):
    """
    标准化角度到指定范围
    
    Args:
        angle: 输入角度（度）
    
    Returns:
        标准化后的角度
    """
    while angle > 180:
        angle -= 360
    while angle <= -180:
        angle += 360
    return angle
    