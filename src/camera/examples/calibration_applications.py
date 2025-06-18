#!/usr/bin/env python3
"""
相机标定应用示例
展示如何使用标定结果进行各种计算和应用
"""

import sys
import os
# 添加正确的路径
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

import cv2
import numpy as np
from camera_utils import create_your_camera_utils

try:
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False
    print("matplotlib未安装，将跳过图像显示功能")


def demo_undistort_comparison():
    """演示去畸变效果对比"""
    print("="*60)
    print("1. 图像去畸变演示")
    print("="*60)
    
    # 创建相机工具
    camera_utils = create_your_camera_utils()
    
    # 创建测试图像（带网格）
    img_size = (1280, 720)
    test_image = create_grid_test_image(img_size)
    
    # 添加人工畸变
    distorted_image = add_artificial_distortion(test_image, camera_utils)
    
    # 去畸变
    undistorted_image = camera_utils.undistort_image(distorted_image)
    
    # 显示对比
    if HAS_MATPLOTLIB:
        show_three_images(test_image, distorted_image, undistorted_image,
                         ['原始网格', '人工畸变', '去畸变后'])
    else:
        # 使用OpenCV显示
        cv2.imshow('Original', test_image)
        cv2.imshow('Distorted', distorted_image)
        cv2.imshow('Undistorted', undistorted_image)
    
    print("按任意键继续...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def demo_distance_measurement():
    """演示距离测量"""
    print("="*60)
    print("2. 距离测量演示")
    print("="*60)
    
    camera_utils = create_your_camera_utils()
    
    # 示例：测量两个点之间的距离
    # 假设这是在深度为1米处的两个像素点
    point1 = np.array([[400, 300]])  # 像素坐标
    point2 = np.array([[800, 400]])  # 像素坐标
    depth = 1.0  # 深度1米
    
    # 转换到相机坐标系
    camera_point1 = camera_utils.pixel_to_camera_coords(point1, depth)
    camera_point2 = camera_utils.pixel_to_camera_coords(point2, depth)
    
    # 计算3D距离
    distance_3d = np.linalg.norm(camera_point1 - camera_point2)
    
    # 计算像素距离
    distance_pixel = np.linalg.norm(point1 - point2)
    
    print(f"像素点1: {point1[0]}")
    print(f"像素点2: {point2[0]}")
    print(f"像素距离: {distance_pixel:.2f} 像素")
    print(f"3D坐标1: [{camera_point1[0][0]:.3f}, {camera_point1[0][1]:.3f}, {camera_point1[0][2]:.3f}] 米")
    print(f"3D坐标2: [{camera_point2[0][0]:.3f}, {camera_point2[0][1]:.3f}, {camera_point2[0][2]:.3f}] 米")
    print(f"实际3D距离: {distance_3d:.3f} 米")


def demo_field_of_view():
    """演示视场角计算"""
    print("="*60)
    print("3. 视场角分析")
    print("="*60)
    
    camera_utils = create_your_camera_utils()
    
    # 获取视场角
    fov_x, fov_y = camera_utils.get_field_of_view()
    
    print(f"水平视场角: {fov_x:.1f}°")
    print(f"垂直视场角: {fov_y:.1f}°")
    print(f"对角线视场角: {np.sqrt(fov_x**2 + fov_y**2):.1f}°")
    
    # 计算不同距离下的视野范围
    distances = [0.5, 1.0, 2.0, 5.0]
    print(f"\n不同距离的视野范围:")
    print(f"{'距离(m)':<8} {'宽度(m)':<8} {'高度(m)':<8}")
    print("-" * 25)
    
    for dist in distances:
        width = 2 * dist * np.tan(np.radians(fov_x / 2))
        height = 2 * dist * np.tan(np.radians(fov_y / 2))
        print(f"{dist:<8.1f} {width:<8.2f} {height:<8.2f}")


def demo_coordinate_conversion():
    """演示坐标转换"""
    print("="*60)
    print("4. 坐标转换演示")
    print("="*60)
    
    camera_utils = create_your_camera_utils()
    
    # 像素坐标转相机坐标
    pixel_points = np.array([[640, 360], [400, 300], [800, 500]])  # 图像中心和两个其他点
    depth = 2.0  # 2米深度
    
    camera_points = camera_utils.pixel_to_camera_coords(pixel_points, depth)
    
    print("像素坐标 -> 相机坐标 (深度2米):")
    for i, (pixel, camera) in enumerate(zip(pixel_points, camera_points)):
        print(f"点{i+1}: 像素{pixel} -> 相机[{camera[0]:.3f}, {camera[1]:.3f}, {camera[2]:.3f}]米")
    
    # 相机坐标转回像素坐标
    pixel_back = camera_utils.camera_to_pixel_coords(camera_points)
    
    print("\n相机坐标 -> 像素坐标 (验证):")
    for i, (original, converted) in enumerate(zip(pixel_points, pixel_back)):
        error = np.linalg.norm(original - converted)
        print(f"点{i+1}: 原始{original} -> 转换{converted.astype(int)} (误差: {error:.3f})")


def demo_real_world_measurements():
    """演示真实世界测量应用"""
    print("="*60)
    print("5. 真实世界测量应用")
    print("="*60)
    
    camera_utils = create_your_camera_utils()
    
    # 模拟一个矩形物体在图像中的四个角点
    print("假设场景：检测到一个矩形物体")
    rect_corners_pixel = np.array([
        [400, 300],  # 左上
        [600, 300],  # 右上
        [600, 450],  # 右下
        [400, 450]   # 左下
    ])
    
    depth = 1.5  # 假设物体距离相机1.5米
    
    # 转换到相机坐标系
    rect_corners_3d = camera_utils.pixel_to_camera_coords(rect_corners_pixel, depth)
    
    # 计算矩形的长度和宽度
    width_3d = np.linalg.norm(rect_corners_3d[1] - rect_corners_3d[0])
    height_3d = np.linalg.norm(rect_corners_3d[3] - rect_corners_3d[0])
    area_3d = width_3d * height_3d
    
    print(f"矩形角点像素坐标:")
    for i, corner in enumerate(rect_corners_pixel):
        print(f"  角点{i+1}: {corner}")
    
    print(f"\n矩形角点3D坐标 (深度{depth}米):")
    for i, corner in enumerate(rect_corners_3d):
        print(f"  角点{i+1}: [{corner[0]:.3f}, {corner[1]:.3f}, {corner[2]:.3f}]米")
    
    print(f"\n矩形实际尺寸:")
    print(f"  宽度: {width_3d:.3f}米")
    print(f"  高度: {height_3d:.3f}米")
    print(f"  面积: {area_3d:.3f}平方米")


def create_grid_test_image(size):
    """创建网格测试图像"""
    width, height = size
    image = np.ones((height, width, 3), dtype=np.uint8) * 255
    
    # 绘制网格
    grid_size = 50
    for x in range(0, width, grid_size):
        cv2.line(image, (x, 0), (x, height), (200, 200, 200), 1)
    for y in range(0, height, grid_size):
        cv2.line(image, (0, y), (width, y), (200, 200, 200), 1)
    
    # 绘制中心十字
    cv2.line(image, (width//2-50, height//2), (width//2+50, height//2), (0, 0, 255), 3)
    cv2.line(image, (width//2, height//2-50), (width//2, height//2+50), (0, 0, 255), 3)
    
    return image


def add_artificial_distortion(image, camera_utils):
    """添加人工畸变"""
    # 创建畸变较大的系数进行演示
    strong_dist = np.array([0.3, -0.2, 0.01, 0.01, 0.1])
    return cv2.undistort(image, camera_utils.camera_matrix, -strong_dist)


def show_three_images(img1, img2, img3, titles):
    """显示三张图像的对比"""
    if HAS_MATPLOTLIB:
        fig, axes = plt.subplots(1, 3, figsize=(15, 5))
        
        for i, (img, title) in enumerate(zip([img1, img2, img3], titles)):
            axes[i].imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
            axes[i].set_title(title)
            axes[i].axis('off')
        
        plt.tight_layout()
        plt.show()
    else:
        print("matplotlib未安装，无法显示图像对比")


def main():
    print("相机标定应用演示程序")
    print("="*60)
    
    # 创建示例目录
    os.makedirs('examples', exist_ok=True)
    
    try:
        # 1. 去畸变演示
        demo_undistort_comparison()
        
        # 2. 距离测量演示
        demo_distance_measurement()
        
        # 3. 视场角分析
        demo_field_of_view()
        
        # 4. 坐标转换演示
        demo_coordinate_conversion()
        
        # 5. 真实世界测量
        demo_real_world_measurements()
        
        print("\n" + "="*60)
        print("演示完成！")
        print("你可以基于这些示例开发自己的应用")
        print("="*60)
        
    except ImportError as e:
        print(f"缺少依赖包: {e}")
        print("请安装: pip install matplotlib")


if __name__ == '__main__':
    main() 