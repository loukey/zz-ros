import os
import cv2
import numpy as np
import torch
from ultralytics import YOLO
from sklearn.decomposition import PCA
import time


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
    
    # 检查图像目录是否存在
    if not os.path.exists(images_dir):
        print(f"图像目录 {images_dir} 不存在")
        return
    
    # 获取所有jpg图像文件
    image_files = [f for f in os.listdir(images_dir) if f.lower().endswith('.jpg')]
    
    if not image_files:
        print("在images目录中未找到jpg图像文件")
        return
    
    print(f"找到 {len(image_files)} 个图像文件，开始进行语义分割检测...")
    print(f"获取图像文件时间: {time.time() - start_time:.2f}秒")
    start_time = time.time()
    # 批量处理图像
    for img_file in image_files:
        img_path = os.path.join(images_dir, img_file)
        print(f"正在处理: {img_file}")
        
        # 使用YOLO进行语义分割，明确指定设备
        with torch.amp.autocast(device_type=device) if device == 'cuda' else torch.no_grad():
            results = model(img_path, device=device)
        print(f"语义分割时间: {time.time() - start_time:.2f}秒")
        start_time = time.time()
        # 显示分割结果
        for r in results:
            # 获取分割掩码
            if r.masks is not None:
                print(f"  - 检测到 {len(r.masks)} 个分割区域")
                
                # 获取检测框信息
                boxes = r.boxes
                if boxes is not None:
                    for i, box in enumerate(boxes):
                        cls = int(box.cls)
                        conf = float(box.conf)
                        class_name = model.names[cls]
                        print(f"    区域 {i+1}: {class_name}, 置信度: {conf:.2f}")
                
                # 保存分割掩码
                masks = r.masks.data  # 获取掩码数据（保持在GPU上）
                original_img = cv2.imread(img_path)
                
                # 创建彩色掩码和方向可视化图像
                colored_mask = np.zeros_like(original_img)
                orientation_img = original_img.copy()
                colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), 
                         (255, 0, 255), (0, 255, 255), (128, 0, 128), (255, 165, 0)]
                
                # 批量处理掩码，减少GPU-CPU数据传输
                if device == 'cuda':
                    # 在GPU上进行resize操作
                    target_size = (original_img.shape[1], original_img.shape[0])
                    masks_resized = torch.nn.functional.interpolate(
                        masks.unsqueeze(1).float(), 
                        size=target_size, 
                        mode='bilinear', 
                        align_corners=False
                    ).squeeze(1)
                    # 转换为CPU进行后续处理
                    masks_cpu = masks_resized.cpu().numpy()
                else:
                    masks_cpu = masks.cpu().numpy()
                
                for i, mask in enumerate(masks_cpu):
                    # 如果在CPU模式下需要resize
                    mask_resized = cv2.resize(mask, 
                                            (original_img.shape[1], original_img.shape[0]))
                    mask_bool = mask_resized > 0.5
                    
                    # 计算零件方向
                    orientation, centroid, direction_vector = calculate_part_orientation(mask_resized)
                    print(f"计算方向时间: {time.time() - start_time:.2f}秒")
                    start_time = time.time()
                    if orientation is not None:
                        print(f"      零件方向: {orientation:.1f}°")
                        # 在方向可视化图像上绘制箭头
                        orientation_img = draw_orientation_arrow(orientation_img, centroid, 
                                                               direction_vector, orientation)
                    
                    # 应用颜色
                    color = colors[i % len(colors)]
                    colored_mask[mask_bool] = color
                
                # 将掩码与原图融合
                overlay = cv2.addWeighted(original_img, 0.7, colored_mask, 0.3, 0)
                
                # 保存分割结果
                mask_output_path = os.path.join(images_dir, f'result/segmented_{img_file}')
                cv2.imwrite(mask_output_path, overlay)
                print(f"  - 分割结果已保存到: {mask_output_path}")
                
                # 保存方向可视化结果
                orientation_output_path = os.path.join(images_dir, f'orientation_{img_file}')
                cv2.imwrite(orientation_output_path, orientation_img)
                print(f"  - 方向结果已保存到: {orientation_output_path}")
                
                # 清理GPU内存
                if device == 'cuda':
                    torch.cuda.empty_cache()
                
            else:
                print("  - 未检测到分割区域")
        
        # 保存带有检测框和掩码的完整结果
        annotated_frame = results[0].plot()
        output_path = os.path.join(images_dir, f'detected_{img_file}')
        cv2.imwrite(output_path, annotated_frame)
        print(f"  - 完整结果已保存到: {output_path}")
        
        # 处理完一张图片后清理GPU内存
        if device == 'cuda':
            torch.cuda.empty_cache()

if __name__ == "__main__":
    main()
