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
    cv2.putText(image, f'{angle:.1f}deg', text_pos, 
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    
    return image


def process_image_segmentation_and_orientation(images_dir='./data/images', model_path='yolo11x-seg.pt'):
    """
    处理图像分割和方向识别的核心函数
    专注于纯粹的计算和识别，不进行任何可视化操作
    
    Args:
        images_dir (str): 输入图像目录路径
        model_path (str): YOLO模型文件路径
        
    Returns:
        list: 每张图像的处理结果列表
        [
            {
                'image_file': 图像文件名,
                'image_path': 图像完整路径,
                'original_image': 原始图像数组,
                'detection_results': YOLO检测结果,
                'objects': [
                    {
                        'class_name': 类别名称,
                        'confidence': 置信度,
                        'mask': 掩码数组,
                        'orientation': 方向角度,
                        'centroid': 质心坐标,
                        'direction_vector': 方向向量
                    },
                    ...
                ]
            },
            ...
        ]
    """
    start_time = time.time()
    
    # 检查GPU可用性
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    print(f"使用设备: {device}")
    
    if device == 'cuda':
        print(f"GPU型号: {torch.cuda.get_device_name(0)}")
        print(f"GPU内存: {torch.cuda.get_device_properties(0).total_memory / 1024**3:.1f} GB")
    
    # 加载YOLO11分割模型并指定GPU设备
    model = YOLO(model_path)
    model.to(device)  # 将模型移动到GPU
    
    # 检查图像目录是否存在
    if not os.path.exists(images_dir):
        print(f"图像目录 {images_dir} 不存在")
        return []
    
    # 获取所有jpg图像文件
    image_files = [f for f in os.listdir(images_dir) if f.lower().endswith('.jpg')]
    
    if not image_files:
        print("在images目录中未找到jpg图像文件")
        return []
    
    print(f"找到 {len(image_files)} 个图像文件，开始进行语义分割检测...")
    
    # 存储所有处理结果
    all_results = []
    
    # 批量处理图像
    for img_file in image_files:
        img_path = os.path.join(images_dir, img_file)
        print(f"正在处理: {img_file}")
        
        # 读取原始图像
        original_img = cv2.imread(img_path)
        img_height, img_width = original_img.shape[:2]
        
        # 使用YOLO进行语义分割
        if device == 'cuda':
            with torch.autocast(device_type='cuda'):
                results = model(img_path, device=device)
        else:
            results = model(img_path, device=device)
        
        # 初始化当前图像的结果
        current_result = {
            'image_file': img_file,
            'image_path': img_path,
            'original_image': original_img.copy(),
            'detection_results': results,
            'objects': []
        }
        
        # 处理分割结果
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
                        original_class_name = model.names[cls]
                        
                        print(f"    区域 {i+1}: {original_class_name} -> part, 置信度: {conf:.2f}")
                
                # 处理掩码数据
                masks = r.masks.data
                
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
                    
                    # 确保mask_bool的形状与原图像兼容
                    if mask_bool.shape != original_img.shape[:2]:
                        print(f"警告：掩码尺寸不匹配，重新调整 - mask: {mask_bool.shape}, img: {original_img.shape[:2]}")
                        mask_bool = cv2.resize(mask_bool.astype(np.uint8), 
                                             (original_img.shape[1], original_img.shape[0])) > 0.5
                    
                    # 计算零件方向
                    orientation, centroid, direction_vector = calculate_part_orientation(mask_resized)
                    
                    # 存储对象信息
                    obj_info = {
                        'class_name': 'part',
                        'confidence': float(boxes[i].conf) if boxes is not None and i < len(boxes) else 0.0,
                        'mask': mask_bool.copy(),
                        'orientation': orientation,
                        'centroid': centroid,
                        'direction_vector': direction_vector
                    }
                    current_result['objects'].append(obj_info)
                    
                    if orientation is not None:
                        print(f"      零件方向: {orientation:.1f}度")
                
                # 清理GPU内存
                if device == 'cuda':
                    torch.cuda.empty_cache()
                
            else:
                print("  - 未检测到分割区域")
        
        # 处理完一张图片后清理GPU内存
        if device == 'cuda':
            torch.cuda.empty_cache()
        
        all_results.append(current_result)
    
    # 处理完成
    total_time = time.time() - start_time
    total_objects = sum(len(result['objects']) for result in all_results)
    total_orientations = sum(1 for result in all_results for obj in result['objects'] if obj['orientation'] is not None)
    
    print(f"\n=== 语义分割和方向识别完成 ===")
    print(f"处理了 {len(all_results)} 张图像")
    print(f"检测到 {total_objects} 个目标对象")
    print(f"计算了 {total_orientations} 个方向角度")
    print(f"总用时: {total_time:.2f}秒")
    
    return all_results


def save_results(results, results_dir='./data/images/result'):
    """
    保存识别结果到文件，并进行所有的可视化工作
    
    Args:
        results (list): process_image_segmentation_and_orientation的返回结果
        results_dir (str): 结果保存目录
    """
    if not results:
        print("没有结果需要保存")
        return
    
    # 创建结果目录
    os.makedirs(results_dir, exist_ok=True)
    
    print(f"\n开始生成可视化图像并保存结果到: {results_dir}")
    
    # 定义颜色列表
    colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), 
              (255, 0, 255), (0, 255, 255), (128, 0, 128), (255, 165, 0)]
    
    for result in results:
        img_file = result['image_file']
        original_img = result['original_image']
        objects = result['objects']
        detection_results = result['detection_results']
        
        print(f"正在生成 {img_file} 的可视化图像...")
        
        # 1. 生成分割结果图像
        if objects:
            colored_mask = np.zeros_like(original_img)
            
            for i, obj in enumerate(objects):
                mask_bool = obj['mask']
                color = colors[i % len(colors)]
                colored_mask[mask_bool] = color
            
            # 混合原图和彩色掩码
            segmented_image = cv2.addWeighted(original_img, 0.7, colored_mask, 0.3, 0)
            
            # 保存分割结果
            mask_output_path = os.path.join(results_dir, f'segmented_{img_file}')
            cv2.imwrite(mask_output_path, segmented_image)
            print(f"  - 分割结果已保存到: {mask_output_path}")
        
        # 2. 生成方向可视化图像
        if objects:
            orientation_img = original_img.copy()
            
            for obj in objects:
                if obj['orientation'] is not None:
                    orientation_img = draw_orientation_arrow(
                        orientation_img, 
                        obj['centroid'], 
                        obj['direction_vector'], 
                        obj['orientation']
                    )
            
            # 保存方向可视化结果
            orientation_output_path = os.path.join(results_dir, f'orientation_{img_file}')
            cv2.imwrite(orientation_output_path, orientation_img)
            print(f"  - 方向结果已保存到: {orientation_output_path}")
        
        # 3. 生成完整检测标注图像
        if detection_results:
            annotated_image = detection_results[0].plot()
            
            # 保存完整检测结果
            output_path = os.path.join(results_dir, f'detected_{img_file}')
            cv2.imwrite(output_path, annotated_image)
            print(f"  - 完整结果已保存到: {output_path}")
    
    print("所有可视化图像生成和保存完成")


def main():
    """
    主函数 - 执行完整的识别和保存流程
    """
    print("=== 语义分割和方向识别工具 ===")
    
    # 执行识别处理
    results = process_image_segmentation_and_orientation()
    
    if results:
        # 保存结果
        save_results(results)
        
        # 显示统计信息
        total_objects = sum(len(result['objects']) for result in results)
        total_orientations = sum(1 for result in results for obj in result['objects'] if obj['orientation'] is not None)
        
        print(f"\n=== 处理完成 ===")
        print(f"✅ 处理图像: {len(results)} 张")
        print(f"✅ 检测对象: {total_objects} 个")
        print(f"✅ 计算方向: {total_orientations} 个")
        
        # 显示每张图像的详细结果
        print(f"\n📊 详细结果:")
        for i, result in enumerate(results, 1):
            print(f"  {i}. {result['image_file']}: {len(result['objects'])} 个对象")
            for j, obj in enumerate(result['objects'], 1):
                if obj['orientation'] is not None:
                    print(f"     对象{j}: 方向 {obj['orientation']:.1f}度, 置信度 {obj['confidence']:.2f}")
                else:
                    print(f"     对象{j}: 方向计算失败, 置信度 {obj['confidence']:.2f}")
    else:
        print("❌ 没有找到可处理的图像或处理失败")


if __name__ == "__main__":
    main()
