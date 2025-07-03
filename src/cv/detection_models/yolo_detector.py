#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO检测器模块
用于图像分割和目标检测
"""

from .base import *
import os
from ultralytics import YOLO
import time


class YOLODetector:
    """YOLO分割检测器"""
    
    def __init__(self, model_path='yolo11x-seg.pt', device=None):
        """
        初始化YOLO检测器
        
        Args:
            model_path (str): YOLO模型路径
            device (str): 设备选择，None为自动选择
        """
        self.model_path = model_path
        
        # 检查GPU可用性
        if device is None:
            self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        else:
            self.device = device
            
        print(f"使用设备: {self.device}")
        
        if self.device == 'cuda':
            print(f"GPU型号: {torch.cuda.get_device_name(0)}")
            print(f"GPU内存: {torch.cuda.get_device_properties(0).total_memory / 1024**3:.1f} GB")
        
        # 加载YOLO11分割模型并指定设备
        self.model = YOLO(model_path)
        self.model.to(self.device)
    
    def detect_single_image(self, image_path):
        """
        检测单张图像
        
        Args:
            image_path (str): 图像路径
            
        Returns:
            dict: 检测结果
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
            }
        """
        if not os.path.exists(image_path):
            print(f"图像文件不存在: {image_path}")
            return None
        
        img_file = os.path.basename(image_path)
        print(f"正在处理: {img_file}")
        
        # 读取原始图像
        original_img = cv2.imread(image_path)
        if original_img is None:
            print(f"无法读取图像: {image_path}")
            return None
            
        img_height, img_width = original_img.shape[:2]
        
        # 使用YOLO进行语义分割
        if self.device == 'cuda':
            with torch.autocast(device_type='cuda'):
                results = self.model(image_path, device=self.device)
        else:
            results = self.model(image_path, device=self.device)
        
        # 初始化检测结果
        detection_result = {
            'image_file': img_file,
            'image_path': image_path,
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
                        original_class_name = self.model.names[cls]
                        
                        print(f"    区域 {i+1}: {original_class_name} -> part, 置信度: {conf:.2f}")
                
                # 处理掩码数据
                masks = r.masks.data
                
                # 批量处理掩码，减少GPU-CPU数据传输
                if self.device == 'cuda':
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
                    if self.device != 'cuda':
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
                    
                    # 计算零件尖端方向
                    orientation, centroid, tip_vector = calculate_part_orientation(mask_resized)
                    
                    # 存储对象信息
                    obj_info = {
                        'class_name': 'part',
                        'confidence': float(boxes[i].conf) if boxes is not None and i < len(boxes) else 0.0,
                        'mask': mask_bool.copy(),
                        'orientation': orientation,
                        'centroid': centroid,
                        'tip_vector': tip_vector
                    }
                    detection_result['objects'].append(obj_info)
                    
                    if orientation is not None:
                        print(f"      零件尖端方向: {orientation:.1f}度")
                
                # 清理GPU内存
                if self.device == 'cuda':
                    torch.cuda.empty_cache()
                    
            else:
                print("  - 未检测到分割区域")
        
        # 处理完一张图片后清理GPU内存
        if self.device == 'cuda':
            torch.cuda.empty_cache()
            
        return detection_result
    
    def detect_batch_images(self, images_dir='./data/images'):
        """
        批量处理图像分割和方向识别
        
        Args:
            images_dir (str): 输入图像目录路径
            
        Returns:
            list: 所有图像的处理结果列表
        """
        start_time = time.time()
        
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
            result = self.detect_single_image(img_path)
            if result:
                all_results.append(result)
        
        # 处理完成统计
        total_time = time.time() - start_time
        total_objects = sum(len(result['objects']) for result in all_results)
        total_orientations = sum(1 for result in all_results for obj in result['objects'] if obj['orientation'] is not None)
        
        print(f"\n=== 语义分割和方向识别完成 ===")
        print(f"处理了 {len(all_results)} 张图像")
        print(f"检测到 {total_objects} 个目标对象")
        print(f"计算了 {total_orientations} 个方向角度")
        print(f"总用时: {total_time:.2f}秒")
        
        return all_results
    
    def save_results(self, results, results_dir='./data/images/result'):
        """
        保存识别结果到文件，并进行所有的可视化工作
        
        Args:
            results (list): 检测结果列表
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
                            obj['tip_vector'], 
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
    
    def process_and_save(self, images_dir='./data/images/origin_img', results_dir='./data/images/result'):
        """
        完整的检测和保存流程
        
        Args:
            images_dir (str): 输入图像目录
            results_dir (str): 结果保存目录
        """
        # 执行检测
        results = self.detect_batch_images(images_dir)
        
        if results:
            # 保存结果
            self.save_results(results, results_dir)
            
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
        
        return results

