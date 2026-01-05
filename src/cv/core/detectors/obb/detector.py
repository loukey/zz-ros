import os
import cv2
import numpy as np
import torch
from ultralytics import YOLO
from typing import Dict, Any, List, Union, Tuple
from datetime import datetime

from .types import OBBDetection
from .post_processor import RobotPoseProcessor
from .visualizer import OBBVisualizer

class YOLOOBBDetector:
    """
    YOLO11-OBB 检测器
    集成模型推理、几何计算和可视化功能
    """
    
    def __init__(self, model_path: str, conf_threshold: float = 0.001, iou_threshold: float = 0.45):
        self.model_path = model_path
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        
        # 初始化组件
        self.processor = RobotPoseProcessor()
        self.visualizer = OBBVisualizer()
        
        self.model = None
        self.class_names = []
        self._load_model()
        
    def _load_model(self):
        """加载 YOLO 模型"""
        try:
            self.model = YOLO(self.model_path)
            if hasattr(self.model, 'names'):
                self.class_names = list(self.model.names.values())
            print(f"✅ 模型加载成功: {self.model_path} ({self.device})")
        except Exception as e:
            print(f"❌ 模型加载失败: {e}")
            raise

    def detect_image(self, image: Union[str, np.ndarray], 
                    save_result: bool = True, 
                    output_dir: str = "./results") -> Dict[str, Any]:
        """
        对单张图像进行检测
        
        Args:
            image: 图像路径或 numpy 数组
            save_result: 是否保存可视化结果
            output_dir: 结果保存目录
        """
        # 1. 准备图像
        if isinstance(image, str):
            if not os.path.exists(image):
                raise FileNotFoundError(f"图像不存在: {image}")
            img_path = image
            img = cv2.imread(image)
            base_name = os.path.basename(image)
        else:
            img_path = "memory_image"
            img = image.copy()
            base_name = f"image_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
            
        if img is None:
            raise ValueError("无法读取图像")
            
        # 2. 模型推理
        raw_results = self.model(img, conf=self.conf_threshold, iou=self.iou_threshold, verbose=False)
        
        # 3. 解析结果为标准数据结构
        detections = self._parse_results(raw_results)
        
        # 4. 业务后处理 (计算 Head+Central 关系)
        pose_result = self.processor.process(detections)
        
        # 5. 可视化
        vis_img = None
        if save_result:
            vis_img = img.copy()
            # 绘制所有检测框
            for det in detections:
                vis_img = self.visualizer.draw_detection(vis_img, det)
            
            # 绘制姿态信息 (如果有)
            if pose_result:
                vis_img = self.visualizer.draw_pose(vis_img, pose_result)
                
            # 保存
            os.makedirs(output_dir, exist_ok=True)
            output_path = os.path.join(output_dir, f"result_{base_name}")
            cv2.imwrite(output_path, vis_img)
            
        # 6. 构造返回结果
        return {
            'image_shape': img.shape,
            'detections': detections,
            'pose_result': pose_result,
            'vis_image': vis_img
        }

    def _parse_results(self, raw_results) -> List[OBBDetection]:
        """将 YOLO 原始结果转换为 OBBDetection 列表"""
        detections = []
        for result in raw_results:
            # 优先处理 OBB 结果
            if result.obb is not None:
                obb = result.obb
                for i in range(len(obb)):
                    coords = obb.xyxyxyxy[i].cpu().numpy() # 4x2
                    corners = coords.reshape(4, 2)
                    
                    # 计算几何中心
                    center_x = corners[:, 0].mean()
                    center_y = corners[:, 1].mean()
                    
                    cls_id = int(obb.cls[i].item())
                    conf = float(obb.conf[i].item())
                    cls_name = self.class_names[cls_id] if cls_id < len(self.class_names) else str(cls_id)
                    
                    detections.append(OBBDetection(
                        class_id=cls_id,
                        class_name=cls_name,
                        confidence=conf,
                        corners=corners,
                        center=np.array([center_x, center_y]),
                        raw_coords=coords
                    ))
            
            # 兼容普通 Box 结果 (转换为 OBB)
            elif result.boxes is not None:
                boxes = result.boxes
                for i in range(len(boxes)):
                    x1, y1, x2, y2 = boxes.xyxy[i].cpu().numpy()
                    cls_id = int(boxes.cls[i].item())
                    conf = float(boxes.conf[i].item())
                    cls_name = self.class_names[cls_id] if cls_id < len(self.class_names) else str(cls_id)
                    
                    corners = np.array([
                        [x1, y1], [x2, y1], [x2, y2], [x1, y2]
                    ])
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2
                    
                    detections.append(OBBDetection(
                        class_id=cls_id,
                        class_name=cls_name,
                        confidence=conf,
                        corners=corners,
                        center=np.array([center_x, center_y])
                    ))
                    
        return detections

