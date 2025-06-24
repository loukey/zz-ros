import torch
import cv2
import numpy as np
from ultralytics import YOLO
from sklearn.decomposition import PCA
import os


class YOLO11DetectionModel():
    def __init__(self):
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model = YOLO('yolo11x-seg.pt')
        self.model.to(self.device)
        
    def calculate_part_orientation(self, mask):
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

    def draw_orientation_arrow(self, image, centroid, direction_vector, angle, length=100):
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

    def detect_objects(self, image):
        """检测图像中的对象"""
        with torch.amp.autocast(device_type=self.device) if self.device == 'cuda' else torch.no_grad():
            results = self.model(image, device=self.device)
        
        detections = []
        for r in results:
            if r.masks is not None and r.boxes is not None:
                boxes = r.boxes
                masks = r.masks.data
                print(masks)
                print(boxes)
                # 调整mask尺寸到原图大小
                if self.device == 'cuda':
                    target_size = (image.shape[1], image.shape[0])
                    masks_resized = torch.nn.functional.interpolate(
                        masks.unsqueeze(1).float(), 
                        size=target_size, 
                        mode='bilinear', 
                        align_corners=False
                    ).squeeze(1)
                    masks_cpu = masks_resized.cpu().numpy()
                else:
                    masks_cpu = masks.cpu().numpy()
                
                for i, (box, mask) in enumerate(zip(boxes, masks_cpu)):
                    cls = int(box.cls)
                    conf = float(box.conf)
                    class_name = self.model.names[cls]
                    
                    # 调整mask尺寸
                    mask_resized = cv2.resize(mask, (image.shape[1], image.shape[0]))
                    
                    # 计算方向
                    orientation, centroid, direction_vector = self.calculate_part_orientation(mask_resized)
                    
                    # 从box获取bbox信息
                    x1, y1, x2, y2 = [float(coord) for coord in box.xyxy[0]]
                    bbox = [x1, y1, x2 - x1, y2 - y1]  # x, y, w, h
                    
                    detection = {
                        'bbox': bbox,
                        'class': class_name,
                        'confidence': conf,
                        'orientation': orientation if orientation is not None else 0.0,
                        'centroid': centroid if centroid is not None else [(x1 + x2) / 2, (y1 + y2) / 2],
                        'direction_vector': direction_vector if direction_vector is not None else [1, 0],
                    }
                    detections.append(detection)
            
            # 如果没有分割结果，只使用边界框
            elif r.boxes is not None:
                boxes = r.boxes
                for box in boxes:
                    cls = int(box.cls)
                    conf = float(box.conf)
                    class_name = self.model.names[cls]
                    
                    # 从box获取bbox信息
                    x1, y1, x2, y2 = [float(coord) for coord in box.xyxy[0]]
                    bbox = [x1, y1, x2 - x1, y2 - y1]  # x, y, w, h
                    centroid = [(x1 + x2) / 2, (y1 + y2) / 2]
                    
                    detection = {
                        'bbox': bbox,
                        'class': class_name,
                        'confidence': conf,
                        'orientation': 0.0,
                        'centroid': centroid,
                        'direction_vector': [1, 0],
                    }
                    detections.append(detection)

        if self.device == 'cuda':
            torch.cuda.empty_cache()

        return detections
        

yolo_model = YOLO11DetectionModel()
img_path = os.path.join(os.path.dirname(__file__), "data", "images", "img1.jpg")
img = cv2.imread(img_path)
detections = yolo_model.detect_objects(img)
print(detections)
