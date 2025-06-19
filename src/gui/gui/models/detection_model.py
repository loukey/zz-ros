"""
检测数据模型
"""
import numpy as np
from PyQt5.QtCore import QObject, pyqtSignal
import cv2
import numpy as np
import torch
from ultralytics import YOLO
from sklearn.decomposition import PCA
import time


class DetectionModel(QObject):
    """检测数据模型类"""
    
    # 定义信号
    detection_result_updated = pyqtSignal(dict)  # 检测结果更新信号
    error_occurred = pyqtSignal(str)  # 错误信号
    
    def __init__(self, camera_model=None):
        super().__init__()
        self.camera_model = camera_model
        self.detection_enabled = False
        
        # 检测结果
        self.latest_detection_result = {}
        
        # 初始化YOLO11检测模型
        try:
            self.yolo_detector = YOLO11DetectionModel()
            print("YOLO11检测模型初始化成功")
        except Exception as e:
            print(f"YOLO11检测模型初始化失败: {str(e)}")
            self.yolo_detector = None
        
    def set_camera_model(self, camera_model):
        """设置摄像头模型"""
        self.camera_model = camera_model
    
    def enable_detection(self):
        """启用检测"""
        try:
            if not self.camera_model:
                self.error_occurred.emit("摄像头模型未设置")
                return False
            
            if not self.camera_model.is_camera_connected:
                self.error_occurred.emit("摄像头未连接")
                return False
            
            if not self.yolo_detector:
                self.error_occurred.emit("YOLO11检测模型未初始化")
                return False
            
            self.detection_enabled = True
            return True
            
        except Exception as e:
            self.error_occurred.emit(f"启用检测失败: {str(e)}")
            return False
    
    def disable_detection(self):
        """禁用检测"""
        self.detection_enabled = False
        
    def get_current_color_image(self):
        """获取当前彩色图像"""
        if self.camera_model and self.camera_model.is_camera_connected:
            return self.camera_model.get_latest_color_image()
        return None
    
    def get_current_depth_image(self):
        """获取当前深度图像"""
        if self.camera_model and self.camera_model.is_camera_connected:
            return self.camera_model.get_latest_depth_image()
        return None
    
    def is_camera_available(self):
        """检查摄像头是否可用"""
        return (self.camera_model and 
                self.camera_model.is_camera_connected and
                self.camera_model.is_color_available() and
                self.camera_model.is_depth_available())
    
    def get_depth_at_position(self, depth_image, x, y):
        """获取指定位置的深度值"""
        try:
            if depth_image is None:
                return None
            
            height, width = depth_image.shape[:2]
            
            # 确保坐标在有效范围内
            x = max(0, min(int(x), width - 1))
            y = max(0, min(int(y), height - 1))
            
            # 获取深度值
            depth_value = depth_image[y, x]
            
            # 如果深度值为0或无效，尝试获取周围像素的平均值
            if depth_value == 0 or np.isnan(depth_value):
                # 获取5x5区域的平均深度
                y_start = max(0, y - 2)
                y_end = min(height, y + 3)
                x_start = max(0, x - 2)
                x_end = min(width, x + 3)
                
                region = depth_image[y_start:y_end, x_start:x_end]
                valid_depths = region[region > 0]
                
                if len(valid_depths) > 0:
                    depth_value = np.mean(valid_depths)
                else:
                    depth_value = 0
            
            return float(depth_value)
            
        except Exception as e:
            print(f"获取深度值失败: {str(e)}")
            return None
    
    def process_detection(self, detection_type="color"):
        """处理检测 - 只检测彩色图像
        
        Args:
            detection_type: 检测类型，现在只支持 'color'
        
        Returns:
            dict: 检测结果
        """
        try:
            if not self.detection_enabled:
                return {}
            
            if not self.is_camera_available():
                self.error_occurred.emit("摄像头不可用")
                return {}
            
            if not self.yolo_detector:
                self.error_occurred.emit("YOLO11检测模型不可用")
                return {}
            
            # 只检测彩色图像
            color_image = self.get_current_color_image()
            depth_image = self.get_current_depth_image()
            
            if color_image is None:
                self.error_occurred.emit("无法获取彩色图像")
                return {}
            
            # 使用YOLO11进行检测
            start_time = time.time()
            detections = self.yolo_detector.detect_objects(color_image)
            detection_time = time.time() - start_time
            
            # 处理检测结果，添加深度信息
            processed_objects = []
            for detection in detections:
                obj = {
                    'type': detection.get('class', 'unknown'),
                    'confidence': detection.get('confidence', 0.0),
                    'bbox': detection.get('bbox', [0, 0, 0, 0]),
                    'centroid': detection.get('centroid', [0, 0]).tolist() if hasattr(detection.get('centroid', [0, 0]), 'tolist') else detection.get('centroid', [0, 0]),
                    'orientation': detection.get('orientation', 0.0),
                    'direction_vector': detection.get('direction_vector', [1, 0]).tolist() if hasattr(detection.get('direction_vector', [1, 0]), 'tolist') else detection.get('direction_vector', [1, 0])
                }
                
                # 计算中心点（如果没有centroid，从bbox计算）
                if 'centroid' not in detection or detection['centroid'] is None:
                    bbox = detection.get('bbox', [0, 0, 0, 0])
                    if len(bbox) >= 4:
                        center_x = bbox[0] + bbox[2] / 2
                        center_y = bbox[1] + bbox[3] / 2
                        obj['centroid'] = [center_x, center_y]
                
                # 获取对应位置的深度值
                if depth_image is not None and obj['centroid']:
                    center_x, center_y = obj['centroid']
                    depth_value = self.get_depth_at_position(depth_image, center_x, center_y)
                    obj['depth'] = depth_value
                else:
                    obj['depth'] = None
                
                processed_objects.append(obj)
            
            # 构建结果
            result = {
                'color': {
                    'image_size': (color_image.shape[1], color_image.shape[0]),
                    'objects': processed_objects,
                    'detection_time': detection_time,
                    'total_objects': len(processed_objects)
                }
            }
            
            # 更新检测结果
            self.latest_detection_result = result
            self.detection_result_updated.emit(result)
            
            return result
            
        except Exception as e:
            self.error_occurred.emit(f"检测处理失败: {str(e)}")
            return {}
    
    def _detect_objects_in_color(self, color_image):
        """在彩色图像中检测对象 - 已弃用，使用process_detection代替"""
        # 为了保持兼容性，调用新的检测方法
        result = self.process_detection("color")
        return result.get('color', {})
    
    def _detect_objects_in_depth(self, depth_image):
        """在深度图像中检测对象 - 已弃用，不再使用"""
        return {}
    
    def get_latest_detection_result(self):
        """获取最新检测结果"""
        return self.latest_detection_result
    
    def clear_detection_result(self):
        """清除检测结果"""
        self.latest_detection_result = {}
        self.detection_result_updated.emit({})
    
    def get_detection_stats(self):
        """获取检测统计信息"""
        result = self.get_latest_detection_result()
        
        stats = {
            'total_objects': 0,
            'color_objects': 0,
            'depth_objects': 0,
            'last_detection_time': 0
        }
        
        if 'color' in result:
            color_objects = len(result['color'].get('objects', []))
            stats['color_objects'] = color_objects
            stats['total_objects'] += color_objects
            stats['last_detection_time'] = result['color'].get('detection_time', 0)
        
        return stats
    
    def detect_objects(self, image, image_type='color'):
        """
        在指定图像中检测对象
        
        Args:
            image (numpy.ndarray): 输入图像
            image_type (str): 图像类型，现在只支持 'color'
            
        Returns:
            list: 检测结果列表，格式为 [{'bbox': [x, y, w, h], 'class': '类名', 'confidence': 置信度}, ...]
        """
        try:
            if image is None or image_type != 'color':
                return []
            
            if not self.yolo_detector:
                return []
            
            # 使用YOLO11检测
            detections = self.yolo_detector.detect_objects(image)
            
            # 将结果格式化为标准格式
            formatted_detections = []
            for detection in detections:
                formatted_detection = {
                    'bbox': detection.get('bbox', [0, 0, 0, 0]),
                    'class': detection.get('class', 'unknown'),
                    'confidence': detection.get('confidence', 0.0),
                    'centroid': detection.get('centroid', [0, 0]),
                    'orientation': detection.get('orientation', 0.0)
                }
                formatted_detections.append(formatted_detection)
            
            return formatted_detections
            
        except Exception as e:
            self.error_occurred.emit(f"对象检测失败: {str(e)}")
            return []


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
        try:
            with torch.amp.autocast(device_type=self.device) if self.device == 'cuda' else torch.no_grad():
                results = self.model(image, device=self.device)
            
            detections = []
            for r in results:
                if r.masks is not None and r.boxes is not None:
                    boxes = r.boxes
                    masks = r.masks.data
                    
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
            
        except Exception as e:
            print(f"YOLO11检测失败: {str(e)}")
            return []
