"""
检测数据模型
"""
import numpy as np
from PyQt5.QtCore import QObject, pyqtSignal


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
                (self.camera_model.is_color_available() or self.camera_model.is_depth_available()))
    
    def process_detection(self, detection_type="both"):
        """处理检测
        
        Args:
            detection_type: 检测类型 ('color', 'depth', 'both')
        
        Returns:
            dict: 检测结果
        """
        try:
            if not self.detection_enabled:
                return {}
            
            if not self.is_camera_available():
                self.error_occurred.emit("摄像头不可用")
                return {}
            
            result = {}
            
            # 获取彩色图像并进行检测
            if detection_type in ['color', 'both']:
                color_image = self.get_current_color_image()
                if color_image is not None:
                    color_result = self._detect_objects_in_color(color_image)
                    result['color'] = color_result
            
            # 获取深度图像并进行检测
            if detection_type in ['depth', 'both']:
                depth_image = self.get_current_depth_image()
                if depth_image is not None:
                    depth_result = self._detect_objects_in_depth(depth_image)
                    result['depth'] = depth_result
            
            # 更新检测结果
            self.latest_detection_result = result
            self.detection_result_updated.emit(result)
            
            return result
            
        except Exception as e:
            self.error_occurred.emit(f"检测处理失败: {str(e)}")
            return {}
    
    def _detect_objects_in_color(self, color_image):
        """在彩色图像中检测对象"""
        try:
            if color_image is None:
                return {}
            
            height, width = color_image.shape[:2]
            
            # 模拟检测结果
            result = {
                'image_size': (width, height),
                'objects': [
                    {
                        'type': 'object1',
                        'confidence': 0.85,
                        'bbox': [100, 100, 200, 200],  # x, y, w, h
                        'center': [150, 150]
                    }
                ],
                'detection_time': 0.1  # 模拟检测时间
            }
            
            return result
            
        except Exception as e:
            self.error_occurred.emit(f"彩色图像检测失败: {str(e)}")
            return {}
    
    def _detect_objects_in_depth(self, depth_image):
        """在深度图像中检测对象"""
        try:
            if depth_image is None:
                return {}
            
            height, width = depth_image.shape[:2]
            
            # 模拟检测结果
            result = {
                'image_size': (width, height),
                'objects': [
                    {
                        'type': 'depth_object1',
                        'confidence': 0.78,
                        'bbox': [120, 80, 180, 220],  # x, y, w, h
                        'center': [210, 190],
                        'depth': 1.2  # 深度值（米）
                    }
                ],
                'detection_time': 0.12  # 模拟检测时间
            }
            
            return result
            
        except Exception as e:
            self.error_occurred.emit(f"深度图像检测失败: {str(e)}")
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
            stats['last_detection_time'] = max(stats['last_detection_time'], 
                                             result['color'].get('detection_time', 0))
        
        if 'depth' in result:
            depth_objects = len(result['depth'].get('objects', []))
            stats['depth_objects'] = depth_objects
            stats['total_objects'] += depth_objects
            stats['last_detection_time'] = max(stats['last_detection_time'], 
                                             result['depth'].get('detection_time', 0))
        
        return stats
    
    def detect_objects(self, image, image_type='color'):
        """
        在指定图像中检测对象
        
        Args:
            image (numpy.ndarray): 输入图像
            image_type (str): 图像类型 ('color' 或 'depth')
            
        Returns:
            list: 检测结果列表，格式为 [{'bbox': [x, y, w, h], 'class': '类名', 'confidence': 置信度}, ...]
        """
        try:
            if image is None:
                return []
            
            if image_type == 'color':
                result = self._detect_objects_in_color(image)
            elif image_type == 'depth':
                result = self._detect_objects_in_depth(image)
            else:
                return []
            
            # 将结果格式化为标准格式
            detections = []
            for obj in result.get('objects', []):
                detection = {
                    'bbox': obj.get('bbox', [0, 0, 0, 0]),
                    'class': obj.get('type', 'unknown'),
                    'confidence': obj.get('confidence', 0.0)
                }
                detections.append(detection)
            
            return detections
            
        except Exception as e:
            self.error_occurred.emit(f"对象检测失败: {str(e)}")
            return []
    