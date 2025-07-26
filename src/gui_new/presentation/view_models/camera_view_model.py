"""
相机视图模型
"""
from typing import List, Dict, Any, Optional, Tuple
from PyQt5.QtCore import pyqtSignal, QTimer
from PyQt5.QtGui import QPixmap
from .base_view_model import BaseViewModel
from application.services.robot_application_service import RobotApplicationService
from shared.config.container import ServiceLocator
import numpy as np


class CameraViewModel(BaseViewModel):
    """相机视图模型"""
    
    # 相机特有信号
    frame_captured = pyqtSignal(object)  # 捕获的帧数据
    detection_completed = pyqtSignal(list, dict)  # 检测结果 [检测对象], 统计信息
    camera_status_changed = pyqtSignal(str)  # 相机状态变化
    settings_updated = pyqtSignal(dict)  # 相机设置更新
    pose_published = pyqtSignal(dict)  # ROS位姿发布
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # 获取Application Service
        self.robot_service = ServiceLocator.resolve(RobotApplicationService)
        
        # 相机状态
        self.is_camera_connected = False
        self.is_capturing = False
        self.camera_status = "未连接"
        self.current_frame = None
        
        # 相机设置
        self.camera_settings = {
            'resolution': (640, 480),
            'fps': 30,
            'exposure': 0,
            'brightness': 0,
            'contrast': 0,
            'saturation': 0,
            'gamma': 100,
            'white_balance': 'auto'
        }
        
        # 检测设置
        self.detection_enabled = False
        self.detection_mode = 'color'  # 'color', 'depth', 'combined'
        self.detection_objects = ['red', 'green', 'blue', 'custom']
        
        # 颜色阈值设置
        self.color_thresholds = {
            'red': {'hue': [0, 10], 'saturation': [120, 255], 'value': [70, 255]},
            'green': {'hue': [40, 80], 'saturation': [120, 255], 'value': [70, 255]},
            'blue': {'hue': [100, 130], 'saturation': [120, 255], 'value': [70, 255]},
            'custom': {'hue': [0, 180], 'saturation': [0, 255], 'value': [0, 255]}
        }
        
        # ROS设置
        self.ros_enabled = False
        self.pose_topic = '/detected_objects_pose'
        
        # 检测结果
        self.last_detection_results = []
        self.detection_statistics = {
            'total_objects': 0,
            'objects_by_color': {},
            'average_confidence': 0.0
        }
        
        # 捕获定时器
        self.capture_timer = QTimer()
        self.capture_timer.timeout.connect(self._capture_frame)
    
    def connect_camera(self, camera_index: int = 0) -> bool:
        """连接相机"""
        try:
            # TODO: 当Infrastructure层实现后，这里调用真实的相机连接服务
            # 目前使用模拟连接
            self.is_camera_connected = True
            self.camera_status = "已连接"
            self.camera_status_changed.emit("已连接")
            
            self.emit_status(f"相机 {camera_index} 连接成功")
            return True
            
        except Exception as e:
            self.emit_error(f"连接相机失败: {str(e)}")
            self.camera_status = "连接失败"
            self.camera_status_changed.emit("连接失败")
            return False
    
    def disconnect_camera(self) -> bool:
        """断开相机连接"""
        try:
            # 停止捕获
            if self.is_capturing:
                self.stop_capture()
            
            # TODO: 当Infrastructure层实现后，这里调用真实的相机断开服务
            self.is_camera_connected = False
            self.camera_status = "未连接"
            self.camera_status_changed.emit("未连接")
            
            self.emit_status("相机已断开连接")
            return True
            
        except Exception as e:
            self.emit_error(f"断开相机连接失败: {str(e)}")
            return False
    
    def start_capture(self, fps: Optional[int] = None) -> bool:
        """开始捕获画面"""
        if not self.is_camera_connected:
            self.emit_error("相机未连接")
            return False
        
        if self.is_capturing:
            self.emit_status("相机已在捕获中")
            return True
        
        try:
            capture_fps = fps or self.camera_settings['fps']
            interval_ms = int(1000 / capture_fps)
            
            self.capture_timer.start(interval_ms)
            self.is_capturing = True
            self.camera_status = "捕获中"
            self.camera_status_changed.emit("捕获中")
            
            self.emit_status(f"开始捕获画面，帧率: {capture_fps} FPS")
            return True
            
        except Exception as e:
            self.emit_error(f"开始捕获失败: {str(e)}")
            return False
    
    def stop_capture(self) -> bool:
        """停止捕获画面"""
        try:
            if self.capture_timer.isActive():
                self.capture_timer.stop()
            
            self.is_capturing = False
            self.camera_status = "已连接" if self.is_camera_connected else "未连接"
            self.camera_status_changed.emit(self.camera_status)
            
            self.emit_status("已停止捕获画面")
            return True
            
        except Exception as e:
            self.emit_error(f"停止捕获失败: {str(e)}")
            return False
    
    def _capture_frame(self):
        """捕获一帧画面（定时调用）"""
        try:
            # TODO: 当Infrastructure层实现后，这里获取真实的相机帧
            # 目前生成模拟帧数据
            width, height = self.camera_settings['resolution']
            
            # 生成模拟帧（随机彩色图像）
            frame_data = {
                'timestamp': self._get_timestamp(),
                'width': width,
                'height': height,
                'format': 'RGB',
                'data': np.random.randint(0, 255, (height, width, 3), dtype=np.uint8)
            }
            
            self.current_frame = frame_data
            self.frame_captured.emit(frame_data)
            
            # 如果启用检测，执行检测
            if self.detection_enabled and self.current_frame:
                self._perform_detection(frame_data)
                
        except Exception as e:
            # 在定时器中不显示错误，避免过多错误消息
            pass
    
    def _perform_detection(self, frame_data: Dict[str, Any]):
        """执行目标检测"""
        try:
            # TODO: 当Domain层视觉检测服务实现后，调用真实的检测算法
            # 目前使用模拟检测
            detections = []
            
            # 模拟检测结果
            for color in self.detection_objects:
                if np.random.random() > 0.7:  # 30%概率检测到物体
                    detection = {
                        'color': color,
                        'center': [
                            np.random.randint(50, frame_data['width'] - 50),
                            np.random.randint(50, frame_data['height'] - 50)
                        ],
                        'area': np.random.randint(100, 5000),
                        'confidence': np.random.uniform(0.7, 0.98),
                        'bounding_box': self._generate_bounding_box(frame_data),
                        'timestamp': frame_data['timestamp']
                    }
                    detections.append(detection)
            
            # 更新检测结果
            self.last_detection_results = detections
            self._update_detection_statistics(detections)
            
            # 发出检测完成信号
            self.detection_completed.emit(detections, self.detection_statistics.copy())
            
            # 如果启用ROS，发布位姿信息
            if self.ros_enabled and detections:
                self._publish_pose_to_ros(detections)
                
        except Exception as e:
            self.emit_error(f"目标检测失败: {str(e)}")
    
    def _generate_bounding_box(self, frame_data: Dict[str, Any]) -> List[int]:
        """生成模拟边界框"""
        x = np.random.randint(0, frame_data['width'] - 100)
        y = np.random.randint(0, frame_data['height'] - 100)
        w = np.random.randint(50, min(100, frame_data['width'] - x))
        h = np.random.randint(50, min(100, frame_data['height'] - y))
        return [x, y, w, h]
    
    def _update_detection_statistics(self, detections: List[Dict[str, Any]]):
        """更新检测统计信息"""
        try:
            self.detection_statistics['total_objects'] = len(detections)
            
            # 按颜色统计
            color_counts = {}
            total_confidence = 0.0
            
            for detection in detections:
                color = detection['color']
                color_counts[color] = color_counts.get(color, 0) + 1
                total_confidence += detection['confidence']
            
            self.detection_statistics['objects_by_color'] = color_counts
            self.detection_statistics['average_confidence'] = (
                total_confidence / len(detections) if detections else 0.0
            )
            
        except Exception as e:
            self.emit_error(f"更新检测统计失败: {str(e)}")
    
    def _publish_pose_to_ros(self, detections: List[Dict[str, Any]]):
        """发布检测结果到ROS（模拟）"""
        try:
            for detection in detections:
                # 转换像素坐标为3D位姿（需要相机标定参数）
                # 这里使用简化转换
                pixel_x, pixel_y = detection['center']
                
                # 模拟3D位姿计算
                pose_data = {
                    'object_id': detection['color'],
                    'position': {
                        'x': (pixel_x - self.camera_settings['resolution'][0] / 2) * 0.001,  # mm转m
                        'y': (pixel_y - self.camera_settings['resolution'][1] / 2) * 0.001,
                        'z': 0.5  # 假设固定深度
                    },
                    'orientation': {
                        'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0  # 无旋转
                    },
                    'confidence': detection['confidence'],
                    'timestamp': detection['timestamp']
                }
                
                self.pose_published.emit(pose_data)
                
        except Exception as e:
            self.emit_error(f"发布ROS位姿失败: {str(e)}")
    
    def set_camera_settings(self, settings: Dict[str, Any]) -> bool:
        """设置相机参数"""
        try:
            # 验证设置
            valid_keys = self.camera_settings.keys()
            for key in settings:
                if key not in valid_keys:
                    self.emit_error(f"无效的相机设置: {key}")
                    return False
            
            # 更新设置
            self.camera_settings.update(settings)
            self.settings_updated.emit(self.camera_settings.copy())
            
            # TODO: 当Infrastructure层实现后，应用到真实相机
            self.emit_status(f"相机设置已更新: {settings}")
            return True
            
        except Exception as e:
            self.emit_error(f"设置相机参数失败: {str(e)}")
            return False
    
    def set_detection_mode(self, mode: str, objects: List[str]) -> bool:
        """设置检测模式和对象"""
        try:
            if mode not in ['color', 'depth', 'combined']:
                self.emit_error("检测模式必须是 'color', 'depth' 或 'combined'")
                return False
            
            self.detection_mode = mode
            self.detection_objects = objects.copy()
            
            self.emit_status(f"检测模式设置为: {mode}, 检测对象: {objects}")
            return True
            
        except Exception as e:
            self.emit_error(f"设置检测模式失败: {str(e)}")
            return False
    
    def set_color_thresholds(self, color: str, thresholds: Dict[str, List[int]]) -> bool:
        """设置颜色阈值"""
        try:
            if color not in self.color_thresholds:
                self.emit_error(f"未知颜色: {color}")
                return False
            
            # 验证阈值格式
            required_keys = ['hue', 'saturation', 'value']
            for key in required_keys:
                if key not in thresholds or len(thresholds[key]) != 2:
                    self.emit_error(f"阈值 {key} 必须包含2个值 [min, max]")
                    return False
            
            self.color_thresholds[color] = thresholds.copy()
            
            self.emit_status(f"{color} 颜色阈值已更新")
            return True
            
        except Exception as e:
            self.emit_error(f"设置颜色阈值失败: {str(e)}")
            return False
    
    def enable_detection(self, enabled: bool) -> bool:
        """启用/禁用检测"""
        try:
            self.detection_enabled = enabled
            status = "启用" if enabled else "禁用"
            self.emit_status(f"目标检测已{status}")
            return True
            
        except Exception as e:
            self.emit_error(f"切换检测状态失败: {str(e)}")
            return False
    
    def enable_ros(self, enabled: bool, topic: Optional[str] = None) -> bool:
        """启用/禁用ROS发布"""
        try:
            self.ros_enabled = enabled
            if topic:
                self.pose_topic = topic
            
            status = "启用" if enabled else "禁用"
            self.emit_status(f"ROS位姿发布已{status}")
            return True
            
        except Exception as e:
            self.emit_error(f"切换ROS状态失败: {str(e)}")
            return False
    
    def get_camera_status(self) -> Dict[str, Any]:
        """获取相机状态"""
        return {
            'connected': self.is_camera_connected,
            'capturing': self.is_capturing,
            'status': self.camera_status,
            'settings': self.camera_settings.copy(),
            'detection_enabled': self.detection_enabled,
            'detection_mode': self.detection_mode,
            'ros_enabled': self.ros_enabled
        }
    
    def get_detection_results(self) -> Tuple[List[Dict], Dict[str, Any]]:
        """获取最新检测结果"""
        return self.last_detection_results.copy(), self.detection_statistics.copy()
    
    def _get_timestamp(self) -> str:
        """获取时间戳"""
        from datetime import datetime
        return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    
    def cleanup(self):
        """清理资源"""
        super().cleanup()
        if self.is_capturing:
            self.stop_capture()
        if self.is_camera_connected:
            self.disconnect_camera() 