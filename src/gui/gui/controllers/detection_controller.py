"""
检测控制器
"""
from PyQt5.QtCore import pyqtSignal, QTimer
from .base_controller import BaseController


class DetectionController(BaseController):
    """检测控制器类"""
    
    # 定义信号
    detection_result_updated = pyqtSignal(dict)  # 检测结果更新信号
    detection_stats_updated = pyqtSignal(dict)   # 检测统计信息更新信号
    
    def __init__(self, detection_model, camera_model=None):
        super().__init__()
        self.detection_model = detection_model
        self.camera_model = camera_model
        self.auto_detection_timer = QTimer()
        self.auto_detection_timer.timeout.connect(self._auto_detect)
        self.auto_detection_enabled = False
        
        # 连接模型信号
        self._connect_model_signals()
        
        # 设置摄像头模型
        if camera_model:
            self.set_camera_model(camera_model)
    
    def _connect_model_signals(self):
        """连接模型信号"""
        self.detection_model.detection_result_updated.connect(self._on_detection_result_updated)
        self.detection_model.error_occurred.connect(self._on_detection_error)
    
    def set_camera_model(self, camera_model):
        """设置摄像头模型"""
        self.camera_model = camera_model
        self.detection_model.set_camera_model(camera_model)
        self.display("摄像头模型已设置", "检测")
    
    def _on_detection_result_updated(self, result):
        """检测结果更新处理"""
        self.detection_result_updated.emit(result)
        
        # 更新统计信息
        stats = self.detection_model.get_detection_stats()
        self.detection_stats_updated.emit(stats)
        
        # 记录检测结果
        if result:
            total_objects = stats.get('total_objects', 0)
            detection_time = stats.get('last_detection_time', 0)
            self.display(f"检测完成: 发现{total_objects}个对象, 耗时{detection_time:.3f}s", "检测")
    
    def _on_detection_error(self, error_msg):
        """检测错误处理"""
        self.display(f"检测错误: {error_msg}", "错误")
    
    def enable_detection(self):
        """启用检测"""
        try:
            success = self.detection_model.enable_detection()
            if success:
                self.display("检测功能已启用", "检测")
            else:
                self.display("启用检测功能失败", "错误")
            return success
        except Exception as e:
            self.display(f"启用检测异常: {str(e)}", "错误")
            return False
    
    def disable_detection(self):
        """禁用检测"""
        try:
            # 停止自动检测
            self.stop_auto_detection()
            
            self.detection_model.disable_detection()
            self.display("检测功能已禁用", "检测")
        except Exception as e:
            self.display(f"禁用检测异常: {str(e)}", "错误")
    
    def detect_once(self, detection_type="both"):
        """执行一次检测
        
        Args:
            detection_type: 检测类型 ('color', 'depth', 'both')
        
        Returns:
            dict: 检测结果
        """
        try:
            if not self.detection_model.detection_enabled:
                self.display("请先启用检测功能", "提示")
                return {}
            
            result = self.detection_model.process_detection(detection_type)
            return result
            
        except Exception as e:
            self.display(f"执行检测异常: {str(e)}", "错误")
            return {}
    
    def start_auto_detection(self, interval_ms=500, detection_type="both"):
        """开始自动检测
        
        Args:
            interval_ms: 检测间隔（毫秒）
            detection_type: 检测类型 ('color', 'depth', 'both')
        """
        try:
            if not self.detection_model.detection_enabled:
                self.display("请先启用检测功能", "提示")
                return False
            
            self.auto_detection_type = detection_type
            self.auto_detection_enabled = True
            self.auto_detection_timer.start(interval_ms)
            self.display(f"自动检测已启动，间隔{interval_ms}ms", "检测")
            return True
            
        except Exception as e:
            self.display(f"启动自动检测异常: {str(e)}", "错误")
            return False
    
    def stop_auto_detection(self):
        """停止自动检测"""
        try:
            self.auto_detection_timer.stop()
            self.auto_detection_enabled = False
            self.display("自动检测已停止", "检测")
        except Exception as e:
            self.display(f"停止自动检测异常: {str(e)}", "错误")
    
    def _auto_detect(self):
        """自动检测定时器回调"""
        if self.auto_detection_enabled:
            self.detect_once(getattr(self, 'auto_detection_type', 'both'))
    
    def get_current_color_image(self):
        """获取当前彩色图像"""
        return self.detection_model.get_current_color_image()
    
    def get_current_depth_image(self):
        """获取当前深度图像"""
        return self.detection_model.get_current_depth_image()
    
    def is_camera_available(self):
        """检查摄像头是否可用"""
        return self.detection_model.is_camera_available()
    
    def get_latest_detection_result(self):
        """获取最新检测结果"""
        return self.detection_model.get_latest_detection_result()
    
    def get_detection_stats(self):
        """获取检测统计信息"""
        return self.detection_model.get_detection_stats()
    
    def clear_detection_result(self):
        """清除检测结果"""
        try:
            self.detection_model.clear_detection_result()
            self.display("检测结果已清除", "检测")
        except Exception as e:
            self.display(f"清除检测结果异常: {str(e)}", "错误")
    
    def detect_color_only(self):
        """仅检测彩色图像"""
        return self.detect_once("color")
    
    def detect_depth_only(self):
        """仅检测深度图像"""
        return self.detect_once("depth")
    
    def detect_both(self):
        """检测彩色和深度图像"""
        return self.detect_once("both")
    
    def get_detection_status(self):
        """获取检测状态"""
        return {
            'detection_enabled': self.detection_model.detection_enabled,
            'auto_detection_enabled': self.auto_detection_enabled,
            'camera_available': self.is_camera_available(),
            'auto_detection_interval': self.auto_detection_timer.interval() if self.auto_detection_timer.isActive() else 0
        }
