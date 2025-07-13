"""
参数控制器
"""
from PyQt5.QtCore import QTimer, pyqtSignal
from .base_controller import BaseController
from gui.config.params import ParamConfig
import time


class ParamController(BaseController):
    """参数控制器类"""
    
    # 定义信号
    velocity_updated = pyqtSignal(float, float)  # 速度更新信号 (vx, vy)
    history_velocity_updated = pyqtSignal(float, float)  # 历史速度更新信号 (vx, vy)
    
    def __init__(self, detection_model):
        super().__init__()
        self.detection_model = detection_model
        self.is_collecting = False
        
        # 速度计算相关
        self.prev_center = None
        self.prev_time = None
        self.velocity_timer = QTimer()
        self.velocity_timer.timeout.connect(self._calculate_velocity)
        self.velocity_timer.setInterval(100)  # 100ms更新一次
        
        # 当前速度值
        self.current_velocity_x = 0.0
        self.current_velocity_y = 0.0
    
    def start_velocity_collection(self):
        """开始速度收集"""
        if not self.is_collecting:
            self.is_collecting = True
            self.prev_center = None
            self.prev_time = None
            self.current_velocity_x = 0.0
            self.current_velocity_y = 0.0
            self.velocity_timer.start()
            self.display("开始速度收集", "参数")
    
    def stop_velocity_collection(self):
        """停止速度收集"""
        if self.is_collecting:
            self.is_collecting = False
            self.velocity_timer.stop()
            self.current_velocity_x = 0.0
            self.current_velocity_y = 0.0
            self.velocity_updated.emit(0.0, 0.0)
            self.display("停止速度收集", "参数")
    
    def _calculate_velocity(self):
        """计算速度值"""
        if not self.is_collecting:
            return
        
        try:
            # 获取最新检测结果
            detection_result = self.detection_model.get_latest_detection_result()
            
            if detection_result and 'central_center' in detection_result:
                current_center = detection_result['central_center']
                current_time = time.time()
                
                if self.prev_center is not None and self.prev_time is not None:
                    # 计算位移
                    dx = current_center[0] - self.prev_center[0]
                    dy = current_center[1] - self.prev_center[1]
                    
                    # 计算时间差
                    dt = current_time - self.prev_time
                    
                    if dt > 0:
                        # 计算速度 (像素/秒)
                        vx = dx / dt
                        vy = dy / dt
                        
                        # 更新当前速度值
                        self.current_velocity_x = vx
                        self.current_velocity_y = vy
                        
                        # 发送速度更新信号
                        self.velocity_updated.emit(vx, vy)
                
                # 更新前一帧数据
                self.prev_center = current_center
                self.prev_time = current_time
            else:
                # 没有检测结果时，速度为0
                self.current_velocity_x = 0.0
                self.current_velocity_y = 0.0
                self.velocity_updated.emit(0.0, 0.0)
                
        except Exception as e:
            self.display(f"速度计算错误: {str(e)}", "错误")
    
    def get_current_velocity(self):
        """获取当前速度值"""
        return self.current_velocity_x, self.current_velocity_y
    
    def is_velocity_collecting(self):
        """检查是否正在收集速度"""
        return self.is_collecting
    
    def save_velocity_to_config(self):
        """保存当前速度值到配置文件"""
        try:
            # 更新全局配置
            ParamConfig.update_velocity(self.current_velocity_x, self.current_velocity_y)
            
            # 保存到文件
            if ParamConfig.save_config():
                self.display(f"速度参数已保存: Vx={self.current_velocity_x:.2f}, Vy={self.current_velocity_y:.2f}", "参数")
                
                # 发送历史速度更新信号
                self.history_velocity_updated.emit(self.current_velocity_x, self.current_velocity_y)
                
                return True
            else:
                self.display("保存速度参数失败", "错误")
                return False
                
        except Exception as e:
            self.display(f"保存速度参数时出错: {str(e)}", "错误")
            return False 