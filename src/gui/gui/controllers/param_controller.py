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
        self.start_center = None
        self.start_time = None
        self.end_center = None
        self.end_time = None
        self.monitoring_timer = QTimer()
        self.monitoring_timer.timeout.connect(self._monitor_detection)
        self.monitoring_timer.setInterval(100)  # 100ms检查一次检测结果
        
        # 当前速度值
        self.current_velocity_x = 0.0
        self.current_velocity_y = 0.0
    
    def start_velocity_collection(self):
        """开始速度收集"""
        if not self.is_collecting:
            self.is_collecting = True
            self.start_center = None
            self.start_time = None
            self.current_center = None
            self.end_center = None
            self.end_time = None
            self.current_velocity_x = 0.0
            self.current_velocity_y = 0.0
            
            # 启动监控定时器，等待有效的检测结果
            self.monitoring_timer.start()
            self.display("开始速度收集，等待检测结果...", "参数")
    
    def stop_velocity_collection(self):
        """停止速度收集"""
        if self.is_collecting:
            self.is_collecting = False
            self.monitoring_timer.stop()
            
            # 记录结束时间和位置
            self._record_end_position()
            
            # 计算速度
            self._calculate_final_velocity()
            
            self.display("速度收集已停止", "参数")
    
    def _monitor_detection(self):
        """监控检测结果，记录起始位置"""
        if not self.is_collecting:
            return
            
        try:
            # 获取最新检测结果
            detection_result = self.detection_model.get_latest_detection_result()
            
            if detection_result and 'central_center' in detection_result:
                self.current_center = detection_result['central_center']
                self.current_time = time.time()
                
                # 如果还没有记录起始位置，记录起始位置
                if self.start_center is None:
                    self.start_center = self.current_center
                    self.start_time = self.current_time
                    self.display(f"记录起始位置: ({self.current_center[0]:.1f}, {self.current_center[1]:.1f})", "参数")
                    
        except Exception as e:
            self.display(f"监控检测结果错误: {str(e)}", "错误")
    
    def _record_end_position(self):
        """记录结束位置"""
        try:
            # 获取最新检测结果
            detection_result = self.detection_model.get_latest_detection_result()
            
            if detection_result and 'central_center' in detection_result:
                self.end_center = detection_result['central_center']
                self.end_time = time.time()
                self.display(f"记录结束位置: ({self.end_center[0]:.1f}, {self.end_center[1]:.1f})", "参数")
            else:
                self.end_center = self.current_center
                self.end_time = self.current_time
                self.display(f"记录结束位置: ({self.end_center[0]:.1f}, {self.end_center[1]:.1f})", "参数")

        except Exception as e:
            self.display(f"记录结束位置错误: {str(e)}", "错误")
    
    def _calculate_final_velocity(self):
        """计算最终速度"""
        try:
            # 检查是否有有效的起始和结束数据
            if self.start_center is None or self.start_time is None:
                self.display("错误：未记录到起始位置", "错误")
                self.current_velocity_x = 0.0
                self.current_velocity_y = 0.0
                self.velocity_updated.emit(0.0, 0.0)
                return
                
            if self.end_center is None or self.end_time is None:
                self.display("错误：未记录到结束位置", "错误")
                self.current_velocity_x = 0.0
                self.current_velocity_y = 0.0
                self.velocity_updated.emit(0.0, 0.0)
                return
            
            # 计算时间差
            time_duration = self.end_time - self.start_time
            
            if time_duration <= 0:
                self.display("错误：时间间隔无效", "错误")
                self.current_velocity_x = 0.0
                self.current_velocity_y = 0.0
                self.velocity_updated.emit(0.0, 0.0)
                return
            
            # 计算位移
            dx = self.end_center[0] - self.start_center[0]
            dy = self.end_center[1] - self.start_center[1]
            
            # 计算平均速度 (像素/秒)
            self.current_velocity_x = dx / time_duration
            self.current_velocity_y = dy / time_duration
            
            # 显示结果
            self.display(f"测量完成 - 时长: {time_duration:.2f}s, 位移: ({dx:.1f}, {dy:.1f})px", "参数")
            self.display(f"计算速度: Vx={self.current_velocity_x:.2f} px/s, Vy={self.current_velocity_y:.2f} px/s", "参数")
            
            # 发送速度更新信号
            self.velocity_updated.emit(self.current_velocity_x, self.current_velocity_y)
            
        except Exception as e:
            self.display(f"计算速度错误: {str(e)}", "错误")
            self.current_velocity_x = 0.0
            self.current_velocity_y = 0.0
            self.velocity_updated.emit(0.0, 0.0)
    
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
