"""
显示视图模型
"""
from typing import List, Dict, Any, Optional
from PyQt5.QtCore import pyqtSignal, QTimer
from .base_view_model import BaseViewModel
from application.services.robot_application_service import RobotApplicationService
from shared.config.container import ServiceLocator
from datetime import datetime


class DisplayViewModel(BaseViewModel):
    """显示视图模型"""
    
    # 显示特有信号
    message_received = pyqtSignal(str, str)  # message, message_type
    data_updated = pyqtSignal(dict)  # 数据更新
    log_message = pyqtSignal(str, str, str)  # timestamp, level, message
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # 获取Application Service
        self.robot_service = ServiceLocator.resolve(RobotApplicationService)
        
        # UI状态
        self.message_history = []  # 消息历史
        self.current_data = {}  # 当前显示的数据
        self.max_history_size = 1000  # 最大历史记录数
        
        # 数据更新定时器
        self.data_timer = QTimer()
        self.data_timer.timeout.connect(self._update_robot_data)
        self.auto_update_enabled = False
        self.update_interval = 100  # 更新间隔（毫秒）
    
    def add_message(self, message: str, message_type: str = "info", 
                   timestamp: Optional[str] = None) -> None:
        """添加消息到显示"""
        try:
            if timestamp is None:
                timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            
            # 添加到历史记录
            message_entry = {
                'timestamp': timestamp,
                'type': message_type,
                'message': message
            }
            
            self.message_history.append(message_entry)
            
            # 限制历史记录大小
            if len(self.message_history) > self.max_history_size:
                self.message_history.pop(0)
            
            # 发出信号
            self.message_received.emit(message, message_type)
            self.log_message.emit(timestamp, message_type, message)
            
        except Exception as e:
            print(f"添加消息失败: {str(e)}")
    
    def display_robot_data(self, data_dict: Dict[str, Any]) -> None:
        """显示机器人数据"""
        try:
            self.current_data.update(data_dict)
            self.data_updated.emit(self.current_data.copy())
            
        except Exception as e:
            self.emit_error(f"显示机器人数据失败: {str(e)}")
    
    def start_auto_update(self, interval_ms: int = 100) -> bool:
        """开始自动更新数据"""
        try:
            if self.auto_update_enabled:
                self.emit_status("数据自动更新已启动")
                return True
            
            self.update_interval = interval_ms
            self.data_timer.start(interval_ms)
            self.auto_update_enabled = True
            
            self.emit_status(f"开始自动更新数据，间隔: {interval_ms}ms")
            return True
            
        except Exception as e:
            self.emit_error(f"启动自动更新失败: {str(e)}")
            return False
    
    def stop_auto_update(self) -> bool:
        """停止自动更新数据"""
        try:
            if not self.auto_update_enabled:
                self.emit_status("数据自动更新未启动")
                return True
            
            self.data_timer.stop()
            self.auto_update_enabled = False
            
            self.emit_status("已停止自动更新数据")
            return True
            
        except Exception as e:
            self.emit_error(f"停止自动更新失败: {str(e)}")
            return False
    
    def _update_robot_data(self):
        """更新机器人数据（定时调用）"""
        try:
            # 获取机器人状态
            robot_state = self.robot_service.get_robot_state()
            
            if robot_state:
                # 构建显示数据
                display_data = {
                    'joint_angles': [f"{angle:.2f}°" for angle in [
                        180 * a / 3.14159 for a in robot_state.joint_angles
                    ]],
                    'position': robot_state.position if robot_state.position else [0, 0, 0],
                    'is_moving': robot_state.is_moving,
                    'is_connected': robot_state.is_connected,
                    'last_updated': robot_state.last_updated.strftime("%H:%M:%S") if robot_state.last_updated else "N/A"
                }
                
                # 添加速度和力矩信息（如果有）
                if robot_state.velocities:
                    display_data['velocities'] = [f"{v:.3f}" for v in robot_state.velocities]
                
                if robot_state.torques:
                    display_data['torques'] = [f"{t:.3f}" for t in robot_state.torques]
                
                self.display_robot_data(display_data)
            
        except Exception as e:
            # 不在定时器中发出错误，避免过多错误消息
            pass
    
    def get_message_history(self, message_type: Optional[str] = None, 
                          count: Optional[int] = None) -> List[Dict[str, str]]:
        """获取消息历史"""
        try:
            history = self.message_history.copy()
            
            # 按类型过滤
            if message_type:
                history = [msg for msg in history if msg['type'] == message_type]
            
            # 限制数量
            if count:
                history = history[-count:]
            
            return history
            
        except Exception as e:
            self.emit_error(f"获取消息历史失败: {str(e)}")
            return []
    
    def clear_messages(self, message_type: Optional[str] = None) -> bool:
        """清除消息"""
        try:
            if message_type:
                # 清除特定类型的消息
                self.message_history = [
                    msg for msg in self.message_history 
                    if msg['type'] != message_type
                ]
                self.emit_status(f"已清除 {message_type} 类型的消息")
            else:
                # 清除所有消息
                self.message_history.clear()
                self.emit_status("已清除所有消息")
            
            return True
            
        except Exception as e:
            self.emit_error(f"清除消息失败: {str(e)}")
            return False
    
    def get_current_data(self) -> Dict[str, Any]:
        """获取当前显示数据"""
        return self.current_data.copy()
    
    def set_max_history_size(self, size: int) -> bool:
        """设置最大历史记录大小"""
        try:
            if size <= 0:
                self.emit_error("历史记录大小必须大于0")
                return False
            
            self.max_history_size = size
            
            # 如果当前历史超过新限制，截断
            if len(self.message_history) > size:
                self.message_history = self.message_history[-size:]
            
            self.emit_status(f"最大历史记录大小设置为: {size}")
            return True
            
        except Exception as e:
            self.emit_error(f"设置历史记录大小失败: {str(e)}")
            return False
    
    def is_auto_update_enabled(self) -> bool:
        """检查是否启用自动更新"""
        return self.auto_update_enabled
    
    def get_update_interval(self) -> int:
        """获取更新间隔"""
        return self.update_interval
    
    def cleanup(self):
        """清理资源"""
        super().cleanup()
        if self.data_timer.isActive():
            self.data_timer.stop()
        self.message_history.clear()
        self.current_data.clear() 
        