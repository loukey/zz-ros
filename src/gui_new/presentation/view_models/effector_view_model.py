"""
末端执行器视图模型
"""
from typing import Dict, Any, Optional
from PyQt5.QtCore import pyqtSignal
from .base_view_model import BaseViewModel
from application.services.robot_application_service import RobotApplicationService
from shared.config.container import ServiceLocator


class EffectorViewModel(BaseViewModel):
    """末端执行器视图模型"""
    
    # 末端执行器特有信号
    connection_status_changed = pyqtSignal(bool)
    effector_mode_changed = pyqtSignal(int)  # 执行器模式变化
    effector_data_changed = pyqtSignal(float)  # 执行器数据变化
    effector_status_changed = pyqtSignal(str)  # 执行器状态变化
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # 获取Application Service
        self.robot_service = ServiceLocator.resolve(RobotApplicationService)
        
        # UI状态
        self.is_connected = False
        self.current_mode = 0x00  # 当前执行器模式
        self.current_data = 0.0   # 当前执行器数据
        self.effector_status = "停止"  # 执行器状态
        
        # 模式映射
        self.mode_mapping = {
            "停止": 0x00,
            "夹取": 0x01,
            "释放": 0x02,
            "吸取": 0x03,
            "放置": 0x04,
            "旋转": 0x05
        }
        
        # 状态映射
        self.status_mapping = {
            0x00: "停止",
            0x01: "运行中",
            0x02: "完成", 
            0x03: "错误"
        }
    
    def set_effector_mode(self, mode_name: str) -> bool:
        """设置执行器模式"""
        try:
            if mode_name not in self.mode_mapping:
                self.emit_error(f"无效的执行器模式: {mode_name}")
                return False
            
            mode_value = self.mode_mapping[mode_name]
            
            # TODO: 当Infrastructure层实现后，这里调用真实的执行器控制服务
            # 目前仅更新状态
            self.current_mode = mode_value
            self.effector_mode_changed.emit(mode_value)
            self.effector_status = mode_name
            self.effector_status_changed.emit(mode_name)
            
            self.emit_status(f"设置执行器模式: {mode_name} (0x{mode_value:02X})")
            return True
            
        except Exception as e:
            self.emit_error(f"设置执行器模式失败: {str(e)}")
            return False
    
    def set_effector_data(self, data: float) -> bool:
        """设置执行器数据（如夹取力度、旋转角度等）"""
        try:
            # 验证数据范围（0-100）
            if not (0.0 <= data <= 100.0):
                self.emit_error("执行器数据必须在0-100范围内")
                return False
            
            # TODO: 当Infrastructure层实现后，这里调用真实的执行器控制服务
            # 目前仅更新状态
            self.current_data = data
            self.effector_data_changed.emit(data)
            
            self.emit_status(f"设置执行器数据: {data:.1f}")
            return True
            
        except Exception as e:
            self.emit_error(f"设置执行器数据失败: {str(e)}")
            return False
    
    def execute_effector_command(self, command_data: Dict[str, Any]) -> bool:
        """执行执行器命令"""
        if not self.is_connected:
            self.emit_error("设备未连接")
            return False
        
        try:
            mode = command_data.get('mode', self.current_mode)
            data = command_data.get('data', self.current_data)
            
            # TODO: 当Infrastructure层实现后，这里调用RobotApplicationService发送执行器命令
            # 目前仅记录日志
            self.emit_status(f"执行执行器命令 - 模式: 0x{mode:02X}, 数据: {data:.1f}")
            
            # 更新状态
            self.current_mode = mode
            self.current_data = data
            self.effector_mode_changed.emit(mode)
            self.effector_data_changed.emit(data)
            
            # 模拟执行过程
            mode_name = self._get_mode_name(mode)
            self.effector_status = "运行中"
            self.effector_status_changed.emit("运行中")
            
            # TODO: 实际执行后，根据结果更新状态
            # 这里模拟成功完成
            self.effector_status = "完成"
            self.effector_status_changed.emit("完成")
            
            return True
            
        except Exception as e:
            self.emit_error(f"执行执行器命令失败: {str(e)}")
            self.effector_status = "错误"
            self.effector_status_changed.emit("错误")
            return False
    
    def stop_effector(self) -> bool:
        """停止执行器"""
        try:
            self.current_mode = 0x00
            self.effector_mode_changed.emit(0x00)
            self.effector_status = "停止"
            self.effector_status_changed.emit("停止")
            
            self.emit_status("执行器已停止")
            return True
            
        except Exception as e:
            self.emit_error(f"停止执行器失败: {str(e)}")
            return False
    
    def get_effector_status(self) -> Dict[str, Any]:
        """获取执行器状态"""
        return {
            'mode': self.current_mode,
            'mode_name': self._get_mode_name(self.current_mode),
            'data': self.current_data,
            'status': self.effector_status,
            'connected': self.is_connected
        }
    
    def set_connection_status(self, connected: bool):
        """设置连接状态"""
        self.is_connected = connected
        self.connection_status_changed.emit(connected)
        
        if not connected:
            # 断开连接时停止执行器
            self.stop_effector()
    
    def get_connection_status(self) -> bool:
        """获取连接状态"""
        return self.is_connected
    
    def get_available_modes(self) -> Dict[str, int]:
        """获取可用模式列表"""
        return self.mode_mapping.copy()
    
    def _get_mode_name(self, mode_value: int) -> str:
        """根据模式值获取模式名称"""
        for name, value in self.mode_mapping.items():
            if value == mode_value:
                return name
        return "未知模式" 