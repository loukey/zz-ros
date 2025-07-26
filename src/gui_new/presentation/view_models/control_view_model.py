"""
控制视图模型
"""
from typing import Dict, Any, Optional
from PyQt5.QtCore import pyqtSignal
from .base_view_model import BaseViewModel
from application.services.robot_application_service import RobotApplicationService
from application.dto.command_dto import ControlCommandDTO
from shared.config.container import ServiceLocator


class ControlViewModel(BaseViewModel):
    """控制视图模型"""
    
    # 控制特有信号
    robot_started = pyqtSignal()
    robot_stopped = pyqtSignal()
    emergency_stop_triggered = pyqtSignal()
    mode_changed = pyqtSignal(str)
    connection_status_changed = pyqtSignal(bool)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # 获取Application Service
        self.robot_service = ServiceLocator.resolve(RobotApplicationService)
        
        # UI状态
        self.is_running = False
        self.current_mode = "manual"  # manual, auto, teach
        self.encoding_type = "hex"
        self.is_connected = False
        
        # 模式映射
        self.mode_mapping = {
            "manual": 0x01,
            "auto": 0x02,
            "teach": 0x03
        }
    
    def send_control_command(self, command_data: Dict[str, Any]) -> bool:
        """发送控制命令"""
        if not self.is_connected:
            self.emit_error("设备未连接")
            return False
        
        try:
            # 获取当前机器人状态
            robot_state = self.robot_service.get_robot_state()
            if not robot_state:
                self.emit_error("无法获取机器人状态")
                return False
            
            # 创建控制命令DTO
            control_dto = ControlCommandDTO(
                joint_angles=robot_state.joint_angles,
                control=command_data.get('control', 0x06),
                mode=command_data.get('mode', self.mode_mapping.get(self.current_mode, 0x01)),
                contour_speed=command_data.get('contour_speed'),
                contour_acceleration=command_data.get('contour_acceleration'),
                contour_deceleration=command_data.get('contour_deceleration'),
                torque=command_data.get('torque'),
                effector_mode=command_data.get('effector_mode', 0x00),
                effector_data=command_data.get('effector_data', 0.0),
                encoding=command_data.get('encoding', self.encoding_type)
            )
            
            # TODO: 当Infrastructure层实现后，这里调用真实的命令发送服务
            # 目前仅记录日志
            self.emit_status(f"发送控制命令: 控制字={control_dto.control:02X}, "
                           f"模式={control_dto.mode:02X}, 编码={control_dto.encoding}")
            return True
            
        except Exception as e:
            self.emit_error(f"发送控制命令失败: {str(e)}")
            return False
    
    def start_robot(self) -> bool:
        """启动机器人"""
        if self.is_running:
            self.emit_status("机器人已在运行中")
            return True
        
        try:
            success, message = self.robot_service.start_movement()
            
            if success:
                self.is_running = True
                self.robot_started.emit()
                self.emit_status("机器人已启动")
                return True
            else:
                self.emit_error(f"启动机器人失败: {message}")
                return False
                
        except Exception as e:
            self.emit_error(f"启动机器人失败: {str(e)}")
            return False
    
    def stop_robot(self) -> bool:
        """停止机器人"""
        if not self.is_running:
            self.emit_status("机器人已停止")
            return True
        
        try:
            success, message = self.robot_service.stop_movement()
            
            if success:
                self.is_running = False
                self.robot_stopped.emit()
                self.emit_status("机器人已停止")
                return True
            else:
                self.emit_error(f"停止机器人失败: {message}")
                return False
                
        except Exception as e:
            self.emit_error(f"停止机器人失败: {str(e)}")
            return False
    
    def emergency_stop(self) -> bool:
        """急停"""
        try:
            # 强制停止机器人
            success, message = self.robot_service.stop_movement()
            
            self.is_running = False
            self.emergency_stop_triggered.emit()
            self.emit_status("急停已触发")
            
            return success
            
        except Exception as e:
            self.emit_error(f"急停执行失败: {str(e)}")
            return False
    
    def set_connection_status(self, connected: bool):
        """设置连接状态"""
        self.is_connected = connected
        self.connection_status_changed.emit(connected)
        
        # 如果断开连接，自动停止机器人
        if not connected and self.is_running:
            self.is_running = False
            self.robot_stopped.emit()
    
    def set_mode(self, mode: str) -> bool:
        """设置运行模式"""
        if mode in self.mode_mapping:
            self.current_mode = mode
            self.mode_changed.emit(mode)
            self.emit_status(f"切换到{mode}模式")
            return True
        else:
            self.emit_error(f"无效的运行模式: {mode}")
            return False
    
    def get_run_mode(self) -> str:
        """获取运行模式"""
        return self.current_mode
    
    def get_run_mode_value(self) -> int:
        """获取运行模式对应的数值"""
        return self.mode_mapping.get(self.current_mode, 0x01)
    
    def set_encoding_type(self, encoding: str):
        """设置编码类型"""
        if encoding in ["hex", "decimal", "binary"]:
            self.encoding_type = encoding
            self.emit_status(f"编码类型切换为: {encoding}")
        else:
            self.emit_error(f"无效的编码类型: {encoding}")
    
    def get_encoding_type(self) -> str:
        """获取编码类型"""
        return self.encoding_type
    
    def get_running_status(self) -> bool:
        """获取运行状态"""
        return self.is_running
    
    def get_connection_status(self) -> bool:
        """获取连接状态"""
        return self.is_connected 