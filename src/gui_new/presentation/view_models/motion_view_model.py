"""
运动视图模型
"""
from typing import List, Dict, Any, Tuple, Optional
from PyQt5.QtCore import pyqtSignal
from .base_view_model import BaseViewModel
from application.services.robot_application_service import RobotApplicationService
from application.dto.command_dto import SendAnglesDTO
from application.dto.robot_dto import JointAnglesDTO
from shared.config.container import ServiceLocator
import numpy as np


class MotionViewModel(BaseViewModel):
    """运动视图模型"""
    
    # 运动特有信号
    joint_angles_changed = pyqtSignal(list)
    target_angles_changed = pyqtSignal(list)
    motion_started = pyqtSignal()
    motion_completed = pyqtSignal()
    connection_status_changed = pyqtSignal(bool)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # 获取Application Service
        self.robot_service = ServiceLocator.resolve(RobotApplicationService)
        
        # UI状态
        self.current_angles = [0.0] * 6  # 当前关节角度（度）
        self.target_angles = [0.0] * 6   # 目标关节角度（度）
        self.is_connected = False
        
        # 轨迹参数
        self.curve_type = "S型"
        self.frequency = 0.01  # 频率（秒）
        self.contour_params = None  # 轮廓参数
        
        # 曲线类型映射
        self.curve_type_mapping = {
            "S型": "S_CURVE",
            "梯形": "TRAPEZOIDAL", 
            "直线": "LINEAR"
        }
    
    def send_angles(self, angle_data: Dict[str, Any]) -> bool:
        """发送角度数据"""
        if not self.is_connected:
            self.emit_error("设备未连接")
            return False
        
        try:
            # 提取角度数据
            target_angles_deg = angle_data.get('target_angles', self.target_angles)
            curve_type_info = angle_data.get('curve_type', ('S型', 4.0, 0.01))
            
            # 解析曲线类型信息
            if isinstance(curve_type_info, tuple) and len(curve_type_info) >= 3:
                curve_name, max_vel, frequency = curve_type_info[:3]
                self.curve_type = curve_name
                self.frequency = frequency
            else:
                curve_name = self.curve_type
                frequency = self.frequency
            
            # 转换为弧度
            target_angles_rad = [np.radians(angle) for angle in target_angles_deg]
            
            # 创建发送角度DTO
            send_dto = SendAnglesDTO(
                target_angles=target_angles_rad,
                curve_type=self.curve_type_mapping.get(curve_name, "S_CURVE"),
                frequency=frequency,
                contour_params=angle_data.get('contour_params', self.contour_params),
                encoding_type=angle_data.get('encoding_type', 'hex'),
                run_mode=angle_data.get('run_mode', 0x08)
            )
            
            # 调用Application服务发送角度
            success, message, command_str = self.robot_service.send_angles(send_dto)
            
            if success:
                self.target_angles = target_angles_deg
                self.target_angles_changed.emit(target_angles_deg)
                self.emit_status(f"发送角度成功: {[f'{a:.2f}°' for a in target_angles_deg]}")
                
                if command_str:
                    self.emit_status(f"命令字符串: {command_str}")
                return True
            else:
                self.emit_error(f"发送角度失败: {message}")
                return False
                
        except Exception as e:
            self.emit_error(f"发送角度失败: {str(e)}")
            return False
    
    def move_to_angles(self, angles: List[float]) -> bool:
        """移动到指定角度"""
        try:
            # 验证角度数据
            if len(angles) != 6:
                self.emit_error("角度数据必须包含6个关节")
                return False
            
            # 转换为弧度并更新
            angles_rad = [np.radians(angle) for angle in angles]
            joint_angles_dto = JointAnglesDTO(angles=angles_rad)
            
            success, message = self.robot_service.update_joint_angles(joint_angles_dto)
            
            if success:
                self.target_angles = angles
                self.target_angles_changed.emit(angles)
                self.emit_status(f"设置目标角度: {[f'{a:.2f}°' for a in angles]}")
                return True
            else:
                self.emit_error(f"设置目标角度失败: {message}")
                return False
                
        except Exception as e:
            self.emit_error(f"移动到指定角度失败: {str(e)}")
            return False
    
    def update_current_angles(self, angles: List[float]):
        """更新当前角度"""
        try:
            if len(angles) == 6:
                self.current_angles = angles.copy()
                self.joint_angles_changed.emit(angles)
            else:
                self.emit_error("当前角度数据必须包含6个关节")
                
        except Exception as e:
            self.emit_error(f"更新当前角度失败: {str(e)}")
    
    def get_current_angles(self) -> List[float]:
        """获取当前角度（度）"""
        try:
            # 从Application Service获取最新状态
            robot_state = self.robot_service.get_robot_state()
            if robot_state:
                # 转换为度数
                angles_deg = [np.degrees(angle) for angle in robot_state.joint_angles]
                self.current_angles = angles_deg
                return angles_deg
            else:
                return self.current_angles
                
        except Exception as e:
            self.emit_error(f"获取当前角度失败: {str(e)}")
            return self.current_angles
    
    def get_target_angles(self) -> List[float]:
        """获取目标角度（度）"""
        return self.target_angles.copy()
    
    def set_curve_type(self, curve_type: str):
        """设置曲线类型"""
        if curve_type in self.curve_type_mapping:
            self.curve_type = curve_type
            self.emit_status(f"设置曲线类型: {curve_type}")
        else:
            self.emit_error(f"无效的曲线类型: {curve_type}")
    
    def get_curve_type(self) -> str:
        """获取当前曲线类型"""
        return self.curve_type
    
    def set_frequency(self, frequency: float):
        """设置频率"""
        if 0.001 <= frequency <= 1.0:
            self.frequency = frequency
            self.emit_status(f"设置频率: {frequency}s")
        else:
            self.emit_error("频率必须在0.001-1.0秒之间")
    
    def get_frequency(self) -> float:
        """获取频率"""
        return self.frequency
    
    def set_contour_params(self, params: Optional[List[List[float]]]):
        """设置轮廓参数"""
        self.contour_params = params
        if params:
            self.emit_status("已设置轮廓参数")
        else:
            self.emit_status("已清除轮廓参数")
    
    def get_contour_params(self) -> Optional[List[List[float]]]:
        """获取轮廓参数"""
        return self.contour_params
    
    def start_motion(self) -> bool:
        """开始运动"""
        try:
            success, message = self.robot_service.start_movement()
            
            if success:
                self.motion_started.emit()
                self.emit_status("开始运动")
                return True
            else:
                self.emit_error(f"开始运动失败: {message}")
                return False
                
        except Exception as e:
            self.emit_error(f"开始运动失败: {str(e)}")
            return False
    
    def stop_motion(self) -> bool:
        """停止运动"""
        try:
            success, message = self.robot_service.stop_movement()
            
            if success:
                self.motion_completed.emit()
                self.emit_status("停止运动")
                return True
            else:
                self.emit_error(f"停止运动失败: {message}")
                return False
                
        except Exception as e:
            self.emit_error(f"停止运动失败: {str(e)}")
            return False
    
    def set_connection_status(self, connected: bool):
        """设置连接状态"""
        self.is_connected = connected
        self.connection_status_changed.emit(connected)
        
        if connected:
            # 连接后获取当前角度
            self.get_current_angles()
        else:
            # 断开连接时停止运动
            self.motion_completed.emit()
    
    def get_connection_status(self) -> bool:
        """获取连接状态"""
        return self.is_connected 