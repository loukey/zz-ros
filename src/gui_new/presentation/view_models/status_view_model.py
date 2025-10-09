"""
状态视图模型 - 管理机器人状态显示
订阅 RobotStateDomainService，不再自己维护状态
"""
from PyQt5.QtCore import pyqtSignal
from .base_view_model import BaseViewModel
from domain import RobotStateDomainService, RobotStateSnapshot


class StatusViewModel(BaseViewModel):
    """
    状态视图模型 - 订阅状态服务
    
    职责：
    1. 订阅 RobotStateDomainService 的状态更新
    2. 转换数据格式（兼容UI组件）
    3. 发射信号给UI组件
    
    注意：不再自己维护状态，所有状态来自 RobotStateDomainService
    """
    
    # 状态更新信号
    status_updated = pyqtSignal(dict)
    
    def __init__(self, robot_state_service: RobotStateDomainService, parent=None):
        """
        初始化状态视图模型
        
        Args:
            robot_state_service: 机械臂状态服务（依赖注入）
        """
        super().__init__(parent)
        self.robot_state_service = robot_state_service
        
        # 订阅状态服务的信号
        self.robot_state_service.state_updated.connect(self._on_state_updated)
    
    def _on_state_updated(self, snapshot: RobotStateSnapshot):
        """
        状态更新回调
        
        Args:
            snapshot: 状态快照（不可变）
        """
        try:
            # 转换为dict格式（兼容现有UI组件）
            status_data = {
                'init_status': snapshot.init_status,
                'control': snapshot.control,
                'mode': snapshot.mode,
                'positions': list(snapshot.joint_angles),  # 显示角度（弧度）
                'status': list(snapshot.joint_status),
                'speeds': list(snapshot.joint_speeds),
                'torques': list(snapshot.joint_torques),
                'double_encoder_interpolations': list(snapshot.double_encoder_interpolations),
                'errors': list(snapshot.errors),
                'effector_data': snapshot.effector_data
            }
            
            # 发射信号给UI组件
            self.status_updated.emit(status_data)
            
        except Exception as e:
            pass
    
    def get_position_string(self):
        """
        获取位置信息的字符串表示
        
        Returns:
            str: 位置字符串，如 "[0.50, -1.20, 0.30, 0.00, 0.00, 0.00]"
        """
        snapshot = self.robot_state_service.get_current_state()
        if snapshot:
            positions = snapshot.joint_angles
            return f"[{', '.join(f'{pos:.3f}' for pos in positions[:6])}]"
        return "--"
    
    def get_status_summary(self):
        """
        获取状态摘要
        
        Returns:
            str: 状态摘要字符串，如 "CMD:0x06 | MODE:0x08 | STA:0x01"
        """
        snapshot = self.robot_state_service.get_current_state()
        if not snapshot:
            return "无状态数据"
        
        summary = []
        summary.append(f"CMD:0x{snapshot.control:02X}")
        summary.append(f"MODE:0x{snapshot.mode:02X}")
        summary.append(f"STA:0x{snapshot.init_status:02X}")
        
        # 检查错误
        if any(err != 0 for err in snapshot.errors):
            summary.append(f"ERR:有错误")
        
        return " | ".join(summary)
