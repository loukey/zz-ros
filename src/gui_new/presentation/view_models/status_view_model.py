"""
状态视图模型 - 管理机器人状态显示
"""
from PyQt5.QtCore import pyqtSignal
from .base_view_model import BaseViewModel


class StatusViewModel(BaseViewModel):
    """状态视图模型 - 管理解码后的消息状态显示"""
    
    # 状态更新信号
    status_updated = pyqtSignal(dict)
    
    def __init__(self, parent=None):
        """初始化状态视图模型"""
        super().__init__(parent)
        
        # 当前状态数据
        self.current_status = {
            'init_status': None,
            'control': None,
            'mode': None,
            'positions': None,
            'status': None,
            'speeds': None,
            'torques': None,
            'double_encoder_interpolations': None,
            'errors': None,
            'effector_data': None
        }
    
    def update_robot_status(self, decoded_message):
        """更新机器人状态"""
        try:
            # 提取解码消息中的各个字段
            status_data = {
                'init_status': getattr(decoded_message, 'init_status', None),
                'control': getattr(decoded_message, 'control', None),
                'mode': getattr(decoded_message, 'mode', None),
                'positions': getattr(decoded_message, 'positions', None),
                'status': getattr(decoded_message, 'status', None),
                'speeds': getattr(decoded_message, 'speeds', None),
                'torques': getattr(decoded_message, 'torques', None),
                'double_encoder_interpolations': getattr(decoded_message, 'double_encoder_interpolations', None),
                'errors': getattr(decoded_message, 'errors', None),
                'effector_data': getattr(decoded_message, 'effector_data', None)
            }
            
            # 更新当前状态
            self.current_status.update(status_data)
            
            # 发送更新信号
            self.status_updated.emit(status_data)
            
        except Exception as e:
            print(f"更新机器人状态失败: {str(e)}")
    
    def get_current_status(self):
        """获取当前状态"""
        return self.current_status.copy()
    
    def clear_status(self):
        """清空状态显示"""
        cleared_status = {key: None for key in self.current_status.keys()}
        self.current_status = cleared_status
        self.status_updated.emit(cleared_status)
    
    def get_position_string(self):
        """获取位置信息的字符串表示"""
        positions = self.current_status.get('positions')
        if positions and isinstance(positions, (list, tuple)):
            return f"[{', '.join(f'{pos:.3f}' for pos in positions[:6])}]"
        return "--"
    
    def get_status_summary(self):
        """获取状态摘要"""
        control = self.current_status.get('control')
        mode = self.current_status.get('mode')
        status = self.current_status.get('status')
        errors = self.current_status.get('errors')
        
        summary = []
        if control is not None:
            summary.append(f"CMD:0x{control:02X}")
        if mode is not None:
            summary.append(f"MODE:0x{mode:02X}")
        if status is not None:
            summary.append(f"STA:0x{status:02X}")
        if errors is not None and errors != 0:
            summary.append(f"ERR:0x{errors:02X}")
        
        return " | ".join(summary) if summary else "无状态数据"
