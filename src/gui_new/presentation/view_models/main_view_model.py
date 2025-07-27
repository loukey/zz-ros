"""
主视图模型 - 简化版本
"""
from typing import Dict, Any, Optional
from PyQt5.QtCore import pyqtSignal
from .base_view_model import BaseViewModel


class MainViewModel(BaseViewModel):
    """主视图模型 - 简化版本"""
    
    # 全局信号
    connection_status_changed = pyqtSignal(bool)
    global_error_occurred = pyqtSignal(str, str)  # module, error_message
    application_closing = pyqtSignal()
    progress_changed = pyqtSignal(int, bool)  # 进度百分比, 是否可见
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # 初始化子ViewModels（暂时为空实现）
        self._initialize_view_models()
        
        # 应用状态
        self.is_connected = False
        self.current_module = "serial"  # 当前活动模块
        
        # 初始化完成
        self.emit_status("主视图模型初始化完成（简化版）")
    
    def _initialize_view_models(self):
        """初始化所有子ViewModels（简化版）"""
        try:
            # TODO: 逐步添加子ViewModels
            # 暂时创建空的属性，避免AttributeError
            self.serial_vm = None
            self.control_vm = None
            self.motion_vm = None
            self.effector_vm = None
            self.trajectory_vm = None
            self.camera_vm = None
            self.dynamics_vm = None
            self.display_vm = None
            
            # ViewModels字典，便于管理
            self.view_models = {
                'serial': self.serial_vm,
                'control': self.control_vm,
                'motion': self.motion_vm,
                'effector': self.effector_vm,
                'trajectory': self.trajectory_vm,
                'camera': self.camera_vm,
                'dynamics': self.dynamics_vm,
                'display': self.display_vm,
            }
            
        except Exception as e:
            self.emit_error(f"初始化子ViewModels失败: {str(e)}")
    
    def get_connection_status(self) -> bool:
        """获取连接状态"""
        return self.is_connected
    
    def set_active_module(self, module_name: str):
        """设置当前活动模块"""
        if module_name in self.view_models:
            self.current_module = module_name
            self.emit_status(f"切换到{module_name.upper()}模块")
        else:
            self.emit_error(f"未知模块: {module_name}")
    
    def cleanup_all(self):
        """清理所有资源"""
        try:
            # 发出应用关闭信号
            self.application_closing.emit()
            
            # 清理自己
            super().cleanup()
            
            self.emit_status("所有资源已清理")
            
        except Exception as e:
            self.emit_error(f"清理资源失败: {str(e)}")
    
    def get_system_info(self) -> Dict[str, Any]:
        """获取系统信息"""
        try:
            return {
                'connected': self.is_connected,
                'active_module': self.current_module,
                'total_modules': len(self.view_models),
                'version': 'simplified',
                'view_models': list(self.view_models.keys())
            }
            
        except Exception as e:
            self.emit_error(f"获取系统信息失败: {str(e)}")
            return {} 