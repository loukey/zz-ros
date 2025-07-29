"""
主视图模型 - 从零开始实现
"""
from PyQt5.QtCore import pyqtSignal
from .base_view_model import BaseViewModel
from .serial_view_model import SerialViewModel
from .display_view_model import DisplayViewModel


class MainViewModel(BaseViewModel):
    """主视图模型 - 从零开始实现"""
    
    # 基础信号定义 - 等待实现具体功能
    connection_status_changed = pyqtSignal(bool)
    status_message_changed = pyqtSignal(str)
    progress_changed = pyqtSignal(int, bool)
    
    def __init__(self, serial_vm: SerialViewModel, display_vm: DisplayViewModel, parent=None):
        super().__init__(parent)
        
        # 基础状态
        self.is_connected = False
        
        # 通过依赖注入接收子ViewModel
        self.serial_vm = serial_vm
        self.display_vm = display_vm
        
        # 连接服务信号到显示ViewModel
        self._connect_service_signals()
        
        # 预留的子ViewModel属性 - 等待逐步实现
        self.control_vm = None  
        self.motion_vm = None
        self.effector_vm = None
        self.trajectory_vm = None
        self.camera_vm = None
        self.dynamics_vm = None
        
        # TODO: 逐步添加具体功能实现
    
    def _connect_service_signals(self) -> None:
        """连接各个服务的信号到DisplayViewModel"""
        # 连接串口服务的消息显示信号
        if hasattr(self.serial_vm, 'serial_service'):
            self.serial_vm.serial_service.message_display.connect(self.display_vm.append_message)
        
        # TODO: 未来添加其他服务的信号连接
        # self.control_service.message_display.connect(self.display_vm.append_message)
        # self.motion_service.message_display.connect(self.display_vm.append_message)
    
    def cleanup(self):
        """清理资源"""
        # 断开服务信号连接
        self._disconnect_service_signals()
        
        # 清理子ViewModels（如果已实现）
        for vm_attr in ['serial_vm', 'control_vm', 'motion_vm', 'effector_vm',
                       'trajectory_vm', 'camera_vm', 'dynamics_vm', 'display_vm', 'serial_config_vm']:
            vm = getattr(self, vm_attr, None)
            if vm and hasattr(vm, 'cleanup'):
                vm.cleanup()
        
        super().cleanup()
    
    def _disconnect_service_signals(self) -> None:
        """断开服务信号连接"""
        try:
            if hasattr(self.serial_vm, 'serial_service'):
                self.serial_vm.serial_service.message_display.disconnect(self.display_vm.append_message)
            
            # 断开SerialConfigViewModel的信号连接
            self.serial_config_vm.send_serial_config_sig.disconnect(self.serial_vm.connect_serial)
            
            # TODO: 断开其他服务的信号连接
        except (TypeError, AttributeError):
            # 如果信号未连接或属性不存在，忽略即可
            pass 