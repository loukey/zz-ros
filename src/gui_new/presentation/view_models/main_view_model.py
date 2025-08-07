"""
主视图模型 - 从零开始实现
"""
from PyQt5.QtCore import pyqtSignal
from .base_view_model import BaseViewModel
from .serial_view_model import SerialViewModel
from .display_view_model import DisplayViewModel
from .control_view_model import ControlViewModel
from .status_view_model import StatusViewModel


class MainViewModel(BaseViewModel):
    """主视图模型 - 从零开始实现"""
    
    # 基础信号定义 - 等待实现具体功能
    connection_status_changed = pyqtSignal(bool)
    status_message_changed = pyqtSignal(str)
    progress_changed = pyqtSignal(int, bool)
    
    def __init__(self, serial_vm: SerialViewModel, display_vm: DisplayViewModel, control_vm: ControlViewModel, status_vm: StatusViewModel, parent=None):
        super().__init__(parent)
        
        # 基础状态
        self.is_connected = False
        
        # 通过依赖注入接收子ViewModel
        self.serial_vm = serial_vm
        self.display_vm = display_vm
        self.control_vm = control_vm
        self.status_vm = status_vm
        
        # 连接服务信号到显示ViewModel
        self._connect_service_signals()
        
        # 连接MessageResponseService信号到StatusViewModel
        self._connect_message_response_signals()
        
        # 预留的子ViewModel属性 - 等待逐步实现
        self.motion_vm = None
        self.effector_vm = None
        self.trajectory_vm = None
        self.camera_vm = None
        self.dynamics_vm = None
    
    def _connect_service_signals(self) -> None:
        """连接各个服务的信号到DisplayViewModel"""
        # 连接状态改变
        self.serial_vm.connection_status_changed.connect(self.control_vm.connection_status_changed.emit)
    
    def _connect_message_response_signals(self) -> None:
        """连接MessageResponseService信号到StatusViewModel"""
        # 这里需要通过DI容器获取MessageResponseService
        from shared.config.service_registry import resolve
        from application.services import MessageResponseService
        
        try:
            message_response_service = resolve(MessageResponseService)
            message_response_service.decoded_message_received.connect(self.status_vm.update_robot_status)
        except Exception as e:
            print(f"连接MessageResponseService信号失败: {str(e)}")

    def cleanup(self):
        """清理资源"""
        # 断开服务信号连接
        self._disconnect_service_signals()
        
        # 清理子ViewModels（如果已实现）
        for vm_attr in ['serial_vm', 'control_vm', 'motion_vm', 'effector_vm',
                       'trajectory_vm', 'camera_vm', 'dynamics_vm', 'display_vm', 'status_vm']:
            vm = getattr(self, vm_attr, None)
            if vm and hasattr(vm, 'cleanup'):
                vm.cleanup()
        
        super().cleanup()
    
    def _disconnect_service_signals(self) -> None:
        """断开服务信号连接"""
        try:
            if hasattr(self.serial_vm, 'serial_service'):
                self.serial_vm.serial_service.message_display.disconnect(self.display_vm.append_message)
        except (TypeError, AttributeError):
            pass 
        