"""
主视图模型 - 整合所有子视图模型
"""
from PyQt5.QtCore import pyqtSignal
from .base_view_model import BaseViewModel
from .serial_view_model import SerialViewModel
from .display_view_model import DisplayViewModel
from .control_view_model import ControlViewModel
from .status_view_model import StatusViewModel
from .effector_view_model import EffectorViewModel
from .trajectory_view_model import TrajectoryViewModel
from .dynamics_view_model import DynamicsViewModel
from .camera_view_model import CameraViewModel
from .motion_planning_view_model import MotionPlanningViewModel
from .tools_view_model import ToolsViewModel


class MainViewModel(BaseViewModel):
    """主视图模型 - 整合所有子视图模型"""
    
    # 基础信号定义
    connection_status_changed = pyqtSignal(bool)
    status_message_changed = pyqtSignal(str)
    progress_changed = pyqtSignal(int, bool)
    
    def __init__(
        self, 
        serial_vm: SerialViewModel, 
        display_vm: DisplayViewModel, 
        control_vm: ControlViewModel, 
        status_vm: StatusViewModel,
        effector_vm: EffectorViewModel,
        trajectory_vm: TrajectoryViewModel,
        dynamics_vm: DynamicsViewModel,
        camera_vm: CameraViewModel,
        motion_planning_vm: MotionPlanningViewModel,
        tools_vm: ToolsViewModel,
        parent=None
    ):
        super().__init__(parent)
        self.serial_vm = serial_vm
        self.display_vm = display_vm
        self.control_vm = control_vm
        self.status_vm = status_vm
        self.effector_vm = effector_vm
        self.trajectory_vm = trajectory_vm
        self.dynamics_vm = dynamics_vm
        self.camera_vm = camera_vm
        self.motion_planning_vm = motion_planning_vm
        self.tools_vm = tools_vm
        
        # 连接连接状态信号
        self._connect_status_update_signals()

    def _connect_status_update_signals(self) -> None:
        """连接串口连接状态到各个ViewModel"""
        # 将串口连接状态转发到所有需要的ViewModel
        self.serial_vm.connection_status_changed.connect(
            self.control_vm.connection_status_changed.emit
        )
        self.serial_vm.connection_status_changed.connect(
            self.effector_vm.connection_status_changed.emit
        )
        self.serial_vm.connection_status_changed.connect(
            self.trajectory_vm.connection_status_changed.emit
        )
        self.serial_vm.connection_status_changed.connect(
            self.dynamics_vm.connection_status_changed.emit
        )

    def cleanup(self):
        """清理资源"""
        # 清理所有子ViewModels
        for vm_attr in ['serial_vm', 'display_vm', 'control_vm', 'status_vm',
                       'effector_vm', 'trajectory_vm', 'dynamics_vm', 'camera_vm',
                       'motion_planning_vm', 'tools_vm']:
            vm = getattr(self, vm_attr, None)
            if vm and hasattr(vm, 'cleanup'):
                try:
                    vm.cleanup()
                except Exception as e:
                    pass
        
        super().cleanup() 
        