"""
主视图模型 - 协调所有子视图模型
"""
from typing import Dict, Any, Optional
from PyQt5.QtCore import pyqtSignal
from .base_view_model import BaseViewModel
from .serial_view_model import SerialViewModel
from .control_view_model import ControlViewModel
from .motion_view_model import MotionViewModel
from .kinematic_view_model import KinematicViewModel
from .effector_view_model import EffectorViewModel
from .trajectory_view_model import TrajectoryViewModel
from .camera_view_model import CameraViewModel
from .dynamics_view_model import DynamicsViewModel
from .parameter_view_model import ParameterViewModel
from .display_view_model import DisplayViewModel
from .contour_view_model import ContourViewModel
from application.services.robot_application_service import RobotApplicationService
from shared.config.container import ServiceLocator


class MainViewModel(BaseViewModel):
    """主视图模型 - 协调管理所有功能模块"""
    
    # 全局信号
    connection_status_changed = pyqtSignal(bool)
    global_error_occurred = pyqtSignal(str, str)  # module, error_message
    application_closing = pyqtSignal()
    progress_changed = pyqtSignal(int, bool)  # 进度百分比, 是否可见
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # 获取Application Service
        self.robot_service = ServiceLocator.resolve(RobotApplicationService)
        
        # 初始化所有子ViewModels
        self._initialize_view_models()
        
        # 建立信号连接
        self._setup_signal_connections()
        
        # 应用状态
        self.is_connected = False
        self.current_module = "serial"  # 当前活动模块
        
        # 初始化完成
        self.emit_status("主视图模型初始化完成")
    
    def _initialize_view_models(self):
        """初始化所有子ViewModels"""
        try:
            # 创建所有子ViewModels
            self.serial_vm = SerialViewModel(self)
            self.control_vm = ControlViewModel(self)
            self.motion_vm = MotionViewModel(self)
            self.kinematic_vm = KinematicViewModel(self)
            self.effector_vm = EffectorViewModel(self)
            self.trajectory_vm = TrajectoryViewModel(self)
            self.camera_vm = CameraViewModel(self)
            self.dynamics_vm = DynamicsViewModel(self)
            self.parameter_vm = ParameterViewModel(self)
            self.display_vm = DisplayViewModel(self)
            self.contour_vm = ContourViewModel(self)
            
            # ViewModels字典，便于管理
            self.view_models = {
                'serial': self.serial_vm,
                'control': self.control_vm,
                'motion': self.motion_vm,
                'kinematic': self.kinematic_vm,
                'effector': self.effector_vm,
                'trajectory': self.trajectory_vm,
                'camera': self.camera_vm,
                'dynamics': self.dynamics_vm,
                'parameter': self.parameter_vm,
                'display': self.display_vm,
                'contour': self.contour_vm
            }
            

            
        except Exception as e:
            self.emit_error(f"初始化子ViewModels失败: {str(e)}")
    
    def _setup_signal_connections(self):
        """建立信号连接"""
        try:
            # 1. 连接状态同步 - 串口连接状态影响所有模块
            self.serial_vm.connection_status_changed.connect(self._on_connection_status_changed)
            
            # 2. 错误处理 - 所有模块的错误都转发到主模块
            for name, vm in self.view_models.items():
                vm.error_occurred.connect(lambda msg, module=name: self._on_module_error(module, msg))
                vm.status_message_changed.connect(self._on_module_status)
            
            # 3. 显示模块连接 - 将所有状态消息转发到显示模块
            self.status_message_changed.connect(self.display_vm.add_message)
            self.error_occurred.connect(lambda msg: self.display_vm.add_message(msg, "error"))
            
            # 4. 运动学计算结果应用到运动控制
            self.kinematic_vm.angles_calculated.connect(self._apply_kinematic_results)
            
            # 5. 轮廓参数同步到运动和轨迹模块
            self.contour_vm.contour_parameters_changed.connect(self._sync_contour_parameters)
            
            # 6. 参数更新同步到相关模块
            self.parameter_vm.parameters_updated.connect(self._sync_robot_parameters)
            
            # 7. 相机检测结果可以用于运动学计算
            self.camera_vm.pose_published.connect(self._handle_camera_pose)
            
            # 8. 轨迹执行状态同步
            self.trajectory_vm.execution_started.connect(lambda: self._set_module_busy('trajectory', True))
            self.trajectory_vm.execution_completed.connect(lambda success, msg: self._set_module_busy('trajectory', False))
            
            # 9. 进度信号转发
            self.trajectory_vm.progress_changed.connect(self._on_progress_changed)
            
        except Exception as e:
            self.emit_error(f"建立信号连接失败: {str(e)}")
    
    def _on_connection_status_changed(self, connected: bool):
        """处理连接状态变化"""
        try:
            self.is_connected = connected
            self.connection_status_changed.emit(connected)
            
            # 同步连接状态到所有需要的模块
            modules_need_connection = ['control', 'motion', 'effector', 'dynamics']
            for module_name in modules_need_connection:
                if module_name in self.view_models:
                    vm = self.view_models[module_name]
                    if hasattr(vm, 'set_connection_status'):
                        vm.set_connection_status(connected)
            
            # 连接状态变化时启动/停止显示数据自动更新
            if connected:
                self.display_vm.start_auto_update(100)  # 100ms间隔
                self.emit_status("机器人已连接，开始实时数据更新")
            else:
                self.display_vm.stop_auto_update()
                self.emit_status("机器人已断开连接")
                
        except Exception as e:
            self.emit_error(f"处理连接状态变化失败: {str(e)}")
    
    def _on_module_error(self, module: str, error_message: str):
        """处理模块错误"""
        try:
            error_msg = f"[{module.upper()}] {error_message}"
            self.global_error_occurred.emit(module, error_message)
            self.emit_error(error_msg)
            
        except Exception as e:
            print(f"处理模块错误失败: {str(e)}")
    
    def _on_module_status(self, status_message: str):
        """处理模块状态消息"""
        try:
            # 转发状态消息到显示模块
            self.display_vm.add_message(status_message, "info")
            
        except Exception as e:
            print(f"处理模块状态失败: {str(e)}")
    
    def _apply_kinematic_results(self, angles: list):
        """应用运动学计算结果到运动控制"""
        try:
            if not self.is_connected:
                return
            
            # 将计算出的角度应用到运动控制模块
            self.motion_vm.move_to_angles(angles)
            self.emit_status(f"运动学计算结果已应用: {[f'{a:.2f}°' for a in angles]}")
            
        except Exception as e:
            self.emit_error(f"应用运动学结果失败: {str(e)}")
    
    def _sync_contour_parameters(self, contour_params: Dict[str, Any]):
        """同步轮廓参数到相关模块"""
        try:
            # 同步到运动模块
            if hasattr(self.motion_vm, 'set_contour_params'):
                effective_params = [
                    contour_params.get('effective_speeds', []),
                    contour_params.get('effective_accelerations', []),
                    contour_params.get('effective_decelerations', [])
                ]
                self.motion_vm.set_contour_params(effective_params)
            
            # 同步到轨迹模块
            if hasattr(self.trajectory_vm, 'set_trajectory_parameters'):
                self.trajectory_vm.set_trajectory_parameters(
                    max_velocity=max(contour_params.get('effective_speeds', [1.0])),
                    max_acceleration=max(contour_params.get('effective_accelerations', [2.0])),
                    max_jerk=10.0  # 使用默认值
                )
            
            self.emit_status("轮廓参数已同步到相关模块")
            
        except Exception as e:
            self.emit_error(f"同步轮廓参数失败: {str(e)}")
    
    def _sync_robot_parameters(self, robot_params: Dict[str, Any]):
        """同步机器人参数到相关模块"""
        try:
            # 同步到动力学模块
            if 'payload_params' in robot_params:
                payload = robot_params['payload_params']
                self.dynamics_vm.set_payload_parameters(
                    payload.get('mass', 0.0),
                    payload.get('center_of_mass', [0.0, 0.0, 0.0])
                )
            
            # 同步到运动模块（关节限位）
            if 'joint_limits' in robot_params:
                limits = robot_params['joint_limits']
                # 这里可以设置运动模块的限位检查参数
                pass
            
            self.emit_status("机器人参数已同步到相关模块")
            
        except Exception as e:
            self.emit_error(f"同步机器人参数失败: {str(e)}")
    
    def _handle_camera_pose(self, pose_data: Dict[str, Any]):
        """处理相机位姿数据"""
        try:
            # 将相机检测到的位姿转换为运动学模块可用的格式
            position = [
                pose_data['position']['x'] * 1000,  # m转mm
                pose_data['position']['y'] * 1000,
                pose_data['position']['z'] * 1000
            ]
            
            # 提取欧拉角（简化）
            orientation = [0.0, 0.0, 0.0]  # 简化为无旋转
            
            # 设置为运动学模块的目标位姿
            self.kinematic_vm.set_target_position(position)
            self.kinematic_vm.set_target_orientation(orientation)
            
            self.emit_status(f"相机检测位姿已设置为目标: {pose_data['object_id']}")
            
        except Exception as e:
            self.emit_error(f"处理相机位姿失败: {str(e)}")
    
    def _set_module_busy(self, module: str, busy: bool):
        """设置模块忙碌状态"""
        try:
            status = "忙碌" if busy else "空闲"
            self.emit_status(f"{module.upper()}模块状态: {status}")
            
        except Exception as e:
            self.emit_error(f"设置模块状态失败: {str(e)}")
    
    def _on_progress_changed(self, progress: int, completed: bool):
        """处理进度变化"""
        try:
            # 转发进度信号到主窗口
            if completed:
                # 完成时隐藏进度条
                self.progress_changed.emit(100, False)
            else:
                # 执行中显示进度条
                self.progress_changed.emit(progress, True)
                
        except Exception as e:
            self.emit_error(f"处理进度变化失败: {str(e)}")
    
    def set_active_module(self, module_name: str):
        """设置当前活动模块"""
        if module_name in self.view_models:
            self.current_module = module_name
            self.emit_status(f"切换到{module_name.upper()}模块")
        else:
            self.emit_error(f"未知模块: {module_name}")
    
    def get_module_view_model(self, module_name: str):
        """获取指定模块的ViewModel"""
        return self.view_models.get(module_name)
    
    def get_connection_status(self) -> bool:
        """获取连接状态"""
        return self.is_connected
    
    def get_all_module_status(self) -> Dict[str, Any]:
        """获取所有模块状态"""
        try:
            status = {}
            for name, vm in self.view_models.items():
                if hasattr(vm, 'get_connection_status'):
                    status[name] = {
                        'connected': vm.get_connection_status() if hasattr(vm, 'get_connection_status') else self.is_connected,
                        'active': name == self.current_module
                    }
                else:
                    status[name] = {
                        'connected': self.is_connected,
                        'active': name == self.current_module
                    }
            
            return status
            
        except Exception as e:
            self.emit_error(f"获取模块状态失败: {str(e)}")
            return {}
    
    def emergency_stop(self) -> bool:
        """紧急停止所有运动"""
        try:
            success = True
            
            # 停止所有可能的运动
            if hasattr(self.control_vm, 'emergency_stop'):
                success &= self.control_vm.emergency_stop()
            
            if hasattr(self.trajectory_vm, 'stop_trajectory_execution'):
                success &= self.trajectory_vm.stop_trajectory_execution()
            
            if hasattr(self.motion_vm, 'stop_motion'):
                success &= self.motion_vm.stop_motion()
            
            self.emit_status("紧急停止已执行")
            return success
            
        except Exception as e:
            self.emit_error(f"紧急停止失败: {str(e)}")
            return False
    
    def cleanup_all(self):
        """清理所有资源"""
        try:
            # 发出应用关闭信号
            self.application_closing.emit()
            
            # 清理所有子ViewModels
            for vm in self.view_models.values():
                if hasattr(vm, 'cleanup'):
                    vm.cleanup()
            
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
                'robot_service_available': self.robot_service is not None,
                'view_models': list(self.view_models.keys())
            }
            
        except Exception as e:
            self.emit_error(f"获取系统信息失败: {str(e)}")
            return {} 