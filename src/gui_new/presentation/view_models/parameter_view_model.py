"""
参数视图模型
"""
from typing import List, Dict, Any, Optional
from PyQt5.QtCore import pyqtSignal, QTimer
from .base_view_model import BaseViewModel
from application.services.robot_application_service import RobotApplicationService
from shared.config.container import ServiceLocator
from shared.config.settings import AppSettings


class ParameterViewModel(BaseViewModel):
    """参数视图模型"""
    
    # 参数特有信号
    parameters_updated = pyqtSignal(dict)  # 参数更新
    calibration_completed = pyqtSignal(bool, str)  # 标定完成 (成功, 消息)
    parameters_saved = pyqtSignal(str)  # 参数保存完成
    parameters_loaded = pyqtSignal(dict)  # 参数加载完成
    
    # 速度标定信号（匹配原始功能）
    velocity_updated = pyqtSignal(float, float)  # 速度更新信号 (vx, vy)
    history_velocity_updated = pyqtSignal(float, float)  # 历史速度更新信号 (vx, vy)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # 获取Application Service和设置
        self.robot_service = ServiceLocator.resolve(RobotApplicationService)
        self.app_settings = AppSettings()
        
        # 机器人参数
        self.robot_parameters = {
            # DH参数 (6个关节)
            'dh_params': {
                'a': [0.0, 425.0, 392.25, 0.0, 0.0, 0.0],  # 连杆长度 (mm)
                'd': [162.5, 0.0, 0.0, 133.3, 99.7, 99.6],  # 偏移 (mm)
                'alpha': [1.570796, 0.0, 0.0, 1.570796, -1.570796, 0.0],  # 扭转角 (rad)
                'theta_offset': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 角度偏移 (rad)
            },
            
            # 关节限位
            'joint_limits': {
                'min_angles': [-3.14159, -1.5708, -1.5708, -3.14159, -1.5708, -3.14159],  # rad
                'max_angles': [3.14159, 1.5708, 1.5708, 3.14159, 1.5708, 3.14159],  # rad
                'max_velocities': [3.14, 3.14, 3.14, 6.28, 6.28, 6.28],  # rad/s
                'max_accelerations': [10.0, 10.0, 10.0, 20.0, 20.0, 20.0]  # rad/s²
            },
            
            # 工具参数
            'tool_params': {
                'tcp_offset': [0.0, 0.0, 100.0, 0.0, 0.0, 0.0],  # TCP偏移 [x,y,z,rx,ry,rz]
                'tool_mass': 0.5,  # 工具质量 (kg)
                'tool_center_of_mass': [0.0, 0.0, 50.0]  # 工具质心 (mm)
            },
            
            # 负载参数
            'payload_params': {
                'mass': 0.0,  # 负载质量 (kg)
                'center_of_mass': [0.0, 0.0, 0.0]  # 负载质心 (mm)
            },
            
            # 速度标定参数
            'velocity_calibration': {
                'joint_speeds': [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],  # 各关节速度系数
                'acceleration_time': 0.5,  # 加速时间 (s)
                'deceleration_time': 0.5   # 减速时间 (s)
            }
        }
        
        # 标定状态
        self.calibration_status = {
            'is_calibrating': False,
            'current_step': 0,
            'total_steps': 0,
            'calibration_type': None
        }
        
        # 速度标定状态（匹配原始功能）
        self.velocity_collection_status = {
            'is_collecting': False,
            'start_time': None,
            'start_position': None,
            'end_time': None,
            'end_position': None,
            'current_velocity_x': 0.0,
            'current_velocity_y': 0.0
        }
        
        # 加载保存的参数
        self._load_parameters_from_settings()
    
    def set_dh_parameters(self, dh_params: Dict[str, List[float]]) -> bool:
        """设置DH参数"""
        try:
            required_keys = ['a', 'd', 'alpha', 'theta_offset']
            
            # 验证参数格式
            for key in required_keys:
                if key not in dh_params:
                    self.emit_error(f"缺少DH参数: {key}")
                    return False
                if len(dh_params[key]) != 6:
                    self.emit_error(f"DH参数 {key} 必须包含6个值")
                    return False
            
            # 更新DH参数
            self.robot_parameters['dh_params'] = dh_params.copy()
            
            # TODO: 当Domain层运动学服务支持DH参数更新后，调用相应接口
            
            # 发出更新信号
            self.parameters_updated.emit(self.robot_parameters.copy())
            
            self.emit_status("DH参数已更新")
            return True
            
        except Exception as e:
            self.emit_error(f"设置DH参数失败: {str(e)}")
            return False
    
    def set_joint_limits(self, limits: Dict[str, List[float]]) -> bool:
        """设置关节限位"""
        try:
            required_keys = ['min_angles', 'max_angles', 'max_velocities', 'max_accelerations']
            
            # 验证限位格式
            for key in required_keys:
                if key not in limits:
                    self.emit_error(f"缺少关节限位参数: {key}")
                    return False
                if len(limits[key]) != 6:
                    self.emit_error(f"关节限位 {key} 必须包含6个值")
                    return False
            
            # 验证限位逻辑
            for i in range(6):
                if limits['min_angles'][i] >= limits['max_angles'][i]:
                    self.emit_error(f"第{i+1}关节最小角度必须小于最大角度")
                    return False
                if limits['max_velocities'][i] <= 0:
                    self.emit_error(f"第{i+1}关节最大速度必须大于0")
                    return False
                if limits['max_accelerations'][i] <= 0:
                    self.emit_error(f"第{i+1}关节最大加速度必须大于0")
                    return False
            
            # 更新关节限位
            self.robot_parameters['joint_limits'] = limits.copy()
            
            # 发出更新信号
            self.parameters_updated.emit(self.robot_parameters.copy())
            
            self.emit_status("关节限位已更新")
            return True
            
        except Exception as e:
            self.emit_error(f"设置关节限位失败: {str(e)}")
            return False
    
    def set_tool_parameters(self, tool_params: Dict[str, Any]) -> bool:
        """设置工具参数"""
        try:
            # 验证工具参数
            if 'tcp_offset' in tool_params:
                if len(tool_params['tcp_offset']) != 6:
                    self.emit_error("TCP偏移必须包含6个值 [x,y,z,rx,ry,rz]")
                    return False
            
            if 'tool_center_of_mass' in tool_params:
                if len(tool_params['tool_center_of_mass']) != 3:
                    self.emit_error("工具质心必须包含3个值 [x,y,z]")
                    return False
            
            if 'tool_mass' in tool_params:
                if tool_params['tool_mass'] < 0:
                    self.emit_error("工具质量不能为负数")
                    return False
            
            # 更新工具参数
            self.robot_parameters['tool_params'].update(tool_params)
            
            # 发出更新信号
            self.parameters_updated.emit(self.robot_parameters.copy())
            
            self.emit_status("工具参数已更新")
            return True
            
        except Exception as e:
            self.emit_error(f"设置工具参数失败: {str(e)}")
            return False
    
    def set_payload_parameters(self, mass: float, center_of_mass: List[float]) -> bool:
        """设置负载参数"""
        try:
            if mass < 0:
                self.emit_error("负载质量不能为负数")
                return False
            
            if len(center_of_mass) != 3:
                self.emit_error("负载质心必须包含3个值 [x,y,z]")
                return False
            
            # 更新负载参数
            self.robot_parameters['payload_params'] = {
                'mass': mass,
                'center_of_mass': center_of_mass.copy()
            }
            
            # 发出更新信号
            self.parameters_updated.emit(self.robot_parameters.copy())
            
            self.emit_status(f"负载参数已更新 - 质量: {mass}kg, 质心: {center_of_mass}mm")
            return True
            
        except Exception as e:
            self.emit_error(f"设置负载参数失败: {str(e)}")
            return False
    
    def set_velocity_calibration(self, calibration: Dict[str, Any]) -> bool:
        """设置速度标定参数"""
        try:
            if 'joint_speeds' in calibration:
                if len(calibration['joint_speeds']) != 6:
                    self.emit_error("关节速度系数必须包含6个值")
                    return False
                # 验证速度系数范围
                for i, speed in enumerate(calibration['joint_speeds']):
                    if not (0.1 <= speed <= 2.0):
                        self.emit_error(f"第{i+1}关节速度系数必须在0.1-2.0之间")
                        return False
            
            if 'acceleration_time' in calibration:
                if calibration['acceleration_time'] <= 0:
                    self.emit_error("加速时间必须大于0")
                    return False
            
            if 'deceleration_time' in calibration:
                if calibration['deceleration_time'] <= 0:
                    self.emit_error("减速时间必须大于0")
                    return False
            
            # 更新速度标定参数
            self.robot_parameters['velocity_calibration'].update(calibration)
            
            # 发出更新信号
            self.parameters_updated.emit(self.robot_parameters.copy())
            
            self.emit_status("速度标定参数已更新")
            return True
            
        except Exception as e:
            self.emit_error(f"设置速度标定参数失败: {str(e)}")
            return False
    
    def start_calibration(self, calibration_type: str) -> bool:
        """开始参数标定"""
        if self.calibration_status['is_calibrating']:
            self.emit_error("标定正在进行中")
            return False
        
        try:
            if calibration_type not in ['joint_limits', 'tool_tcp', 'payload', 'velocity']:
                self.emit_error(f"无效的标定类型: {calibration_type}")
                return False
            
            # 开始标定
            self.calibration_status = {
                'is_calibrating': True,
                'current_step': 0,
                'total_steps': self._get_calibration_steps(calibration_type),
                'calibration_type': calibration_type
            }
            
            # TODO: 当Infrastructure层实现后，启动真实的标定程序
            # 目前模拟标定过程
            self.emit_status(f"开始{calibration_type}标定 - {self.calibration_status['total_steps']}步")
            
            # 模拟标定完成
            self._simulate_calibration_completion(True, "标定完成")
            
            return True
            
        except Exception as e:
            self.emit_error(f"开始标定失败: {str(e)}")
            self.calibration_status['is_calibrating'] = False
            return False
    
    def stop_calibration(self) -> bool:
        """停止标定"""
        try:
            if not self.calibration_status['is_calibrating']:
                self.emit_status("没有正在进行的标定")
                return True
            
            self.calibration_status['is_calibrating'] = False
            self.calibration_completed.emit(False, "标定已停止")
            
            self.emit_status("标定已停止")
            return True
            
        except Exception as e:
            self.emit_error(f"停止标定失败: {str(e)}")
            return False
    
    def _get_calibration_steps(self, calibration_type: str) -> int:
        """获取标定步骤数"""
        steps_map = {
            'joint_limits': 12,  # 每个关节正负限位
            'tool_tcp': 6,      # TCP标定点
            'payload': 3,       # 负载测量点
            'velocity': 6       # 各关节速度测试
        }
        return steps_map.get(calibration_type, 1)
    
    def _simulate_calibration_completion(self, success: bool, message: str):
        """模拟标定完成"""
        self.calibration_status['is_calibrating'] = False
        self.calibration_completed.emit(success, message)
    
    def save_parameters(self, file_path: Optional[str] = None) -> bool:
        """保存参数到文件"""
        try:
            if file_path:
                # 保存到指定文件
                import json
                with open(file_path, 'w', encoding='utf-8') as f:
                    json.dump(self.robot_parameters, f, indent=2, ensure_ascii=False)
                self.emit_status(f"参数已保存到: {file_path}")
            else:
                # 保存到应用设置
                self._save_parameters_to_settings()
                self.emit_status("参数已保存到应用设置")
            
            self.parameters_saved.emit(file_path or "应用设置")
            return True
            
        except Exception as e:
            self.emit_error(f"保存参数失败: {str(e)}")
            return False
    
    def load_parameters(self, file_path: Optional[str] = None) -> bool:
        """从文件加载参数"""
        try:
            if file_path:
                # 从指定文件加载
                import json
                with open(file_path, 'r', encoding='utf-8') as f:
                    loaded_params = json.load(f)
                
                # 验证参数格式
                if self._validate_parameters(loaded_params):
                    self.robot_parameters = loaded_params
                    self.emit_status(f"参数已从文件加载: {file_path}")
                else:
                    self.emit_error("参数文件格式无效")
                    return False
            else:
                # 从应用设置加载
                self._load_parameters_from_settings()
                self.emit_status("参数已从应用设置加载")
            
            # 发出参数加载信号
            self.parameters_loaded.emit(self.robot_parameters.copy())
            self.parameters_updated.emit(self.robot_parameters.copy())
            
            return True
            
        except Exception as e:
            self.emit_error(f"加载参数失败: {str(e)}")
            return False
    
    def _validate_parameters(self, params: Dict[str, Any]) -> bool:
        """验证参数格式"""
        try:
            required_keys = ['dh_params', 'joint_limits', 'tool_params', 'payload_params', 'velocity_calibration']
            for key in required_keys:
                if key not in params:
                    return False
            
            # 验证DH参数
            dh_keys = ['a', 'd', 'alpha', 'theta_offset']
            for key in dh_keys:
                if key not in params['dh_params'] or len(params['dh_params'][key]) != 6:
                    return False
            
            return True
            
        except Exception:
            return False
    
    def _save_parameters_to_settings(self):
        """保存参数到应用设置"""
        # 将机器人参数保存到配置中
        self.app_settings.robot_config.__dict__.update({
            'dh_params': self.robot_parameters['dh_params'],
            'joint_limits': self.robot_parameters['joint_limits'],
            'tool_params': self.robot_parameters['tool_params'],
            'payload_params': self.robot_parameters['payload_params'],
            'velocity_calibration': self.robot_parameters['velocity_calibration']
        })
        self.app_settings.save()
    
    def _load_parameters_from_settings(self):
        """从应用设置加载参数"""
        try:
            self.app_settings.load()
            # 从配置中恢复参数（如果存在）
            # 目前使用默认参数
            
            # 加载历史速度参数并发出信号（匹配原始功能）
            if 'measured_velocity' in self.robot_parameters['velocity_calibration']:
                vx = self.robot_parameters['velocity_calibration']['measured_velocity'].get('vx', 0.0)
                vy = self.robot_parameters['velocity_calibration']['measured_velocity'].get('vy', 0.0)
                # 延迟发出历史速度信号，确保UI组件已经初始化
                QTimer.singleShot(100, lambda: self.history_velocity_updated.emit(vx, vy))
            else:
                # 如果没有历史数据，发出默认值
                QTimer.singleShot(100, lambda: self.history_velocity_updated.emit(0.0, 0.0))
                
        except Exception as e:
            self.emit_error(f"从设置加载参数失败: {str(e)}")
    
    def get_parameters(self) -> Dict[str, Any]:
        """获取当前参数"""
        return self.robot_parameters.copy()
    
    def get_calibration_status(self) -> Dict[str, Any]:
        """获取标定状态"""
        return self.calibration_status.copy()
    
    def reset_to_defaults(self) -> bool:
        """重置为默认参数"""
        try:
            # 重置为默认参数
            self.__init__(self.parent())
            
            # 发出更新信号
            self.parameters_updated.emit(self.robot_parameters.copy())
            
            self.emit_status("参数已重置为默认值")
            return True
            
        except Exception as e:
            self.emit_error(f"重置参数失败: {str(e)}")
            return False
    
    def start_velocity_collection(self):
        """开始速度收集（匹配原始功能）"""
        if not self.velocity_collection_status['is_collecting']:
            self.velocity_collection_status = {
                'is_collecting': True,
                'start_time': None,
                'start_position': None,
                'end_time': None,
                'end_position': None,
                'current_velocity_x': 0.0,
                'current_velocity_y': 0.0
            }
            
            self.emit_status("开始速度收集，等待检测结果...")
            
            # 模拟检测到起始位置（真实应用中会从摄像头检测服务获取）
            import time
            self.velocity_collection_status['start_time'] = time.time()
            self.velocity_collection_status['start_position'] = [100.0, 100.0]  # 模拟位置
            
            self.emit_status(f"记录起始位置: ({self.velocity_collection_status['start_position'][0]:.1f}, {self.velocity_collection_status['start_position'][1]:.1f})")
    
    def stop_velocity_collection(self):
        """停止速度收集（匹配原始功能）"""
        if self.velocity_collection_status['is_collecting']:
            self.velocity_collection_status['is_collecting'] = False
            
            # 记录结束位置和时间
            import time
            self.velocity_collection_status['end_time'] = time.time()
            self.velocity_collection_status['end_position'] = [200.0, 150.0]  # 模拟位置
            
            self.emit_status(f"记录结束位置: ({self.velocity_collection_status['end_position'][0]:.1f}, {self.velocity_collection_status['end_position'][1]:.1f})")
            
            # 计算速度
            self._calculate_velocity()
            
            self.emit_status("速度收集已停止")
    
    def _calculate_velocity(self):
        """计算速度（匹配原始功能）"""
        try:
            # 检查是否有有效的起始和结束数据
            if (self.velocity_collection_status['start_position'] is None or 
                self.velocity_collection_status['start_time'] is None):
                self.emit_error("错误：未记录到起始位置")
                self.velocity_collection_status['current_velocity_x'] = 0.0
                self.velocity_collection_status['current_velocity_y'] = 0.0
                self.velocity_updated.emit(0.0, 0.0)
                return
                
            if (self.velocity_collection_status['end_position'] is None or 
                self.velocity_collection_status['end_time'] is None):
                self.emit_error("错误：未记录到结束位置")
                self.velocity_collection_status['current_velocity_x'] = 0.0
                self.velocity_collection_status['current_velocity_y'] = 0.0
                self.velocity_updated.emit(0.0, 0.0)
                return
            
            # 计算时间差
            time_duration = self.velocity_collection_status['end_time'] - self.velocity_collection_status['start_time']
            
            if time_duration <= 0:
                self.emit_error("错误：时间间隔无效")
                self.velocity_collection_status['current_velocity_x'] = 0.0
                self.velocity_collection_status['current_velocity_y'] = 0.0
                self.velocity_updated.emit(0.0, 0.0)
                return
            
            # 计算位移
            dx = self.velocity_collection_status['end_position'][0] - self.velocity_collection_status['start_position'][0]
            dy = self.velocity_collection_status['end_position'][1] - self.velocity_collection_status['start_position'][1]
            
            # 计算平均速度 (像素/秒)
            self.velocity_collection_status['current_velocity_x'] = dx / time_duration
            self.velocity_collection_status['current_velocity_y'] = dy / time_duration
            
            # 显示结果
            self.emit_status(f"测量完成 - 时长: {time_duration:.2f}s, 位移: ({dx:.1f}, {dy:.1f})px")
            self.emit_status(f"计算速度: Vx={self.velocity_collection_status['current_velocity_x']:.2f} px/s, Vy={self.velocity_collection_status['current_velocity_y']:.2f} px/s")
            
            # 发送速度更新信号
            self.velocity_updated.emit(
                self.velocity_collection_status['current_velocity_x'], 
                self.velocity_collection_status['current_velocity_y']
            )
            
        except Exception as e:
            self.emit_error(f"计算速度错误: {str(e)}")
            self.velocity_collection_status['current_velocity_x'] = 0.0
            self.velocity_collection_status['current_velocity_y'] = 0.0
            self.velocity_updated.emit(0.0, 0.0)
    
    def get_current_velocity(self):
        """获取当前速度值（匹配原始功能）"""
        return (self.velocity_collection_status['current_velocity_x'], 
                self.velocity_collection_status['current_velocity_y'])
    
    def is_velocity_collecting(self):
        """检查是否正在收集速度（匹配原始功能）"""
        return self.velocity_collection_status['is_collecting']
    
    def save_velocity_to_config(self):
        """保存当前速度值到配置文件（匹配原始功能）"""
        try:
            # 更新速度标定参数
            self.robot_parameters['velocity_calibration']['measured_velocity'] = {
                'vx': self.velocity_collection_status['current_velocity_x'],
                'vy': self.velocity_collection_status['current_velocity_y']
            }
            
            # 保存到应用设置
            self._save_parameters_to_settings()
            
            self.emit_status(f"速度参数已保存: Vx={self.velocity_collection_status['current_velocity_x']:.2f}, Vy={self.velocity_collection_status['current_velocity_y']:.2f}")
            
            # 发送历史速度更新信号
            self.history_velocity_updated.emit(
                self.velocity_collection_status['current_velocity_x'], 
                self.velocity_collection_status['current_velocity_y']
            )
            
            return True
            
        except Exception as e:
            self.emit_error(f"保存速度参数时出错: {str(e)}")
            return False 