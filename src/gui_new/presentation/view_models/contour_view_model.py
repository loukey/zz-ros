"""
轮廓视图模型
"""
from typing import List, Dict, Any, Optional
from PyQt5.QtCore import pyqtSignal
from .base_view_model import BaseViewModel
from application.services.robot_application_service import RobotApplicationService
from shared.config.container import ServiceLocator


class ContourViewModel(BaseViewModel):
    """轮廓视图模型"""
    
    # 轮廓特有信号
    contour_parameters_changed = pyqtSignal(dict)  # 轮廓参数变化
    speed_profile_updated = pyqtSignal(list)  # 速度曲线更新
    acceleration_profile_updated = pyqtSignal(list)  # 加速度曲线更新
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # 获取Application Service
        self.robot_service = ServiceLocator.resolve(RobotApplicationService)
        
        # 轮廓参数
        self.contour_speeds = [1.0] * 6  # 各关节轮廓速度 (rad/s)
        self.contour_accelerations = [2.0] * 6  # 各关节轮廓加速度 (rad/s²)
        self.contour_decelerations = [2.0] * 6  # 各关节轮廓减速度 (rad/s²)
        
        # 全局轮廓参数
        self.global_speed_ratio = 1.0  # 全局速度比例 (0.1 - 2.0)
        self.global_acceleration_ratio = 1.0  # 全局加速度比例 (0.1 - 2.0)
        
        # 轮廓模式
        self.contour_mode = "independent"  # independent, synchronized
        self.curve_type = "S_CURVE"  # 曲线类型
        
        # 速度限制
        self.max_speeds = [3.14, 3.14, 3.14, 6.28, 6.28, 6.28]  # 各关节最大速度限制
        self.max_accelerations = [10.0] * 6  # 各关节最大加速度限制
    
    def set_contour_speeds(self, speeds: List[float]) -> bool:
        """设置轮廓速度"""
        try:
            if len(speeds) != 6:
                self.emit_error("轮廓速度必须包含6个关节")
                return False
            
            # 验证速度范围
            for i, speed in enumerate(speeds):
                if speed <= 0 or speed > self.max_speeds[i]:
                    self.emit_error(f"第{i+1}关节速度超出范围 (0 - {self.max_speeds[i]:.2f})")
                    return False
            
            self.contour_speeds = speeds.copy()
            self.speed_profile_updated.emit(speeds)
            
            # 发出参数变化信号
            params = self._get_contour_parameters()
            self.contour_parameters_changed.emit(params)
            
            self.emit_status(f"轮廓速度已设置: {[f'{s:.3f}' for s in speeds]} rad/s")
            return True
            
        except Exception as e:
            self.emit_error(f"设置轮廓速度失败: {str(e)}")
            return False
    
    def set_contour_accelerations(self, accelerations: List[float], 
                                decelerations: Optional[List[float]] = None) -> bool:
        """设置轮廓加速度和减速度"""
        try:
            if len(accelerations) != 6:
                self.emit_error("轮廓加速度必须包含6个关节")
                return False
            
            if decelerations and len(decelerations) != 6:
                self.emit_error("轮廓减速度必须包含6个关节")
                return False
            
            # 如果未提供减速度，使用加速度值
            if decelerations is None:
                decelerations = accelerations.copy()
            
            # 验证加速度范围
            for i, (accel, decel) in enumerate(zip(accelerations, decelerations)):
                if accel <= 0 or accel > self.max_accelerations[i]:
                    self.emit_error(f"第{i+1}关节加速度超出范围 (0 - {self.max_accelerations[i]:.2f})")
                    return False
                if decel <= 0 or decel > self.max_accelerations[i]:
                    self.emit_error(f"第{i+1}关节减速度超出范围 (0 - {self.max_accelerations[i]:.2f})")
                    return False
            
            self.contour_accelerations = accelerations.copy()
            self.contour_decelerations = decelerations.copy()
            self.acceleration_profile_updated.emit(accelerations)
            
            # 发出参数变化信号
            params = self._get_contour_parameters()
            self.contour_parameters_changed.emit(params)
            
            self.emit_status(f"轮廓加速度已设置: {[f'{a:.2f}' for a in accelerations]} rad/s²")
            return True
            
        except Exception as e:
            self.emit_error(f"设置轮廓加速度失败: {str(e)}")
            return False
    
    def set_global_ratios(self, speed_ratio: float, acceleration_ratio: float) -> bool:
        """设置全局速度和加速度比例"""
        try:
            if not (0.1 <= speed_ratio <= 2.0):
                self.emit_error("速度比例必须在0.1-2.0之间")
                return False
            
            if not (0.1 <= acceleration_ratio <= 2.0):
                self.emit_error("加速度比例必须在0.1-2.0之间")
                return False
            
            self.global_speed_ratio = speed_ratio
            self.global_acceleration_ratio = acceleration_ratio
            
            # 发出参数变化信号
            params = self._get_contour_parameters()
            self.contour_parameters_changed.emit(params)
            
            self.emit_status(f"全局比例已设置 - 速度: {speed_ratio:.2f}, 加速度: {acceleration_ratio:.2f}")
            return True
            
        except Exception as e:
            self.emit_error(f"设置全局比例失败: {str(e)}")
            return False
    
    def set_contour_mode(self, mode: str) -> bool:
        """设置轮廓模式"""
        try:
            if mode not in ["independent", "synchronized"]:
                self.emit_error("轮廓模式必须是 'independent' 或 'synchronized'")
                return False
            
            self.contour_mode = mode
            
            # 发出参数变化信号
            params = self._get_contour_parameters()
            self.contour_parameters_changed.emit(params)
            
            self.emit_status(f"轮廓模式设置为: {mode}")
            return True
            
        except Exception as e:
            self.emit_error(f"设置轮廓模式失败: {str(e)}")
            return False
    
    def set_curve_type(self, curve_type: str) -> bool:
        """设置曲线类型"""
        try:
            if curve_type not in ["S_CURVE", "TRAPEZOIDAL", "LINEAR"]:
                self.emit_error("曲线类型必须是 'S_CURVE', 'TRAPEZOIDAL' 或 'LINEAR'")
                return False
            
            self.curve_type = curve_type
            
            # 发出参数变化信号
            params = self._get_contour_parameters()
            self.contour_parameters_changed.emit(params)
            
            self.emit_status(f"曲线类型设置为: {curve_type}")
            return True
            
        except Exception as e:
            self.emit_error(f"设置曲线类型失败: {str(e)}")
            return False
    
    def get_effective_speeds(self) -> List[float]:
        """获取有效速度（考虑全局比例）"""
        return [speed * self.global_speed_ratio for speed in self.contour_speeds]
    
    def get_effective_accelerations(self) -> List[float]:
        """获取有效加速度（考虑全局比例）"""
        return [accel * self.global_acceleration_ratio for accel in self.contour_accelerations]
    
    def get_effective_decelerations(self) -> List[float]:
        """获取有效减速度（考虑全局比例）"""
        return [decel * self.global_acceleration_ratio for decel in self.contour_decelerations]
    
    def _get_contour_parameters(self) -> Dict[str, Any]:
        """获取完整的轮廓参数"""
        return {
            'speeds': self.contour_speeds.copy(),
            'accelerations': self.contour_accelerations.copy(),
            'decelerations': self.contour_decelerations.copy(),
            'effective_speeds': self.get_effective_speeds(),
            'effective_accelerations': self.get_effective_accelerations(),
            'effective_decelerations': self.get_effective_decelerations(),
            'global_speed_ratio': self.global_speed_ratio,
            'global_acceleration_ratio': self.global_acceleration_ratio,
            'contour_mode': self.contour_mode,
            'curve_type': self.curve_type
        }
    
    def get_contour_parameters(self) -> Dict[str, Any]:
        """获取轮廓参数（供外部调用）"""
        return self._get_contour_parameters()
    
    def reset_to_defaults(self) -> bool:
        """重置为默认参数"""
        try:
            self.contour_speeds = [1.0] * 6
            self.contour_accelerations = [2.0] * 6
            self.contour_decelerations = [2.0] * 6
            self.global_speed_ratio = 1.0
            self.global_acceleration_ratio = 1.0
            self.contour_mode = "independent"
            self.curve_type = "S_CURVE"
            
            # 发出更新信号
            self.speed_profile_updated.emit(self.contour_speeds)
            self.acceleration_profile_updated.emit(self.contour_accelerations)
            
            params = self._get_contour_parameters()
            self.contour_parameters_changed.emit(params)
            
            self.emit_status("轮廓参数已重置为默认值")
            return True
            
        except Exception as e:
            self.emit_error(f"重置轮廓参数失败: {str(e)}")
            return False
    
    def validate_parameters(self) -> bool:
        """验证当前参数的有效性"""
        try:
            # 验证速度
            for i, speed in enumerate(self.contour_speeds):
                if speed <= 0 or speed > self.max_speeds[i]:
                    self.emit_error(f"第{i+1}关节速度无效")
                    return False
            
            # 验证加速度
            for i, accel in enumerate(self.contour_accelerations):
                if accel <= 0 or accel > self.max_accelerations[i]:
                    self.emit_error(f"第{i+1}关节加速度无效")
                    return False
            
            # 验证减速度
            for i, decel in enumerate(self.contour_decelerations):
                if decel <= 0 or decel > self.max_accelerations[i]:
                    self.emit_error(f"第{i+1}关节减速度无效")
                    return False
            
            self.emit_status("轮廓参数验证通过")
            return True
            
        except Exception as e:
            self.emit_error(f"参数验证失败: {str(e)}")
            return False 