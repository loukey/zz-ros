"""
机器人状态数据模型
"""
from kinematic.kinematic_6dof import Kinematic6DOF


class RobotModel:
    """机器人数据模型类，包含机器人的当前状态和运动学计算"""
    
    def __init__(self):
        self.current_angles = [0.0] * 6  # 当前关节角度
        self.target_angles = [0.0] * 6   # 目标关节角度
        self.is_enabled = False          # 是否使能
        self.is_brake_released = False   # 刹车是否释放
        self.kinematics = Kinematic6DOF()  # 运动学模型
        self.trajectory_data = None      # 轨迹数据
        
    def update_current_angles(self, angles):
        """
        更新当前关节角度
        
        参数:
            angles: 关节角度列表，包含6个元素
        """
        if len(angles) != 6:
            raise ValueError("必须提供6个关节角度")
        self.current_angles = angles.copy()
        
    def update_target_angles(self, angles):
        """
        更新目标关节角度
        
        参数:
            angles: 关节角度列表，包含6个元素
        """
        if len(angles) != 6:
            raise ValueError("必须提供6个关节角度")
        self.target_angles = angles.copy()
        
    def set_enabled(self, enabled):
        """
        设置使能状态
        
        参数:
            enabled: 是否使能
        """
        self.is_enabled = enabled
        
    def set_brake_released(self, released):
        """
        设置刹车状态
        
        参数:
            released: 是否释放刹车
        """
        self.is_brake_released = released
        
    def get_end_position(self):
        """
        获取末端位置和姿态
        
        返回:
            position: 包含位置和欧拉角的元组 (x, y, z, A, B, C)
        """
        return self.kinematics.get_end_position(self.current_angles)
    
    def calculate_inverse_kinematics(self, x, y, z, A, B, C):
        """
        计算逆运动学
        
        参数:
            x, y, z: 目标位置
            A, B, C: 目标欧拉角
            
        返回:
            angles: 关节角度列表
        """
        return self.kinematics.inverse_kinematic(A, B, C, x, y, z) 