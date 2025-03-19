"""
机器人控制器
"""
from PyQt5.QtCore import QTimer
from models.robot_model import RobotModel
from controllers.serial_controller import SerialController
from kinematic.velocity_planning import trapezoidal_velocity_planning, s_curve_velocity_planning


class RobotController:
    """机器人控制器，负责控制机器人的状态和运动"""
    
    def __init__(self, serial_controller=None):
        """
        初始化机器人控制器
        
        参数:
            serial_controller: 串口控制器实例，如果为None则自动创建
        """
        self.robot_model = RobotModel()
        self.serial_controller = serial_controller or SerialController()
        self.trajectory_timer = None      # 轨迹发送定时器
        self.trajectory_data = None       # 轨迹数据
        self.trajectory_index = 0         # 当前轨迹点索引
    
    def enable_robot(self):
        """启用机器人"""
        self.serial_controller.send_control_command("ENABLE")
    
    def disable_robot(self):
        """禁用机器人"""
        self.serial_controller.send_control_command("DISABLE")
    
    def release_brake(self):
        """释放制动"""
        self.serial_controller.send_control_command("RELEASE")
    
    def lock_brake(self):
        """锁定制动"""
        self.serial_controller.send_control_command("LOCK")
    
    def stop_robot(self):
        """停止机器人"""
        self.serial_controller.send_control_command("STOP")
    
    def move_robot(self, target_angles, duration=5.0, frequency=0.01, curve_type="Trapezoid"):
        """移动机器人到指定角度
        
        参数:
            target_angles: 目标角度列表，包含6个关节角度
            duration: 运动持续时间，默认5秒
            frequency: 采样频率，默认0.01秒
            curve_type: 曲线类型，默认梯形曲线
        """
        return self.serial_controller.send_angles(
            angles=target_angles,
            curve_type=curve_type,
            duration=duration,
            frequency=frequency
        )
    
    def calculate_inverse_kinematics(self, x, y, z, A, B, C):
        """
        计算逆运动学
        
        参数:
            x, y, z: 目标位置
            A, B, C: 目标欧拉角
            
        返回:
            angles: 关节角度列表
        """
        return self.robot_model.calculate_inverse_kinematics(x, y, z, A, B, C)
    
    def get_end_position(self):
        """
        获取末端位置和姿态
        
        返回:
            position: 包含位置和欧拉角的元组 (x, y, z, A, B, C)
        """
        return self.robot_model.get_end_position()
    
    def register_data_received_callback(self, callback):
        """
        注册数据接收回调函数
        
        参数:
            callback: 回调函数，接收一个字符串参数
        """
        self.serial_controller.register_data_received_callback(callback)
    
    def register_connection_changed_callback(self, callback):
        """
        注册连接状态变化回调函数
        
        参数:
            callback: 回调函数，接收一个布尔参数
        """
        self.serial_controller.register_connection_changed_callback(callback)
    
    def register_error_occurred_callback(self, callback):
        """
        注册错误发生回调函数
        
        参数:
            callback: 回调函数，接收一个字符串参数
        """
        self.serial_controller.register_error_occurred_callback(callback) 