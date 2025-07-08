from .base_controller import BaseController
from PyQt5.QtCore import pyqtSlot
from gui.kinematic import *
from gui.config import GlobalVars


class DynamicController(BaseController):
    def __init__(self, serial_model, motion_controller=None):
        super().__init__()
        self.serial_model = serial_model
        self.motion_controller = motion_controller
        self.dynamic = Dynamic()
        self.recording_enabled = False

    @pyqtSlot(list)
    def handle_dynamic_torque_calculation_command_requested(self, joint_angles):
        torque = self.dynamic.compute_gravity_compensation(joint_angles)
        torque = [torque[i] + GlobalVars.get_work_array()[i] for i in range(6)]
        self.display(f"静摩擦力: {GlobalVars.get_work_array()}", "发送")
        success, cmd = self.serial_model.send_control_command(control=0x06, mode=0x0A, torque=torque, return_cmd=True)
        self.display(f"重力补偿力矩: {torque}", "发送")
        self.display(f"重力补偿力矩: {cmd}", "发送")

    @pyqtSlot()
    def handle_enable_dynamic_command_requested(self):
        success, cmd = self.serial_model.send_control_command(control=0x01, mode=0x0A, return_cmd=True)
        self.display(f"周期力矩模式使能: {cmd}", "发送")

    @pyqtSlot(bool)
    def handle_dynamic_command_requested(self, teaching_mode):
        # 设置全局变量示教模式标志位
        GlobalVars.set_dynamic_teach_flag(teaching_mode)
        if teaching_mode:
            success, cmd = self.serial_model.send_control_command(control=0x07, mode=0x0A, return_cmd=True)
            GlobalVars.clear_array()
            self.display(f"开启示教模式: {cmd}", "发送")
        else:
            success, cmd = self.serial_model.send_control_command(control=0x05, mode=0x0A, return_cmd=True)
            GlobalVars.clear_array()
            self.display(f"关闭示教模式: {cmd}", "发送")

    @pyqtSlot(list)
    def send_dynamic_torque_command(self, torque):
        success, cmd = self.serial_model.send_control_command(control=0x06, mode=0x0A, torque=torque, return_cmd=True)
        self.display(f"力矩补偿力矩: {torque}", "发送")
        self.display(f"力矩补偿力矩: {cmd}", "发送")
    
    @pyqtSlot()
    def handle_start_recording_requested(self):
        """处理开始记录请求"""
        self.recording_enabled = True
        self.display("开始记录示教轨迹", "记录")
    
    @pyqtSlot()
    def handle_stop_recording_requested(self):
        """处理停止记录请求"""
        self.recording_enabled = False
        self.display("停止记录示教轨迹", "记录")
    
    @pyqtSlot(str)
    def handle_delete_record_requested(self, record_name):
        """处理删除记录请求"""
        self.display(f"删除记录: {record_name}", "记录")
    
    @pyqtSlot(list)
    def handle_run_record_requested(self, angles_list):
        """处理运行记录请求"""
        if self.motion_controller:
            self.display(f"运行记录: 共{len(angles_list)}个点", "记录")
            self.motion_controller.teach_motion_setting(angles_list)
        else:
            self.display("运动控制器未初始化", "错误")
    
    def handle_record_timer_timeout(self, dynamics_frame):
        """处理记录定时器超时"""
        if self.recording_enabled:
            # 获取当前关节角度
            current_angles = GlobalVars.get_current_joint_angles()
            
            # 添加到当前记录
            dynamics_frame.add_angle_to_current_record(current_angles)
    
    def connect_dynamics_frame_signals(self, dynamics_frame):
        """连接动力学框架的信号"""
        dynamics_frame.start_recording_requested.connect(self.handle_start_recording_requested)
        dynamics_frame.stop_recording_requested.connect(self.handle_stop_recording_requested)
        dynamics_frame.delete_record_requested.connect(self.handle_delete_record_requested)
        dynamics_frame.run_record_requested.connect(self.handle_run_record_requested)
        
        # 连接记录定时器
        record_timer = dynamics_frame.get_record_timer()
        record_timer.timeout.connect(lambda: self.handle_record_timer_timeout(dynamics_frame))
