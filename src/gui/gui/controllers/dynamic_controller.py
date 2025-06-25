from .base_controller import BaseController
from PyQt5.QtCore import pyqtSlot
from kinematic import *
from config import GlobalVars


class DynamicController(BaseController):
    def __init__(self, serial_model):
        super().__init__()
        self.serial_model = serial_model
        self.dynamic = Dynamic()

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
    def send_dynamic_torque_command(self, torgue):
        success, cmd = self.serial_model.send_control_command(control=0x06, mode=0x0A, torque=torgue, return_cmd=True)
        self.display(f"力矩补偿力矩: {torgue}", "发送")
        self.display(f"力矩补偿力矩: {cmd}", "发送")
