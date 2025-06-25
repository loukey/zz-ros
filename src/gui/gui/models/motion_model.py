from .trajectory_model import *
from PyQt5.QtCore import QTimer, QObject, pyqtSignal, Qt
from utils import format_command


class MotionModel(QObject):
    motion_send_signal = pyqtSignal(int, list, int, float)

    def __init__(self, serial_model):
        super().__init__()
        self.serial_model = serial_model
        self.s_curve = SCurve()
        self.motion_timer = QTimer()
        self.motion_timer.setTimerType(Qt.PreciseTimer)
        self.motion_timer.setInterval(10)
        self.motion_timer.timeout.connect(self.motion_timeout)
        self.motion_params = []
        self.motion_index = 0


    def curve_planning(self, start_angles, target_angles, dt=0.01):
        times, accelerations, velocities, positions = self.s_curve.planning(start_angles=start_angles, 
                                                                            target_angles=target_angles, 
                                                                            dt=dt)
        return times, positions

    def set_interval(self, interval):
        self.motion_timer.setInterval(int(interval))

    def clear_motion_data(self):
        self.motion_params = []

    def add_motion_data(self, mode, params):
        if mode == "运动":
            positions = params['positions']  # (n, 6) 形状的数组
            motion_data = []
            for position in positions:
                # 格式：[控制命令, 模式, [六个角度], [夹爪命令, 夹爪参数]]
                row_data = [
                    0x06,  # 控制命令
                    0x08,  # 模式
                    position.tolist(),  # [六个角度]
                    [0x00, 0.0]  # [夹爪命令, 夹爪参数] - 运动时设为默认值
                ]
                motion_data.append(row_data)
            
            if len(self.motion_params) == 0:
                self.motion_params = motion_data
            else:
                self.motion_params.extend(motion_data)
                
        elif mode == "夹爪":
            gripper_command = params['gripper_command']
            gripper_params = params['gripper_param']
            # 格式：[控制命令, 模式, [六个角度], [夹爪命令, 夹爪参数]]
            gripper_data = [
                0x00,  # 控制命令 - 夹爪也使用0x06
                0x08,  # 模式
                self.motion_params[-1][2],  # [六个角度] - 夹爪时设为默认值
                [gripper_command, gripper_params]  # [夹爪命令, 夹爪参数]
            ]
            
            if len(self.motion_params) == 0:
                self.motion_params = [gripper_data]
            else:
                self.motion_params.append(gripper_data)

    def start_motion(self):
        self.motion_index = 0
        self.motion_timer.start()

    def stop_motion(self):
        self.motion_index = 0
        self.motion_timer.stop()

    def motion_timeout(self):
        if self.motion_index >= len(self.motion_params):
            self.stop_motion()
            return

        current_data = self.motion_params[self.motion_index]
        control = current_data[0]
        mode = current_data[1]
        positions = current_data[2]
        gripper_data = current_data[3]
        self.motion_send_signal.emit(mode, positions, gripper_data[0], gripper_data[1])
        self.serial_model.send_control_command(joint_angles=positions, 
                                               control=control, 
                                               mode=mode, 
                                               effector_mode=gripper_data[0],
                                               effector_data=float(gripper_data[1]),
                                               encoding='hex')
        self.motion_index += 1
