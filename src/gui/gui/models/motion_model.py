from .trajectory_model import *
from PyQt5.QtCore import QTimer, QObject, pyqtSignal


class MotionModel(QObject):
    motion_send_signal = pyqtSignal(list)

    def __init__(self, serial_model):
        super().__init__()
        self.serial_model = serial_model
        self.s_curve = SCurve()
        self.motion_timer = QTimer()
        self.motion_timer.setInterval(10)
        self.motion_timer.timeout.connect(self.motion_timeout)
        self.motion_times = np.array([])
        self.motion_positions = np.array([])
        self.motion_times_index = 0


    def curve_planning(self, start_angles, target_angles, dt=0.01):
        times, accelerations, velocities, positions = self.s_curve.planning(start_angles=start_angles, 
                                                                            target_angles=target_angles, 
                                                                            dt=dt)
        return times, positions

    def set_interval(self, interval):
        self.motion_timer.setInterval(int(interval))

    def clear_motion_data(self):
        self.motion_times = np.array([])
        self.motion_positions = np.array([])

    def add_motion_data(self, times, positions):
        if len(self.motion_times) == 0:
            self.motion_times = times
            self.motion_positions = positions
        else:
            times += self.motion_times[-1]
            self.motion_times = np.concatenate((self.motion_times, times))
            self.motion_positions = np.concatenate((self.motion_positions, positions))

    def start_motion(self):
        self.motion_times_index = 0
        self.motion_timer.start()

    def stop_motion(self):
        self.motion_times_index = 0
        self.motion_timer.stop()

    def motion_timeout(self):
        if self.motion_times_index >= len(self.motion_times):
            self.stop_motion()
            return
        positions = self.motion_positions[self.motion_times_index].tolist()
        self.serial_model.send_control_command(joint_angles=positions, 
                                               control=0x06, 
                                               mode=0x08, 
                                               encoding='hex')
        self.motion_send_signal.emit(positions)
        self.motion_times_index += 1
