from .base_controller import BaseController
from PyQt5.QtCore import pyqtSlot, QThread, pyqtSignal
from utils import *
from config import GlobalVars
import time
from math import pi
import numpy as np
from scipy.signal import savgol_filter
from scipy.interpolate import CubicSpline


class MotionController(BaseController):    
    only_get_current_position_signal = pyqtSignal(list)
    torque_calculation_signal = pyqtSignal(list)

    def __init__(self, serial_model, motion_model, robot_model=None):
        super().__init__()
        self.serial_model = serial_model
        self.motion_model = motion_model
        self.robot_model = robot_model
        # 0: 只获取当前位置
        # 1: 单次轨迹发送   
        # 2: 周期轨迹发送
        self.motion_mode = 0
        self.target_angles = [0.0] * 6
        self.target_angles_list = []
        self.dt = 0.01
        self.buffer = ""
        self.motion_data = []
        self.tech_data = []
        self.motion_model.motion_send_signal.connect(self.single_motion_send)
        self.serial_model.data_received.connect(self.handle_data_received)
        self.serial_model.error_occurred.connect(self.handle_error_occurred)

        # 示教模式标志位
        self.dynamic_teach_flag = False

    @pyqtSlot(dict)
    def handle_angles_requested(self, params):
        self.send_angles(**params)

    @pyqtSlot(dict)
    def handle_command_requested(self, params):
        self.send_control_command(**params)

    def send_control_command(self, 
                             joint_angles=[0.0] * 6, 
                             control=0x00, 
                             mode=0x08, 
                             contour_speed=[0.0] * 6, 
                             contour_acceleration=[0.0] * 6, 
                             contour_deceleration=[0.0] * 6, 
                             torque=[0.0] * 6,
                             effector_mode=0x00, 
                             effector_data=0.0, 
                             encoding='hex', 
                             return_cmd=True):


        try:
            if control == 0x05:
                self.display("轨迹发送已暂停", "控制")
                self.motion_model.stop_motion()
            if control == 0x08:
                self.display("轨迹发送已停止", "控制")
                self.motion_model.stop_motion()
                
            success, cmd = self.serial_model.send_control_command(joint_angles=joint_angles, 
                                                                  control=control, 
                                                                  mode=mode, 
                                                                  contour_speed=contour_speed, 
                                                                  contour_acceleration=contour_acceleration, 
                                                                  contour_deceleration=contour_deceleration, 
                                                                  torque=torque,
                                                                  effector_mode=effector_mode, 
                                                                  effector_data=effector_data,
                                                                  encoding=encoding,
                                                                  return_cmd=return_cmd)
            if not success:
                self.display(f"发送命令失败: control: {control}, mode: {mode}", "错误")
                return (False, "") if return_cmd else False
            if return_cmd:
                self.display(f"发送命令成功: {cmd}", "控制")
                return True, cmd
            return True
            
        except Exception as e:
            if hasattr(self, 'error_occurred'):
                self.error_occurred.emit(f"发送控制命令失败: {str(e)}")
            return (False, "") if return_cmd else False  
    
    def send_angles(self, 
                    target_angles, 
                    curve_type='S型', 
                    frequency=0.01,
                    contour_params=[[0.0] * 6, [0.0] * 6, [0.0] * 6],
                    encoding_type='hex',
                    run_mode=0x08):
        self.display(f"目标角度: [{', '.join([f'{angle:.4f}' for angle in target_angles])}]", "控制")
        contour_speed, contour_acceleration, contour_deceleration = contour_params
        if run_mode == 0x01:
            success, cmd = self.send_control_command(
                joint_angles=target_angles,
                control=0x06,
                mode=run_mode,
                contour_speed=contour_speed,
                contour_acceleration=contour_acceleration,
                contour_deceleration=contour_deceleration,
                encoding=encoding_type,
                return_cmd=True
            )
            return
        
        self.display(f"曲线类型: {curve_type}, 频率: {frequency}秒", "参数")
        self.target_angles = target_angles
        self.dt = frequency
        self.motion_mode = 1
        self.get_current_position(encoding_type=encoding_type)
    
    def get_current_position(self, encoding_type='hex'):
        # 获取当前位置
        self.display("正在获取当前位置...", "控制")
        success, cmd = self.serial_model.send_control_command(
            control=0x07, 
            encoding=encoding_type,
            return_cmd=True
        )
        self.display(f"{cmd}", "发送")
        if not success:
            self.display("发送获取当前位置命令失败", "错误")

    def handle_data_received(self, data):
        """处理接收到的数据"""
        clean_data = data.strip()
        self.display(f"接收数据: '{clean_data}'", "接收")
        self.buffer += clean_data
        if "0D0A" in self.buffer:
            lines = self.buffer.split("0D0A")
            self.buffer = lines[-1]
            command_line = lines[0]
            
            if command_line.startswith("AA55"):
                try:
                    # 帧头 (2字节)
                    header = command_line[0:4]  # AA55
                    
                    # 初始化状态 (1字节)
                    init_status = command_line[4:6]
                    
                    # 当前命令 (1字节)
                    current_command = command_line[6:8]
                    
                    # 运行模式 (1字节)
                    run_mode = command_line[8:10]
                    
                    # 位置1-6 (每个4字节，共24字节)
                    positions = []
                    start = 10
                    for _ in range(6):
                        pos = int(command_line[start:start+8], 16)
                        if pos & 0x80000000:
                            pos = pos - 0x100000000
                        positions.append(pos)
                        start += 8
                    
                    # 将编码器位置转换为弧度角度
                    positions_rad = position_to_radian(positions)
                    init_rad = [0.0, -pi/2, 0.0, pi/2, 0.0, 0.0]
                    # 更新全局位姿变量（每次解析到位置数据都更新）
                    GlobalVars.update_joint_pose(positions, (np.array(positions_rad) + np.array(init_rad)).tolist())

                    # 状态字1-6 (每个2字节，共12字节)
                    status = []
                    for _ in range(6):
                        stat = command_line[start:start+4]
                        status.append(stat)
                        start += 4
                    # 实际速度1-6 (每个4字节，24字节)
                    speeds = []
                    for _ in range(6):
                        speed = int(command_line[start:start+8], 16)
                        if speed & 0x80000000:
                            speed = speed - 0x100000000
                        speeds.append(speed)
                        start += 8
                    # 力矩1-6 (每个2字节，12字节)
                    torques = []
                    for _ in range(6):
                        torque = int(command_line[start:start+4], 16)
                        if torque & 0x8000:
                            torque = torque - 0x10000
                        torques.append(torque)
                        start += 4
                    # 双编码器插值 (每个4字节，24字节)
                    double_encoder_interpolations = []
                    for _ in range(6):
                        double_encoder_interpolation = int(command_line[start:start+8], 16)
                        if double_encoder_interpolation & 0x80000000:
                            double_encoder_interpolation = double_encoder_interpolation - 0x100000000
                        double_encoder_interpolations.append(double_encoder_interpolation)
                        start += 8
                    # 错误码1-6 (每个2字节，共12字节)
                    errors = []
                    for _ in range(6):
                        error = command_line[start:start+4]
                        errors.append(error)
                        start += 4
                    # 夹爪数据 (4字节)
                    effector_data_1 = int(command_line[start:start+4], 16)
                    effector_data_2 = int(command_line[start+4:start+8], 16)
                    effector_data = "{}.{}".format(effector_data_1, effector_data_2)
                    start += 8
                    # CRC16 (2字节)
                    crc = command_line[start:start+4]
                    crc_message = command_line[:-4]
                    calculated_crc = calculate_crc16(crc_message)
                    calculated_crc_hex = f"{calculated_crc:04X}"
                    if calculated_crc_hex == crc:
                        self.display(f"CRC校验: 正确 (接收: {crc}, 计算: {calculated_crc_hex})", "接收")
                    else:
                        self.display(f"CRC校验: 错误 (接收: {crc}, 计算: {calculated_crc_hex})", "错误")
                    # 记录解析后的详细信息
                    return_msg = f"帧头: {header}, 初始状态: {init_status}, 当前命令: {current_command}, 运行模式: {run_mode}, 位置数据: {positions}, 状态字: {status}, 实际速度: {speeds}, 力矩: {torques}, 双编码器插值: {double_encoder_interpolations}, 错误码:{errors}, 夹爪数据: {effector_data}, CRC16: {crc}"
                    self.display(return_msg, "接收")
                except Exception as e:
                    self.display(f"解析AA55数据帧失败: {str(e)}", "错误")
                    self.display(f"重发数据: {GlobalVars.temp_cmd}", "发送")
                    if self.serial_model.send_handler:
                        self.serial_model.send_handler.add_to_queue(GlobalVars.temp_cmd)
                    return
                if run_mode == "0A":                            
                    if GlobalVars.dynamic_teach_flag and current_command in ["06", "07"]:
                        GlobalVars.add_to_array(positions)
                        self.torque_calculation_signal.emit(positions_rad)
                elif run_mode == "08":
                    if current_command == "07":
                        if self.motion_mode == 0:
                            self.only_get_current_position(positions_rad)
                        elif self.motion_mode == 1:
                            self.motion_mode = 0
                            self.single_motion_starter(positions_rad)
                        elif self.motion_mode == 2:
                            self.motion_mode = 0
                            self.multiple_motion_starter(positions_rad)
                        elif self.motion_mode == 3:
                            self.motion_mode = 0
                            self.teach_motion_starter(positions_rad)
    
    def handle_error_occurred(self, error_msg):
        self.display(f"{error_msg}", "错误")

    def single_motion_send(self, mode, positions, gripper_command, gripper_param):
        self.display(f"模式: {mode}, 目标角度: [{', '.join([f'{pos:.4f}' for pos in positions])}], 夹爪命令: {gripper_command}, 夹爪参数: {gripper_param}", "控制")

    def single_motion_starter(self, start_angles):
        times, positions = self.motion_model.curve_planning(start_angles, self.target_angles, dt=self.dt)
        self.display(f"轨迹计算完成: 共{len(positions)}个点, 总时长{times[-1] if len(times) > 0 else 0}秒", 
                    "控制")
        self.motion_model.clear_motion_data()
        self.motion_model.set_interval(self.dt * 1000)
        self.motion_model.add_motion_data("运动", {'positions': positions})
        self.display(f"开始发送轨迹: 共{len(positions)}个点", "控制")
        self.motion_model.start_motion()

    def only_get_current_position(self, positions):
        self.display(f"获取当前角度: [{', '.join([f'{pos:.4f}' for pos in positions])}]", "控制")
        self.only_get_current_position_signal.emit(positions)

    def multiple_motion_setting(self, motion_data):
        self.motion_data = motion_data
        self.motion_mode = 2
        self.display(f"启动混合运动规划", "控制")
        self.get_current_position()

    def multiple_motion_starter(self, start_angles):
        self.motion_model.clear_motion_data()
        self.motion_model.set_interval(self.dt * 1000)
        for i, motion_data in enumerate(self.motion_data):
            mode = motion_data['mode']
            curve_type = motion_data['curve_type']
            frequency = motion_data['frequency']
            gripper_command = motion_data['gripper_command']
            gripper_param = motion_data['gripper_param']
            target_angles = [motion_data[f'joint{i+1}'] for i in range(6)]
            note = motion_data['note']
            if mode == "夹爪":
                command_mode = {
                    "00: 不进行任何操作": 0x00,
                    "01: 夹爪手动使能": 0x01,
                    "02: 设置夹爪目标位置": 0x02,
                    "03: 设置夹爪速度": 0x03,
                    "04: 设置夹爪电流": 0x04,
                    "05: 查询夹爪抓取状态": 0x05,
                    "06: 查询夹爪目前位置": 0x06,
                    "07: 查询夹爪电流": 0x07
                }
                self.motion_model.add_motion_data(mode, {'gripper_command': command_mode[gripper_command], 'gripper_param': gripper_param})
            elif mode == "运动":
                _, positions = self.motion_model.curve_planning(start_angles, target_angles, dt=frequency)
                start_angles = target_angles
                self.motion_model.add_motion_data(mode, {'positions': positions})
        self.motion_model.start_motion()        

    def teach_motion_setting(self, positions_list):
        self.motion_model.clear_teach_data()
        self.tech_data = positions_list
        self.motion_mode = 3
        self.display(f"启动示教轨迹", "控制")
        self.get_current_position()

    def teach_motion_starter(self, start_angles):
        positions_list = self.tech_data
        init_rad = [0.0, -pi/2, 0.0, pi/2, 0.0, 0.0]
        target_angles = (np.array(positions_list[0]) - np.array(init_rad)).tolist()
        _, positions = self.motion_model.curve_planning(start_angles, target_angles, dt=self.dt)
        positions = positions.tolist()
        positions_list = self.spline_then_savgol(positions_list).tolist()
        for p in positions_list:
            p = (np.array(p) - np.array(init_rad)).tolist()
            positions.append(p)
        self.motion_model.add_teach_data(positions)
        self.motion_model.start_teach()
        self.display(f"开始发送示教轨迹: 共{len(positions)}个点", "控制")

    def spline_then_savgol(self,
        position_list,
        upsample=5,
        sg_window=211,
        sg_poly=3
    ):
        """
        先用三次样条做上采样插值，再对插值结果做 Savitzky–Golay 滤波。

        参数:
        - position_list: 原始 N×D 数组
        - upsample: 上采样倍数（整数 >1）
        - sg_window: SG 滤波窗口长度（奇数）
        - sg_poly: SG 拟合多项式阶数 (< sg_window)

        返回:
        - smoothed: (N*upsample)×D 数组
        """
        arr = np.asarray(position_list, dtype=float)
        N, D = arr.shape

        # 1. Savitzky–Golay平滑 (先平滑)
        smoothed = savgol_filter(arr,
                                 window_length=sg_window,
                                 polyorder=sg_poly,
                                 axis=0)

        # 2. 三次样条插值 (再插值)
        t = np.arange(N)
        t_new = np.linspace(0, N - 1, N * upsample)
        interp = np.empty((len(t_new), D), dtype=float)
        for j in range(D):
            cs = CubicSpline(t, smoothed[:, j])
            interp[:, j] = cs(t_new)
        
        indices = np.linspace(0,len(t_new)-1,N).astype(int)
        resampled = interp[indices,:]
        resampled = savgol_filter(resampled,
                                 window_length=sg_window,
                                 polyorder=sg_poly,
                                 axis=0)
        return resampled

    def smooth_via_velocity_reconstruction(self,
        position_list,
        dt=0.01,
        sg_window=11,
        sg_poly=3
    ):
        """
        通过平滑速度曲线重建位置。

        参数
        ----------
        position_list : array-like, shape (N, D)
            原始关节位置序列，共 N 步，每步 D 维。
        dt : float, optional
            原始采样间隔（秒），默认 0.01s (10ms)。
        sg_window : int, optional
            对速度做 Savitzky–Golay 滤波时的窗口长度（奇数），默认 11。
        sg_poly : int, optional
            SG 滤波时的多项式阶数，默认 3。

        返回
        ----------
        pos_recon : np.ndarray, shape (N, D)
            重建后的平滑位置序列，与输入长度一致。
        """
        arr = np.asarray(position_list, dtype=float)
        N, D = arr.shape

        # 1) 样条求导得到速度
        t = np.arange(N) * dt
        vel = np.empty_like(arr)
        for j in range(D):
            cs = CubicSpline(t, arr[:, j])
            vel[:, j] = cs(t, 1)  # 一阶导数

        # 2) 对速度再做 SG 滤波
        vel_smooth = savgol_filter(vel,
                                window_length=sg_window,
                                polyorder=sg_poly,
                                axis=0)

        # 3) 对平滑速度积分重建位置
        pos_recon = np.empty_like(arr)
        pos_recon[0] = arr[0]
        for i in range(1, N):
            pos_recon[i] = pos_recon[i-1] + vel_smooth[i] * dt

        return pos_recon
