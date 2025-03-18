"""
工具类模块，包含 CRC16 校验和命令格式化功能
"""

def calculate_crc16(data):
    """
    计算 CRC16 校验值
    
    参数:
        data: 字节数组或字符串
    返回:
        crc16: 16位校验值
    """
    if isinstance(data, str):
        data = data.encode()
    
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc

def format_command(command_type, data=None):
    """
    格式化命令，添加帧头和 CRC16 校验
    
    参数:
        command_type: 命令类型（如 'ENABLE', 'DISABLE' 等）
        data: 命令数据（可选）
    返回:
        formatted_command: 格式化后的命令字符串
    """
    # 命令类型到命令代码的映射
    command_codes = {
        'ENABLE': '01',
        'DISABLE': '02',
        'RELEASE': '03',
        'LOCK': '04',
        'STOP': '05',
        'MOTION': '06'
    }
    
    # 获取命令代码
    cmd_code = command_codes.get(command_type, '00')
    
    # 构建命令字符串
    if data is None:
        cmd_str = f"AA{cmd_code}"
    else:
        # 将数据转换为十六进制字符串
        if isinstance(data, (int, float)):
            data = f"{data:08X}"
        cmd_str = f"AA{cmd_code}{data}"
    
    # 计算 CRC16
    crc = calculate_crc16(cmd_str)
    cmd_str += f"{crc:04X}"
    
    return cmd_str

def generate_trajectory(start_angles, end_angles, duration, frequency, curve_type='trapezoidal'):
    """
    生成轨迹数据
    
    参数:
        start_angles: 起始角度列表
        end_angles: 目标角度列表
        duration: 运动时长（秒）
        frequency: 发送频率（秒）
        curve_type: 曲线类型，'trapezoidal' 或 's_curve'
    返回:
        time_points: 时间点列表
        position_data: 位置数据列表
        velocity_data: 速度数据列表
        acceleration_data: 加速度数据列表
    """
    import numpy as np
    
    # 计算时间点
    time_points = np.arange(0, duration, frequency)
    
    # 计算每个关节的轨迹
    position_data = []
    velocity_data = []
    acceleration_data = []
    
    for start_angle, end_angle in zip(start_angles, end_angles):
        # 计算角度差
        angle_diff = end_angle - start_angle
        
        if curve_type == 'trapezoidal':
            # 梯形速度曲线
            # 加速和减速时间各占总时间的 1/4
            t_acc = duration / 4
            t_dec = duration / 4
            t_const = duration / 2
            
            # 计算最大速度
            v_max = angle_diff / (t_const + t_acc/2 + t_dec/2)
            
            # 计算加速度
            a_max = v_max / t_acc
            
            # 生成轨迹
            pos = []
            vel = []
            acc = []
            
            for t in time_points:
                if t < t_acc:
                    # 加速段
                    a = a_max
                    v = a_max * t
                    p = start_angle + 0.5 * a_max * t**2
                elif t < (t_acc + t_const):
                    # 匀速段
                    a = 0
                    v = v_max
                    p = start_angle + v_max * (t - t_acc/2)
                else:
                    # 减速段
                    t_dec_start = t_acc + t_const
                    t_dec_elapsed = t - t_dec_start
                    a = -a_max
                    v = v_max - a_max * t_dec_elapsed
                    p = end_angle - 0.5 * a_max * (duration - t)**2
                
                pos.append(p)
                vel.append(v)
                acc.append(a)
            
            position_data.append(pos)
            velocity_data.append(vel)
            acceleration_data.append(acc)
            
        else:  # s_curve
            # S形加减速曲线
            # 使用正弦函数生成平滑的速度曲线
            v_max = angle_diff / duration
            pos = []
            vel = []
            acc = []
            
            for t in time_points:
                # 使用正弦函数生成平滑的速度曲线
                v = v_max * (1 - np.cos(np.pi * t / duration)) / 2
                # 计算位置（积分速度）
                p = start_angle + v_max * (t - duration * np.sin(np.pi * t / duration) / (2 * np.pi))
                # 计算加速度（速度的导数）
                a = v_max * np.pi * np.sin(np.pi * t / duration) / (2 * duration)
                
                pos.append(p)
                vel.append(v)
                acc.append(a)
            
            position_data.append(pos)
            velocity_data.append(vel)
            acceleration_data.append(acc)
    
    return time_points, position_data, velocity_data, acceleration_data 