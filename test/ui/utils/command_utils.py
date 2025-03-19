"""
命令格式化工具模块
"""
from utils.crc import calculate_crc16
import math

def format_command(angles, control=0x01, mode=0x01, result_type='binary'):
    """
    格式化角度命令
    
    参数:
        angles: 角度值列表，包含6个关节角度（弧度值）。当control不为0x06时可以为空列表。
        control: 控制字节，默认为0x01（使能）
        mode: 运动模式，默认为0x01（轮廓位置模式）
        result_type: 返回结果类型，'binary'(二进制字节)、'string'(字符串格式)或'hex'(十六进制字符串)
        
    返回:
        result: 根据result_type返回不同格式的命令:
               - 'binary': 返回命令字节数组
               - 'string': 返回可读的文本命令字符串，包含\r\n
               - 'hex': 返回以空格分隔的十六进制字符串
    """
    # 如果不是运动模式（control != 0x06），使用默认角度值
    if control != 0x06:
        angles = [0.0] * 6
    elif len(angles) != 6:
        raise ValueError("运动模式下必须提供6个关节角度值")
    
    # 偏移值
    OFFSETS = [78623, 369707, 83986, 391414, 508006, 455123]
    
    # 转换系数 rad * 2^19 / (2π)
    SCALE_FACTOR = (2**19) / (2 * math.pi)
    
    # 转换角度值并添加偏移
    converted_angles = []
    for angle, offset in zip(angles, OFFSETS):
        # 将弧度值转换为整数值
        scaled_value = int(angle * SCALE_FACTOR)
        # 添加偏移
        final_value = (scaled_value + offset) & 0xFFFFFF  # 确保是24位
        converted_angles.append(final_value)
    
    # 构建字符串命令用于计算CRC
    cmd_str = f"cmd {control:02d} {mode:02d}"
    for value in converted_angles:
        cmd_str += f" {value}"
    
    # 计算CRC16校验
    crc = calculate_crc16(cmd_str)
    
    # 如果需要返回字符串格式
    if result_type == 'string':
        # 构建可读的文本命令字符串，包含CRC
        return f"{cmd_str} {crc:04X}\r\n"
    
    # 构建二进制命令
    cmd = bytearray()
    
    # 添加 "cmd" 字符串作为命令字节
    cmd.extend("cmd".encode())
    
    # 控制字节
    cmd.append(control)
    
    # 运动模式
    cmd.append(mode)
    
    # 添加转换后的角度值（每个值3字节，大端序）
    for value in converted_angles:
        cmd.extend([(value >> 16) & 0xFF, (value >> 8) & 0xFF, value & 0xFF])
    
    # 添加CRC16校验
    cmd.extend([(crc >> 8) & 0xFF, crc & 0xFF])
    
    # 添加帧尾 \r\n
    cmd.extend([0x0D, 0x0A])
    
    # 根据result_type返回不同格式
    if result_type == 'hex':
        # 返回十六进制字符串
        return ' '.join([f"{b:02X}" for b in cmd])
    
    # 默认返回二进制字节数组
    return cmd

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