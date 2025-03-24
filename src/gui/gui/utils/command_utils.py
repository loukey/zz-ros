"""
命令格式化工具模块
"""
from gui.utils.crc import calculate_crc16
import math
import binascii
import struct
import numpy as np
from math import pi
from kinematic.velocity_planning import trapezoidal_velocity_planning, s_curve_velocity_planning

def format_command(joint_angles, control=0x06, mode=0x08, result_type='string'):
    """
    格式化命令
    
    参数:
        joint_angles: 关节角度列表，包含6个关节角度
        control: 控制字节
        mode: 模式字节
        result_type: 返回类型，'string'或'hex'
    
    返回:
        formatted_command: 格式化后的命令字符串或字节数组
    """
    # 确保有6个关节角度
    if len(joint_angles) != 6:
        raise ValueError("必须提供6个关节角度")
    command = f"cmd {control:02X} {mode:02X}"
    # 偏移值
    OFFSETS = [78623, 369707, 83986, 391414, 508006, 455123]
    
    # 转换系数 rad * 2^19 / (2π)
    SCALE_FACTOR = (2**19) / (2 * math.pi)
    
    # 转换角度值并添加偏移
    counts = []
    for angle, offset in zip(joint_angles, OFFSETS):
        # 将弧度值转换为整数值
        scaled_value = int(angle * SCALE_FACTOR)
        final_value = (scaled_value + offset) & 0xFFFFFF  # 确保是24位
        counts.append(final_value)
    print(counts)
    # 构建命令字符串
    
    for count in counts:
        command += f" {count}"
    # 计算校验和 (CRC16)
    crc = calculate_crc16(command)
    command += f" {crc:04X}"
    
    # 添加行终止符
    command += "\r\n"
    print(command)
    # 根据结果类型返回
    if result_type == 'hex':
        return command.encode('ascii')
    
    return command

def calculate_crc16(data):
    """
    计算CRC16校验和
    
    参数:
        data: 字节数组或字符串
        
    返回:
        crc16: 16位校验和
    """
    if isinstance(data, str):
        # 将字符串转换为ASCII字节
        data = data.encode('ascii')
        
    # 计算CRC16-CCITT
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc = crc << 1
            crc &= 0xFFFF
            
    return crc

def generate_trajectory(start_angles, end_angles, duration=5.0, frequency=0.01, curve_type="trapezoidal"):
    """
    生成轨迹
    
    参数:
        start_angles: 起始角度列表，包含6个关节角度
        end_angles: 结束角度列表，包含6个关节角度
        duration: 运动持续时间，默认5秒
        frequency: 采样频率，默认0.01秒
        curve_type: 曲线类型，"trapezoidal"或"s_curve"
        
    返回:
        tuple: (时间点列表, 位置数组, 速度数组, 加速度数组)
    """
    # 确保有6个关节角度
    if len(start_angles) != 6 or len(end_angles) != 6:
        raise ValueError("起始和结束角度必须各有6个")
    
    # 创建时间点
    time_points = np.arange(0, duration + frequency, frequency)
    
    # 创建位置数组
    num_points = len(time_points)
    positions = np.zeros((6, num_points))
    velocities = np.zeros(num_points)
    accelerations = np.zeros(num_points)
    
    # 为每个关节生成轨迹
    for i in range(6):
        # 计算单轴的角度差
        delta_angle = end_angles[i] - start_angles[i]
        
        # 根据曲线类型选择轨迹规划方法
        if curve_type.lower() == "s_curve":
            t, v, a, p = s_curve_velocity_planning(
                [delta_angle], 
                v_max=abs(delta_angle) / (0.8 * duration) if delta_angle != 0 else 0.001,
                t_acc=0.2 * duration,
                dt=frequency
            )
        else:  # 默认使用梯形曲线
            t, v, a, p = trapezoidal_velocity_planning(
                [delta_angle], 
                v_max=abs(delta_angle) / (0.8 * duration) if delta_angle != 0 else 0.001,
                t_acc=0.2 * duration,
                dt=frequency
            )
        
        # 调整数组长度以匹配时间点
        if len(t) > num_points:
            t = t[:num_points]
            p = p[:, :num_points]
            v = v[:num_points]
            a = a[:num_points]
        elif len(t) < num_points:
            # 如果生成的点少于预期，补充末尾点
            extra_points = num_points - len(t)
            p = np.pad(p, ((0, 0), (0, extra_points)), 'edge')
            v = np.pad(v, (0, extra_points), 'edge')
            a = np.pad(a, (0, extra_points), 'edge')
        
        # 加上起始角度得到实际轨迹
        positions[i] = p[0] + start_angles[i]
        
        # 更新最大速度和加速度
        if np.max(np.abs(v)) > np.max(np.abs(velocities)):
            velocities = v
        if np.max(np.abs(a)) > np.max(np.abs(accelerations)):
            accelerations = a
    
    return time_points, positions, velocities, accelerations 