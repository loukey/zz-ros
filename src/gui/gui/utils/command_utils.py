"""
命令格式化工具模块
"""
import math
import struct
JOINT_OFFSETS = [78623, 369707, 83986, 391414, 508006, 455123]
RADIAN_TO_POS_SCALE_FACTOR = (2**19) / (2 * math.pi)  # 弧度转位置值的系数
POS_TO_RADIAN_SCALE_FACTOR = (2 * math.pi) / (2**19)  # 位置值转弧度的系数


def calculate_crc16(data):
    """
    计算CRC16校验和
    
    参数:
        data: 字节数组或字符串
        
    返回:
        crc16: 16位校验和
    """
    if isinstance(data, str):
        try:
            data = bytes.fromhex(data.strip())
        except Exception as e:
            print(e)
            print(data)
        
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

def position_to_radian(position, joint_index=None):
    """
    将位置值转换为弧度
    
    参数:
        position: 位置值(整数)或位置值列表
        joint_index: 关节索引(0-5)，如果处理单个位置值则必须提供
        
    返回:
        radian: 弧度值或弧度值列表
    """
    if isinstance(position, list):
        # 处理位置值列表
        if len(position) > 6:
            raise ValueError("位置值列表长度不能超过6")
            
        radians = []
        for i, pos in enumerate(position):
            if isinstance(pos, str):
                pos = int(pos)  # 字符串转整数
            radians.append((pos - JOINT_OFFSETS[i]) * POS_TO_RADIAN_SCALE_FACTOR)
        return radians
    else:
        # 处理单个位置值
        if joint_index is None or not 0 <= joint_index <= 5:
            raise ValueError("处理单个位置值时，必须提供有效的关节索引(0-5)")
            
        if isinstance(position, str):
            position = int(position)  # 字符串转整数
            
        return (position - JOINT_OFFSETS[joint_index]) * POS_TO_RADIAN_SCALE_FACTOR

def radian_to_position(radian, joint_index=None):
    """
    将弧度值转换为位置值
    
    参数:
        radian: 弧度值(浮点数)或弧度值列表
        joint_index: 关节索引(0-5)，如果处理单个弧度值则必须提供
        
    返回:
        position: 位置值(整数)或位置值列表
    """
    if isinstance(radian, list):
        # 处理弧度值列表
        if len(radian) > 6:
            raise ValueError("弧度值列表长度不能超过6")
            
        positions = []
        for i, rad in enumerate(radian):
            position = int(rad * RADIAN_TO_POS_SCALE_FACTOR + JOINT_OFFSETS[i]) & 0xFFFFFFFF
            positions.append(position)
        return positions
    else:
        # 处理单个弧度值
        if joint_index is None or not 0 <= joint_index <= 5:
            raise ValueError("处理单个弧度值时，必须提供有效的关节索引(0-5)")
            
        return radian * RADIAN_TO_POS_SCALE_FACTOR + JOINT_OFFSETS[joint_index]

def speed_to_position(speed):
    if len(speed) > 6:
        raise ValueError("弧度值列表长度不能超过6")
    
    return [int(position * RADIAN_TO_POS_SCALE_FACTOR) & 0xFFFFFF for position in speed]

def torque_transfer(torque):
    if len(torque) > 6:
        raise ValueError("力矩值列表长度不能超过6")
    # 前三个 / 87 * 1000
    # 后三个 * 100
    transfer_torque = []
    for t in torque[:3]:
        transfer_torque.append(int(t) / 87 * 1000)
    for t in torque[3:]:
        transfer_torque.append(int(t) * 100)
    return [int(t) & 0xFFFF for t in transfer_torque]

def effector_data_to_hex(effector_data):
    effector_data = str(float(effector_data))
    arr = effector_data.split('.')
    return [int(arr[0]).to_bytes(2, 'big').hex(), int(arr[1]).to_bytes(2, 'big').hex()]

def format_command(joint_angles=[0.0] * 6, 
                   control=0x06, 
                   mode=0x08, 
                   contour_speed=[0.0] * 6, 
                   contour_acceleration=[0.0] * 6, 
                   contour_deceleration=[0.0] * 6, 
                   torque=[0.0] * 6,
                   effector_mode=0x00, 
                   effector_data=0.0, 
                   encoding='hex'):
    """
    格式化命令
    
    参数:
        joint_angles: 关节角度列表，包含6个关节角度(弧度值)
        control: 控制字节，默认为0x06
        mode: 运行模式，默认为0x08
        result_type: 返回结果类型，'string'或'hex'，默认为'string'
        
    返回:
        command: 格式化后的命令字符串或十六进制字符串
    """
    # 确保有6个关节角度
    if len(joint_angles) != 6:
        raise ValueError("必须提供6个关节角度")
    # 1. 构建命令头部
    command = "AA55"  # 添加头部标识 AA55
    command += f"{control:02X}"  # 添加控制字节
    command += f"{mode:02X}"     # 添加模式字节
    
    # 2. 添加位置值
    positions = radian_to_position(joint_angles)
    for pos in positions:
        # 将24位位置值转换为4个字节的十六进制
        command += f"{(pos >> 24) & 0xFF:02X}"  # 高字节
        command += f"{(pos >> 16) & 0xFF:02X}"  # 高字节
        command += f"{(pos >> 8) & 0xFF:02X}"   # 中字节
        command += f"{pos & 0xFF:02X}"          # 低字节
    
    # 3. 添加速度值
    speeds = speed_to_position(contour_speed)
    for speed in speeds:
        command += f"{(speed >> 16) & 0xFF:02X}"  # 高字节
        command += f"{(speed >> 8) & 0xFF:02X}"   # 中字节
        command += f"{speed & 0xFF:02X}"          # 低字节

    # 4. 添加加速度值
    accelerations = speed_to_position(contour_acceleration)
    for acceleration in accelerations:
        command += f"{(acceleration >> 16) & 0xFF:02X}"  # 高字节
        command += f"{(acceleration >> 8) & 0xFF:02X}"   # 中字节
        command += f"{acceleration & 0xFF:02X}"          # 低字节

    # 5. 添加减速度值
    decelerations = speed_to_position(contour_deceleration)
    for deceleration in decelerations:
        command += f"{(deceleration >> 16) & 0xFF:02X}"  # 高字节
        command += f"{(deceleration >> 8) & 0xFF:02X}"   # 中字节
        command += f"{deceleration & 0xFF:02X}"          # 低字节

    # 6. 添加力矩值 
    torques = torque_transfer(torque)
    for torque in torques:
        command += f"{(torque >> 8) & 0xFF:02X}"   # 中字节
        command += f"{torque & 0xFF:02X}"          # 低字节

    # 6. 添加末端执行器模式字节
    command += f"{effector_mode:02X}"  # 添加末端执行器模式字节

    # 7. 添加末端执行器数据字节
    effector_data_bytes = struct.pack('>f', effector_data)
    command += f"{effector_data_bytes[0]:02X}{effector_data_bytes[1]:02X}{effector_data_bytes[2]:02X}{effector_data_bytes[3]:02X}"  # 添加末端执行器数据字节

    # 8. 计算并添加CRC16
    # 从控制字节开始计算CRC，不包括头部标识
    crc = calculate_crc16(bytes.fromhex(command))  # 从控制字节开始计算
    command += f"{(crc >> 8) & 0xFF:02X}"  # CRC高字节
    command += f"{crc & 0xFF:02X}"         # CRC低字节
    
    # 9. 添加尾部标识
    command += "0D0A"  # 添加尾部标识 0D0A

    return command
