# CRC16 计算测试脚本 - 特定案例测试

def calculate_crc16_ccitt(data):
    """
    计算 CRC16-CCITT 校验值 (多项式 0x1021)
    """
    if isinstance(data, str):
        data = data.encode('ascii')
        
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

def calculate_crc16_modbus(data):
    """
    计算 CRC16-Modbus 校验值 (多项式 0xA001)
    """
    if isinstance(data, str):
        data = data.encode('ascii')
        
    crc = 0xFFFF
    
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc = crc >> 1
    
    return crc

def calculate_crc16_xmodem(data):
    """
    计算 CRC16-XMODEM 校验值 (多项式 0x1021, 初始值 0x0000)
    """
    if isinstance(data, str):
        data = data.encode('ascii')
        
    crc = 0x0000
    
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc = crc << 1
        crc &= 0xFFFF
    
    return crc

def calculate_crc16_kermit(data):
    """
    计算 CRC16-KERMIT 校验值 (多项式 0x1021, LSB first)
    """
    if isinstance(data, str):
        data = data.encode('ascii')
        
    crc = 0x0000
    
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0x8408  # 0x1021 的反转值
            else:
                crc = crc >> 1
    
    # 交换字节顺序
    return ((crc & 0xFF) << 8) | ((crc >> 8) & 0xFF)

def calculate_crc16_dnp(data):
    """
    计算 CRC16-DNP 校验值 (多项式 0x3D65, 初始值 0x0000, 结果取反)
    """
    if isinstance(data, str):
        data = data.encode('ascii')
        
    crc = 0x0000
    
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA6BC  # 0x3D65的反转
            else:
                crc = crc >> 1
    
    # 反转所有位
    crc = ~crc & 0xFFFF
    # 交换字节顺序
    return ((crc & 0xFF) << 8) | ((crc >> 8) & 0xFF)

# 测试用户反馈的特定案例
def test_specific_cases():
    # 测试案例
    test_cmd = "cmd 01 08 78623 369707 83986 391414 508006 455123"
    
    # 计算不同CRC16值
    crc_ccitt = calculate_crc16_ccitt(test_cmd)
    crc_modbus = calculate_crc16_modbus(test_cmd)
    crc_xmodem = calculate_crc16_xmodem(test_cmd)
    crc_kermit = calculate_crc16_kermit(test_cmd)
    crc_dnp = calculate_crc16_dnp(test_cmd)
    
    print(f"输入字符串: {test_cmd}")
    print(f"CRC16-CCITT: 0x{crc_ccitt:04X}")
    print(f"CRC16-Modbus: 0x{crc_modbus:04X}")
    print(f"CRC16-XMODEM: 0x{crc_xmodem:04X}")
    print(f"CRC16-KERMIT: 0x{crc_kermit:04X}")
    print(f"CRC16-DNP: 0x{crc_dnp:04X}")
    print(f"目标CRC16: 0x4570")
    
    # 将结果写入文件
    with open("crc16_comparison.txt", "w") as f:
        f.write(f"输入字符串: {test_cmd}\n")
        f.write(f"CRC16-CCITT: 0x{crc_ccitt:04X}\n")
        f.write(f"CRC16-Modbus: 0x{crc_modbus:04X}\n")
        f.write(f"CRC16-XMODEM: 0x{crc_xmodem:04X}\n")
        f.write(f"CRC16-KERMIT: 0x{crc_kermit:04X}\n")
        f.write(f"CRC16-DNP: 0x{crc_dnp:04X}\n")
        f.write(f"目标CRC16: 0x4570\n")
    
    # 测试不同的字符串格式
    cmd_variations = [
        "cmd 01 08 78623 369707 83986 391414 508006 455123",
        "cmd0108786233697078398639141450800645512"  # 无空格
    ]
    
    for cmd in cmd_variations:
        crc = calculate_crc16_ccitt(cmd)
        print(f"\n输入: {cmd}")
        print(f"CRC16-CCITT: 0x{crc:04X}")

if __name__ == "__main__":
    test_specific_cases() 