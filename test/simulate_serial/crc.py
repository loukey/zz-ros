"""
CRC16 校验工具模块
"""

def calculate_crc16(data):
    """
    计算 CRC16 校验值 (CRC-CCITT 标准，多项式 0x1021)
    
    参数:
        data: 字节数组或字符串
    返回:
        crc16: 16位校验值
    """
    if isinstance(data, str):
        # 如果输入是字符串，转换为ASCII字节数组
        data = data.encode('ascii')
        
    crc = 0xFFFF  # 初始值为 0xFFFF
    
    for byte in data:
        crc ^= (byte << 8)  # 将当前字节移到高8位并异或
        
        for _ in range(8):
            if crc & 0x8000:  # 如果最高位为1
                crc = (crc << 1) ^ 0x1021  # 左移并异或多项式
            else:
                crc = crc << 1  # 仅左移
                
            crc &= 0xFFFF  # 保持为16位
    
    return crc 