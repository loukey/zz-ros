def _calculate_crc16(data):
    """
    计算CRC16校验值 (CRC-CCITT, 多项式0x1021)
    
    参数:
        data: 要计算校验的数据（字节数组）
        
    返回:
        crc: 16位校验值
    """
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

cmd_str = "cmd 01 08 78623 369707 83986 391414 508006 455123"
data_bytes = cmd_str.encode('ascii')
crc = _calculate_crc16(data_bytes)
print(f"{crc:04X}")
