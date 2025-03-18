from utils import calculate_crc16

# 测试特定命令字符串
test_cmd = "cmd 01 08 78623 369707 83986 391414 508006 455123"

# 计算CRC16值
crc = calculate_crc16(test_cmd)

print(f"命令字符串: {test_cmd}")
print(f"CRC16值(十六进制): 0x{crc:04X}")
print(f"CRC16值(十进制): {crc}") 