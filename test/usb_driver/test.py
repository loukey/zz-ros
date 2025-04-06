command_line = "AA55060101331F05A42B01481205F8F607C06606F1D300010101010101010101010101331F05A42B01481205F8F607C06606F1D300010101010101010101010101000b000b82EC0A0D"


print(len(command_line))
# 帧头 (2字节)
header = command_line[0:4]  # AA55

# 初始化状态 (1字节)
init_status = command_line[4:6]

# 当前命令 (1字节)
current_command = command_line[6:8]

# 运行模式 (1字节)
run_mode = command_line[8:10]

# 位置1-6 (每个3字节，共18字节)
positions = []
start = 10
for i in range(6):
    pos = int(command_line[start:start+6], 16)
    positions.append(pos)
    start += 6

# 状态字1-6 (每个2字节，共12字节)
status = []
for i in range(6):
    stat = int(command_line[start:start+4], 16)
    status.append(stat)
    start += 4

# 实际速度1-6 (每个3字节，共18字节)
speeds = []
for i in range(6):
    speed = int(command_line[start:start+6], 16)
    speeds.append(speed)
    start += 6

# 错误码1-6 (每个2字节，共12字节)
errors = []
for i in range(6):
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

print(header, init_status, current_command, run_mode, positions, status, speeds, errors, effector_data, crc)