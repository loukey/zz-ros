import math


joint_angles = [0, 0, 0, 0, 0, 0]
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
