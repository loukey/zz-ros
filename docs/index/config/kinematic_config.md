# 运动学相关配置

## DH参数配置
配置位于`controller/domain/value_objects/dh_param.py`, 如下所示
```python
self.kinematic_dh = np.array([
    [0.0, 0.0, 0.0,  0.0],
    [0.0, -pi/2, 0.0, -pi/2],
    [0.425, pi, 0.0, 0.0],
    [0.401, pi, 0.0856, pi/2],
    [0.0, pi/2, 0.086, 0.0],
    [0.0, -pi/2, 0.231, 0.0]  
], dtype=float)

self.dynamic_dh = np.array([
    [0.0, 0.0, 0.0, 0.0],
    [0.0, -pi/2, 0.0, -pi/2],
    [0.425, pi, 0.0, 0.0],
    [0.401, pi, 0.0856, pi/2],
    [0.0, pi/2, 0.086, 0.0],
    [0.0, -pi/2, 0.0725, 0.0]
])
```

## 编码器零位配置
配置位于`controller/domain/utils/robot_utils.py`, 如下所示
```python
"""初始化机器人参数。"""
self.JOINT_OFFSETS = [79119, 369835, 83627, 392414, 507293, 456372]
self.RADIAN_TO_POS_SCALE_FACTOR = (2**19) / (2 * pi)  # 弧度转位置值的系数
self.POS_TO_RADIAN_SCALE_FACTOR = (2 * pi) / (2**19)  # 位置值转弧度的系数
self.init_radians = [0, -pi/2, 0, pi/2, 0, 0]
```
