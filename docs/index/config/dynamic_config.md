# 动力学相关配置
力矩补偿计算位于`controller/domain/algorithm/dynamic_domain_service.py`, 静摩擦力补偿计算位于`controller/domain/state/robot_state_domain_service.py`,具体配置如下

## 力矩补偿
```python
# 连杆质量 (kg)
self.link_masses = [4.64, 10.755, 3.925, 1.24, 1.24, 1.546]

# 使用动力学DH参数
dh_param = DHParam()
self.dh_matrix = dh_param.get_dynamic_dh()

# 连杆质心在本体坐标系下的位置 (m)
self.link_com_positions = [
    np.array([0, 0, 0]),
    np.array([0.2125, 0, 0.134]),
    np.array([0.1818, 0, 0]),
    np.array([0, 0, 0]),
    np.array([0, 0, 0]),
    np.array([0, 0, 0.05])
]

# 重力向量 (m/s²)
self.g_vector = np.array([0, 0, -9.81])

# 关节数量
self.n = 6

# 补偿系数（根据实际机械臂调整）
self.compensation_factor_front = 0.35  # 前3个关节
self.compensation_factor_rear = 0.25   # 后3个关节
```

## 静摩擦力
静摩擦力在获取力矩达到19时开始计算，取最后5个值的平均来确认摩擦力方向
```python
# 静摩擦状态（每个关节：0=静止, 1=运动）
self._friction_state = [0] * 6

# 静摩擦补偿值（Nm）
self._friction_compensation = [0.0] * 6

# 静摩擦配置（阈值）
self._friction_config = [4, 0, 3, 0.5, 0.5, 0.5]
```