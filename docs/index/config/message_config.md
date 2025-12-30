# 消息协议字段说明

配置修改文件为`controller/message_encoder_config.yaml`和`controller/message_decoder_config.yaml`

## 1. 发送消息 (Message Encoder)

**方向**：上位机 -> 控制器 (PC -> MCU)
**总长度**：103 字节 (包含帧头帧尾和CRC)

| 字段名称 | 字节长度 | 说明 |
| :--- | :---: | :--- |
| **Header** | 2 | 帧头 (0xAA55) |
| `control` | 1 | 控制字 (默认 0x06) |
| `mode` | 1 | 运行模式 (默认 0x08) |
| `joint_angles` | 24 | 6个关节的目标角度 (每个关节4字节) |
| `contour_speed` | 18 | 6个关节的轮廓速度 (每个关节3字节) |
| `contour_acceleration` | 18 | 6个关节的轮廓加速度 (每个关节3字节) |
| `contour_deceleration` | 18 | 6个关节的轮廓减速度 (每个关节3字节) |
| `torque` | 12 | 6个关节的力矩前馈 (每个关节2字节) |
| `effector_mode` | 1 | 末端执行器模式 |
| `effector_data` | 4 | 末端执行器数据 (浮点数) |
| **CRC** | 2 | CRC16 校验码 |
| **Footer** | 2 | 帧尾 (0x0D0A) |

## 2. 接收消息 (Message Decoder)

**方向**：控制器 -> 上位机 (MCU -> PC)
**总长度**：121 字节 (包含帧头帧尾和CRC)

| 字段名称 | 字节长度 | 说明 |
| :--- | :---: | :--- |
| **Header** | 2 | 协议头 (0xAA55) |
| `init_status` | 1 | 初始化状态 |
| `control` | 1 | 当前收到的命令 |
| `mode` | 1 | 当前运行模式 |
| `positions` | 24 | 6个关节的当前位置 (每个关节4字节) |
| `status` | 12 | 6个关节的状态字 (每个关节2字节) |
| `speeds` | 24 | 6个关节的实际速度 (每个关节4字节) |
| `torques` | 12 | 6个关节的当前力矩 (每个关节2字节) |
| `double_encoder_interpolations` | 24 | 双编码器插值数据 (每个关节4字节) |
| `errors` | 12 | 6个关节的错误码 (每个关节2字节) |
| `effector_data` | 4 | 末端执行器(夹爪)数据 |
| **CRC** | 2 | CRC16 校验码 |
| **Footer** | 2 | 帧尾 (0x0D0A) |
