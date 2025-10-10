# 虚拟串口机械臂模拟器

这是一个用于测试controller的虚拟串口模拟器，可以模拟真实机械臂的通信协议。

## 功能特性

- ✅ 完整实现 MessageIn 格式的响应消息
- ✅ 模拟机械臂状态（位置、速度、力矩等）
- ✅ 支持运动控制命令
- ✅ 支持夹爪控制命令
- ✅ 实时显示命令和响应信息
- ✅ 位置数据自动转换为弧度显示

## 安装依赖

```bash
pip install pyserial
sudo apt-get install socat  # Ubuntu/Debian
```

## 使用方法

### 1. 创建虚拟串口对

在一个终端中运行：

```bash
socat -d -d pty,raw,echo=0,link=/tmp/ttyV0 pty,raw,echo=0,link=/tmp/ttyV1
```

这会创建两个虚拟串口：
- `/tmp/ttyV0` - controller连接这个
- `/tmp/ttyV1` - 模拟器连接这个

### 2. 启动虚拟机械臂模拟器

在另一个终端中运行：

```bash
cd /home/za/Codes/zz-ros/src/virtual_serial
python3 virtual_serial_port.py /tmp/ttyV1
```

### 3. 配置controller

在controller中配置串口为 `/tmp/ttyV0`，然后正常启动controller。

### 4. 测试

- 在controller中点击"连接串口"
- 点击"发送角度"测试运动控制
- 观察虚拟机械臂模拟器的输出

## 文件说明

- `virtual_serial_port.py` - 主程序，虚拟串口监听和处理
- `robot_simulator.py` - 机械臂状态模拟器
- `message_builder.py` - MessageIn格式消息构建器
- `README.md` - 本文档

## 消息格式

### MessageOut (controller → 模拟器)
```
AA55 + control(1) + mode(1) + joint_angles(24) + speeds(18) + accel(18) + decel(18) + torque(12) + effector_mode(1) + effector_data(4) + CRC(2) + 0D0A
```

### MessageIn (模拟器 → controller)
```
AA55 + init_status(1) + control(1) + mode(1) + positions(24) + status(12) + speeds(24) + torques(12) + double_encoder_interpolations(24) + errors(12) + effector_data(4) + CRC(2) + 0D0A
```

## 示例输出

```
🤖 虚拟机械臂串口模拟器
============================================================
✅ 虚拟串口已连接: /tmp/ttyV1
   波特率: 115200

🤖 虚拟机械臂已启动
📡 正在监听命令...

📥 [1] 接收命令: AA5506080001331B0005A89B0001479200...
   命令类型: 运动控制
   运行模式: 0x08
📤 [1] 发送响应: AA550106080001331B0005A89B0001479200...
   当前位置(弧度): [0.0000, -1.5708, 0.0000, 1.5708, 0.0000, 0.0000]
```

## 注意事项

1. 确保 socat 一直在运行，否则虚拟串口会消失
2. 如果controller无法连接，检查串口路径是否正确
3. 虚拟串口在重启后会消失，需要重新创建
4. 按 Ctrl+C 停止模拟器

## 故障排除

### 串口设备不存在
```bash
# 检查虚拟串口是否存在
ls -l /tmp/ttyV*

# 如果不存在，重新运行socat命令
socat -d -d pty,raw,echo=0,link=/tmp/ttyV0 pty,raw,echo=0,link=/tmp/ttyV1
```

### 权限问题
```bash
# 添加当前用户到dialout组
sudo usermod -a -G dialout $USER

# 重新登录后生效
```

### 端口被占用
```bash
# 查找占用端口的进程
lsof /tmp/ttyV1

# 杀死进程
kill -9 <PID>
```

