# 快速开始 - 虚拟串口模拟器

## 🚀 快速启动（3步）

### 步骤1：创建虚拟串口对

打开**终端1**，运行：

```bash
socat -d -d pty,raw,echo=0,link=/tmp/ttyV0 pty,raw,echo=0,link=/tmp/ttyV1
```

**保持此终端运行！** 输出应该类似：
```
2025/10/10 10:00:00 socat[12345] N PTY is /dev/pts/3
2025/10/10 10:00:00 socat[12345] N PTY is /dev/pts/4
2025/10/10 10:00:00 socat[12345] N starting data transfer loop with FDs [5,5] and [7,7]
```

### 步骤2：启动虚拟机械臂

打开**终端2**，运行：

```bash
cd /home/za/Codes/zz-ros/src/virtual_serial
./start.sh
```

或者：

```bash
cd /home/za/Codes/zz-ros/src/virtual_serial
python3 virtual_serial_port.py /tmp/ttyV1
```

看到以下输出说明启动成功：
```
✅ 虚拟串口已连接: /tmp/ttyV1
   波特率: 115200

🤖 虚拟机械臂已启动
📡 正在监听命令...
```

### 步骤3：配置并启动controller

在**终端3**，修改controller的串口配置为 `/tmp/ttyV0`，然后运行：

```bash
cd /home/za/Codes/zz-ros
source install/setup.bash
ros2 run controller controller
```

## 📊 预期效果

### Controller端
```
✅ 串口已连接: /tmp/ttyV0
✅ 发送运动命令
[发送] 位置(弧度): [0.0000, -1.5708, 0.0000, 1.5708, 0.0000, 0.0000]
[发送] 原始数据: AA5506080001331B0005A89B...
```

### 虚拟机械臂端（终端2）
```
📥 [1] 接收命令: AA5506080001331B0005A89B...
   命令类型: 运动控制
   运行模式: 0x08
📤 [1] 发送响应: AA550106080001331B0005A89B...
   当前位置(弧度): [0.0000, -1.5708, 0.0000, 1.5708, 0.0000, 0.0000]
```

## 🧪 测试功能

在controller GUI中测试：

1. ✅ **连接串口** - 应该成功连接
2. ✅ **发送角度** - 虚拟机械臂会显示接收到的命令和响应
3. ✅ **夹爪控制** - 测试夹爪开合
4. ✅ **查看消息** - 在消息栏看到弧度值和原始数据

## 🛠️ 故障排除

### 问题1：串口设备不存在
```bash
❌ 串口设备不存在: /tmp/ttyV1
```

**解决**：确保终端1中的socat正在运行

---

### 问题2：权限拒绝
```bash
PermissionError: [Errno 13] Permission denied: '/tmp/ttyV1'
```

**解决**：
```bash
sudo usermod -a -G dialout $USER
# 然后重新登录
```

---

### 问题3：端口被占用
```bash
SerialException: could not open port /tmp/ttyV1
```

**解决**：
```bash
# 查找占用的进程
lsof /tmp/ttyV1

# 杀死进程
kill -9 <PID>

# 重启socat
```

---

### 问题4：controller连接失败

**检查清单**：
- [ ] socat是否在运行？ (`ps aux | grep socat`)
- [ ] 虚拟串口是否存在？ (`ls -l /tmp/ttyV*`)
- [ ] controller配置的串口是 `/tmp/ttyV0` ？
- [ ] 虚拟机械臂连接的是 `/tmp/ttyV1` ？

## 📝 完整测试流程

```bash
# 终端1：创建虚拟串口
socat -d -d pty,raw,echo=0,link=/tmp/ttyV0 pty,raw,echo=0,link=/tmp/ttyV1

# 终端2：启动虚拟机械臂
cd /home/za/Codes/zz-ros/src/virtual_serial
python3 virtual_serial_port.py /tmp/ttyV1

# 终端3：启动controller
cd /home/za/Codes/zz-ros
source install/setup.bash
ros2 run controller controller
```

## 🎯 测试验证

运行单元测试：

```bash
cd /home/za/Codes/zz-ros/src/virtual_serial
python3 test_message.py
```

预期输出：
```
🎉 所有测试通过！
```

## 📚 更多信息

- 详细文档：[README.md](README.md)
- 消息格式说明：参见 `src/controller/controller/config/` 中的YAML文件
- 问题反馈：检查虚拟机械臂的输出日志

## 💡 提示

1. **调试技巧**：虚拟机械臂会实时显示所有命令和响应，方便调试
2. **位置数据**：自动转换为弧度显示，便于验证
3. **持久化**：虚拟串口重启后会消失，需要重新创建
4. **多实例**：可以创建多组虚拟串口对，用于同时测试多个设备

## ⚡ 一键启动脚本（可选）

创建 `start_all.sh`：

```bash
#!/bin/bash

# 启动socat（后台）
socat -d -d pty,raw,echo=0,link=/tmp/ttyV0 pty,raw,echo=0,link=/tmp/ttyV1 &
SOCAT_PID=$!

# 等待串口创建
sleep 1

# 启动虚拟机械臂（后台）
cd /home/za/Codes/zz-ros/src/virtual_serial
python3 virtual_serial_port.py /tmp/ttyV1 &
VIRTUAL_PID=$!

echo "✅ 虚拟环境已启动"
echo "   socat PID: $SOCAT_PID"
echo "   虚拟机械臂 PID: $VIRTUAL_PID"
echo ""
echo "按 Ctrl+C 停止所有进程"

# 等待用户中断
trap "kill $SOCAT_PID $VIRTUAL_PID 2>/dev/null; exit" SIGINT SIGTERM
wait
```

使用：
```bash
chmod +x start_all.sh
./start_all.sh
```

