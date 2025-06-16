# 镇中科技机械臂控制系统 (ZZTech Robotics Control System)

一个基于ROS2的6自由度机械臂控制系统，集成了运动学计算、动力学分析、运动规划和计算机视觉功能。

![Python](https://img.shields.io/badge/Python-3.12+-blue.svg)
![ROS2](https://img.shields.io/badge/ROS2-Foxy+-green.svg)
![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)
![PyQt5](https://img.shields.io/badge/GUI-PyQt5-orange.svg)

## 📋 目录

- [功能特性](#功能特性)
- [系统架构](#系统架构)
- [环境要求](#环境要求)
- [安装说明](#安装说明)
- [使用指南](#使用指南)
- [模块说明](#模块说明)
- [配置说明](#配置说明)
- [开发说明](#开发说明)
- [许可证](#许可证)

## ✨ 功能特性

### 🔧 核心功能
- **6自由度机械臂控制** - 支持6DOF机械臂的精确控制
- **图形化界面** - 基于PyQt5的直观操作界面
- **运动学计算** - 正运动学和逆运动学求解
- **动力学分析** - 力矩计算和动力学建模
- **运动规划** - S曲线运动规划和路径优化
- **串口通信** - 稳定的机械臂通信协议

### 🎯 高级功能
- **计算机视觉** - 集成Orbbec深度相机支持
- **示教模式** - 拖拽示教和轨迹记录
- **实时监控** - 关节角度和末端位置实时显示
- **安全保护** - 运动范围限制和碰撞检测
- **数据记录** - 运动轨迹和状态数据记录

### 🛠 技术特点
- 基于ROS2分布式架构
- 模块化设计，易于扩展
- 支持多种运动控制模式
- 高精度运动学和动力学计算
- 实时图像处理和目标检测

## 🏗 系统架构

```
zz-ros/
├── src/
│   ├── gui/                    # 图形用户界面模块
│   │   └── gui/
│   │       ├── views/          # 界面视图组件
│   │       ├── controllers/    # 控制器逻辑
│   │       ├── models/         # 数据模型
│   │       └── utils/          # 工具函数
│   ├── model_base/             # 核心算法模块
│   │   ├── core/               # 核心计算模块
│   │   │   └── kinematic/      # 运动学计算
│   │   ├── interface/          # ROS2接口定义
│   │   │   ├── msg/            # 消息定义
│   │   │   └── action/         # 动作定义
│   │   └── state_transformer/  # 状态转换器
│   ├── cv/                     # 计算机视觉模块
│   └── orbbec/                 # Orbbec相机SDK (submodule)
├── docs/                       # 文档目录
├── build.sh                    # 构建脚本
└── pyproject.toml             # 项目配置文件
```

## 💻 环境要求

### 系统要求
- **操作系统**: Ubuntu 20.04+ / Windows 10+
- **Python版本**: 3.12+
- **ROS2版本**: Foxy 及以上

### 硬件要求
- **内存**: 最少4GB，推荐8GB+
- **CPU**: 双核处理器及以上
- **USB端口**: 用于机械臂和相机连接
- **显卡**: 支持OpenGL（可选，用于3D显示）

## 🚀 安装说明

### 1. 克隆项目
```bash
git clone https://github.com/your-repo/zz-ros.git
cd zz-ros
git submodule update --init --recursive
```

### 2. 安装依赖
```bash
# 使用uv包管理器（推荐）
pip install uv
uv sync

# 或使用pip
pip install -r requirements.txt
```

### 3. 构建ROS2包
```bash
# 设置ROS2环境
source /opt/ros/foxy/setup.bash

# 构建项目
chmod +x build.sh
./build.sh
```

### 4. 安装Orbbec SDK
```bash
cd src/orbbec
# 按照Orbbec SDK文档进行安装
```

## 🎮 使用指南

### 启动系统

1. **启动ROS2核心**
```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
```

2. **启动GUI界面**
```bash
ros2 run gui main
```

3. **启动核心服务**
```bash
ros2 run core kinematic_service
```

### 基本操作

#### 🔌 连接机械臂
1. 在GUI的"主页"标签中找到串口配置
2. 选择正确的串口号和波特率
3. 点击"连接"按钮

#### 🎯 控制机械臂
1. **关节控制**: 在角度控制面板中输入各关节角度
2. **末端控制**: 在末端位置面板中设置目标位置和姿态
3. **运动规划**: 切换到"运动规划"标签进行轨迹规划

#### 📹 使用相机
1. 切换到"摄像头"标签
2. 连接Orbbec相机
3. 选择彩色或深度显示模式
4. 启动目标检测功能

### 高级功能

#### 🔧 示教模式
```bash
# 启用示教模式
ros2 service call /enable_teaching_mode std_srvs/srv/SetBool "{data: true}"
```

#### 📊 动力学分析
1. 切换到"动力学"标签
2. 设置负载参数
3. 启动力矩计算
4. 查看实时力矩数据

## 📦 模块说明

### GUI模块 (`src/gui/`)
- **主窗口**: 集成所有功能的主界面
- **视图组件**: 模块化的UI组件
- **控制器**: 业务逻辑处理
- **数据模型**: 数据管理和状态维护

### 核心模块 (`src/model_base/core/`)
- **运动学计算**: 
  - `kinematic_6dof.py`: 6DOF运动学正逆解
  - `velocity_planning.py`: 速度规划算法
  - `base.py`: 基础数学函数
- **状态管理**: 机械臂状态跟踪和转换

### 接口模块 (`src/model_base/interface/`)
- **消息定义**: 
  - `Pose.msg`: 位姿消息
- **动作定义**: 
  - `JointStates.action`: 关节状态动作

### 计算机视觉模块 (`src/cv/`)
- 图像处理算法
- 目标检测和识别
- 深度信息处理

## ⚙️ 配置说明

### 串口配置
```python
# 默认串口配置
SERIAL_CONFIG = {
    'port': '/dev/ttyUSB0',  # Windows: 'COM3'
    'baudrate': 115200,
    'timeout': 1.0
}
```

### 运动规划配置
运动规划参数可在 `motion_planning_data.json` 中配置：
```json
{
    "joint1": 0.0,
    "joint2": 0.0,
    "joint3": 0.0,
    "joint4": 0.0,
    "joint5": 0.0,
    "joint6": 1.57,
    "frequency": 0.01,
    "curve_type": "S曲线",
    "note": ""
}
```

### DH参数配置
机械臂的DH参数需要根据具体机械臂型号进行配置。

## 🔧 开发说明

### 项目结构
- 采用MVC架构模式
- 模块化设计，高内聚低耦合
- 基于ROS2的分布式系统架构

### 添加新功能
1. 在相应模块下创建新的组件
2. 实现必要的接口
3. 在主窗口中集成新组件
4. 更新配置文件

### 调试模式
```bash
# 启用调试日志
export PYTHONPATH=$PYTHONPATH:$(pwd)/src
python -m gui.main --debug
```

### 单元测试
```bash
# 运行测试
python -m pytest src/model_base/core/test/
```

## 🤝 贡献指南

1. Fork 项目
2. 创建功能分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 创建 Pull Request

## 📝 更新日志

### v0.2.23
- 优化GUI界面布局
- 增加摄像头支持
- 改进运动规划算法

### v0.2.13
- 初始版本发布
- 基础机械臂控制功能
- 运动学和动力学计算

## 🐛 常见问题

### Q: 串口连接失败
A: 检查串口权限和设备连接，确保机械臂电源开启。

### Q: 相机无法识别
A: 确认Orbbec SDK正确安装，检查USB连接。

### Q: 运动规划计算缓慢
A: 调整规划参数，减少路径点数量。

## 📞 联系方式

- **维护者**: za
- **邮箱**: 529768926@qq.com
- **项目主页**: [GitHub仓库链接]

## 📄 许可证

本项目采用 Apache License 2.0 许可证。详情请参阅 [LICENSE](LICENSE) 文件。

---

**注意**: 本系统涉及机械臂控制，使用前请确保操作安全，遵循相关安全规范。

