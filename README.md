# 镇中科技机械臂控制系统 (ZZTech Robotics Control System)

[![Documentation](https://img.shields.io/badge/docs-online-blue)](https://loukey.github.io/zz-ros/)
![Python](https://img.shields.io/badge/Python-3.12+-blue.svg)
![ROS2](https://img.shields.io/badge/ROS2-Foxy+-green.svg)
![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)
![PyQt5](https://img.shields.io/badge/GUI-PyQt5-orange.svg)

一个基于 ROS2 的 6 自由度机械臂控制系统，采用**领域驱动设计 (DDD)** 架构，集成了运动学计算、动力学分析、运动规划、手眼标定和计算机视觉功能。

[📄 在线文档](https://loukey.github.io/zz-ros/) | [🐛 问题反馈](https://github.com/loukey/zz-ros/issues)

## 📋 目录

- [功能特性](#功能特性)
- [系统架构](#系统架构)
- [环境要求](#环境要求)
- [安装说明](#安装说明)
- [使用指南](#使用指南)
- [许可证](#许可证)

## ✨ 功能特性

### 🔧 核心控制
- **6自由度控制** - 支持 6DOF 机械臂的精确关节/笛卡尔空间控制
- **运动学解算** - 高精度正逆运动学 (DH 参数配置)
- **动力学补偿** - 重力补偿与静摩擦力模型，支持零重力拖动示教
- **平滑轨迹** - 基于 S 曲线和 TOPPRA 的轨迹平滑与时间参数化

### 👁️ 视觉与感知
- **手眼标定** - 集成 Eye-in-Hand 标定流程 (AX=XB 求解)
- **视觉识别** - 支持 YOLOv8 目标检测与位姿估计
- **相机支持** - 深度相机 (Orbbec/Realsense) 集成

### 🛠️ 高级工具
- **示教模式** - 拖拽示教、轨迹录制与回放
- **运动规划** - 支持多点路径规划与自动避障
- **全功能 GUI** - 基于 PyQt5 的现代化操作界面，实时 3D 可视化

## 🏗 系统架构

本项目采用严格的 **领域驱动设计 (DDD)** 分层架构：

```
src/controller/controller/
├── application/        # 应用层：业务流程编排 (Service, Command, Listener)
├── domain/            # 领域层：核心业务逻辑 (Entity, Service, ValueObject)
│   ├── services/      # 领域服务 (Algorithm, Motion, State, Vision)
│   ├── entities/      # 领域实体 (MotionPlan)
│   └── value_objects/ # 值对象 (RobotState, DHParam)
├── infrastructure/    # 基础设施层：外部交互 (Serial, Persistence)
├── presentation/      # 展示层：MVVM 架构 (ViewModel, View)
├── shared/            # 共享层：DI 容器与配置
└── config/            # 配置文件 (YAML/JSON)
```

## 💻 环境要求

### 软件环境
- **OS**: Ubuntu 26.04+
- **Python**: 3.12+
- **ROS2**: jazzy
- **依赖管理**: `uv` (推荐) 或 `pip`

### 硬件支持
- **机械臂**: 6轴串联机械臂 (支持自定义 DH 参数)
- **相机**: Orbbec Gemini 330 系列 / Intel Realsense
- **通信**: USB 转串口 (TTL/RS485)

## 🚀 安装说明

### 1. 克隆项目
```bash
git clone https://github.com/loukey/zz-ros.git
cd zz-ros
```

### 2. 安装依赖
本项目推荐使用 `uv` 进行依赖管理：

```bash
# 安装 uv
pip install uv

# 同步依赖环境
uv sync
```

## 📄 许可证

本项目采用 [Apache 2.0 许可证](LICENSE)。

---
**Safe Operation Warning**: 请确保在安全环境下操作机械臂，随时准备按下急停按钮。
