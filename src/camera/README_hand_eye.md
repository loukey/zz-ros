# Eye-in-Hand 手眼标定使用指南

## 📋 什么是手眼标定

手眼标定用于确定安装在机器人末端执行器上的相机与机器人坐标系之间的精确变换关系。

### Eye-in-Hand 配置
- **相机位置**: 安装在机器人末端执行器上
- **标定板位置**: 固定在工作空间中（不移动）
- **标定目标**: 求解从末端执行器到相机的固定变换

## 🛠️ 准备工作

### 1. 硬件准备
- ✅ 机器人系统正常运行
- ✅ 相机已安装在末端执行器上
- ✅ 相机已完成内参标定
- ✅ 标定板（棋盘格）

### 2. 软件准备
- ✅ ROS2机器人位姿话题发布正常
- ✅ 相机图像话题发布正常
- ✅ 手眼标定包已编译

### 3. 话题检查
```bash
# 检查相机话题
ros2 topic list | grep image

# 检查机器人位姿话题
ros2 topic list | grep pose

# 查看话题数据
ros2 topic echo /your_robot_pose_topic
```

## 🚀 标定流程

### 1. 编译包
```bash
cd /home/za/Codes/zzros
colcon build --packages-select camera
source install/setup.bash
```

### 2. 启动手眼标定节点
```bash
# 基本启动
ros2 run camera hand_eye_calibration

# 自定义参数启动
ros2 run camera hand_eye_calibration --ros-args \
    -p camera_topic:=/camera/color/image_raw \
    -p robot_pose_topic:=/robot_pose \
    -p chessboard_size:=9,6 \
    -p square_size:=0.025 \
    -p min_poses:=10
```

### 3. 标定操作步骤

#### 步骤1: 固定标定板
- 将棋盘格标定板固定在机器人工作空间内
- 确保标定板在所有预定位姿下都能被相机观察到
- 标定板应平整、无弯曲

#### 步骤2: 开始数据收集
在程序界面中输入：
```
start
```

#### 步骤3: 移动机器人并采集数据
1. **移动机器人到第一个位姿**
   - 确保相机能清晰观察到完整的标定板
   - 等待程序检测到棋盘格角点（显示绿色角点）

2. **按空格键保存当前位姿数据**
   - 程序会同时记录机器人位姿和相机观察到的标定板位姿

3. **重复移动和保存过程**
   - 建议采集至少 **10-15个** 不同位姿
   - 位姿应覆盖相机工作空间的各个区域

#### 步骤4: 位姿采集建议

📍 **推荐的机器人位姿分布：**
- **距离变化**: 近距离、中距离、远距离
- **角度变化**: 正面观察、倾斜观察、侧面观察
- **位置变化**: 工作空间的不同位置
- **姿态变化**: 不同的末端执行器姿态

⚠️ **避免的位姿：**
- 标定板不完整出现在图像中
- 图像过于模糊
- 光照条件很差
- 机器人位姿过于相似

#### 步骤5: 执行标定
当收集足够位姿后，输入：
```
calibrate
```

### 4. 标定结果解读

标定完成后会显示：
```
==================================================
Eye-in-Hand 手眼标定结果
==================================================
使用位姿数量: 12
标定误差: 0.003245 米

末端执行器到相机的变换:
平移 (x, y, z): (0.020000, 0.050000, 0.100000) 米
四元数 (x, y, z, w): (0.258819, 0.000000, 0.000000, 0.965926)
欧拉角 (roll, pitch, yaw): (30.00°, 0.00°, 0.00°)

变换矩阵:
[[ 1.000000  0.000000  0.000000  0.020000]
 [ 0.000000  0.866025 -0.500000  0.050000]
 [ 0.000000  0.500000  0.866025  0.100000]
 [ 0.000000  0.000000  0.000000  1.000000]]
==================================================
```

#### 质量评估标准：
- **标定误差 < 0.005米** : 优秀
- **标定误差 < 0.010米** : 良好  
- **标定误差 > 0.010米** : 需要重新标定

## 📁 输出文件

标定完成后会生成：
- `hand_eye_calibration.yaml` - 人类可读的标定结果
- `hand_eye_calibration.npz` - 程序使用的标定结果
- `calibration_poses.json` - 标定位姿数据
- `pose_*.jpg` - 标定图像

## 💻 使用标定结果

### 1. 在Python中使用
```python
from hand_eye_utils import HandEyeUtils

# 加载标定结果
hand_eye = HandEyeUtils('./hand_eye_calibration_data/hand_eye_calibration.yaml')

# 机器人位姿转相机位姿
robot_pose = np.eye(4)  # 当前机器人位姿
camera_pose = hand_eye.robot_pose_to_camera_pose(robot_pose)

# 相机坐标转机器人坐标
point_camera = np.array([0.1, 0.05, 0.5])  # 相机坐标系中的点
point_robot = hand_eye.transform_point_camera_to_robot(point_camera, robot_pose)
```

### 2. 验证标定质量
```python
# 使用验证数据测试标定质量
validation_result = hand_eye.validate_hand_eye_calibration(test_poses, test_observations)
print(f"平均平移误差: {validation_result['translation_error']['mean']:.6f} 米")
print(f"平均旋转误差: {validation_result['rotation_error_degrees']['mean']:.2f} 度")
```

## 🔧 常见问题排除

### 1. 检测不到棋盘格
- 检查光照条件
- 确认棋盘格尺寸参数正确
- 调整相机到标定板的距离

### 2. 机器人位姿话题问题
```bash
# 检查话题是否存在
ros2 topic list | grep pose

# 检查话题数据格式
ros2 topic info /your_robot_pose_topic

# 检查话题发布频率
ros2 topic hz /your_robot_pose_topic
```

### 3. 标定误差过大
- 增加标定位姿数量（推荐15-20个）
- 提高位姿分布的多样性
- 检查机器人位姿精度
- 重新进行相机内参标定

### 4. 程序无法启动
```bash
# 检查依赖是否安装
pip install opencv-python numpy PyYAML

# 重新编译
colcon build --packages-select camera --symlink-install
source install/setup.bash
```

## 📊 参数说明

| 参数名称 | 默认值 | 说明 |
|----------|--------|------|
| `camera_topic` | `/camera/color/image_raw` | 相机图像话题 |
| `robot_pose_topic` | `/robot_pose` | 机器人位姿话题 |
| `chessboard_size` | `9,6` | 棋盘格内角点数目 |
| `square_size` | `0.025` | 棋盘格方格尺寸(米) |
| `save_dir` | `./hand_eye_calibration_data` | 保存目录 |
| `min_poses` | `10` | 最少标定位姿数 |
| `calibration_file` | `./calibration_data/camera_calibration.yaml` | 相机标定文件 |

## 🎯 应用示例

手眼标定完成后，可以用于：

1. **视觉引导抓取**: 将相机检测到的物体坐标转换为机器人坐标
2. **路径规划**: 基于视觉信息进行机器人路径规划
3. **质量检测**: 使用相机进行工件检测和测量
4. **人机协作**: 基于视觉的安全监控

## 📈 标定质量提升技巧

1. **位姿多样性**: 确保采集的位姿覆盖整个工作空间
2. **角度变化**: 包含不同观察角度的位姿
3. **距离分布**: 近、中、远距离的位姿组合
4. **重复验证**: 使用独立的测试数据验证标定结果
5. **环境稳定**: 保持稳定的光照和工作环境

完成手眼标定后，你的机器人视觉系统就可以精确地在相机坐标系和机器人坐标系之间进行转换了！ 