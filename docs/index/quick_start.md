# 快速开始

## 环境配置
- `ubuntu 26.4` + `ros2 jazzy`
```shell
sudo apt install curl
sudo curl -sSL https://mirrors.tuna.tsinghua.edu.cn/rosdistro/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade
sudo apt install libfuse2 git
# 添加 151.101.84.133 raw.githubusercontent.com 到/etc/hosts文件中
echo "151.101.84.133 raw.githubusercontent.com" | sudo tee -a /etc/hosts
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
sudo apt install python3-colcon-common-extensions
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install rosdep
sudo rosdep init
rosdep update
cd ~
mkdir Codes
cd Codes
git clone https://github.com/loukey/zz-ros.git
git checkout -b v0.5 origin/v0.5
colcon build
source install/setup.bash
ros2 run controller controller
```

## 配置清华源
```shell
sudo apt install curl
sudo curl -sSL https://mirrors.tuna.tsinghua.edu.cn/rosdistro/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade
sudo apt install libfuse2 git
```

## 安装ros2-jazzy
```shell
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
sudo apt install python3-colcon-common-extensions
```

## 测试ros2-jazzy
```shell
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
ros2 run demo_nodes_cpp talker
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```

## 下载代码
```shell
cd ~
mkdir Codes
cd Codes
git clone https://github.com/loukey/zz-ros.git
cd zz-ros
git checkout -b v0.6 origin/v0.6
git branch
colcon build
source install/setup.bash
ros2 run controller controller
ros2 run recognition recognition_pub

sudo apt install python3-pip
pip install pyserial
pip install ultralytics -i https://pypi.tuna.tsinghua.edu.cn/simple
pip install "numpy<2"
```

## 配置相机
```shell
cd ~/Codes
git clone https://github.com/orbbec/OrbbecSDK_ROS2.git

sudo apt install libgflags-dev nlohmann-json3-dev 
sudo apt install ros-$ROS_DISTRO-image-transport  ros-${ROS_DISTRO}-image-transport-plugins ros-${ROS_DISTRO}-compressed-image-transport
sudo apt install ros-$ROS_DISTRO-image-publisher ros-$ROS_DISTRO-camera-info-manager 
sudo apt install ros-$ROS_DISTRO-diagnostic-updater ros-$ROS_DISTRO-diagnostic-msgs ros-$ROS_DISTRO-statistics-msgs 
sudo apt install ros-$ROS_DISTRO-backward-ros libdw-dev

cd OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
sudo udevadm control --reload-rules && sudo udevadm trigger
cd ../../
colcon build --event-handlers  console_direct+  --cmake-args  -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch orbbec_camera gemini_330_series.launch.py
```

## 配置USB
```shell
sudo systemctl stop brltty.service
sudo usermod -aG dialout $USER
```

## URDF配置
```shell
# 安装urdf依赖
sudo apt install ros-jazzy-urdf-tutorial
```

