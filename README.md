# ZhenZhong Tech Ros

A ROS 2 package for controlling and visualizing a 6-DOF robot arm with forward/inverse kinematics.

## Overview

This project implements a complete robot arm control system in ROS 2, featuring:

- 6-DOF robot arm model defined in URDF/XACRO
- Forward and inverse kinematics solver
- GUI for controlling the robot in joint space or Cartesian space
- Joint state publisher for smooth motion between positions
- RViz visualization

## Project Structure

```
src/
├── model_config/          # Robot model, URDF, launch files
│   ├── urdf/              # XACRO/URDF definition
│   ├── launch/            # ROS 2 launch files
│   └── meshes/            # 3D model files (STL)
├── core/                  # Core functionality
│   └── kinematic/         # Forward/inverse kinematics implementation
├── state_transformer/     # Transforms between different states
│   └── joint_state_pub/   # Joint state publisher
├── gui/                   # User interface
│   └── gui.py             # Tkinter-based control GUI
└── interface/             # Message definitions
```

## Features

### Robot Model

The robot is a 6-DOF arm defined using XACRO for modularity. Each joint and link is parameterized with physical properties including inertial parameters, visual meshes, and collision geometrics.

### Kinematics

The `Kinematic6DOF` class provides:

- **Forward Kinematics**: Computes end-effector position/orientation from joint angles
- **Inverse Kinematics**: Computes joint angles from desired end-effector position/orientation
- Singularity detection and handling
- DH parameter-based transformations
- Solution verification and optimization

### GUI Control

The control GUI provides:

- Input fields for Cartesian pose (X, Y, Z, px, py, pz)
- Input fields for joint angles (angles 1-6)
- Buttons to send commands to the robot
- Threaded ROS 2 communication

### Motion Control

The joint state publisher:

- Smoothly interpolates between current and target joint positions
- Configurable transition duration (default: 5.0 seconds)
- Publishes joint states at 10Hz

## Installation

### Prerequisites

- ROS 2 (tested on Humble)
- Python 3.8+
- Required Python packages: tkinter, numpy

### Building the Package

```bash
# Create a ROS 2 workspace if you don't have one
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src

# Clone this repository
git clone https://github.com/your-username/zhenzhong_tech_ros.git .

# Install dependencies
cd ~/robot_ws
rosdep install --from-paths src --ignore-src -r -y

# Build the packages
colcon build
```

## Usage

### Launch the Robot Visualization

```bash
# Source the workspace
source ~/robot_ws/install/setup.bash

# Launch the robot model with RViz
ros2 launch model_config display.model.launch.py
```

### Control the Robot

```bash
# In a new terminal, source the workspace
source ~/robot_ws/install/setup.bash

# Launch the GUI
ros2 run gui gui
```

Using the GUI:
1. Enter target joint angles or Cartesian pose
2. Click "Send Angles" or "Send Pose" to command the robot
3. The robot will smoothly move to the target position

## Implementation Details

### Kinematics

The robot uses DH parameters for kinematic calculations:

```
i | link length a_{i-1} | link twist α_{i-1} | link offset d_i | joint angle θ_i
-----------------------------------------------------------------------------
1 | 0.0                | 0.0                | 0.0             | θ1
2 | 0.0                | -π/2               | 0.0             | θ2
3 | 0.425              | 0.0                | 0.0             | θ3
4 | 0.401              | 0.0                | 0.0856          | θ4
5 | 0.0                | π/2                | 0.086           | θ5
6 | 0.0                | -π/2               | 0.0725          | θ6
```

The inverse kinematics solver handles multiple solutions and chooses the optimal one based on the current configuration.

### Joint Control

The joint state publisher implements smooth transitions between positions by linearly interpolating joint values over a configurable duration. This creates natural-looking robot movements.

### Message Interface

Communication between components uses ROS 2 topics:
- `/joint_states` - Current joint positions
- `/target_pose` - Target joint angles

## License

[Add your license information here]

## Contributing

[Add contribution guidelines here]
