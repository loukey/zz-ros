# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ZZTech Robotics Control System (镇中科技机械臂控制系统) — a ROS2-based 6-DOF robotic arm control system with PyQt5 GUI. Written primarily in Chinese (comments, docs, UI strings). Python 3.12+, ROS2 Jazzy, dependency management via `uv`.

## Build & Run Commands

```bash
# Install dependencies
uv sync

# Build ROS2 packages (on Ubuntu with ROS2 installed)
colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace after building
source install/setup.bash

# Run the GUI application
cd src/gui/gui && uv run python main.py

# Run controller as ROS2 node
ros2 run controller controller

# Build and deploy docs
mkdocs gh-deploy --force
```

## Architecture

The project uses **Domain-Driven Design (DDD)** with strict layered architecture. All core logic lives under `src/controller/controller/`:

### DDD Layers

- **`domain/`** — Core business logic, no external dependencies
  - `services/algorithm/` — Kinematics (FK/IK), trajectory planning (S-curve, TOPPRA), dynamics, linear/curve motion, hand-eye transforms
  - `services/communication/` — Serial protocol encoding/decoding, message domain service
  - `services/state/` — Robot state management, teach-record
  - `services/planning/` — Motion planning with multi-point paths
  - `services/vision/` — Camera and recognition domain services
  - `entities/` — MotionPlan entity
  - `value_objects/` — DHParam, RobotStateSnapshot, HandEyeCalibrationConfig, MotionOperationMode
  - `utils/` — Kinematic math, message encoder/decoder, robot utilities

- **`application/`** — Orchestration layer
  - `services/` — Application services (serial, camera, command hub, tools, message response, data recording)
  - `commands/` — Command objects (message display)
  - `listener/` — Motion listener

- **`infrastructure/`** — External system integration
  - `communication/` — Serial port adapter, reader, writer, port scanner (USB TTL/RS485)
  - `persistence/` — Repositories for records, motion plans, trajectories, hand-eye calibration data

- **`presentation/`** — PyQt5 GUI using MVVM pattern
  - `view_models/` — One ViewModel per feature area (control, serial, camera, dynamics, effector, tools, trajectory, etc.)
  - `components/` — UI widget components grouped by feature
  - `gui/main_window.py` — Main application window

- **`shared/config/`** — Dependency injection
  - `di_container.py` — Custom DI container (singleton/transient lifetime)
  - `service_registry.py` — Registers all services; entry point: `configure_services()`

### Other Source Packages (ROS2 packages under `src/`)

- `src/calibration/` — Hand-eye calibration ROS2 package
- `src/cv/` — Computer vision (YOLOv8 OBB detection, data tools)
- `src/model_base/` — Robot URDF model, kinematics library (`core/kinematic/`), joint state publisher, RViz launch files
- `src/recognition/` — Recognition ROS2 package
- `src/record/` — Recording ROS2 package
- `src/orbbec/` — Git submodule for Orbbec camera ROS2 SDK

### Key Patterns

- **DI Container**: Global singleton via `get_container()` / `resolve(Type)`. Services are registered in `service_registry.py` with explicit dependency order.
- **MVVM in Presentation**: ViewModels expose state and commands; Components bind to ViewModels. `MainWindow` is composed from feature components.
- **Serial Communication**: Custom binary protocol with `MessageEncoder`/`MessageDecoder` over USB serial. The `SerialDomainService` handles low-level comms, `MessageDomainService` handles protocol logic.
- **Configuration**: YAML/JSON config files under `src/controller/controller/config/` for DH parameters, dynamics, hand-eye calibration, message protocol, and save paths.
