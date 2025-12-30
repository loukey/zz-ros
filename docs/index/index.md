# 欢迎来到镇中科技机械臂控制系统文档

这里是镇中科技机械臂控制系统的官方文档。

本系统旨在提供一个灵活、高效且易于扩展的框架，用于控制机械臂进行各种任务，包括运动控制、轨迹规划、视觉识别和示教记录等。

## 项目结构概览

本项目的代码结构遵循 **领域驱动设计 (DDD)** 原则。下文将严格按照 `src/controller/controller` 目录下的实际文件分布进行介绍。

### 1. Application (应用层)
**目录路径**: `src/controller/controller/application`

业务流程的协调者。负责编排领域对象来完成具体的应用用例。

*   **commands (`application/commands`)**: 封装具体的命令对象。
    *   `message_display.py`: 处理消息显示的命令逻辑。
*   **listener (`application/listener`)**: 事件监听器。
    *   `motion_listener.py`: 监听运动状态变化的事件处理器。
*   **services (`application/services`)**: 应用服务，封装具体的业务场景。
    *   `camera_application_service.py`: 协调相机连接、图像获取与视觉算法。
    *   `motion_planning_application_service.py`: 协调运动规划请求与轨迹生成。
    *   `command_hub_service.py`: 统一的指令分发中心，处理用户命令并路由到相应的领域服务。
    *   `serial_application_service.py`: 串口连接管理服务。
    *   `message_response_service.py`: 处理下位机反馈消息的服务。
    *   `tools_application_service.py`: 提供工具类功能（如正逆解计算）的应用服务。
    *   `base_service.py`: 应用服务基类。

### 2. Config (系统配置)
**目录路径**: `src/controller/controller/config`

存放系统运行所需的静态配置文件（YAML 格式），主要用于定义通信协议。

*   `message_decoder_config.yaml`: 定义接收消息的解码规则。
*   `message_encoder_config.yaml`: 定义发送消息的编码规则。

### 3. Domain (领域层)
**目录路径**: `src/controller/controller/domain`

核心业务逻辑的心脏。包含纯粹的业务规则、算法和状态模型。

*   **entities (`domain/entities`)**: 具有唯一标识的领域实体。
    *   `motion_plan.py`: 定义运动计划实体。
*   **services (`domain/services`)**: 领域服务，封装核心业务能力。
    *   **algorithm**: 核心算法。
        *   `kinematic_domain_service.py`: 正逆运动学解算。
        *   `trajectory_domain_service.py`: S型速度曲线规划。
        *   `curve_motion.py`: 空间曲线运动规划（TOPPRA）。
        *   `smooth_domain_service.py`: 轨迹平滑算法。
        *   `hand_eye_transform_domain_service.py`: 手眼标定计算。
        *   `dynamic_domain_service.py`: 动力学与重力补偿计算。
    *   **communication**: 通信逻辑。
        *   `message_domain_service.py`: 消息编解码服务。
        *   `serial_domain_service.py`: 串口数据收发逻辑。
    *   **motion**: 运动控制。
        *   `motion_constructor.py`: 运动指令构造器。
        *   `motion_runner.py`: 运动执行器。
        *   `trajectory_planning_service.py`: 轨迹规划服务。
    *   **planning**: 高级规划。
        *   `motion_planning_domain_service.py`: 运动计划管理。
    *   **state**: 状态管理。
        *   `robot_state_domain_service.py`: 机器人运行时状态管理。
        *   `teach_record_domain_service.py`: 示教记录管理。
    *   **vision**: 视觉处理。
        *   `camera_domain_service.py`: 相机图像流处理。
        *   `recognition_domain_service.py`: 视觉识别结果处理。
*   **utils (`domain/utils`)**: 领域层通用工具。
    *   `message_decoder.py` / `message_encoder.py`: 协议解析工具。
    *   `kinematic_utils.py`: 运动学数学工具。
    *   `image_drawing_utils.py`: 图像绘制工具。
*   **value_objects (`domain/value_objects`)**: 不可变的值对象。
    *   `robot_state_snapshot.py`: 机器人状态快照。
    *   `dh_param.py`: DH 参数配置。
    *   `hand_eye_calibration_config.py`: 标定配置。

### 4. Infrastructure (基础设施层)
**目录路径**: `src/controller/controller/infrastructure`

与外部世界的连接器。提供技术实现细节。

*   **communication (`infrastructure/communication`)**: 通信基础设施。
    *   `serial_adapter.py`: `pyserial` 适配器。
    *   `serial_reader.py` / `serial_writer.py`: 串口读写线程实现。
    *   `port_scanner.py`: 串口扫描工具。
*   **persistence (`infrastructure/persistence`)**: 数据持久化仓储 (Repository)。
    *   `trajectory_repository.py`: 轨迹文件存取。
    *   `record_repository.py`: 示教记录存取。
    *   `motion_plan_repository.py`: 运动计划存取。
    *   `hand_eye_calibration_repository.py`: 标定参数存取。
*   **external / hardware**: （预留）外部库适配与硬件驱动。

### 5. Presentation (展示层)
**目录路径**: `src/controller/controller/presentation`

用户交互界面。采用 MVVM 模式。

*   **components (`presentation/components`)**: 可复用的 UI 组件（按功能模块分类）。
    *   包含 `camera`, `display`, `dynamics`, `effector`, `main`, `motion_planning`, `settings`, `status`, `tools` 等子目录。
*   **gui (`presentation/gui`)**: 主窗口实现。
    *   `main_window.py`: 应用程序主窗口组装。
*   **view_models (`presentation/view_models`)**: 视图模型，连接 UI 与 Application/Domain。
    *   `main_view_model.py`: 主视图模型。
    *   `control_view_model.py`: 控制逻辑视图模型。
    *   `camera_view_model.py`: 相机视图模型。
    *   `dynamics_view_model.py`: 动力学视图模型。
    *   `status_view_model.py`: 状态显示视图模型。
    *   ... (以及其他功能对应的 ViewModel)

### 6. Shared (共享层)
**目录路径**: `src/controller/controller/shared`

跨层通用的工具与配置。

*   **config (`shared/config`)**: 全局代码配置。
    *   `di_container.py`: 依赖注入 (DI) 容器实现。
    *   `service_registry.py`: 服务注册与组装逻辑。

