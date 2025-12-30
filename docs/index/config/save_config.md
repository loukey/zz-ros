# 轨迹保存说明

本系统中有两种类型的轨迹保存机制，分别对应**示教记录**和**运动规划方案**。

## 1. 示教记录保存 (Teach Records)

这是指在"动力学/示教"模式下录制的原始关节角度数据。

*   **保存位置**: 项目根目录下的 `teach_record.json` 文件。
*   **文件格式**: JSON 格式。
*   **数据结构**: 字典格式，键为记录名称，值为角度列表的列表。
    ```json
    {
      "record1": [[0.1, 0.2, ...], [0.11, 0.21, ...]],
      "record2": [...]
    }
    ```
*   **命名规则**:
    *   自动生成: `record1`, `record2`, `record3`... (递增计数)
    *   反转记录: `{原名}-反转` (例如 `record1-反转`)
    *   命名冲突时: `{原名}-反转-{序号}` (例如 `record1-反转-2`)
*   **负责类**: `RecordRepository` (位于 `infrastructure/persistence/record_repository.py`)

## 2. 运动规划轨迹保存 (Motion Plan Trajectories)

这是指在"运动规划"功能中，将规划好的平滑轨迹（经过插值和滤波后的最终执行点）保存下来的文件。这些文件通常用于调试、分析或作为本地轨迹再次加载。

*   **保存位置**: 项目根目录下的 `plans/` 文件夹。
*   **文件格式**: JSON 格式。
*   **数据结构**: 纯列表格式，包含一系列关节角度列表。
    ```json
    [
      [0.0, -1.57, 0.0, 1.57, 0.0, 0.0],
      [0.01, -1.56, 0.0, 1.57, 0.0, 0.0],
      ...
    ]
    ```
*   **命名规则**:
    *   **保存整个方案**: 使用方案名称作为文件名。例如 `默认方案.json`。
    *   **保存单个节点**: 使用 `{方案名}-{节点索引}` 作为文件名。例如 `默认方案-0.json`, `默认方案-1.json`。
*   **负责类**: `TrajectoryRepository` (位于 `infrastructure/persistence/trajectory_repository.py`)

## 3. 运动方案配置保存 (Motion Plan Configuration)

这是指保存运动规划的**配置参数**（如节点列表、目标点、曲线类型等），而不是具体的轨迹点。

*   **保存位置**: 项目根目录下的 `motion_planning_plans.json` 文件。
*   **文件格式**: JSON 格式。
*   **命名规则**: 只有一个固定文件。
*   **负责类**: `MotionPlanRepository` (位于 `infrastructure/persistence/motion_plan_repository.py`)
