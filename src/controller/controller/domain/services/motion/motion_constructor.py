"""
运动构造器 - Domain层
协调轨迹规划和运动执行
"""
from typing import List, Dict
from .motion_runner import MotionRunner
from .trajectory_planning_service import TrajectoryPlanningService
from controller.domain import MotionOperationMode


class MotionConstructor:
    """运动构造器 - 领域服务。
    
    职责：
    1. 管理操作状态（执行/保存/预览）
    2. 协调 TrajectoryPlanningService（规划）
    3. 协调 MotionRunner（执行）
    
    Attributes:
        motion_runner (MotionRunner): 运动执行器。
        trajectory_planner (TrajectoryPlanningService): 轨迹规划服务。
    """
    
    def __init__(
        self, 
        motion_runner: MotionRunner,
        trajectory_planner: TrajectoryPlanningService
    ):
        """初始化运动构造器。
        
        Args:
            motion_runner (MotionRunner): 运动执行器。
            trajectory_planner (TrajectoryPlanningService): 轨迹规划服务。
        """
        self.motion_runner = motion_runner
        self.trajectory_planner = trajectory_planner
        
        # 统一的操作状态
        self._operation_mode = MotionOperationMode.NONE
        self._pending_tasks = []
        self._operation_context = {}
    
    def prepare_operation(
        self,
        mode: MotionOperationMode,
        tasks: List[Dict],
        context: Dict = None
    ):
        """准备操作（统一入口）。
        
        Args:
            mode (MotionOperationMode): 操作模式。
                - EXECUTE: 执行运动
                - SAVE: 保存轨迹
                - PREVIEW: 预览轨迹
            tasks (List[Dict]): 任务列表。
            context (Dict, optional): 操作上下文。
                - EXECUTE: {}
                - SAVE: {"filename": "xxx", "type": "node|plan"}
                - PREVIEW: {"type": "node|plan", "node_index": 0}
        """
        self._operation_mode = mode
        self._pending_tasks = tasks.copy()
        self._operation_context = context or {}
    
    def has_pending_operation(self) -> bool:
        """检查是否有待处理的操作。
        
        Returns:
            bool: True=有待处理的操作, False=无操作。
        """
        return self._operation_mode != MotionOperationMode.NONE
    
    def get_operation_mode(self) -> MotionOperationMode:
        """获取当前操作模式。
        
        Returns:
            MotionOperationMode: 当前操作模式枚举。
        """
        return self._operation_mode
    
    def get_pending_tasks(self) -> List[Dict]:
        """获取待处理的任务列表。
        
        Returns:
            List[Dict]: 任务列表的副本。
        """
        return self._pending_tasks.copy()
    
    def get_operation_context(self) -> Dict:
        """获取操作上下文。
        
        Returns:
            Dict: 上下文字典的副本。
        """
        return self._operation_context.copy()
    
    def clear_operation(self):
        """清除操作状态。"""
        self._operation_mode = MotionOperationMode.NONE
        self._pending_tasks.clear()
        self._operation_context.clear()
    
    def execute_motion(self, start_position: List[float]):
        """执行运动（仅用于EXECUTE模式）。
        
        流程：
        1. 按任务顺序规划每个任务
        2. 按顺序添加运动数据和夹爪命令
        3. 清除状态
        
        Args:
            start_position (List[float]): 起始位置（当前关节角度）。
            
        Raises:
            RuntimeError: 如果当前模式不是EXECUTE。
        """
        if self._operation_mode != MotionOperationMode.EXECUTE:
            raise RuntimeError(f"当前模式不是EXECUTE: {self._operation_mode}")
        
        if not self._pending_tasks:
            return
        
        # 执行
        self.motion_runner.clear_data()
        
        # 按顺序处理每个任务，保持原有顺序
        current_position = start_position
        
        for task in self._pending_tasks:
            if task["type"] == "gripper":
                # 夹爪任务：直接添加夹爪命令（带延迟）
                effector_mode = task["effector_mode"]
                effector_data = task["effector_data"]
                pre_delay = task.get("pre_delay", 0.0)
                post_delay = task.get("post_delay", 1.0)
                self.motion_runner.add_gripper_data(
                    effector_mode, 
                    effector_data,
                    pre_delay=pre_delay,
                    post_delay=post_delay
                )
            elif task["type"] == "detect":
                pass
            else:
                # 运动任务：规划轨迹并添加
                positions, end_position = self.trajectory_planner._plan_single_task(
                    task, 
                    current_position
                )
                # 注意：positions 可能是 numpy 数组，不能直接用 if positions
                if len(positions) > 0:
                    self.motion_runner.add_motion_data(positions)
                    current_position = end_position
        
        self.motion_runner.start_motion()
        
        # 清理
        self.clear_operation()
