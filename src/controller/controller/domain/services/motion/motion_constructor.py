"""
运动构造器 - Domain层
协调轨迹规划和运动执行
"""
from typing import List, Dict
from .motion_runner import MotionRunner
from .trajectory_planning_service import TrajectoryPlanningService
from controller.domain import MotionOperationMode


class MotionConstructor:
    """
    运动构造器 - 领域服务
    
    职责：
    1. 管理操作状态（执行/保存/预览）
    2. 协调 TrajectoryPlanningService（规划）
    3. 协调 MotionRunner（执行）
    """
    
    def __init__(
        self, 
        motion_runner: MotionRunner,
        trajectory_planner: TrajectoryPlanningService
    ):
        """
        初始化运动构造器
        
        Args:
            motion_runner: 运动执行器
            trajectory_planner: 轨迹规划服务
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
        """
        准备操作（统一入口）
        
        Args:
            mode: 操作模式
                - EXECUTE: 执行运动
                - SAVE: 保存轨迹
                - PREVIEW: 预览轨迹
            tasks: 任务列表
            context: 操作上下文（可选）
                - EXECUTE: {}
                - SAVE: {"filename": "xxx", "type": "node|plan"}
                - PREVIEW: {"type": "node|plan", "node_index": 0}
        """
        self._operation_mode = mode
        self._pending_tasks = tasks.copy()
        self._operation_context = context or {}
    
    def has_pending_operation(self) -> bool:
        """
        是否有待处理的操作
        
        Returns:
            True: 有待处理的操作
            False: 无操作
        """
        return self._operation_mode != MotionOperationMode.NONE
    
    def get_operation_mode(self) -> MotionOperationMode:
        """
        获取当前操作模式
        
        Returns:
            当前操作模式枚举
        """
        return self._operation_mode
    
    def get_pending_tasks(self) -> List[Dict]:
        """
        获取待处理的任务列表
        
        Returns:
            任务列表的副本
        """
        return self._pending_tasks.copy()
    
    def get_operation_context(self) -> Dict:
        """
        获取操作上下文
        
        Returns:
            上下文字典的副本
        """
        return self._operation_context.copy()
    
    def clear_operation(self):
        """清除操作状态"""
        self._operation_mode = MotionOperationMode.NONE
        self._pending_tasks.clear()
        self._operation_context.clear()
    
    def execute_motion(self, start_position: List[float]):
        """
        执行运动（仅用于EXECUTE模式）
        
        流程：
        1. 调用 trajectory_planner 规划轨迹
        2. 将轨迹传给 motion_runner 执行
        3. 清除状态
        
        Args:
            start_position: 起始位置（当前关节角度）
            
        Raises:
            RuntimeError: 如果当前模式不是EXECUTE
        """
        if self._operation_mode != MotionOperationMode.EXECUTE:
            raise RuntimeError(f"当前模式不是EXECUTE: {self._operation_mode}")
        
        if not self._pending_tasks:
            return
        
        # 规划轨迹
        all_positions = self.trajectory_planner.plan_task_sequence(
            self._pending_tasks,
            start_position
        )
        
        # 执行
        self.motion_runner.clear_data()
        
        # 处理每个任务（包括夹爪操作）
        for task in self._pending_tasks:
            if task["type"] == "gripper":
                # 夹爪任务单独处理
                effector_mode = task["effector_mode"]
                effector_data = task["effector_data"]
                self.motion_runner.add_gripper_data(effector_mode, effector_data)
        
        # 添加运动轨迹
        if all_positions:
            self.motion_runner.add_motion_data(all_positions)
        
        self.motion_runner.start_motion()
        
        # 清理
        self.clear_operation()
