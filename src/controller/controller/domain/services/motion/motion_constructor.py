"""
运动构造器 - Domain层
协调轨迹规划和运动执行
"""
from typing import List, Dict, Callable
from .motion_runner import MotionRunner
from .trajectory_planning_service import TrajectoryPlanningService
from controller.domain import MotionOperationMode
from PyQt5.QtCore import QObject, pyqtSignal


class MotionConstructor(QObject):
    """运动构造器 - 领域服务。
    
    职责：
    1. 管理操作状态（执行/保存/预览）
    2. 协调 TrajectoryPlanningService（规划）
    3. 协调 MotionRunner（执行）
    
    Attributes:
        motion_runner (MotionRunner): 运动执行器。
        trajectory_planner (TrajectoryPlanningService): 轨迹规划服务。
        detection_requested (pyqtSignal): 请求检测信号，携带(current_position)。
    """
    
    detection_requested = pyqtSignal(list)
    motion_plan_finished = pyqtSignal() # 整个运动方案执行完成信号
    
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
        super().__init__()
        self.motion_runner = motion_runner
        self.trajectory_planner = trajectory_planner
        
        # 统一的操作状态
        self._operation_mode = MotionOperationMode.NONE
        self._pending_tasks = []
        self._operation_context = {}
        
        # 中断恢复相关的状态
        self._planning_cursor_position = None  # 当前规划游标（上一个任务的终点）
        self._is_waiting_for_detection = False # 是否正在等待检测结果
        
        # 连接MotionRunner的信号
        self.motion_runner.motion_finished.connect(self._on_motion_finished)
    
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
        1. 初始化状态
        2. 启动任务处理循环
        
        Args:
            start_position (List[float]): 起始位置（当前关节角度）。
            
        Raises:
            RuntimeError: 如果当前模式不是EXECUTE。
        """
        if self._operation_mode != MotionOperationMode.EXECUTE:
            raise RuntimeError(f"当前模式不是EXECUTE: {self._operation_mode}")
        
        if not self._pending_tasks:
            return
        
        # 执行前清理
        self.motion_runner.clear_data()
        
        # 初始化规划游标
        self._planning_cursor_position = start_position
        self._is_waiting_for_detection = False
        
        # 启动任务处理
        self._process_task_queue()

    def _process_task_queue(self):
        """处理任务队列，直到队列为空或遇到阻断节点（检测）。"""
        
        # 循环处理队列中的任务
        while self._pending_tasks:
            task = self._pending_tasks[0]  # 窥视队首任务
            
            if task["type"] == "detect":
                # === 遇到检测节点：阻断！===
                
                # 1. 设置等待状态
                self._is_waiting_for_detection = True
                
                # 2. 如果目前有积攒的运动数据，立即执行，让机器人走到检测点
                if len(self.motion_runner.data_list) > 0:
                    self.motion_runner.start_motion()
                else:
                    # 如果没有运动数据（说明机器人已经在检测点了），直接触发完成逻辑
                    # 我们手动触发一次回调来处理这种情况
                    self._on_motion_finished()
                
                # 3. 退出循环，暂停规划
                return
                
            else:
                # === 普通任务：正常规划 ===
                self._pending_tasks.pop(0)  # 确认处理该任务，移出队列
                
                if task["type"] == "gripper":
                    # 夹爪任务：直接添加夹爪命令
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
                else:
                    # 运动任务：规划轨迹并添加
                    positions, end_position = self.trajectory_planner._plan_single_task(
                        task, 
                        self._planning_cursor_position
                    )
                    
                    if len(positions) > 0:
                        self.motion_runner.add_motion_data(positions)
                        # 更新游标：指向该任务的终点
                        self._planning_cursor_position = end_position

        # 循环正常结束（无中断），启动执行所有积攒的指令
        if len(self.motion_runner.data_list) > 0:
            self.motion_runner.start_motion()
        
        # 如果队列空了，清理操作状态
        if not self._pending_tasks:
            self.clear_operation()
            self.motion_plan_finished.emit() # 发射方案完成信号

    def _on_motion_finished(self):
        """当MotionRunner完成一组运动时触发。"""
        # 检查是否是因为遇到检测节点而暂停的
        if self._is_waiting_for_detection:
            # 确认队首确实是检测任务（双重保险）
            if self._pending_tasks and self._pending_tasks[0]["type"] == "detect":
                # 发射信号，请求外部进行检测
                # 此时机器人应该已经到达了 _planning_cursor_position
                self.detection_requested.emit(self._planning_cursor_position)

    def resume_after_detection(self, target_position: List[float]):
        """从检测中断中恢复，并注入目标位置。
        
        外部服务在完成检测和计算后调用此方法。
        此方法会自动规划一段从当前位置（拍照点）到目标位置的过渡轨迹。
        
        Args:
            target_position (List[float]): 视觉算出的目标关节角度。
        """
        if not self._is_waiting_for_detection:
            return
            
        # 1. 移除那个阻断的 detect 任务
        if self._pending_tasks and self._pending_tasks[0]["type"] == "detect":
            self._pending_tasks.pop(0)
            
        # 2. 规划过渡运动（从拍照点 -> 目标点）
        # 获取当前物理位置（即拍照点，因为游标尚未更新）
        current_physical_pos = self._planning_cursor_position
        
        # 构造过渡任务
        transition_task = {
            "type": "motion",
            "target_angles": target_position,
            "curve_type": "s_curve",  # 默认使用S曲线平滑过渡
            "frequency": 0.01
        }
        
        # 规划轨迹
        positions, _ = self.trajectory_planner._plan_single_task(
            transition_task,
            current_physical_pos
        )
        
        # 将过渡轨迹添加到执行器缓冲
        if len(positions) > 0:
            self.motion_runner.add_motion_data(positions)
            
        # 3. 更新规划游标
        # 逻辑上机器人现在位于目标点
        self._planning_cursor_position = target_position
        self._is_waiting_for_detection = False
        
        # 4. 恢复处理循环
        # 注意：这里不需要 clear_data，因为我们要保留刚才生成的过渡轨迹
        self._process_task_queue()
