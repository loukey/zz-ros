from .motion_runner import MotionRunner
from ..algorithm import SCurve, SmoothDomainService, LinearMotionDomainService, CurveMotionDomainService
from typing import List, Dict
import numpy as np

class MotionConstructor:
    """
    运动构造器 - Domain层
    
    职责：
    1. 管理运动任务队列
    2. 调用算法服务进行轨迹规划（S曲线/直线）
    3. 管理运动执行状态（是否需要自动构建）
    4. 维护位置状态，一次性规划所有任务
    """
    
    def __init__(
        self, 
        motion_runner: MotionRunner,
        smooth_service: SmoothDomainService,
        linear_motion_service: LinearMotionDomainService,
        curve_motion_service: CurveMotionDomainService):
        """
        初始化运动构造器
        
        Args:
            motion_runner: 运动执行器
            smooth_service: 轨迹平滑服务（依赖注入）
            linear_motion_service: 直线运动规划服务（依赖注入）
        """
        self.motion_runner = motion_runner
        self.smooth_service = smooth_service
        self.linear_motion_service = linear_motion_service
        self.curve_motion_service = curve_motion_service
        self.s_curve = SCurve()
        self._has_pending_motion = False
        self._pending_tasks = []  # 待规划的任务列表

    def clear_data(self):
        """清空任务队列"""
        self._has_pending_motion = False
        self._pending_tasks.clear()
    
    def prepare_motion(self, task: Dict):
        """
        准备单个运动任务（等待位置数据后自动执行）
        
        内部统一调用 prepare_motion_sequence，将单个任务包装为列表
        
        Args:
            task: 任务字典（格式同 add_motion_data）
        """
        self.prepare_motion_sequence([task])
    
    def prepare_motion_sequence(self, tasks_list: List[Dict]):
        """
        准备运动序列（等待位置数据后自动执行）
        
        用于批量运动任务（如运行整个运动规划方案）：
        1. 暂存所有任务
        2. 标记有待执行的运动
        3. 等待获取当前位置
        4. 位置到达后一次性规划所有任务
        
        Args:
            tasks_list: 任务字典列表
        """
        self.clear_data()
        self._pending_tasks = tasks_list
        self._has_pending_motion = True
    
    def has_pending_motion(self) -> bool:
        """
        检查是否有待执行的运动任务
        
        Returns:
            bool: True 表示有待执行的运动，收到位置后应自动构建并执行
        """
        return self._has_pending_motion

    def construct_motion_data(self, start_position: List[float]):
        """
        根据任务队列构建运动轨迹（统一调用 construct_motion_sequence）
        
        Args:
            start_position: 起始位置（当前关节角度）
        """
        self.construct_motion_sequence(start_position)

    def construct_motion_sequence(self, start_position: List[float]):
        """
        规划所有任务的轨迹
        
        核心逻辑：
        1. 维护 current_position 状态
        2. 每个运动任务的终点 = 下一个任务的起点
        3. 夹爪任务不改变位置
        4. 所有任务规划完成后，MotionRunner 按序发送
        
        Args:
            start_position: 起始位置（只在最开始查询一次）
        """
        if not self._pending_tasks:
            return
        
        self.motion_runner.clear_data()
        current_position = start_position
        
        for task in self._pending_tasks:
            current_position = self._process_task(task, current_position)
        
        self._has_pending_motion = False
        self._pending_tasks.clear()
        
        # 启动运动执行
        self.motion_runner.start_motion()
    
    def _process_task(self, task: Dict, current_position: List[float]) -> List[float]:
        """
        处理单个任务并返回新的当前位置
        
        Args:
            task: 任务字典
            current_position: 当前位置
            
        Returns:
            新的当前位置
        """
        task_type = task["type"]
        
        if task_type == "motion":
            # 普通运动点（S曲线或直线）
            return self._construct_motion(task, current_position)
            
        elif task_type == "teach":
            # 示教轨迹
            return self._construct_teach(task, current_position)
            
        elif task_type == "vector_motion":
            # 向量直线运动
            return self._construct_vector_motion(task, current_position)
            
        elif task_type == "curve_motion":
            # 曲线运动
            return self._construct_curve_motion(task, current_position)
            
        elif task_type == "gripper":
            # 夹爪操作（不改变位置）
            self._construct_gripper(task)
            return current_position  # 位置不变
        
        else:
            return current_position
    
    def _construct_motion(self, task: Dict, start_position: List[float]) -> List[float]:
        """
        构建普通运动任务（S曲线或直线）
        
        Args:
            task: 任务字典，包含 target_angles, curve_type, frequency
            start_position: 起始位置
            
        Returns:
            终点位置
        """
        end_position = task["target_angles"]
        curve_type = task.get("curve_type", "s_curve")
        frequency = task.get("frequency", 0.01)
        
        if curve_type == "linear":
            # 直线运动规划
            positions = self.linear_motion_service.linear_motion(
                start_position, 
                end_position
            )

        else:
            # S曲线运动规划（默认）
            # 使用关键字参数 dt 指定采样时间间隔，v_start 使用默认值 [0]*6
            _, _, _, positions = self.s_curve.planning(
                start_position, 
                end_position,
                dt=frequency
            )
        
        # 添加到运动执行队列
        self.motion_runner.add_motion_data(positions)
        
        return end_position
    
    def _construct_teach(self, task: Dict, start_position: List[float]) -> List[float]:
        """
        构建示教轨迹运动
        
        流程：
        1. S-curve 规划：从当前位置到示教第一个点
        2. 调用平滑服务：对示教轨迹进行平滑
        3. 合并轨迹：初始段 + 平滑后的示教段
        4. 添加到 Runner
        
        Args:
            task: 任务字典，包含 teach_data
            start_position: 起始位置
            
        Returns:
            终点位置（示教数据的最后一个点）
        """
        teach_data = task["teach_data"]
        
        # 检查示教数据是否为空
        if not teach_data or len(teach_data) == 0:
            return start_position  # 位置不变
        
        target_first_point = teach_data[0]
        
        # 从当前位置规划到示教第一个点
        _, _, _, initial_positions = self.s_curve.planning(
            start_position, 
            target_first_point
        )
        
        # 对示教数据进行平滑处理
        smoothed_positions = self.smooth_service.smooth_trajectory(
            teach_data,
            method="spline_savgol",
            upsample=5,
            sg_window=211,
            sg_poly=3
        )
        
        # 合并轨迹
        all_positions = initial_positions.tolist()
        all_positions.extend(smoothed_positions)
        
        # 添加到运动执行队列
        self.motion_runner.add_motion_data(all_positions)
        
        return teach_data[-1]
    
    def _construct_vector_motion(self, task: Dict, start_position: List[float]) -> List[float]:
        """
        构建向量直线运动任务
        
        使用 linear_motion_z_axis 方法，从当前位置沿指定方向移动指定距离
        
        Args:
            task: 任务字典，包含 distance, direction, frequency
            start_position: 起始位置
            
        Returns:
            终点位置
        """
        distance = task["distance"]
        direction = task["direction"]
        frequency = task.get("frequency", 0.01)
        
        # 调用 linear_motion_z_axis，直接返回关节角度序列
        positions = self.linear_motion_service.linear_motion_z_axis(
            start_position=start_position,
            distance=distance,
            direction=direction
        )
        
        # 添加到运动执行队列
        self.motion_runner.add_motion_data(positions)
        
        # 返回终点关节角度（用于下一个任务的起点）
        return positions[-1] if positions else start_position
    
    def _construct_gripper(self, task: Dict):
        """
        构建夹爪操作
        
        注意：夹爪操作会自动添加1秒延迟（在 MotionRunner.add_gripper_data 中）
        
        Args:
            task: 任务字典，包含 effector_mode, effector_data
        """
        effector_mode = task["effector_mode"]
        effector_data = task["effector_data"]
        
        self.motion_runner.add_gripper_data(effector_mode, effector_data)

    def _construct_curve_motion(self, task: Dict, start_position: List[float]) -> List[float]:
        """
        构建曲线运动任务
        
        Args:
            task: 任务字典，包含 curve_type, frequency
            start_position: 起始位置
            
        Returns:
            终点位置
        """
        end_position = task["target_position"]
        mid_points = task["mid_points"]
        pos_fun,s = self.curve_motion_service.make_pos_fun_spline(start_position, end_position, mid_points, bc_type="natural")


        positions = self.curve_motion_service.curve_motion(
            pos_fun=pos_fun, 
            u0=0, 
            u1=1, 
            start_position=start_position, 
            end_position=end_position, 
            ds = 0.002, 
            include_end=True,
            orientation_mode = "slerp", 
            tool_axis = "z", 
            up_hint= np.array([0, 0, 1.0]))
        
        self.motion_runner.add_motion_data(positions)
        
        return positions[-1] if positions else start_position

