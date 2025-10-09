from .motion_runner import MotionRunner
from ..algorithm import SCurve, SmoothDomainService


class MotionConstructor:
    """
    运动构造器 - Domain层
    
    职责：
    1. 管理运动任务队列
    2. 调用算法服务进行轨迹规划
    3. 管理运动执行状态（是否需要自动构建）
    """
    
    def __init__(
        self, 
        motion_runner: MotionRunner,
        smooth_service: SmoothDomainService):
        """
        初始化运动构造器
        
        Args:
            motion_runner: 运动执行器
            smooth_service: 轨迹平滑服务（依赖注入）
        """
        self.motion_runner = motion_runner
        self.smooth_service = smooth_service
        self.s_curve = SCurve()
        self.data_list = []
        self._has_pending_motion = False

    def clear_data(self):
        """清空任务队列"""
        self.data_list = []
        self._has_pending_motion = False

    def add_motion_data(self, motion_type: str, data):
        """
        添加运动任务到队列
        
        Args:
            motion_type: 运动类型
                - "motion": 单个目标点，data = [θ1, ..., θ6]
                - "gripper": 夹爪控制，data = {"effector_mode": ..., "effector_data": ...}
                - "teach": 示教轨迹，data = [[θ1,...], [θ2,...], ...]
            data: 运动数据
        """
        self.data_list.append((motion_type, data))
    
    def prepare_motion(self, motion_type: str, data):
        """
        准备运动任务（等待位置数据后自动执行）
        
        此方法用于异步运动流程：
        1. 添加运动任务到队列
        2. 标记有待执行的运动
        3. 等待获取当前位置
        4. 位置到达后自动构建并执行
        
        Args:
            motion_type: 运动类型 ("motion", "teach", "gripper")
            data: 运动数据
        """
        self.clear_data()
        self.add_motion_data(motion_type, data)
        self._has_pending_motion = True
    
    def has_pending_motion(self) -> bool:
        """
        检查是否有待执行的运动任务
        
        Returns:
            bool: True 表示有待执行的运动，收到位置后应自动构建并执行
        """
        return self._has_pending_motion

    def construct_motion_data(self, start_position: list):
        """
        根据任务队列构建完整运动轨迹
        
        流程：
        1. 清空 Runner 的消息列表
        2. 遍历任务队列
        3. 根据任务类型调用不同的算法服务
        4. 将编码后的消息添加到 Runner
        5. 清除待执行标志
        
        Args:
            start_position: 起始位置（当前关节角度）
        """
        if not self.data_list:
            return
        
        self.motion_runner.clear_data()
        last_position = start_position
        
        for motion_type, data in self.data_list:
            if motion_type == "motion":
                _, _, _, positions = self.s_curve.planning(last_position, data)
                self.motion_runner.add_motion_data(positions)
                last_position = data
                
            elif motion_type == "gripper":
                self.motion_runner.add_gripper_data(**data)
                
            elif motion_type == "teach":
                self._construct_teach_motion(last_position, data)
                last_position = data[-1]
        
        self._has_pending_motion = False
    
    def _construct_teach_motion(self, start_position: list, angles_list: list):
        """
        构建示教轨迹运动（内部方法）
        
        流程：
        1. S-curve 规划：从当前位置到示教第一个点
        2. 调用平滑服务：对示教轨迹进行平滑
        3. 合并轨迹：初始段 + 平滑后的示教段
        4. 添加到 Runner
        
        Args:
            start_position: 起始位置
            angles_list: 示教记录的角度列表
        """
        target_first_point = angles_list[0]
        
        _, _, _, initial_positions = self.s_curve.planning(
            start_position, 
            target_first_point
        )
        
        smoothed_positions = self.smooth_service.smooth_trajectory(
            angles_list,
            method="spline_savgol",
            upsample=5,
            sg_window=211,
            sg_poly=3
        )
        
        all_positions = initial_positions.tolist()
        all_positions.extend(smoothed_positions)
        
        self.motion_runner.add_motion_data(all_positions)
