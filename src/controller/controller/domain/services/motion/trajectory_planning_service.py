"""
轨迹规划服务 - Domain层
纯粹的轨迹规划逻辑，不涉及执行或持久化
"""
from typing import List, Dict, Tuple
import numpy as np
from math import pi
from ..algorithm import SCurve, SmoothDomainService, LinearMotionDomainService, CurveMotionDomainService


class TrajectoryPlanningService:
    """轨迹规划服务 - 领域服务。
    
    职责：
    1. 根据任务类型规划轨迹点序列
    2. 维护位置状态（起点->终点）
    3. 返回纯数据，不执行任何副作用
    
    核心原则：纯函数，可测试，可复用
    
    Attributes:
        s_curve (SCurve): S曲线算法服务。
        smooth_service (SmoothDomainService): 平滑算法服务。
        linear_motion_service (LinearMotionDomainService): 直线运动服务。
        curve_motion_service (CurveMotionDomainService): 曲线运动服务。
    """
    
    def __init__(
        self,
        s_curve: SCurve,
        smooth_service: SmoothDomainService,
        linear_motion_service: LinearMotionDomainService,
        curve_motion_service: CurveMotionDomainService
    ):
        """初始化轨迹规划服务。
        
        Args:
            s_curve (SCurve): S曲线算法服务。
            smooth_service (SmoothDomainService): 平滑算法服务。
            linear_motion_service (LinearMotionDomainService): 直线运动服务。
            curve_motion_service (CurveMotionDomainService): 曲线运动服务。
        """
        self.s_curve = s_curve
        self.smooth_service = smooth_service
        self.linear_motion_service = linear_motion_service
        self.curve_motion_service = curve_motion_service
    
    def plan_task_sequence(
        self,
        tasks: List[Dict],
        start_position: List[float]
    ) -> List[List[float]]:
        """规划任务序列（仅返回位置）。
        
        用于：执行运动、保存轨迹。
        
        Args:
            tasks (List[Dict]): 任务列表。
            start_position (List[float]): 起始位置。
            
        Returns:
            List[List[float]]: 轨迹点序列 [[q1,...,q6], ...]。
        """
        all_positions = []
        current_position = start_position
        
        for task in tasks:
            positions, end_position = self._plan_single_task(task, current_position)
            all_positions.extend(positions)
            current_position = end_position
        
        return all_positions
    
    def plan_task_sequence_with_derivatives(
        self,
        tasks: List[Dict],
        start_position: List[float]
    ) -> Dict:
        """规划任务序列（返回位置、速度、加速度、时间）。
        
        用于：预览轨迹曲线。
        
        Args:
            tasks (List[Dict]): 任务列表。
            start_position (List[float]): 起始位置。
            
        Returns:
            Dict: 包含以下键的字典:
                - time: [t0, t1, ...]
                - positions: [[q1,...,q6], ...]
                - velocities: [[qd1,...,qd6], ...]
                - accelerations: [[qdd1,...,qdd6], ...]
        """
        all_time = []
        all_positions = []
        all_velocities = []
        all_accelerations = []
        
        current_position = start_position
        current_time = 0.0
        
        for task in tasks:
            result = self._plan_single_task_full(task, current_position)
            
            # 合并数据（时间累加）
            all_time.extend([t + current_time for t in result["time"]])
            all_positions.extend(result["positions"])
            all_velocities.extend(result["velocities"])
            all_accelerations.extend(result["accelerations"])
            
            # 更新状态
            if len(result["positions"]) > 0:
                current_position = result["positions"][-1]
                if len(result["time"]) > 0:
                    current_time = all_time[-1]
        
        return {
            "time": all_time,
            "positions": all_positions,
            "velocities": all_velocities,
            "accelerations": all_accelerations
        }
    
    def _plan_single_task(
        self, 
        task: Dict, 
        start_position: List[float]
    ) -> Tuple[List[List[float]], List[float]]:
        """规划单个任务（仅返回位置）。
        
        Args:
            task (Dict): 任务字典。
            start_position (List[float]): 起始位置。
            
        Returns:
            Tuple[List[List[float]], List[float]]: (轨迹点序列, 终点位置)。
        """
        task_type = task["type"]
        
        if task_type == "motion":
            return self._plan_motion(task, start_position)
        elif task_type == "teach":
            return self._plan_teach(task, start_position)
        elif task_type == "vector_motion":
            return self._plan_vector_motion(task, start_position)
        elif task_type == "curve_motion":
            return self._plan_curve_motion(task, start_position)
        elif task_type == "gripper":
            return ([], start_position)
        else:
            return ([], start_position)
    
    def _plan_single_task_full(
        self,
        task: Dict,
        start_position: List[float]
    ) -> Dict:
        """规划单个任务（返回完整数据：时间、位置、速度、加速度）。
        
        Args:
            task (Dict): 任务字典。
            start_position (List[float]): 起始位置。
            
        Returns:
            Dict: 包含 time, positions, velocities, accelerations 的字典。
        """
        task_type = task["type"]
        
        if task_type == "motion":
            return self._plan_motion_full(task, start_position)
        elif task_type == "teach":
            return self._plan_teach_full(task, start_position)
        elif task_type == "vector_motion":
            return self._plan_vector_motion_full(task, start_position)
        elif task_type == "curve_motion":
            return self._plan_curve_motion_full(task, start_position)
        elif task_type == "gripper":
            return {"time": [], "positions": [], "velocities": [], "accelerations": []}
        else:
            return {"time": [], "positions": [], "velocities": [], "accelerations": []}
    
    # ========== 具体规划方法（位置版本） ==========
    
    def _plan_motion(self, task: Dict, start_position: List[float]) -> Tuple[List[List[float]], List[float]]:
        """S曲线/直线运动规划（仅返回位置）。"""
        end_position = task["target_angles"]
        curve_type = task.get("curve_type", "s_curve")
        frequency = task.get("frequency", 0.01)
        
        if curve_type == "linear":
            # 直线运动规划
            _, positions, _, _ = self.linear_motion_service.linear_motion(
                start_position, 
                end_position
            )
        else:
            # S曲线运动规划（默认）
            _, _, _, positions = self.s_curve.planning(
                start_position, 
                end_position,
                dt=frequency
            )
            # S曲线返回numpy数组，需要转换为列表
            if hasattr(positions, 'tolist'):
                positions = positions.tolist()
        
        return (positions, end_position)
    
    def _plan_teach(self, task: Dict, start_position: List[float]) -> Tuple[List[List[float]], List[float]]:
        """示教轨迹规划（仅返回位置）。"""
        teach_data = task["teach_data"]
        
        if not teach_data or len(teach_data) == 0:
            return ([], start_position)
        
        target_first_point = teach_data[0]
        end_position = teach_data[-1]
        
        # 从当前位置规划到示教第一个点
        _, _, _, initial_positions = self.s_curve.planning(
            start_position, 
            target_first_point
        )
        
        # 对示教数据进行平滑处理
        smoothed_positions = self.smooth_service.teach_smooth(
            teach_data,
            target_first_point,
            end_position,
            step=0.5,
            eps=1e-6            
        )
        
        toppra_positions = self.smooth_service.toppra_smooth(
            smoothed_positions,
            v_max=[pi/4] * 6,
            a_max=[pi/8] * 6,
            dt = 0.01,
            grid_n = 400
        )
        
        # 合并轨迹
        all_positions = initial_positions.tolist()
        all_positions.extend(toppra_positions)
        
        return (all_positions, teach_data[-1])
    
    def _plan_vector_motion(self, task: Dict, start_position: List[float]) -> Tuple[List[List[float]], List[float]]:
        """向量直线运动规划（仅返回位置）。"""
        distance = task["distance"]
        direction = task["direction"]
        
        _, positions, _, _ = self.linear_motion_service.linear_motion_z_axis(
            start_position=start_position,
            distance=distance,
            direction=direction
        )
        
        end_position = positions[-1] if len(positions) > 0 else start_position
        return (positions, end_position)
    
    def _plan_curve_motion(self, task: Dict, start_position: List[float]) -> Tuple[List[List[float]], List[float]]:
        """曲线运动规划（仅返回位置）。"""
        end_position = task["target_position"]
        mid_points = task["mid_points"]
        
        pos_fun, s = self.curve_motion_service.make_pos_fun_spline(
            start_position, 
            end_position, 
            mid_points, 
            bc_type="natural"
        )
        
        _, positions, _, _ = self.curve_motion_service.curve_motion(
            pos_fun=pos_fun, 
            u0=0, 
            u1=1, 
            start_position=start_position, 
            end_position=end_position, 
            ds=0.002, 
            include_end=True,
            orientation_mode="slerp", 
            tool_axis="z", 
            up_hint=np.array([0, 0, 1.0])
        )
        
        final_position = positions[-1] if len(positions) > 0 else start_position
        return (positions, final_position)
    
    # ========== 具体规划方法（完整数据版本） ==========
    
    def _plan_motion_full(self, task: Dict, start_position: List[float]) -> Dict:
        """S曲线/直线运动规划（完整数据）。"""
        end_position = task["target_angles"]
        curve_type = task.get("curve_type", "s_curve")
        frequency = task.get("frequency", 0.01)
        
        if curve_type == "linear":
            # 直线运动：只有位置，没有速度加速度数据
            time, positions, velocities, accelerations = self.linear_motion_service.linear_motion(
                start_position, 
                end_position
            )

        else:
            # S曲线运动规划
            # 注意：s_curve.planning 返回顺序是 times, accelerations, velocities, positions
            time, accelerations, velocities, positions = self.s_curve.planning(
                start_position, 
                end_position,
                dt=frequency
            )
        # 确保转换为列表
        time = time if isinstance(time, list) else time.tolist() if hasattr(time, 'tolist') else list(time)
        positions = positions if isinstance(positions, list) else positions.tolist() if hasattr(positions, 'tolist') else list(positions)
        velocities = velocities if isinstance(velocities, list) else velocities.tolist() if hasattr(velocities, 'tolist') else list(velocities)
        accelerations = accelerations if isinstance(accelerations, list) else accelerations.tolist() if hasattr(accelerations, 'tolist') else list(accelerations)
        
        return {
            "time": time,
            "positions": positions,
            "velocities": velocities,
            "accelerations": accelerations
        }
    
    def _plan_teach_full(self, task: Dict, start_position: List[float]) -> Dict:
        """示教轨迹规划（完整数据）。"""
        teach_data = task["teach_data"]
        
        if not teach_data or len(teach_data) == 0:
            return {"time": [], "positions": [], "velocities": [], "accelerations": []}
        
        target_first_point = teach_data[0]
        end_position = teach_data[-1]
        
        # 从当前位置规划到示教第一个点
        # 注意：s_curve.planning 返回顺序是 times, accelerations, velocities, positions
        t1, a1, v1, p1 = self.s_curve.planning(start_position, target_first_point)
        
        # 确保转换为列表
        t1 = t1 if isinstance(t1, list) else t1.tolist() if hasattr(t1, 'tolist') else list(t1)
        p1 = p1 if isinstance(p1, list) else p1.tolist() if hasattr(p1, 'tolist') else list(p1)
        v1 = v1 if isinstance(v1, list) else v1.tolist() if hasattr(v1, 'tolist') else list(v1)
        a1 = a1 if isinstance(a1, list) else a1.tolist() if hasattr(a1, 'tolist') else list(a1)
        
        # 对示教数据进行平滑处理
        smoothed_positions = self.smooth_service.teach_smooth(
            teach_data,
            target_first_point,
            end_position,
            step=0.5,
            eps=1e-6            
        )
        
        toppra_positions = self.smooth_service.toppra_smooth(
            smoothed_positions,
            v_max=[pi/4] * 6,
            a_max=[pi/8] * 6,
            dt=0.01
        )
        
        # 合并轨迹
        all_positions = p1 + toppra_positions
        
        # 生成时间序列
        time = [i * 0.01 for i in range(len(all_positions))]
        
        # 速度和加速度（简化处理）
        velocities = v1 + [[0.0] * 6 for _ in toppra_positions]
        accelerations = a1 + [[0.0] * 6 for _ in toppra_positions]
        
        return {
            "time": time,
            "positions": all_positions,
            "velocities": velocities,
            "accelerations": accelerations
        }
    
    def _plan_vector_motion_full(self, task: Dict, start_position: List[float]) -> Dict:
        """向量直线运动规划（完整数据）。"""
        distance = task["distance"]
        direction = task["direction"]
        
        time, positions, velocities, accelerations = self.linear_motion_service.linear_motion_z_axis(
            start_position=start_position,
            distance=distance,
            direction=direction
        )

        # 确保转换为列表
        time = time if isinstance(time, list) else time.tolist() if hasattr(time, 'tolist') else list(time)
        positions = positions if isinstance(positions, list) else positions.tolist() if hasattr(positions, 'tolist') else list(positions)
        velocities = velocities if isinstance(velocities, list) else velocities.tolist() if hasattr(velocities, 'tolist') else list(velocities)
        accelerations = accelerations if isinstance(accelerations, list) else accelerations.tolist() if hasattr(accelerations, 'tolist') else list(accelerations)
        
        return {
            "time": time,
            "positions": positions,
            "velocities": velocities,
            "accelerations": accelerations
        }
    
    def _plan_curve_motion_full(self, task: Dict, start_position: List[float]) -> Dict:
        """曲线运动规划（完整数据）。"""
        end_position = task["target_position"]
        mid_points = task["mid_points"]
        
        pos_fun, s = self.curve_motion_service.make_pos_fun_spline(
            start_position, 
            end_position, 
            mid_points, 
            bc_type="natural"
        )
        
        time, positions, velocities, accelerations = self.curve_motion_service.curve_motion(
            pos_fun=pos_fun, 
            u0=0, 
            u1=1, 
            start_position=start_position, 
            end_position=end_position, 
            ds=0.002, 
            include_end=True,
            orientation_mode="slerp", 
            tool_axis="z", 
            up_hint=np.array([0, 0, 1.0])
        )
        
        # 确保转换为列表
        time = time if isinstance(time, list) else time.tolist() if hasattr(time, 'tolist') else list(time)
        positions = positions if isinstance(positions, list) else positions.tolist() if hasattr(positions, 'tolist') else list(positions)
        velocities = velocities if isinstance(velocities, list) else velocities.tolist() if hasattr(velocities, 'tolist') else list(velocities)
        accelerations = accelerations if isinstance(accelerations, list) else accelerations.tolist() if hasattr(accelerations, 'tolist') else list(accelerations)
        
        return {
            "time": time,
            "positions": positions,
            "velocities": velocities,
            "accelerations": accelerations
        }

