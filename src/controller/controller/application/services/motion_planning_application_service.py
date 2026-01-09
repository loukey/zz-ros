"""
运动规划应用服务
"""
from PyQt5.QtCore import QObject, pyqtSignal
from controller.domain import (
    MotionPlanningDomainService, 
    MotionPlan, 
    MotionConstructor,
    MotionOperationMode
)
from controller.infrastructure import MotionPlanRepository, TrajectoryRepository
from .command_hub_service import CommandHubService
from .message_response_service import MessageResponseService
from typing import List, Dict, Any
from pathlib import Path


class MotionPlanningApplicationService(QObject):
    """运动规划应用服务。
    
    职责：
    1. 协调Domain和Infrastructure
    2. 自动保存
    3. 发送UI更新信号
    4. 执行运动规划（单点和整体方案）
    5. 处理"获取位置"功能

    Attributes:
        plan_list_changed (pyqtSignal): 方案列表变化信号。
        current_plan_changed (pyqtSignal): 当前方案切换信号。
        point_list_changed (pyqtSignal): 节点列表变化信号。
        current_position_received (pyqtSignal): 当前位置数据信号（弧度值）。
        trajectory_preview_signal (pyqtSignal): 轨迹预览数据信号（轨迹数据，上下文）。
    """
    
    # 信号
    plan_list_changed = pyqtSignal()  # 方案列表变化
    current_plan_changed = pyqtSignal(int)  # 当前方案切换
    point_list_changed = pyqtSignal()  # 节点列表变化
    current_position_received = pyqtSignal(list)  # 当前位置数据（弧度值）
    trajectory_preview_signal = pyqtSignal(dict, dict)  # 轨迹预览数据（轨迹数据，上下文）
    detection_service_requested = pyqtSignal(bool)  # 检测服务控制请求 True=Start, False=Stop
    
    # 夹爪命令映射表：从字符串转为 effector_mode
    GRIPPER_MODE_MAP = {
        "00: 不进行任何操作": 0x00,
        "01: 夹爪手动使能": 0x01,
        "02: 设置夹爪目标位置": 0x02,
        "03: 设置夹爪速度": 0x03,
        "04: 设置夹爪电流": 0x04,
        "05: 查询夹爪抓取状态": 0x05,
        "06: 查询夹爪目前位置": 0x06,
        "07: 查询夹爪电流": 0x07
    }
    
    def __init__(
        self,
        domain_service: MotionPlanningDomainService,
        repository: MotionPlanRepository,
        motion_constructor: MotionConstructor,
        command_hub: CommandHubService,
        message_response: MessageResponseService,
        trajectory_repository: TrajectoryRepository
    ):
        """初始化运动规划应用服务。
        
        Args:
            domain_service: 运动规划领域服务。
            repository: 运动方案仓储。
            motion_constructor: 运动构造器。
            command_hub: 命令中心服务。
            message_response: 消息响应服务。
            trajectory_repository: 轨迹仓储。
        """
        super().__init__()
        self.domain_service = domain_service
        self.repository = repository
        self.motion_constructor = motion_constructor
        self.command_hub = command_hub
        self.message_response = message_response
        self.trajectory_repository = trajectory_repository
        
        # 连接MessageResponse的位置数据信号
        self.message_response.get_current_position_signal.connect(
            self.current_position_received.emit
        )
        
        # 连接MessageResponse的轨迹预览信号
        self.message_response.trajectory_preview_signal.connect(
            self.trajectory_preview_signal.emit
        )
        
        # 监听方案完成信号，用于自动停止检测
        self.motion_constructor.motion_plan_finished.connect(
            self._on_motion_plan_finished_stop_detection
        )
        
        # 标记是否由本服务自动开启了检测
        self._auto_started_detection = False
        
        self._load_data()
    
    def _load_data(self):
        """加载数据"""
        plans_data, current_index = self.repository.load()
        
        # 通过Domain Service的公共方法初始化数据
        plans = [
            MotionPlan(name=p["name"], points=p.get("points", []))
            for p in plans_data
        ]
        self.domain_service.initialize(plans, current_index)
    
    def _save_data(self):
        """保存数据"""
        plans_data = [
            {"name": plan.name, "points": plan.points}
            for plan in self.domain_service.get_all_plans()
        ]
        self.repository.save(plans_data, self.domain_service.get_current_index())
    
    # ========== 方案操作 ==========
    
    def create_plan(self, name: str):
        """创建方案。
        
        Args:
            name (str): 方案名称。
        """
        new_index = self.domain_service.create_plan(name)
        self._save_data()
        self.plan_list_changed.emit()
        self.current_plan_changed.emit(new_index)
        self.point_list_changed.emit()
    
    def delete_plan(self, index: int) -> bool:
        """删除方案。
        
        Args:
            index (int): 方案索引。
        
        Returns:
            bool: True=删除成功, False=删除失败（违反业务规则）。
        """
        if self.domain_service.delete_plan(index):
            self._save_data()
            self.plan_list_changed.emit()
            self.current_plan_changed.emit(self.domain_service.get_current_index())
            self.point_list_changed.emit()
            return True
        return False
    
    def switch_plan(self, index: int):
        """切换方案。
        
        Args:
            index (int): 目标方案索引。
        """
        if self.domain_service.set_current_index(index):
            self._save_data()
            self.current_plan_changed.emit(index)
            self.point_list_changed.emit()
    
    def rename_plan(self, index: int, new_name: str):
        """重命名方案。
        
        Args:
            index (int): 方案索引。
            new_name (str): 新名称。
        """
        if self.domain_service.rename_plan(index, new_name):
            self._save_data()
            self.plan_list_changed.emit()
    
    # ========== 节点操作 ==========
    
    def add_point(self, point_data: dict):
        """添加节点"""
        self.domain_service.add_point(point_data)
        self._save_data()
        self.point_list_changed.emit()
    
    def delete_point(self, index: int):
        """删除节点"""
        self.domain_service.remove_point(index)
        self._save_data()
        self.point_list_changed.emit()
    
    def move_point_up(self, index: int):
        """上移节点"""
        if self.domain_service.move_point_up(index):
            self._save_data()
            self.point_list_changed.emit()
    
    def move_point_down(self, index: int):
        """下移节点"""
        if self.domain_service.move_point_down(index):
            self._save_data()
            self.point_list_changed.emit()
    
    def update_point(self, index: int, point_data: dict):
        """更新节点"""
        self.domain_service.update_point(index, point_data)
        self._save_data()
        self.point_list_changed.emit()
    
    def get_single_point(self, index: int) -> dict:
        """获取单个节点数据。
        
        Args:
            index (int): 节点索引。
            
        Returns:
            dict: 节点数据字典，如果索引无效则返回None。
        """
        points = self.domain_service.get_all_points()
        if 0 <= index < len(points):
            return points[index]
        return None
    
    # ========== 执行操作 ==========
    
    def execute_single_point(self, index: int):
        """执行单个节点。
        
        流程：
        1. 获取节点数据
        2. 解析为任务列表
        3. 准备执行操作
        4. 查询当前位置
        
        Args:
            index (int): 节点索引。
        """
        tasks = self._get_node_tasks(index)
        if not tasks:
            return
        
        # 预先扫描：如果包含检测任务，处理检测服务的生命周期
        has_detection = any(task.get("type") == "detect" for task in tasks)
        if has_detection:
            # 1. 启动检测服务，并标记是由我们自动开启的
            self.detection_service_requested.emit(True)
            self._auto_started_detection = True
        else:
            self._auto_started_detection = False

        # 准备执行操作
        self.motion_constructor.prepare_operation(
            MotionOperationMode.EXECUTE,
            tasks
        )
        
        # 查询当前位置，触发执行
        self.command_hub.get_current_position()
    
    def execute_motion_plan(self):
        """执行整个运动规划方案。
        
        流程：
        1. 获取所有节点的任务
        2. 预先扫描任务，若有检测节点则提前启动检测服务并监听结束信号
        3. 准备执行操作
        4. 查询当前位置，触发执行
        """
        tasks = self._get_plan_tasks()
        if not tasks:
            return
        
        # 预先扫描：如果包含检测任务，处理检测服务的生命周期
        has_detection = any(task.get("type") == "detect" for task in tasks)
        if has_detection:
            # 1. 启动检测服务，并标记是由我们自动开启的
            self.detection_service_requested.emit(True)
            self._auto_started_detection = True
        else:
            self._auto_started_detection = False
        
        # 准备执行操作
        self.motion_constructor.prepare_operation(
            MotionOperationMode.EXECUTE,
            tasks
        )
        
        # 查询当前位置，触发执行
        self.command_hub.run_motion_sequence()

    def _on_motion_plan_finished_stop_detection(self):
        """当包含检测的方案执行完毕后，停止检测服务。"""
        # 只有当我们之前自动开启了检测时，才负责关闭
        if self._auto_started_detection:
            self.detection_service_requested.emit(False)
            self._auto_started_detection = False
    
    def _parse_point_to_tasks(self, point: Dict) -> List[Dict]:
        """将节点数据解析为任务列表。
        
        注意：每个节点只会生成一个任务，不会同时有运动和夹爪
        - 如果是示教节点，只返回示教任务
        - 如果是运动点，只返回运动任务
        - 如果是夹爪节点，只返回夹爪任务
        
        Args:
            point (Dict): 节点数据字典，包含：
                - mode: 模式（"运动点"、"示教-xxx"、"夹爪"等）
                - joint_angles: 关节角度
                - curve_type: 曲线类型（"S曲线"、"直线"）
                - frequency: 频率
                - gripper_command: 夹爪命令
                - gripper_param: 夹爪参数
                - teach_data: 示教数据（示教模式节点）
                
        Returns:
            List[Dict]: 任务列表，每个节点只返回一个任务。
        """
        mode = point.get("mode", "")
        
        # 1. 判断是否为示教节点
        if mode.startswith("示教-"):
            # 兼容旧字段名 teach_angles 和新字段名 teach_data
            teach_data = point.get("teach_data") or point.get("teach_angles", [])
            return [{
                "type": "teach",
                "teach_data": teach_data
            }]
            
        # 2. 判断是否为检测节点
        if mode == "检测":
            return [{
                "type": "detect"
            }]

        # 3. 带交融半径的直线运动
        if mode == "混合":
            return [{
                "type": "blend",
                "blend_data": point.get("blend_data", [])
            }]
        
        # 4. 判断是否为夹爪节点
        gripper_command = point.get("gripper_command", "00: 不进行任何操作")
        if gripper_command != "00: 不进行任何操作":
            effector_mode = self._parse_gripper_command(gripper_command)
            effector_data = point.get("gripper_param", 0.0)
            pre_delay = point.get("gripper_pre_delay", 0.0)
            post_delay = point.get("gripper_post_delay", 1.0)
            
            return [{
                "type": "gripper",
                "effector_mode": effector_mode,
                "effector_data": effector_data,
                "pre_delay": pre_delay,
                "post_delay": post_delay
            }]
        
        # 5. 判断是否为向量运动节点
        curve_type = point.get("curve_type", "S曲线")
        if curve_type == "向量":
            direction = [
                point.get("direction_x", 0.0),
                point.get("direction_y", 0.0),
                point.get("direction_z", 1.0)
            ]
            distance = point.get("distance", 0.1)
            
            return [{
                "type": "vector_motion",
                "distance": distance,
                "direction": direction,
                "frequency": point.get("frequency", 0.01)
            }]
        
        # 6. 判断是否为曲线运动节点
        if curve_type == "曲线":
            # 提取中间点坐标
            mid_points = [
                [
                    point.get("mid_point1_x", 0.0),
                    point.get("mid_point1_y", 0.0),
                    point.get("mid_point1_z", 0.0)
                ],
                [
                    point.get("mid_point2_x", 0.0),
                    point.get("mid_point2_y", 0.0),
                    point.get("mid_point2_z", 0.0)
                ]
            ]
            
            # 提取终点位置（关节角度）
            target_angles = [
                point.get("joint1", 0.0),
                point.get("joint2", 0.0),
                point.get("joint3", 0.0),
                point.get("joint4", 0.0),
                point.get("joint5", 0.0),
                point.get("joint6", 0.0)
            ]
            
            return [{
                "type": "curve_motion",
                "target_position": target_angles,
                "mid_points": mid_points,
                "frequency": point.get("frequency", 0.01)
            }]
        
        # 7. 默认为普通运动点
        # 从 UI 字段名 (joint1-joint6) 构建角度列表
        target_angles = [
            point.get("joint1", 0.0),
            point.get("joint2", 0.0),
            point.get("joint3", 0.0),
            point.get("joint4", 0.0),
            point.get("joint5", 0.0),
            point.get("joint6", 0.0)
        ]
        
        return [{
            "type": "motion",
            "target_angles": target_angles,
            "curve_type": "linear" if curve_type == "直线" else "s_curve",
            "frequency": point.get("frequency", 0.01)
        }]
    
    def _parse_gripper_command(self, gripper_str: str) -> int:
        """解析夹爪命令字符串为 effector_mode。
        
        Args:
            gripper_str (str): 夹爪命令字符串，如 "02: 设置夹爪目标位置"。
            
        Returns:
            int: effector_mode.
        """
        return self.GRIPPER_MODE_MAP.get(gripper_str, 0x00)
    
    # ========== 获取位置功能 ==========
    
    def request_current_position(self):
        """请求获取当前位置（仅用于UI显示）。
        
        流程：
        1. 发送 0x07 命令查询位置
        2. 串口返回数据后，MessageResponseService 检查 has_pending_operation()
        3. 因为没有调用 prepare_operation，所以返回 False
        4. MessageResponseService emit get_current_position_signal
        5. 本Service转发信号给ViewModel
        """
        self.command_hub.single_send_command(control=0x07)
    
    # ========== 保存轨迹功能 ==========
    
    def save_node_trajectory(self, node_index: int) -> bool:
        """保存单个节点的轨迹。
        
        流程：
        1. 获取节点任务
        2. 准备保存操作
        3. 查询当前位置
        4. MessageResponseService 自动完成保存
        
        Args:
            node_index (int): 节点索引。
            
        Returns:
            bool: True: 准备成功, False: 准备失败。
        """
        try:
            tasks = self._get_node_tasks(node_index)
            if not tasks:
                return False
            
            current_plan = self.domain_service.get_current_plan()
            if not current_plan:
                return False
            
            context = {
                "filename": f"{current_plan.name}-{node_index}",
                "type": "node"
            }
            
            self.motion_constructor.prepare_operation(
                MotionOperationMode.SAVE,
                tasks,
                context
            )
            
            self.command_hub.get_current_position()
            return True
        except Exception:
            return False
    
    def save_plan_trajectory(self) -> bool:
        """
        保存整个方案的轨迹
        
        流程：
        1. 获取方案所有任务
        2. 准备保存操作
        3. 查询当前位置
        4. MessageResponseService 自动完成保存
        
        Returns:
            True: 准备成功
            False: 准备失败
        """
        try:
            tasks = self._get_plan_tasks()
            if not tasks:
                return False
            
            current_plan = self.domain_service.get_current_plan()
            if not current_plan:
                return False
            
            context = {
                "filename": current_plan.name,
                "type": "plan"
            }
            
            self.motion_constructor.prepare_operation(
                MotionOperationMode.SAVE,
                tasks,
                context
            )
            
            self.command_hub.get_current_position()
            return True
        except Exception:
            return False
    
    # ========== 预览轨迹功能 ==========
    
    def preview_node_trajectory(self, node_index: int) -> bool:
        """
        预览单个节点的轨迹曲线
        
        流程：
        1. 获取节点任务
        2. 准备预览操作
        3. 查询当前位置
        4. MessageResponseService 自动发射预览信号
        
        Args:
            node_index: 节点索引
            
        Returns:
            True: 准备成功
            False: 准备失败
        """
        try:
            tasks = self._get_node_tasks(node_index)
            if not tasks:
                return False
            
            context = {
                "type": "node",
                "node_index": node_index
            }
            
            self.motion_constructor.prepare_operation(
                MotionOperationMode.PREVIEW,
                tasks,
                context
            )
            
            self.command_hub.get_current_position()
            return True
        except Exception:
            return False
    
    def preview_plan_trajectory(self) -> bool:
        """
        预览整个方案的轨迹曲线
        
        流程：
        1. 获取方案所有任务
        2. 准备预览操作
        3. 查询当前位置
        4. MessageResponseService 自动发射预览信号
        
        Returns:
            True: 准备成功
            False: 准备失败
        """
        try:
            tasks = self._get_plan_tasks()
            if not tasks:
                return False
            
            context = {"type": "plan"}
            
            self.motion_constructor.prepare_operation(
                MotionOperationMode.PREVIEW,
                tasks,
                context
            )
            
            self.command_hub.get_current_position()
            return True
        except Exception:
            return False
    
    # ========== 辅助方法 ==========
    
    def _get_node_tasks(self, node_index: int) -> List[Dict]:
        """
        获取节点的任务列表（复用逻辑）
        
        Args:
            node_index: 节点索引
            
        Returns:
            任务列表
        """
        point = self.get_single_point(node_index)
        if not point:
            return []
        return self._parse_point_to_tasks(point)
    
    def _get_plan_tasks(self) -> List[Dict]:
        """
        获取方案的所有任务列表（复用逻辑）
        
        Returns:
            任务列表
        """
        points = self.domain_service.get_all_points()
        if not points:
            return []
        
        all_tasks = []
        for point in points:
            tasks = self._parse_point_to_tasks(point)
            all_tasks.extend(tasks)
        
        return all_tasks
    
    # ========== 加载本地轨迹功能 ==========
    
    def load_local_trajectory(self, file_path: str) -> bool:
        """
        从 plans 目录加载轨迹文件，作为示教节点添加
        
        Args:
            file_path: 轨迹文件的完整路径或文件名
        
        Returns:
            True: 加载成功
            False: 加载失败
        """
        try:
            # 1. 提取文件名（不带路径和扩展名）
            file_stem = Path(file_path).stem
            
            # 2. 从 TrajectoryRepository 加载
            trajectory_data = self.trajectory_repository.load_trajectory(file_stem)
            
            # 3. 验证数据
            if not self._validate_trajectory_data(trajectory_data):
                return False
            
            # 4. 创建示教节点数据（复用示教模式的格式）
            point_data = {
                "mode": f"示教-{file_stem}",
                "joint1": 0.0,
                "joint2": 0.0,
                "joint3": 0.0,
                "joint4": 0.0,
                "joint5": 0.0,
                "joint6": 0.0,
                "frequency": 0.01,
                "curve_type": "S曲线",
                "gripper_command": "00: 不进行任何操作",
                "gripper_param": 0.0,
                "note": f"本地轨迹，共{len(trajectory_data)}个点",
                "teach_record_name": file_stem,
                "teach_data": trajectory_data,
                "source": "local_trajectory"  # 标记来源
            }
            
            # 5. 添加节点
            self.domain_service.add_point(point_data)
            self._save_data()  # 持久化保存
            self.point_list_changed.emit()
            return True
            
        except Exception:
            return False
    
    def get_local_trajectory_files(self) -> List[str]:
        """
        获取所有可用的本地轨迹文件
        
        Returns:
            文件名列表（不带扩展名）
        """
        return self.trajectory_repository.list_trajectory_files()
    
    def _validate_trajectory_data(self, data: Any) -> bool:
        """
        验证轨迹数据格式
        
        要求：
        - 列表类型
        - 至少2个点
        - 每个点是长度为6的列表
        - 所有值都是数字
        
        Args:
            data: 待验证的数据
        
        Returns:
            True: 数据有效
            False: 数据无效
        """
        if not isinstance(data, list) or len(data) < 2:
            return False
        
        for point in data:
            if not isinstance(point, list) or len(point) != 6:
                return False
            if not all(isinstance(x, (int, float)) for x in point):
                return False
        
        return True

