"""
运动规划ViewModel
"""
from PyQt5.QtCore import QObject, pyqtSignal
from typing import List, Dict
from controller.application import MotionPlanningApplicationService
from .dynamics_view_model import DynamicsViewModel


class MotionPlanningViewModel(QObject):
    """运动规划 ViewModel。
    
    职责：
    1. 转发 UI 命令到 Application Service
    2. 转发 Application Service 信号到 UI 组件
    3. 通过 DynamicsViewModel 获取示教记录（避免重复注入）
    
    设计原则：
    - ViewModel 只依赖 Application Service，不直接访问 Domain 或 Infrastructure
    - 所有业务逻辑在 Application Service 中封装
    
    Attributes:
        plan_list_changed (pyqtSignal): 方案列表变更信号。
        current_plan_changed (pyqtSignal): 当前方案变更信号，携带索引。
        point_list_changed (pyqtSignal): 节点列表变更信号。
        current_position_received (pyqtSignal): 当前位置数据信号，携带角度列表（弧度）。
        trajectory_preview_signal (pyqtSignal): 轨迹预览信号，携带 (轨迹数据, 上下文)。
        app_service (MotionPlanningApplicationService): 运动规划应用服务。
        dynamics_vm (DynamicsViewModel): 动力学 ViewModel。
    """
    
    # 转发信号
    plan_list_changed = pyqtSignal()
    current_plan_changed = pyqtSignal(int)
    point_list_changed = pyqtSignal()
    current_position_received = pyqtSignal(list)  # 当前位置数据（弧度值）
    trajectory_preview_signal = pyqtSignal(dict, dict)  # 轨迹预览数据（轨迹数据，上下文）
    
    def __init__(
        self, 
        app_service: MotionPlanningApplicationService,
        dynamics_vm: DynamicsViewModel
    ):
        """初始化运动规划 ViewModel。
        
        Args:
            app_service (MotionPlanningApplicationService): 运动规划应用服务。
            dynamics_vm (DynamicsViewModel): 动力学 ViewModel。
        """
        super().__init__()
        self.app_service = app_service
        self.dynamics_vm = dynamics_vm
        
        # 连接Application Service的信号
        self.app_service.plan_list_changed.connect(self.plan_list_changed.emit)
        self.app_service.current_plan_changed.connect(self.current_plan_changed.emit)
        self.app_service.point_list_changed.connect(self.point_list_changed.emit)
        self.app_service.current_position_received.connect(self.current_position_received.emit)
        self.app_service.trajectory_preview_signal.connect(self.trajectory_preview_signal.emit)
    
    # ========== 方案操作 ==========
    
    def create_plan(self, name: str):
        """创建方案。
        
        Args:
            name (str): 方案名称。
        """
        self.app_service.create_plan(name)
    
    def delete_plan(self, index: int) -> bool:
        """删除方案。
        
        Args:
            index (int): 方案索引。
            
        Returns:
            bool: True=删除成功, False=删除失败（违反业务规则）。
        """
        return self.app_service.delete_plan(index)
    
    def switch_plan(self, index: int):
        """切换方案。
        
        Args:
            index (int): 方案索引。
        """
        self.app_service.switch_plan(index)
    
    def rename_plan(self, index: int, new_name: str):
        """重命名方案。
        
        Args:
            index (int): 方案索引。
            new_name (str): 新名称。
        """
        self.app_service.rename_plan(index, new_name)
    
    def get_plan_names(self) -> List[str]:
        """获取所有方案名称。
        
        Returns:
            List[str]: 方案名称列表。
        """
        return self.app_service.domain_service.get_plan_names()
    
    def get_current_plan_index(self) -> int:
        """获取当前方案索引。
        
        Returns:
            int: 当前索引。
        """
        return self.app_service.domain_service.get_current_index()
    
    def get_plan_count(self) -> int:
        """获取方案数量。
        
        Returns:
            int: 方案数量。
        """
        return self.app_service.domain_service.get_plan_count()
    
    # ========== 节点操作 ==========
    
    def add_point(self, point_data: dict):
        """添加节点。
        
        Args:
            point_data (dict): 节点数据。
        """
        self.app_service.add_point(point_data)
    
    def delete_point(self, index: int):
        """删除节点。
        
        Args:
            index (int): 节点索引。
        """
        self.app_service.delete_point(index)
    
    def move_point_up(self, index: int):
        """上移节点。
        
        Args:
            index (int): 节点索引。
        """
        self.app_service.move_point_up(index)
    
    def move_point_down(self, index: int):
        """下移节点。
        
        Args:
            index (int): 节点索引。
        """
        self.app_service.move_point_down(index)
    
    def update_point(self, index: int, point_data: dict):
        """更新节点。
        
        Args:
            index (int): 节点索引。
            point_data (dict): 节点数据。
        """
        self.app_service.update_point(index, point_data)
    
    def get_all_points(self) -> List[dict]:
        """获取当前方案的所有节点。
        
        Returns:
            List[dict]: 节点列表。
        """
        return self.app_service.domain_service.get_all_points()
    
    def get_single_point(self, index: int) -> dict:
        """获取单个节点数据。
        
        Args:
            index (int): 节点索引。
            
        Returns:
            dict: 节点数据。
        """
        return self.app_service.get_single_point(index)
    
    # ========== 示教记录操作（通过DynamicsViewModel）==========
    
    def get_teach_record_names(self) -> List[str]:
        """获取所有示教记录名称（委托给 DynamicsViewModel）。
        
        Returns:
            List[str]: 记录名称列表。
        """
        return self.dynamics_vm.get_record_names()
    
    def get_teach_record(self, name: str) -> List[List[float]]:
        """获取指定示教记录的数据（委托给 DynamicsViewModel）。
        
        Args:
            name (str): 记录名称。
            
        Returns:
            List[List[float]]: 记录数据。
        """
        angles_list = self.dynamics_vm.teach_record_service.get_record(name)
        return angles_list if angles_list else []
    
    # ========== 执行操作 ==========
    
    def execute_single_point(self, index: int):
        """执行单个节点。
        
        Args:
            index (int): 节点索引。
        """
        self.app_service.execute_single_point(index)
    
    def execute_motion_plan(self):
        """执行整个运动规划方案。"""
        self.app_service.execute_motion_plan()
    
    # ========== 获取位置功能 ==========
    
    def request_current_position(self):
        """请求获取当前位置（仅用于UI显示）。
        
        委托给 Application Service 处理。
        """
        self.app_service.request_current_position()
    
    # ========== 保存轨迹功能 ==========
    
    def save_node_trajectory(self, node_index: int) -> bool:
        """保存单个节点的轨迹。
        
        Args:
            node_index (int): 节点索引。
            
        Returns:
            bool: True=准备成功, False=准备失败。
        """
        return self.app_service.save_node_trajectory(node_index)
    
    def save_plan_trajectory(self) -> bool:
        """保存整个方案的轨迹。
        
        Returns:
            bool: True=准备成功, False=准备失败。
        """
        return self.app_service.save_plan_trajectory()
    
    # ========== 预览轨迹功能 ==========
    
    def preview_node_trajectory(self, node_index: int) -> bool:
        """预览单个节点的轨迹曲线。
        
        Args:
            node_index (int): 节点索引。
            
        Returns:
            bool: True=准备成功, False=准备失败。
        """
        return self.app_service.preview_node_trajectory(node_index)
    
    def preview_plan_trajectory(self) -> bool:
        """预览整个方案的轨迹曲线。
        
        Returns:
            bool: True=准备成功, False=准备失败。
        """
        return self.app_service.preview_plan_trajectory()
    
    # ========== 加载本地轨迹功能 ==========
    
    def load_local_trajectory(self, file_path: str) -> bool:
        """从 plans 目录加载轨迹文件。
        
        Args:
            file_path (str): 轨迹文件的完整路径或文件名。
        
        Returns:
            bool: True=加载成功, False=加载失败。
        """
        return self.app_service.load_local_trajectory(file_path)
    
    def get_local_trajectory_files(self) -> List[str]:
        """获取所有可用的本地轨迹文件。
        
        Returns:
            List[str]: 文件名列表（不带扩展名）。
        """
        return self.app_service.get_local_trajectory_files()

