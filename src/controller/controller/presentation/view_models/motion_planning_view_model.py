"""
运动规划ViewModel
"""
from PyQt5.QtCore import QObject, pyqtSignal
from typing import List, Dict
from controller.application import MotionPlanningApplicationService
from .dynamics_view_model import DynamicsViewModel


class MotionPlanningViewModel(QObject):
    """
    运动规划ViewModel
    
    职责：
    1. 转发UI命令到Application Service
    2. 转发Application Service信号到UI组件
    3. 通过DynamicsViewModel获取示教记录（避免重复注入）
    
    设计原则：
    - ViewModel只依赖Application Service，不直接访问Domain或Infrastructure
    - 所有业务逻辑在Application Service中封装
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
        self.app_service.create_plan(name)
    
    def delete_plan(self, index: int) -> bool:
        """
        删除方案
        
        Returns:
            True=删除成功, False=删除失败（违反业务规则）
        """
        return self.app_service.delete_plan(index)
    
    def switch_plan(self, index: int):
        self.app_service.switch_plan(index)
    
    def rename_plan(self, index: int, new_name: str):
        self.app_service.rename_plan(index, new_name)
    
    def get_plan_names(self) -> List[str]:
        return self.app_service.domain_service.get_plan_names()
    
    def get_current_plan_index(self) -> int:
        return self.app_service.domain_service.get_current_index()
    
    def get_plan_count(self) -> int:
        return self.app_service.domain_service.get_plan_count()
    
    # ========== 节点操作 ==========
    
    def add_point(self, point_data: dict):
        self.app_service.add_point(point_data)
    
    def delete_point(self, index: int):
        self.app_service.delete_point(index)
    
    def move_point_up(self, index: int):
        self.app_service.move_point_up(index)
    
    def move_point_down(self, index: int):
        self.app_service.move_point_down(index)
    
    def update_point(self, index: int, point_data: dict):
        self.app_service.update_point(index, point_data)
    
    def get_all_points(self) -> List[dict]:
        return self.app_service.domain_service.get_all_points()
    
    def get_single_point(self, index: int) -> dict:
        """获取单个节点数据"""
        return self.app_service.get_single_point(index)
    
    # ========== 示教记录操作（通过DynamicsViewModel）==========
    
    def get_teach_record_names(self) -> List[str]:
        """获取所有示教记录名称（委托给DynamicsViewModel）"""
        return self.dynamics_vm.get_record_names()
    
    def get_teach_record(self, name: str) -> List[List[float]]:
        """获取指定示教记录的数据（委托给DynamicsViewModel）"""
        angles_list = self.dynamics_vm.teach_record_service.get_record(name)
        return angles_list if angles_list else []
    
    # ========== 执行操作 ==========
    
    def execute_single_point(self, index: int):
        """执行单个节点"""
        self.app_service.execute_single_point(index)
    
    def execute_motion_plan(self):
        """执行整个运动规划方案"""
        self.app_service.execute_motion_plan()
    
    # ========== 获取位置功能 ==========
    
    def request_current_position(self):
        """
        请求获取当前位置（仅用于UI显示）
        
        委托给Application Service处理
        """
        self.app_service.request_current_position()
    
    # ========== 保存轨迹功能 ==========
    
    def save_node_trajectory(self, node_index: int) -> bool:
        """
        保存单个节点的轨迹
        
        Args:
            node_index: 节点索引
            
        Returns:
            True: 准备成功
            False: 准备失败
        """
        return self.app_service.save_node_trajectory(node_index)
    
    def save_plan_trajectory(self) -> bool:
        """
        保存整个方案的轨迹
        
        Returns:
            True: 准备成功
            False: 准备失败
        """
        return self.app_service.save_plan_trajectory()
    
    # ========== 预览轨迹功能 ==========
    
    def preview_node_trajectory(self, node_index: int) -> bool:
        """
        预览单个节点的轨迹曲线
        
        Args:
            node_index: 节点索引
            
        Returns:
            True: 准备成功
            False: 准备失败
        """
        return self.app_service.preview_node_trajectory(node_index)
    
    def preview_plan_trajectory(self) -> bool:
        """
        预览整个方案的轨迹曲线
        
        Returns:
            True: 准备成功
            False: 准备失败
        """
        return self.app_service.preview_plan_trajectory()

