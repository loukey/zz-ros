"""
运动规划Domain服务
"""
from typing import List, Optional
from ...entities.motion_plan import MotionPlan


class MotionPlanningDomainService:
    """运动规划Domain服务。
    
    职责：
    1. 管理多个方案（内存）
    2. 维护当前激活的方案
    3. 提供方案和节点的增删改查
    4. 实施业务规则
    
    Attributes:
        _plans (List[MotionPlan]): 方案列表。
        _current_index (int): 当前激活方案的索引。
    """
    
    def __init__(self):
        """初始化服务。"""
        self._plans: List[MotionPlan] = []
        self._current_index: int = -1
    
    # ========== 数据初始化 ==========
    
    def initialize(self, plans: List[MotionPlan], current_index: int):
        """初始化数据（从Repository加载后调用）。
        
        Args:
            plans (List[MotionPlan]): 方案列表。
            current_index (int): 当前方案索引。
        """
        self._plans = plans
        self._current_index = current_index if plans else -1
    
    def get_all_plans(self) -> List[MotionPlan]:
        """获取所有方案（用于持久化）。
        
        Returns:
            List[MotionPlan]: 方案列表的副本。
        """
        return self._plans.copy()
    
    # ========== 方案管理 ==========
    
    def create_plan(self, name: str) -> int:
        """创建方案，返回索引。
        
        Args:
            name (str): 方案名称。
            
        Returns:
            int: 新创建方案的索引。
        """
        plan = MotionPlan(name=name)
        self._plans.append(plan)
        self._current_index = len(self._plans) - 1
        return self._current_index
    
    def get_plan_names(self) -> List[str]:
        """获取所有方案名称（用于下拉框）。
        
        Returns:
            List[str]: 方案名称列表。
        """
        return [plan.name for plan in self._plans]
    
    def get_current_index(self) -> int:
        """获取当前方案索引。
        
        Returns:
            int: 当前索引。
        """
        return self._current_index
    
    def set_current_index(self, index: int) -> bool:
        """切换方案。
        
        Args:
            index (int): 目标索引。
            
        Returns:
            bool: 是否切换成功。
        """
        if 0 <= index < len(self._plans):
            self._current_index = index
            return True
        return False
    
    def delete_plan(self, index: int) -> bool:
        """删除方案。
        
        业务规则：至少保留一个方案。
        
        Args:
            index (int): 方案索引。
            
        Returns:
            bool: True=删除成功, False=删除失败（违反业务规则或索引无效）。
        """
        # 业务规则：至少保留一个方案
        if len(self._plans) <= 1:
            return False
        
        if 0 <= index < len(self._plans):
            self._plans.pop(index)
            # 调整当前索引
            if self._current_index >= len(self._plans):
                self._current_index = max(0, len(self._plans) - 1)
            if len(self._plans) == 0:
                self._current_index = -1
            return True
        return False
    
    def can_delete_plan(self) -> bool:
        """检查是否可以删除方案。
        
        Returns:
            bool: True=可以删除, False=不能删除（只有一个方案）。
        """
        return len(self._plans) > 1
    
    def rename_plan(self, index: int, new_name: str) -> bool:
        """重命名方案。
        
        Args:
            index (int): 方案索引。
            new_name (str): 新名称。
            
        Returns:
            bool: 是否成功。
        """
        if 0 <= index < len(self._plans):
            self._plans[index].name = new_name
            return True
        return False
    
    def get_current_plan(self) -> Optional[MotionPlan]:
        """获取当前方案。
        
        Returns:
            Optional[MotionPlan]: 当前方案对象，无方案时返回 None。
        """
        if 0 <= self._current_index < len(self._plans):
            return self._plans[self._current_index]
        return None
    
    def get_plan_count(self) -> int:
        """获取方案数量。
        
        Returns:
            int: 方案数量。
        """
        return len(self._plans)
    
    # ========== 节点管理（针对当前方案）==========
    
    def add_point(self, point_data: dict):
        """添加节点到当前方案。
        
        Args:
            point_data (dict): 节点数据。
        """
        plan = self.get_current_plan()
        if plan:
            plan.add_point(point_data)
    
    def remove_point(self, index: int):
        """删除节点。
        
        Args:
            index (int): 节点索引。
        """
        plan = self.get_current_plan()
        if plan:
            plan.remove_point(index)
    
    def move_point_up(self, index: int) -> bool:
        """上移节点。
        
        Args:
            index (int): 节点索引。
            
        Returns:
            bool: 是否成功。
        """
        plan = self.get_current_plan()
        return plan.move_point_up(index) if plan else False
    
    def move_point_down(self, index: int) -> bool:
        """下移节点。
        
        Args:
            index (int): 节点索引。
            
        Returns:
            bool: 是否成功。
        """
        plan = self.get_current_plan()
        return plan.move_point_down(index) if plan else False
    
    def update_point(self, index: int, point_data: dict):
        """更新节点。
        
        Args:
            index (int): 节点索引。
            point_data (dict): 新数据。
        """
        plan = self.get_current_plan()
        if plan:
            plan.update_point(index, point_data)
    
    def get_all_points(self) -> List[dict]:
        """获取当前方案的所有节点。
        
        Returns:
            List[dict]: 节点列表的副本。
        """
        plan = self.get_current_plan()
        return plan.points.copy() if plan else []
    
    def insert_point(self, index: int, point_data: dict):
        """在指定位置插入节点。
        
        Args:
            index (int): 插入位置索引。
            point_data (dict): 节点数据。
        """
        plan = self.get_current_plan()
        if plan:
            plan.insert_point(index, point_data)
    
    def get_single_point(self, index: int) -> Optional[dict]:
        """获取单个节点数据。
        
        Args:
            index (int): 节点索引。
            
        Returns:
            Optional[dict]: 节点数据，无效索引返回 None。
        """
        plan = self.get_current_plan()
        if plan and 0 <= index < len(plan.points):
            return plan.points[index].copy()
        return None

