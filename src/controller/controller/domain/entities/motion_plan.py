"""
运动规划方案实体
"""
from dataclasses import dataclass, field
from typing import List


@dataclass
class MotionPlan:
    """运动规划方案实体。
    
    Attributes:
        name (str): 方案名称。
        points (List[dict]): 节点列表。
    """
    
    name: str  # 方案名称
    points: List[dict] = field(default_factory=list)  # 节点列表
    
    def add_point(self, point_data: dict):
        """添加节点。
        
        Args:
            point_data (dict): 节点数据。
        """
        self.points.append(point_data)
    
    def remove_point(self, index: int):
        """删除节点。
        
        Args:
            index (int): 节点索引。
        """
        if 0 <= index < len(self.points):
            self.points.pop(index)
    
    def move_point_up(self, index: int) -> bool:
        """上移节点。
        
        Args:
            index (int): 节点索引。
            
        Returns:
            bool: 是否成功移动。
        """
        if index > 0:
            self.points[index], self.points[index-1] = \
                self.points[index-1], self.points[index]
            return True
        return False
    
    def move_point_down(self, index: int) -> bool:
        """下移节点。
        
        Args:
            index (int): 节点索引。
            
        Returns:
            bool: 是否成功移动。
        """
        if index < len(self.points) - 1:
            self.points[index], self.points[index+1] = \
                self.points[index+1], self.points[index]
            return True
        return False
    
    def update_point(self, index: int, point_data: dict):
        """更新节点。
        
        Args:
            index (int): 节点索引。
            point_data (dict): 新的节点数据。
        """
        if 0 <= index < len(self.points):
            self.points[index] = point_data

