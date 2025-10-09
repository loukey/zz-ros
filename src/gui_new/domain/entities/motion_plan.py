"""
运动规划方案实体
"""
from dataclasses import dataclass, field
from typing import List


@dataclass
class MotionPlan:
    """运动规划方案实体"""
    
    name: str  # 方案名称
    points: List[dict] = field(default_factory=list)  # 节点列表
    
    def add_point(self, point_data: dict):
        """添加节点"""
        self.points.append(point_data)
    
    def remove_point(self, index: int):
        """删除节点"""
        if 0 <= index < len(self.points):
            self.points.pop(index)
    
    def move_point_up(self, index: int) -> bool:
        """上移节点"""
        if index > 0:
            self.points[index], self.points[index-1] = \
                self.points[index-1], self.points[index]
            return True
        return False
    
    def move_point_down(self, index: int) -> bool:
        """下移节点"""
        if index < len(self.points) - 1:
            self.points[index], self.points[index+1] = \
                self.points[index+1], self.points[index]
            return True
        return False
    
    def update_point(self, index: int, point_data: dict):
        """更新节点"""
        if 0 <= index < len(self.points):
            self.points[index] = point_data

