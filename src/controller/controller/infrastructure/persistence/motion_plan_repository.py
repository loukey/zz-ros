"""
运动方案持久化仓库
"""
import json
from pathlib import Path
from typing import List, Dict


class MotionPlanRepository:
    """
    运动方案持久化仓库
    
    职责：
    1. 保存/加载方案到JSON文件
    2. 管理文件系统操作
    """
    
    def __init__(self, file_path: str = "./motion_planning_plans.json"):
        self.file_path = Path(file_path)
    
    def save(self, plans_data: List[Dict], current_index: int):
        """保存到文件"""
        data = {
            "current_index": current_index,
            "plans": plans_data
        }
        try:
            with open(self.file_path, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
        except Exception as e:
            pass
    
    def load(self) -> tuple[List[Dict], int]:
        """从文件加载
        
        Returns:
            (plans_data, current_index)
        """
        if not self.file_path.exists():
            # 返回默认方案
            return [{"name": "默认方案", "points": []}], 0
        
        try:
            with open(self.file_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            plans = data.get("plans", [])
            current_index = data.get("current_index", 0)
            
            # 如果没有方案，创建默认方案
            if not plans:
                plans = [{"name": "默认方案", "points": []}]
                current_index = 0
            
            return plans, current_index
            
        except Exception as e:
            return [{"name": "默认方案", "points": []}], 0

