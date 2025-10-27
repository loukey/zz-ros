"""
轨迹数据仓库 - Infrastructure层
"""
from pathlib import Path
import json
from typing import List
import numpy as np


class TrajectoryRepository:
    """
    轨迹数据仓库
    
    职责：
    1. 保存轨迹数据到文件系统
    2. 加载轨迹数据
    3. 管理存储目录
    """
    
    def __init__(self, plans_dir: str = "./plans"):
        self.plans_dir = Path(plans_dir)
        self.plans_dir.mkdir(exist_ok=True)
    
    def save_trajectory(
        self, 
        filename: str, 
        positions: List[List[float]]
    ) -> None:
        """
        保存轨迹数据
        
        Args:
            filename: 文件名（不含扩展名）
            positions: 轨迹点序列 [[q1,...,q6], ...]
        """
        clean_name = self._sanitize_filename(filename)
        filepath = self.plans_dir / f"{clean_name}.json"
        
        # 将numpy数组转换为Python列表
        positions_serializable = self._convert_to_serializable(positions)
        
        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(positions_serializable, f, indent=2, ensure_ascii=False)
    
    def load_trajectory(self, filename: str) -> List[List[float]]:
        """
        加载轨迹数据
        
        Args:
            filename: 文件名（不含扩展名）
            
        Returns:
            轨迹点序列
        """
        filepath = self.plans_dir / f"{filename}.json"
        with open(filepath, 'r', encoding='utf-8') as f:
            return json.load(f)
    
    def _sanitize_filename(self, name: str) -> str:
        """
        清理文件名中的非法字符
        
        Args:
            name: 原始文件名
            
        Returns:
            清理后的文件名
        """
        invalid_chars = r'<>:"/\|?*'
        for char in invalid_chars:
            name = name.replace(char, '_')
        return name
    
    def _convert_to_serializable(self, data):
        """
        递归地将numpy数组转换为Python列表
        
        Args:
            data: 可能包含numpy数组的数据
            
        Returns:
            可JSON序列化的数据
        """
        if isinstance(data, np.ndarray):
            return data.tolist()
        elif isinstance(data, list):
            return [self._convert_to_serializable(item) for item in data]
        elif isinstance(data, tuple):
            return tuple(self._convert_to_serializable(item) for item in data)
        elif isinstance(data, dict):
            return {key: self._convert_to_serializable(value) for key, value in data.items()}
        elif isinstance(data, (np.integer, np.floating)):
            return data.item()
        else:
            return data

