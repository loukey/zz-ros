"""
记录数据持久化仓库
"""
import json
import os
from typing import Dict, List, Optional


class RecordRepository:
    """
    示教记录持久化仓库 - Infrastructure层
    
    职责：
    - 从JSON文件加载记录数据
    - 保存记录数据到JSON文件
    - 管理记录文件的读写
    """
    
    def __init__(self, file_path: str = "teach_record.json"):
        """
        初始化记录仓库
        
        参数:
            file_path: 记录文件路径
        """
        self.file_path = file_path
    
    def load_records(self) -> Dict[str, List[List[float]]]:
        """
        从文件加载记录数据
        
        返回:
            记录数据字典 {记录名: 角度列表}
        """
        try:
            if os.path.exists(self.file_path):
                with open(self.file_path, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                return data
            else:
                return {}
        except Exception as e:
            return {}
    
    def save_records(self, records: Dict[str, List[List[float]]]) -> bool:
        """
        保存记录数据到文件
        
        参数:
            records: 记录数据字典
            
        返回:
            是否保存成功
        """
        try:
            with open(self.file_path, 'w', encoding='utf-8') as f:
                json.dump(records, f, ensure_ascii=False, indent=2)
            return True
        except Exception as e:
            return False
    
    def delete_file(self) -> bool:
        """
        删除记录文件
        
        返回:
            是否删除成功
        """
        try:
            if os.path.exists(self.file_path):
                os.remove(self.file_path)
                return True
            return False
        except Exception as e:
            return False

