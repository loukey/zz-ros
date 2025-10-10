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
                print(f"✅ 已加载 {len(data)} 个记录")
                return data
            else:
                print(f"ℹ️ 记录文件不存在，创建新文件: {self.file_path}")
                return {}
        except Exception as e:
            print(f"❌ 加载记录文件失败: {e}")
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
            print(f"✅ 记录已保存到文件: {self.file_path}")
            return True
        except Exception as e:
            print(f"❌ 保存记录文件失败: {e}")
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
                print(f"✅ 已删除记录文件: {self.file_path}")
                return True
            return False
        except Exception as e:
            print(f"❌ 删除记录文件失败: {e}")
            return False

