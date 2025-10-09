"""
示教记录领域服务
"""
from typing import Dict, List, Optional
from PyQt5.QtCore import QObject, QTimer, pyqtSignal


class TeachRecordDomainService(QObject):
    """
    示教记录领域服务 - Domain层
    
    职责：
    1. 管理示教记录数据（内存中）
    2. 控制记录的开始/停止
    3. 提供记录的增删改查操作
    4. 生成记录名称
    """
    
    # 记录状态变化信号
    recording_state_changed = pyqtSignal(bool)  # 参数：是否正在记录
    record_added = pyqtSignal(str)  # 参数：新增的记录名称
    record_deleted = pyqtSignal(str)  # 参数：删除的记录名称
    
    def __init__(self):
        super().__init__()
        
        # 记录数据 {记录名: 角度列表}
        self._records: Dict[str, List[List[float]]] = {}
        
        # 当前记录状态
        self._is_recording = False
        self._current_record_angles: List[List[float]] = []
        self._record_counter = 0
        
        # 记录定时器（10ms间隔，100Hz采样率）
        self._record_timer = QTimer()
        self._record_timer.setInterval(10)
        self._record_timer.timeout.connect(self._on_record_timer_timeout)
    
    # ════════════════════════════════════════════════════════
    # 记录控制方法
    # ════════════════════════════════════════════════════════
    
    def start_recording(self):
        """开始记录"""
        if not self._is_recording:
            self._is_recording = True
            self._current_record_angles = []
            self._record_counter = 0
            self._record_timer.start()
            self.recording_state_changed.emit(True)
    
    def stop_recording(self) -> Optional[str]:
        """
        停止记录并保存
        
        返回:
            新生成的记录名称，如果没有数据则返回 None
        """
        if self._is_recording:
            self._is_recording = False
            self._record_timer.stop()
            self.recording_state_changed.emit(False)
            
            # 保存当前记录
            if self._current_record_angles:
                record_name = self._generate_record_name()
                self._records[record_name] = self._current_record_angles.copy()
                self.record_added.emit(record_name)
                return record_name
            else:
                return None
        return None
    
    def is_recording(self) -> bool:
        """查询是否正在记录"""
        return self._is_recording
    
    def add_angle_to_current_record(self, angles: List[float]):
        """
        添加角度到当前记录
        
        参数:
            angles: 6维关节角度
        """
        if self._is_recording and len(angles) == 6:
            self._current_record_angles.append(angles.copy())
            self._record_counter += 1
    
    def _on_record_timer_timeout(self):
        """
        记录定时器超时回调
        
        这个方法会在 ViewModel 中被连接到实际的数据源
        """
        pass
    
    def get_record_timer(self) -> QTimer:
        """获取记录定时器（用于外部连接）"""
        return self._record_timer
    
    # ════════════════════════════════════════════════════════
    # 记录数据管理方法
    # ════════════════════════════════════════════════════════
    
    def get_all_records(self) -> Dict[str, List[List[float]]]:
        """获取所有记录"""
        return self._records.copy()
    
    def get_record_names(self) -> List[str]:
        """获取所有记录名称"""
        return list(self._records.keys())
    
    def get_record(self, name: str) -> Optional[List[List[float]]]:
        """
        获取指定记录
        
        参数:
            name: 记录名称
            
        返回:
            角度列表，如果不存在返回 None
        """
        return self._records.get(name)
    
    def delete_record(self, name: str) -> bool:
        """
        删除指定记录
        
        参数:
            name: 记录名称
            
        返回:
            是否删除成功
        """
        if name in self._records:
            del self._records[name]
            self.record_deleted.emit(name)
            return True
        return False
    
    def reverse_record(self, name: str) -> Optional[str]:
        """
        反转指定记录并另存为新记录
        
        参数:
            name: 原记录名称
            
        返回:
            新记录名称，如果失败返回 None
        """
        if name not in self._records:
            return None
        
        try:
            # 反转角度序列
            original_list = self._records[name]
            reversed_list = list(reversed(original_list))
            
            # 生成新名称：原名-反转（如已存在则追加序号）
            base_name = f"{name}-反转"
            new_name = base_name
            suffix = 2
            while new_name in self._records:
                new_name = f"{base_name}-{suffix}"
                suffix += 1
            
            # 保存新记录
            self._records[new_name] = reversed_list
            self.record_added.emit(new_name)
            return new_name
        except Exception as e:
            return None
    
    def load_records(self, records: Dict[str, List[List[float]]]):
        """
        加载记录数据（从Repository）
        
        参数:
            records: 记录数据字典
        """
        self._records = records.copy()
    
    def clear_all_records(self):
        """清空所有记录"""
        self._records.clear()
    
    # ════════════════════════════════════════════════════════
    # 辅助方法
    # ════════════════════════════════════════════════════════
    
    def _generate_record_name(self) -> str:
        """
        生成唯一的记录名称
        
        返回:
            记录名称（如 record1, record2...）
        """
        base_name = "record"
        counter = len(self._records) + 1
        name = f"{base_name}{counter}"
        
        # 确保名称唯一
        while name in self._records:
            counter += 1
            name = f"{base_name}{counter}"
        
        return name

