"""
领域实体基类 - 提供实体的基础功能
"""
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, Optional, List
import uuid


@dataclass
class BaseEntity(ABC):
    """
    领域实体基类
    提供ID、创建时间、修改时间等基础功能
    """
    id: str = field(default_factory=lambda: str(uuid.uuid4()))
    created_at: datetime = field(default_factory=datetime.now)
    updated_at: Optional[datetime] = field(default=None)
    _domain_events: List = field(default_factory=list, init=False)
    
    def __post_init__(self):
        """实体初始化后调用"""
        if self.updated_at is None:
            self.updated_at = self.created_at
    
    def update_timestamp(self):
        """更新修改时间"""
        self.updated_at = datetime.now()
    
    def __eq__(self, other: Any) -> bool:
        """根据ID比较实体"""
        if not isinstance(other, BaseEntity):
            return False
        return self.id == other.id
    
    def __hash__(self) -> int:
        """根据ID计算哈希值"""
        return hash(self.id)
    
    def add_domain_event(self, event: Any) -> None:
        """添加领域事件"""
        self._domain_events.append(event)
    
    def get_domain_events(self) -> List[Any]:
        """获取领域事件"""
        return self._domain_events.copy()
    
    def clear_domain_events(self) -> None:
        """清除领域事件"""
        self._domain_events.clear()