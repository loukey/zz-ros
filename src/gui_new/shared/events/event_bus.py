"""
事件总线 - 提供事件发布和订阅机制
"""
from typing import Dict, List, Callable, Any
from PyQt5.QtCore import QObject, pyqtSignal


class EventBus(QObject):
    """事件总线 - 解耦组件间通信"""
    
    # 通用事件信号
    event_published = pyqtSignal(str, object)  # event_name, event_data
    
    def __init__(self):
        super().__init__()
        self._subscribers: Dict[str, List[Callable]] = {}
    
    def subscribe(self, event_name: str, handler: Callable[[Any], None]):
        """订阅事件"""
        if event_name not in self._subscribers:
            self._subscribers[event_name] = []
        self._subscribers[event_name].append(handler)
    
    def unsubscribe(self, event_name: str, handler: Callable[[Any], None]):
        """取消订阅事件"""
        if event_name in self._subscribers:
            try:
                self._subscribers[event_name].remove(handler)
            except ValueError:
                pass
    
    def publish(self, event):
        """发布事件"""
        event_name = type(event).__name__
        
        # 发出PyQt信号
        self.event_published.emit(event_name, event)
        
        # 调用订阅者
        if event_name in self._subscribers:
            for handler in self._subscribers[event_name]:
                try:
                    handler(event)
                except Exception as e:
                    print(f"事件处理器错误 {event_name}: {str(e)}")
    
    def clear_subscribers(self):
        """清除所有订阅者"""
        self._subscribers.clear() 