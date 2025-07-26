"""
依赖注入容器 - 管理整个应用的依赖关系
"""
from typing import Dict, Any, TypeVar, Type, Optional, Callable
from PyQt5.QtCore import QObject


T = TypeVar('T')


class DIContainer:
    """依赖注入容器"""
    
    def __init__(self):
        self._services: Dict[str, Any] = {}
        self._singletons: Dict[str, Any] = {}
        self._factories: Dict[str, Callable] = {}
    
    def register_singleton(self, interface: Type[T], implementation: T) -> None:
        """注册单例服务"""
        key = self._get_key(interface)
        self._singletons[key] = implementation
    
    def register_transient(self, interface: Type[T], factory: Callable[[], T]) -> None:
        """注册瞬态服务"""
        key = self._get_key(interface)
        self._factories[key] = factory
    
    def register_instance(self, interface: Type[T], instance: T) -> None:
        """注册实例"""
        key = self._get_key(interface)
        self._services[key] = instance
    
    def resolve(self, interface: Type[T]) -> T:
        """解析服务"""
        key = self._get_key(interface)
        
        # 先检查单例
        if key in self._singletons:
            return self._singletons[key]
        
        # 再检查实例
        if key in self._services:
            return self._services[key]
        
        # 最后检查工厂
        if key in self._factories:
            return self._factories[key]()
        
        raise ValueError(f"服务未注册: {interface}")
    
    def _get_key(self, interface: Type) -> str:
        """获取服务键名"""
        return f"{interface.__module__}.{interface.__name__}"
    
    def clear(self):
        """清除所有注册的服务"""
        self._services.clear()
        self._singletons.clear()
        self._factories.clear()


class ServiceLocator:
    """服务定位器 - 全局访问点"""
    
    _container: Optional[DIContainer] = None
    
    @classmethod
    def set_container(cls, container: DIContainer):
        """设置容器实例"""
        cls._container = container
    
    @classmethod
    def resolve(cls, interface: Type[T]) -> T:
        """解析服务"""
        if cls._container is None:
            raise ValueError("DIContainer 未初始化，请先调用 configure_services()")
        
        return cls._container.resolve(interface)
    
    @classmethod
    def clear(cls):
        """清除容器"""
        if cls._container:
            cls._container.clear()
        cls._container = None


def configure_services() -> DIContainer:
    """配置服务依赖"""
    from shared.events.event_bus import EventBus
    from domain.services.trajectory_planning_service import TrajectoryPlanningService
    from domain.services.kinematic_service import KinematicService  
    from domain.services.command_formatting_service import CommandFormattingService
    from application.services.robot_application_service import RobotApplicationService
    from shared.config.settings import AppSettings
    
    container = DIContainer()
    
    # 先设置全局服务定位器，以便其他服务初始化时可以使用
    ServiceLocator.set_container(container)
    
    # 注册事件总线（单例）
    event_bus = EventBus()
    container.register_singleton(EventBus, event_bus)
    
    # 注册领域服务（单例）
    container.register_singleton(TrajectoryPlanningService, TrajectoryPlanningService())
    container.register_singleton(KinematicService, KinematicService())
    container.register_singleton(CommandFormattingService, CommandFormattingService())
    
    # 注册配置服务（单例）
    container.register_singleton(AppSettings, AppSettings())
    
    # 注册应用服务（单例） - 这个必须在其他依赖服务注册后
    container.register_singleton(RobotApplicationService, RobotApplicationService())
    
    return container 