"""
依赖注入容器 - Shared Config
简洁的DI容器实现，只支持单例和瞬时服务
"""
from typing import Dict, Type, Any, Optional, get_type_hints
import inspect


class ServiceLifetime:
    """服务生命周期枚举。
    
    Attributes:
        SINGLETON: 单例模式，整个容器生命周期内只创建一个实例。
        TRANSIENT: 瞬时模式，每次请求都创建新的实例。
    """
    SINGLETON = "singleton"
    TRANSIENT = "transient"


class ServiceDescriptor:
    """服务描述符。
    
    用于存储服务的注册信息。
    
    Attributes:
        service_type (Type): 服务类型（接口或基类）。
        implementation_type (Type): 实现类型（具体类）。
        lifetime (str): 服务生命周期。
    """
    
    def __init__(self, service_type: Type, implementation_type: Type = None, 
                 lifetime: str = ServiceLifetime.SINGLETON):
        """初始化服务描述符。
        
        Args:
            service_type (Type): 服务类型。
            implementation_type (Type, optional): 实现类型. Defaults to None.
            lifetime (str, optional): 生命周期. Defaults to ServiceLifetime.SINGLETON.
        """
        self.service_type = service_type
        self.implementation_type = implementation_type or service_type
        self.lifetime = lifetime


class DIContainer:
    """依赖注入容器。
    
    简洁的 DI 容器实现，只支持单例和瞬时服务。
    
    Attributes:
        _services (Dict[Type, ServiceDescriptor]): 已注册的服务描述符字典。
        _instances (Dict[Type, Any]): 已创建的单例实例字典。
    """
    
    def __init__(self):
        """初始化容器。"""
        self._services: Dict[Type, ServiceDescriptor] = {}
        self._instances: Dict[Type, Any] = {}
    
    def register_singleton(self, service_type: Type, implementation_type: Type = None) -> 'DIContainer':
        """注册单例服务。
        
        Args:
            service_type (Type): 服务类型。
            implementation_type (Type, optional): 实现类型. Defaults to None.
            
        Returns:
            DIContainer: 容器实例本身（支持链式调用）。
        """
        self._services[service_type] = ServiceDescriptor(
            service_type, implementation_type, lifetime=ServiceLifetime.SINGLETON
        )
        return self
    
    def register_transient(self, service_type: Type, implementation_type: Type = None) -> 'DIContainer':
        """注册瞬时服务。
        
        Args:
            service_type (Type): 服务类型。
            implementation_type (Type, optional): 实现类型. Defaults to None.
            
        Returns:
            DIContainer: 容器实例本身（支持链式调用）。
        """
        self._services[service_type] = ServiceDescriptor(
            service_type, implementation_type, lifetime=ServiceLifetime.TRANSIENT
        )
        return self
    
    def get_service(self, service_type: Type) -> Any:
        """获取服务实例。
        
        Args:
            service_type (Type): 服务类型。
            
        Returns:
            Any: 服务实例。
            
        Raises:
            ValueError: 如果服务未注册。
        """
        if service_type not in self._services:
            raise ValueError(f"服务 {service_type.__name__} 未注册")
        
        descriptor = self._services[service_type]
        
        # 如果是单例且已创建，直接返回
        if descriptor.lifetime == ServiceLifetime.SINGLETON and service_type in self._instances:
            return self._instances[service_type]
        
        # 创建新实例
        instance = self._create_with_constructor(descriptor.implementation_type)
        
        # 如果是单例，缓存实例
        if descriptor.lifetime == ServiceLifetime.SINGLETON:
            self._instances[service_type] = instance
        
        return instance
    
    def _create_with_constructor(self, implementation_type: Type) -> Any:
        """使用构造函数创建实例，自动解析依赖。
        
        Args:
            implementation_type (Type): 实现类型。
            
        Returns:
            Any: 创建的实例。
        """
        # 使用inspect.signature获取构造函数参数，避免类级别属性干扰
        sig = inspect.signature(implementation_type.__init__)
        kwargs = {}
        
        for param_name, param in sig.parameters.items():
            if param_name == 'self':
                continue
            
            # 跳过非类型参数（如parent=None）
            if param_name == 'parent':
                continue
            
            # 跳过有默认值的参数（如 file_path: str = "xxx"）
            if param.default != inspect.Parameter.empty:
                continue
            
            # 检查参数是否有类型注解且不为空
            if param.annotation == inspect.Parameter.empty:
                continue
            
            param_type = param.annotation
            
            # 解析字符串类型注解
            if isinstance(param_type, str):
                try:
                    # 尝试解析字符串类型注解
                    type_hints = get_type_hints(implementation_type.__init__)
                    param_type = type_hints.get(param_name, param_type)
                except Exception:
                    # 如果解析失败，跳过这个参数
                    continue
            
            # 确保param_type是一个类型而不是其他对象（如pyqtSignal）
            if not isinstance(param_type, type):
                continue
            
            # 跳过基本类型（str, int, float, bool等）
            if param_type in (str, int, float, bool, list, dict, tuple, set):
                continue
                
            # 递归解析依赖
            kwargs[param_name] = self.get_service(param_type)
        
        return implementation_type(**kwargs)
    
    def is_registered(self, service_type: Type) -> bool:
        """检查服务是否已注册。
        
        Args:
            service_type (Type): 服务类型。
            
        Returns:
            bool: 是否已注册。
        """
        return service_type in self._services
    
    def clear(self) -> None:
        """清空容器。
        
        清理所有注册的服务和已创建的单例实例。
        """
        # 清理单例实例
        for instance in self._instances.values():
            if hasattr(instance, 'cleanup'):
                try:
                    instance.cleanup()
                except Exception:
                    pass  # 忽略清理错误
        
        self._services.clear()
        self._instances.clear()


# 全局DI容器实例
_container: Optional[DIContainer] = None


def get_container() -> DIContainer:
    """获取全局DI容器。
    
    如果容器不存在，则创建一个新实例。
    
    Returns:
        DIContainer: 全局容器实例。
    """
    global _container
    if _container is None:
        _container = DIContainer()
    return _container


def resolve(service_type: Type) -> Any:
    """便捷方法：解析服务。
    
    从全局容器中获取指定类型的服务实例。
    
    Args:
        service_type (Type): 服务类型。
        
    Returns:
        Any: 服务实例。
    """
    return get_container().get_service(service_type) 