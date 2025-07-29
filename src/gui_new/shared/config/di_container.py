"""
依赖注入容器 - Shared Config
简洁的DI容器实现，只支持单例和瞬时服务
"""
from typing import Dict, Type, Any, Optional, get_type_hints
import inspect


class ServiceLifetime:
    """服务生命周期"""
    SINGLETON = "singleton"
    TRANSIENT = "transient"


class ServiceDescriptor:
    """服务描述符"""
    
    def __init__(self, service_type: Type, implementation_type: Type = None, 
                 lifetime: str = ServiceLifetime.SINGLETON):
        self.service_type = service_type
        self.implementation_type = implementation_type or service_type
        self.lifetime = lifetime


class DIContainer:
    """依赖注入容器"""
    
    def __init__(self):
        self._services: Dict[Type, ServiceDescriptor] = {}
        self._instances: Dict[Type, Any] = {}
    
    def register_singleton(self, service_type: Type, implementation_type: Type = None) -> 'DIContainer':
        """注册单例服务"""
        self._services[service_type] = ServiceDescriptor(
            service_type, implementation_type, lifetime=ServiceLifetime.SINGLETON
        )
        return self
    
    def register_transient(self, service_type: Type, implementation_type: Type = None) -> 'DIContainer':
        """注册瞬时服务"""
        self._services[service_type] = ServiceDescriptor(
            service_type, implementation_type, lifetime=ServiceLifetime.TRANSIENT
        )
        return self
    
    def get_service(self, service_type: Type) -> Any:
        """获取服务实例"""
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
        """使用构造函数创建实例"""
        try:
            # 使用get_type_hints来正确解析字符串类型注解
            type_hints = get_type_hints(implementation_type.__init__)
        except Exception:
            # 如果解析失败，回退到inspect方式
            type_hints = {}
            sig = inspect.signature(implementation_type.__init__)
            for param_name, param in sig.parameters.items():
                if param_name != 'self' and param.annotation != inspect.Parameter.empty:
                    type_hints[param_name] = param.annotation
        
        kwargs = {}
        
        for param_name, param_type in type_hints.items():
            if param_name == 'self':
                continue
            
            # 跳过非类型参数（如parent=None）
            if param_name == 'parent':
                continue
                
            # 递归解析依赖
            kwargs[param_name] = self.get_service(param_type)
        
        return implementation_type(**kwargs)
    
    def is_registered(self, service_type: Type) -> bool:
        """检查服务是否已注册"""
        return service_type in self._services
    
    def clear(self) -> None:
        """清空容器"""
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
    """获取全局DI容器"""
    global _container
    if _container is None:
        _container = DIContainer()
    return _container


def resolve(service_type: Type) -> Any:
    """便捷方法：解析服务"""
    return get_container().get_service(service_type) 