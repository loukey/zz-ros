"""
服务注册配置 - Shared Config
注册所有应用服务到DI容器 - 简化版
"""
from .di_container import DIContainer, get_container, resolve
from application import SerialApplicationService
from presentation import SerialViewModel, MainViewModel, DisplayViewModel


def register_infrastructure_services(container: DIContainer) -> None:
    """注册Infrastructure层服务"""
    # Infrastructure层的服务通常是无状态的工具类，可以使用单例
    # 由于它们没有复杂的依赖关系，暂时不需要在DI中注册
    # 如果需要，可以在这里添加PortScanner, SerialAdapter等
    pass


def register_application_services(container: DIContainer) -> None:
    """注册Application层服务"""
    container.register_singleton(SerialApplicationService)


def register_presentation_services(container: DIContainer) -> None:
    """注册Presentation层服务"""
    
    container.register_singleton(DisplayViewModel)
    container.register_singleton(SerialViewModel)
    container.register_singleton(MainViewModel)



def configure_services() -> DIContainer:
    """配置所有服务"""
    container = get_container()
    
    # 按层次注册服务
    register_infrastructure_services(container)
    register_application_services(container)
    register_presentation_services(container)
    
    return container


def get_main_view_model() -> MainViewModel:
    """便捷方法：获取主视图模型"""
    return resolve(MainViewModel)


def get_serial_service() -> SerialApplicationService:
    """便捷方法：获取串口服务"""
    return resolve(SerialApplicationService)


def get_serial_view_model() -> SerialViewModel:
    """便捷方法：获取串口视图模型"""
    return resolve(SerialViewModel)


def get_display_view_model() -> DisplayViewModel:
    """便捷方法：获取显示视图模型"""
    return resolve(DisplayViewModel)
