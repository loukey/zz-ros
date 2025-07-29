"""
Shared Configuration Module
提供依赖注入和配置管理功能 - 简化版
"""

# 导出依赖注入功能
from .di_container import DIContainer, get_container, resolve
from .service_registry import (
    configure_services, 
    get_main_view_model, 
    get_serial_service, 
    get_serial_view_model,
    get_display_view_model
)

__all__ = [
    # DI Container
    "DIContainer",
    "get_container", 
    "resolve",
    
    # Service Registry
    "configure_services",
    "get_main_view_model",
    "get_serial_service", 
    "get_serial_view_model",
    "get_display_view_model"
] 