"""
服务注册配置 - Shared Config
注册所有应用服务到DI容器 - 简化版
"""
from .di_container import DIContainer, get_container, resolve
from controller.application import *
from controller.presentation import *
from controller.domain import *
from controller.infrastructure import *


def register_infrastructure_services(container: DIContainer) -> None:
    """注册 Infrastructure 层服务。
    
    Args:
        container (DIContainer): DI 容器实例。
    """
    # 记录数据持久化仓库
    container.register_singleton(RecordRepository)
    # 运动规划方案持久化仓库
    container.register_singleton(MotionPlanRepository)
    # 手眼标定配置仓库
    container.register_singleton(HandEyeCalibrationRepository)
    # 轨迹数据仓库
    container.register_singleton(TrajectoryRepository)

def register_domain_services(container: DIContainer) -> None:
    """注册 Domain 层服务。
    
    Args:
        container (DIContainer): DI 容器实例。
    """
    container.register_singleton(SerialDomainService)
    container.register_singleton(MessageEncoder)
    container.register_singleton(MessageDecoder)
    container.register_singleton(MessageDomainService)
    container.register_singleton(MotionRunner)
    # 轨迹平滑服务
    container.register_singleton(SmoothDomainService)
    # 运动学和轨迹规划服务
    container.register_singleton(KinematicDomainService)
    container.register_singleton(LinearMotionDomainService)  # 依赖 KinematicDomainService
    container.register_singleton(CurveMotionDomainService)  # 依赖 KinematicDomainService
    # S曲线算法
    container.register_singleton(SCurve)
    # 轨迹规划服务（依赖 SCurve, SmoothDomainService, LinearMotionDomainService, CurveMotionDomainService）
    container.register_singleton(TrajectoryPlanningService)
    # 运动构造器（依赖 MotionRunner, TrajectoryPlanningService）
    container.register_singleton(MotionConstructor)
    # 机械臂状态服务
    container.register_singleton(RobotStateDomainService)
    # 动力学服务
    container.register_singleton(DynamicDomainService)
    # 示教记录服务
    container.register_singleton(TeachRecordDomainService)
    # 运动规划服务
    container.register_singleton(MotionPlanningDomainService)
    # 视觉服务
    container.register_singleton(CameraDomainService)
    container.register_singleton(RecognitionDomainService)
    
    # 手眼标定配置（作为值对象注入）
    def create_hand_eye_config():
        repo = resolve(HandEyeCalibrationRepository)
        return repo.load()
    
    container.register_singleton(HandEyeCalibrationConfig, create_hand_eye_config)
    
    # 手眼标定服务（会自动注入 HandEyeCalibrationConfig 和 KinematicDomainService）
    container.register_singleton(HandEyeTransformDomainService)
    
def register_application_services(container: DIContainer) -> None:
    """注册 Application 层服务。
    
    Args:
        container (DIContainer): DI 容器实例。
    """
    container.register_singleton(MessageDisplay)
    container.register_singleton(SerialApplicationService)
    container.register_singleton(CommandHubService)
    container.register_singleton(MessageResponseService)
    container.register_singleton(MotionListener)
    # 运动规划应用服务
    container.register_singleton(MotionPlanningApplicationService)
    # 摄像头应用服务
    container.register_singleton(CameraApplicationService)
    # 工具应用服务
    container.register_singleton(ToolsApplicationService)
    
    # 手动连接应用服务之间的信号
    # MotionPlanningApplicationService -> CameraApplicationService (检测服务启停)
    motion_planning_app = resolve(MotionPlanningApplicationService)
    camera_app = resolve(CameraApplicationService)
    camera_app.set_start_stop_detection_signal(motion_planning_app.detection_service_requested)

def register_presentation_services(container: DIContainer) -> None:
    """注册 Presentation 层服务。
    
    Args:
        container (DIContainer): DI 容器实例。
    """
    
    container.register_singleton(DisplayViewModel)
    container.register_singleton(SerialViewModel)
    container.register_singleton(StatusViewModel)  
    container.register_singleton(ControlViewModel)
    container.register_singleton(EffectorViewModel)
    container.register_singleton(TrajectoryViewModel)
    container.register_singleton(DynamicsViewModel)  
    # CameraViewModel 只依赖 CameraApplicationService（Application层）
    # DI容器会自动根据构造函数注入依赖
    container.register_singleton(CameraViewModel)
    # 运动规划ViewModel
    container.register_singleton(MotionPlanningViewModel)
    # 工具ViewModel
    container.register_singleton(ToolsViewModel)
    container.register_singleton(MainViewModel)

def configure_services() -> DIContainer:
    """配置所有服务。
    
    按层次注册所有依赖服务。
    
    Returns:
        DIContainer: 配置好的容器实例。
    """
    container = get_container()
    
    # 按层次注册服务
    register_infrastructure_services(container)
    register_domain_services(container)
    register_application_services(container)
    register_presentation_services(container)
    
    return container


def get_main_view_model() -> MainViewModel:
    """便捷方法：获取主视图模型。
    
    Returns:
        MainViewModel: 主视图模型实例。
    """
    return resolve(MainViewModel)


def get_serial_service() -> SerialApplicationService:
    """便捷方法：获取串口服务。
    
    Returns:
        SerialApplicationService: 串口应用服务实例。
    """
    return resolve(SerialApplicationService)


def get_serial_view_model() -> SerialViewModel:
    """便捷方法：获取串口视图模型。
    
    Returns:
        SerialViewModel: 串口视图模型实例。
    """
    return resolve(SerialViewModel)


def get_display_view_model() -> DisplayViewModel:
    """便捷方法：获取显示视图模型。
    
    Returns:
        DisplayViewModel: 显示视图模型实例。
    """
    return resolve(DisplayViewModel)

def get_listener_service() -> MotionListener:
    """便捷方法：获取监听服务。
    
    Returns:
        MotionListener: 监听服务实例。
    """
    return resolve(MotionListener)
