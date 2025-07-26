"""
应用设置管理 - 集中管理各种配置参数
"""
from dataclasses import dataclass, field
from typing import List, Dict, Any, Optional
from pathlib import Path
import json
from math import pi


@dataclass
class RobotConfig:
    """机器人配置"""
    model_name: str = "6DOF机械臂"
    serial_number: str = "DEFAULT001"
    
    # 运动学参数
    max_velocity: List[float] = field(default_factory=lambda: [pi/4] * 6)
    max_acceleration: List[float] = field(default_factory=lambda: [pi/8] * 6)
    max_jerk: List[float] = field(default_factory=lambda: [pi/16] * 6)
    
    # 关节限位（弧度）
    joint_limits_min: List[float] = field(default_factory=lambda: [-pi, -pi/2, -pi, -pi, -pi/2, -pi])
    joint_limits_max: List[float] = field(default_factory=lambda: [pi, pi/2, pi, pi, pi/2, pi])


@dataclass
class SerialConfig:
    """串口配置"""
    default_port: str = "COM1"
    default_baud_rate: int = 115200
    default_data_bits: int = 8
    default_parity: str = "N"
    default_stop_bits: int = 1
    default_flow_control: Optional[str] = None
    timeout: float = 0.1
    encoding: str = "hex"


@dataclass
class CameraConfig:
    """相机配置"""
    default_width: int = 640
    default_height: int = 480
    default_fps: int = 30
    color_format: str = "BGR"
    depth_format: str = "UINT16"
    connection_timeout: float = 5.0


@dataclass
class TrajectoryConfig:
    """轨迹配置"""
    default_dt: float = 0.01  # 采样间隔（秒）
    default_curve_type: str = "S型"
    smoothing_window: int = 11
    smoothing_poly_order: int = 3
    max_history_points: int = 1000


@dataclass
class UIConfig:
    """界面配置"""
    theme: str = "light"
    language: str = "zh_CN"
    window_width: int = 1200
    window_height: int = 800
    status_message_timeout: int = 5000  # 毫秒
    update_interval: int = 33  # 约30FPS


@dataclass
class LoggingConfig:
    """日志配置"""
    level: str = "INFO"
    format: str = "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
    file_path: str = "logs/app.log"
    max_file_size: int = 10 * 1024 * 1024  # 10MB
    backup_count: int = 5


class AppSettings:
    """应用设置管理器"""
    
    def __init__(self, config_path: Optional[Path] = None):
        self.config_path = config_path or Path("config/app_config.json")
        
        # 默认配置
        self.robot = RobotConfig()
        self.serial = SerialConfig()
        self.camera = CameraConfig()
        self.trajectory = TrajectoryConfig()
        self.ui = UIConfig()
        self.logging = LoggingConfig()
        
        # 加载配置文件
        self._load_config()
    
    def _load_config(self) -> None:
        """加载配置文件"""
        if not self.config_path.exists():
            # 创建默认配置文件
            self._save_config()
            return
        
        try:
            with open(self.config_path, 'r', encoding='utf-8') as f:
                config_data = json.load(f)
            
            # 更新配置
            self._update_from_dict(config_data)
            
        except Exception as e:
            print(f"加载配置文件失败: {e}")
            # 使用默认配置
    
    def _save_config(self) -> None:
        """保存配置文件"""
        try:
            # 确保配置目录存在
            self.config_path.parent.mkdir(parents=True, exist_ok=True)
            
            config_data = self._to_dict()
            
            with open(self.config_path, 'w', encoding='utf-8') as f:
                json.dump(config_data, f, indent=2, ensure_ascii=False)
                
        except Exception as e:
            print(f"保存配置文件失败: {e}")
    
    def _to_dict(self) -> Dict[str, Any]:
        """转换为字典"""
        return {
            "robot": {
                "model_name": self.robot.model_name,
                "serial_number": self.robot.serial_number,
                "max_velocity": self.robot.max_velocity,
                "max_acceleration": self.robot.max_acceleration,
                "max_jerk": self.robot.max_jerk,
                "joint_limits_min": self.robot.joint_limits_min,
                "joint_limits_max": self.robot.joint_limits_max
            },
            "serial": {
                "default_port": self.serial.default_port,
                "default_baud_rate": self.serial.default_baud_rate,
                "default_data_bits": self.serial.default_data_bits,
                "default_parity": self.serial.default_parity,
                "default_stop_bits": self.serial.default_stop_bits,
                "default_flow_control": self.serial.default_flow_control,
                "timeout": self.serial.timeout,
                "encoding": self.serial.encoding
            },
            "camera": {
                "default_width": self.camera.default_width,
                "default_height": self.camera.default_height,
                "default_fps": self.camera.default_fps,
                "color_format": self.camera.color_format,
                "depth_format": self.camera.depth_format,
                "connection_timeout": self.camera.connection_timeout
            },
            "trajectory": {
                "default_dt": self.trajectory.default_dt,
                "default_curve_type": self.trajectory.default_curve_type,
                "smoothing_window": self.trajectory.smoothing_window,
                "smoothing_poly_order": self.trajectory.smoothing_poly_order,
                "max_history_points": self.trajectory.max_history_points
            },
            "ui": {
                "theme": self.ui.theme,
                "language": self.ui.language,
                "window_width": self.ui.window_width,
                "window_height": self.ui.window_height,
                "status_message_timeout": self.ui.status_message_timeout,
                "update_interval": self.ui.update_interval
            },
            "logging": {
                "level": self.logging.level,
                "format": self.logging.format,
                "file_path": self.logging.file_path,
                "max_file_size": self.logging.max_file_size,
                "backup_count": self.logging.backup_count
            }
        }
    
    def _update_from_dict(self, config_data: Dict[str, Any]) -> None:
        """从字典更新配置"""
        if "robot" in config_data:
            robot_data = config_data["robot"]
            self.robot.model_name = robot_data.get("model_name", self.robot.model_name)
            self.robot.serial_number = robot_data.get("serial_number", self.robot.serial_number)
            self.robot.max_velocity = robot_data.get("max_velocity", self.robot.max_velocity)
            self.robot.max_acceleration = robot_data.get("max_acceleration", self.robot.max_acceleration)
            self.robot.max_jerk = robot_data.get("max_jerk", self.robot.max_jerk)
            self.robot.joint_limits_min = robot_data.get("joint_limits_min", self.robot.joint_limits_min)
            self.robot.joint_limits_max = robot_data.get("joint_limits_max", self.robot.joint_limits_max)
        
        if "serial" in config_data:
            serial_data = config_data["serial"]
            self.serial.default_port = serial_data.get("default_port", self.serial.default_port)
            self.serial.default_baud_rate = serial_data.get("default_baud_rate", self.serial.default_baud_rate)
            self.serial.default_data_bits = serial_data.get("default_data_bits", self.serial.default_data_bits)
            self.serial.default_parity = serial_data.get("default_parity", self.serial.default_parity)
            self.serial.default_stop_bits = serial_data.get("default_stop_bits", self.serial.default_stop_bits)
            self.serial.default_flow_control = serial_data.get("default_flow_control", self.serial.default_flow_control)
            self.serial.timeout = serial_data.get("timeout", self.serial.timeout)
            self.serial.encoding = serial_data.get("encoding", self.serial.encoding)
        
        # 类似地更新其他配置...
    
    def save(self) -> None:
        """保存当前配置"""
        self._save_config()
    
    def reset_to_defaults(self) -> None:
        """重置为默认配置"""
        self.robot = RobotConfig()
        self.serial = SerialConfig()
        self.camera = CameraConfig()
        self.trajectory = TrajectoryConfig()
        self.ui = UIConfig()
        self.logging = LoggingConfig()
        self.save()
    
    def update_robot_config(self, **kwargs) -> None:
        """更新机器人配置"""
        for key, value in kwargs.items():
            if hasattr(self.robot, key):
                setattr(self.robot, key, value)
        self.save()
    
    def update_serial_config(self, **kwargs) -> None:
        """更新串口配置"""
        for key, value in kwargs.items():
            if hasattr(self.serial, key):
                setattr(self.serial, key, value)
        self.save()
    
    def get_robot_velocity_profile(self):
        """获取机器人速度配置"""
        from domain.value_objects.trajectory import VelocityProfile
        return VelocityProfile(
            max_velocity=tuple(self.robot.max_velocity),
            max_acceleration=tuple(self.robot.max_acceleration),
            max_jerk=tuple(self.robot.max_jerk)
        ) 