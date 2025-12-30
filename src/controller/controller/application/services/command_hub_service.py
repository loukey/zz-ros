from ..commands import MessageDisplay
from .base_service import BaseService
from typing import Any
from controller.domain import (
    MessageDomainService, 
    MotionRunner, 
    MotionConstructor, 
    SerialDomainService,
    MotionOperationMode
)


class CommandHubService(BaseService):
    """命令中心服务 - Application层。
    
    职责：
    1. 分发用户命令到不同的处理逻辑
    2. 协调运动任务的准备和执行
    3. 管理消息显示
    
    Attributes:
        message_domain_service: 消息领域服务。
        motion_runner: 运动执行器。
        serial_domain_service: 串口领域服务。
        motion_constructor: 运动构造器。
    """

    def __init__(self, 
    message_domain_service: MessageDomainService, 
    motion_runner: MotionRunner,
    serial_domain_service: SerialDomainService, 
    message_display: MessageDisplay,
    motion_constructor: MotionConstructor):
        """初始化命令中心服务。"""
        super().__init__(message_display)
        self.message_domain_service = message_domain_service
        self.motion_runner = motion_runner
        self.serial_domain_service = serial_domain_service
        self.motion_constructor = motion_constructor

    def single_send_command(self, **kwargs: Any) -> bool:
        """发送单个命令。
        
        Args:
            **kwargs: 命令参数，将被编码为消息帧。
            
        Returns:
            bool: 是否发送成功。
        """
        try:
            msg = self.message_domain_service.encode_message(**kwargs)
            self._display_message(msg, "发送")
            success = self.serial_domain_service.send_data(msg)
            if not success:
                self._display_message("发送失败：串口未连接", "错误")
            return success
        except Exception as e:
            self._display_message(f"发送命令失败: {str(e)}", "错误")
            return False

    def command_distribution(self, config_dict: dict):
        """分发用户命令到对应的处理逻辑。
        
        Args:
            config_dict (dict): 配置字典，包含 control, mode 等参数。
        """
        control = config_dict.get('control')
        mode = config_dict.get('mode')
        
        if control in [0x00, 0x01, 0x02, 0x03, 0x04]:
            self.single_send_command(**config_dict)
            self.get_current_position()
            
        elif control == 0x05:
            if mode == 0x0A:
                self._display_message("关闭示教模式", "动力学")
            self.single_send_command(**config_dict)
            self.motion_runner.stop_motion()
            
        elif control == 0x07:
            if mode == 0x0A:
                self._display_message("开启示教模式", "动力学")
            self.single_send_command(**config_dict)
            if mode != 0x0A:
                self.get_current_position()
                
        elif control == 0x06:
            if mode == 0x0A:
                torque_values = config_dict.get('torque')
                self._display_message(f"发送力矩: {torque_values}", "动力学")
                self.single_send_command(**config_dict)
            else:
                # 角度控制：默认使用 S 曲线和 0.01 频率
                target_angles = config_dict.get('target_angles')
                task = {
                    "type": "motion",
                    "target_angles": target_angles,
                    "curve_type": "s_curve",
                    "frequency": 0.01
                }
                self.motion_constructor.prepare_operation(
                    MotionOperationMode.EXECUTE,
                    [task]
                )
                self._display_message(f"准备运动到目标位置: {target_angles}", "控制")
                self.get_current_position()
            
        elif control == 0x08:
            self.single_send_command(**config_dict)
            self.motion_runner.stop_motion()
        

    def get_current_position(self):
        """请求获取当前位置。"""
        self.message_display.clear_messages()
        self._display_message("正在获取当前位置...", "控制")
        self.single_send_command(control=0x07)
    
    def run_teach_record(self, angles_list: list):
        """运行示教记录（播放示教轨迹）。
        
        流程：
        1. 准备示教运动任务
        2. 获取当前位置
        3. 异步回调时自动构建平滑轨迹并执行
        
        Args:
            angles_list (list[list[float]]): 示教记录的角度列表 [[θ1,...,θ6], ...]。
        """
        task = {
            "type": "teach",
            "teach_data": angles_list
        }
        self.motion_constructor.prepare_operation(
            MotionOperationMode.EXECUTE,
            [task]
        )
        self._display_message(f"准备播放示教轨迹，共 {len(angles_list)} 个点", "示教")
        self.get_current_position()
    
    def run_motion_sequence(self):
        """开始执行运动序列（用于运动规划方案）。
        
        流程：
        1. 查询当前位置
        2. 收到位置反馈后，MotionConstructor 自动规划所有任务
        """
        self._display_message("开始执行运动规划方案...", "运动规划")
        self.get_current_position()
