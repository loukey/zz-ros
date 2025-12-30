"""
消息响应服务 - Application层
处理串口接收的数据，解码并更新状态服务
"""
from controller.domain import (
    MessageDomainService, 
    SerialDomainService, 
    RobotStateDomainService, 
    MotionConstructor, 
    MotionRunner,
    TrajectoryPlanningService,
    MotionOperationMode
)
from controller.infrastructure import TrajectoryRepository
from .base_service import BaseService
from ..commands import MessageDisplay
from PyQt5.QtCore import pyqtSignal


class MessageResponseService(BaseService):
    """消息响应服务 - Application层。
    
    处理串口接收的数据，解码并更新状态服务。
    
    职责：
    1. 接收串口数据
    2. 拼接缓冲，检测完整帧
    3. 解码消息
    4. 更新状态服务（统一入口）
    5. 根据操作模式分发处理（执行/保存/预览）
    
    Attributes:
        get_current_position_signal (pyqtSignal): 获取当前位置信号（用于UI显示）。
        trajectory_preview_signal (pyqtSignal): 轨迹预览数据信号。
    """
    
    # 信号：获取当前位置（用于UI显示等非运动场景）
    get_current_position_signal = pyqtSignal(object)
    # 信号：轨迹预览数据
    trajectory_preview_signal = pyqtSignal(dict, dict)
    
    def __init__(
        self, 
        message_display: MessageDisplay,
        serial_domain_service: SerialDomainService,
        message_domain_service: MessageDomainService,
        robot_state_service: RobotStateDomainService,
        motion_runner: MotionRunner,
        motion_constructor: MotionConstructor,
        trajectory_planner: TrajectoryPlanningService,
        trajectory_repository: TrajectoryRepository
    ):
        """初始化消息响应服务。"""
        super().__init__(message_display)
        self.serial_domain_service = serial_domain_service
        self.message_domain_service = message_domain_service
        self.robot_state_service = robot_state_service
        self.motion_runner = motion_runner
        self.motion_constructor = motion_constructor
        self.trajectory_planner = trajectory_planner
        self.trajectory_repository = trajectory_repository
        
        self._connect_signals()
        self.message_buffer = ""
        
        # 操作处理器映射（策略模式）
        self._operation_handlers = {
            MotionOperationMode.EXECUTE: self._handle_execute,
            MotionOperationMode.SAVE: self._handle_save,
            MotionOperationMode.PREVIEW: self._handle_preview,
        }

    def _connect_signals(self):
        """连接串口domain层的数据接收信号"""
        self.serial_domain_service.data_received.connect(self.handle_message)

    def handle_message(self, message_in: str):
        """处理接收到的消息。
        
        流程：
        1. 拼接缓冲
        2. 检测完整帧（AA55...0D0A）
        3. 解码消息
        4. 更新状态服务
        5. 处理运动消息
        
        Args:
            message_in (str): 输入的原始串口数据。
        """
        self.message_buffer += message_in
        if "0D0A" in self.message_buffer:
            if self.message_buffer.startswith("AA55") and len(self.message_buffer) < 120:
                return
            lines = self.message_buffer.rsplit("0D0A", 1)            
            self.message_buffer = lines[-1]
            command_line = lines[0]
            self._display_message(command_line + "0D0A", "接收")
            
            if command_line.startswith("AA55"):
                try:
                    # 解码消息
                    decoded_message = self.message_domain_service.decode_message(
                        command_line + "0D0A"
                    )
                    # 更新统一的状态服务（单一入口）
                    self.robot_state_service.update_state(decoded_message)
                    
                    # 处理运动消息
                    if decoded_message.control == 0x07 and decoded_message.mode == 0x08:
                        self.handle_motion_message(decoded_message.positions)

                except Exception as e:
                    self._display_message(f"解码消息失败: {str(e)}", "错误")
            else:
                try:
                    ascii_text = bytes.fromhex(command_line).decode('ascii', errors='replace')
                    printable_text = ''.join(
                        c if c.isprintable() or c in '\n\r\t' 
                        else f'\\x{ord(c):02x}' 
                        for c in ascii_text
                    )
                    if printable_text.strip():
                        self._display_message(f"ASCII: {printable_text}", "接收")
                except Exception:
                    pass

    def handle_motion_message(self, current_position):
        """处理运动消息（获取当前位置的回复）。
        
        根据状态决定行为：
        1. 有待处理的操作 → 调用对应的处理器
        2. 无操作 → 仅发射信号供UI显示
        
        Args:
            current_position (list[float]): 当前关节位置（编码器值）。
        """
        if self.motion_constructor.has_pending_operation():
            # 获取操作模式
            mode = self.motion_constructor.get_operation_mode()
            
            # 从字典中获取对应的处理函数
            handler = self._operation_handlers.get(mode)
            
            if handler:
                # 调用处理函数
                handler(current_position)
            else:
                self._display_message(f"未知操作模式: {mode}", "错误")
                self.motion_constructor.clear_operation()
        else:
            # 无操作：仅发射信号供UI显示
            self.get_current_position_signal.emit(current_position)
    
    def _handle_execute(self, start_position):
        """处理执行运动。
        
        Args:
            start_position (list[float]): 起始位置（当前关节角度）。
        """
        try:
            self.motion_constructor.execute_motion(start_position)
            self._display_message("轨迹构建完成，开始执行运动", "运动")
        except Exception as e:
            self._display_message(f"执行运动失败: {e}", "错误")
            self.motion_constructor.clear_operation()
    
    def _handle_save(self, start_position):
        """处理保存轨迹。
        
        Args:
            start_position (list[float]): 起始位置（当前关节角度）。
        """
        try:
            tasks = self.motion_constructor.get_pending_tasks()
            context = self.motion_constructor.get_operation_context()
            
            if not tasks or not context:
                self._display_message("保存失败：没有待保存的任务", "错误")
                self.motion_constructor.clear_operation()
                return
            
            # 规划轨迹
            all_positions = self.trajectory_planner.plan_task_sequence(
                tasks,
                start_position
            )
            
            if not all_positions:
                self._display_message("保存失败：轨迹为空", "错误")
                self.motion_constructor.clear_operation()
                return
            
            # 保存到文件
            filename = context["filename"]
            self.trajectory_repository.save_trajectory(filename, all_positions)
            
            # 清除状态
            self.motion_constructor.clear_operation()
            
            # 显示成功消息
            save_type = "节点" if context.get("type") == "node" else "方案"
            self._display_message(
                f"✅ {save_type}轨迹已保存：./plans/{filename}.json "
                f"(共 {len(all_positions)} 个轨迹点)",
                "保存"
            )
        except Exception as e:
            self._display_message(f"保存轨迹失败: {e}", "错误")
            self.motion_constructor.clear_operation()
    
    def _handle_preview(self, start_position):
        """处理预览轨迹（显示曲线）。
        
        Args:
            start_position (list[float]): 起始位置（当前关节角度）。
        """
        try:
            tasks = self.motion_constructor.get_pending_tasks()
            context = self.motion_constructor.get_operation_context()
            
            if not tasks:
                self._display_message("预览失败：没有待预览的任务", "错误")
                self.motion_constructor.clear_operation()
                return
            
            # 规划轨迹（获取完整数据：位置、速度、加速度）
            trajectory_data = self.trajectory_planner.plan_task_sequence_with_derivatives(
                tasks,
                start_position
            )
            
            # 清除状态
            self.motion_constructor.clear_operation()
            
            # 发射信号，让UI显示曲线
            preview_type = "节点" if context.get("type") == "node" else "方案"
            self._display_message(f"📊 正在显示{preview_type}轨迹曲线", "预览")
            
            # 通过信号传递数据给UI
            self.trajectory_preview_signal.emit(trajectory_data, context)
        except Exception as e:
            self._display_message(f"预览轨迹失败: {e}", "错误")
            self.motion_constructor.clear_operation()
