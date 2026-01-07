"""
摄像头应用服务 - Application层
协调摄像头Domain服务、检测服务和消息显示
提供统一的查询接口，避免ViewModel直接访问Domain层
"""
from PyQt5.QtCore import QObject, pyqtSignal
from typing import Optional, Dict
import numpy as np
from controller.domain import (
    CameraDomainService,
    RecognitionDomainService,
    ImageDrawingUtils,
    HandEyeTransformDomainService,
    KinematicDomainService,
    RobotStateDomainService,
    MotionConstructor,
    MotionOperationMode
)
from ..commands import MessageDisplay
from .command_hub_service import CommandHubService


class CameraApplicationService(QObject):
    """摄像头应用服务。
    
    协调摄像头Domain服务、检测服务和消息显示，提供统一的查询接口，避免ViewModel直接访问Domain层。
    
    Attributes:
        connection_status_changed (pyqtSignal): 摄像头连接状态变化信号。
        detection_status_changed (pyqtSignal): 检测状态变化信号。
    """
    
    # ========== 信号定义 ==========
    connection_status_changed = pyqtSignal(bool)  # 摄像头连接状态
    detection_status_changed = pyqtSignal(bool)  # 检测状态
    
    def __init__(
        self,
        camera_service: CameraDomainService,
        recognition_service: RecognitionDomainService,
        hand_eye_service: HandEyeTransformDomainService,
        kinematic_service: KinematicDomainService,
        robot_state_service: RobotStateDomainService,
        motion_constructor: MotionConstructor,
        command_hub: CommandHubService,
        message_display: MessageDisplay
    ):
        """初始化摄像头应用服务。"""
        super().__init__()
        self.camera_service = camera_service
        self.recognition_service = recognition_service
        self.hand_eye_service = hand_eye_service
        self.kinematic_service = kinematic_service
        self.robot_state_service = robot_state_service
        self.motion_constructor = motion_constructor
        self.command_hub = command_hub
        self.message_display = message_display
        
        # 连接Domain Service信号
        self._connect_signals()
    
    # ========== 核心方法 ==========
    
    def connect_camera(self):
        """连接摄像头。"""
        self.message_display.clear_messages()
        self._display_message("正在连接摄像头...", "摄像头")
        
        success = self.camera_service.connect()
        
        if success:
            self._display_message("摄像头连接成功", "摄像头")
            self.connection_status_changed.emit(True)
        else:
            self._display_message("摄像头连接失败", "错误")
            self.connection_status_changed.emit(False)
    
    def disconnect_camera(self):
        """断开摄像头。"""
        self.message_display.clear_messages()
        self._display_message("正在断开摄像头...", "摄像头")
        
        success = self.camera_service.disconnect()
        
        if success:
            self._display_message("摄像头已断开", "摄像头")
            self.connection_status_changed.emit(False)
        else:
            self._display_message("断开摄像头失败", "错误")
    
    def start_detection(self):
        """开始检测。"""
        self.message_display.clear_messages()
        self._display_message("正在启动检测...", "检测")
        
        success = self.recognition_service.start_detection()
        
        if success:
            self._display_message("开始零件识别", "检测")
            self.detection_status_changed.emit(True)
        else:
            self._display_message("启动检测失败", "错误")
            self.detection_status_changed.emit(False)
    
    def stop_detection(self):
        """停止检测。"""
        self.message_display.clear_messages()
        self._display_message("正在停止检测...", "检测")
        
        success = self.recognition_service.stop_detection()
        
        if success:
            self._display_message("停止零件识别", "检测")
            self.detection_status_changed.emit(False)
        else:
            self._display_message("停止检测失败", "错误")
    
    def move_to_detected_part(self):
        """运动到检测到的零件位置。
        
        完整流程：
        1. 检查检测状态
        2. 获取检测结果
        3. 获取当前关节角度
        4. 手眼标定计算目标关节角度
        5. 构建运动任务
        6. 触发运动执行
        7. 停止检测（可选）
        """
        # 1. 检查检测是否运行
        if not self.recognition_service.is_detection_running():
            self._display_message("检测未运行，无法执行运动", "错误")
            return
        
        # 2. 获取最新检测结果
        detection_result = self.recognition_service.get_latest_result()
        if not detection_result:
            self._display_message("未检测到零件，无法执行运动", "警告")
            return
        
        # 3. 获取当前关节角度
        current_state = self.robot_state_service.get_current_state()
        if not current_state:
            self._display_message("无法获取当前机器人状态", "错误")
            return
        
        current_joint_angles = current_state.joint_angles  # ✅ 修复：正确的属性名（弧度）
        
        # 4. 手眼标定计算目标关节角度
        try:
            target_angles = self.hand_eye_service.calculate_target_joint_angles(
                central_center=detection_result['central_center'],
                depth=detection_result['depth'],
                real_center=detection_result['real_center'],
                real_depth=detection_result['real_depth'],
                angle=detection_result['angle'],
                current_joint_angles=current_joint_angles
            )
            
            if target_angles is None:
                self._display_message("逆运动学无解，无法到达目标位置", "错误")
                return
            
        except Exception as e:
            self._display_message(f"计算目标位姿失败: {str(e)}", "错误")
            return
        
        # 5. 构建运动任务
        motion_task = {
            'type': 'motion',
            'target_angles': target_angles,
            'curve_type': 's_curve',  # 's_curve' 或 'linear'（笛卡尔直线）
            'frequency': 0.01
        }
        
        # 6. 准备运动并触发执行
        try:
            # 使用统一的 prepare_operation 接口
            self.motion_constructor.prepare_operation(
                MotionOperationMode.EXECUTE,
                [motion_task]  # 需要传入任务列表
            )
            self.command_hub.get_current_position()  # 触发运动执行
            
            self._display_message(
                f"开始运动到零件位置 "
                f"(中心: {detection_result['central_center']}, "
                f"深度: {detection_result['depth']:.2f}mm, "
                f"角度: {np.degrees(detection_result['angle']):.1f}°)",
                "运动"
            )
            
        except Exception as e:
            self._display_message(f"启动运动失败: {str(e)}", "错误")
            return
        
        # 7. 停止检测（可选）
        # self.recognition_service.stop_detection()
        # self._display_message("已自动停止检测", "检测")
    
    # ========== 私有方法 ==========
    
    def _connect_signals(self):
        """连接Domain Service信号"""
        # 摄像头服务信号
        self.camera_service.connection_status_changed.connect(
            self._on_camera_connection_status_changed
        )
        self.camera_service.error_occurred.connect(self._on_camera_error)
        
        # 检测服务信号
        self.recognition_service.detection_status_changed.connect(
            self._on_detection_status_changed
        )
        self.recognition_service.detection_result_received.connect(
            self._on_detection_result_received
        )
        self.recognition_service.error_occurred.connect(self._on_detection_error)
        
        # MotionConstructor 信号
        self.motion_constructor.detection_requested.connect(
            self._on_motion_detection_requested
        )
        
    def set_start_stop_detection_signal(self, detection_service_requested: pyqtSignal):
        """设置来自MotionPlanningApplicationService的信号连接"""
        detection_service_requested.connect(self._on_detection_service_requested)
    
    def _on_detection_service_requested(self, start: bool):
        """处理检测服务启停请求"""
        if start:
            self.start_detection()
        else:
            self.stop_detection()
    
    def _on_camera_connection_status_changed(self, connected: bool, message: str):
        """处理摄像头连接状态变化"""
        # 显示消息（Domain Service提供的详细消息）
        msg_type = "摄像头" if not message.startswith("错误") else "错误"
        self._display_message(message, msg_type)
        
        # 发射信号给ViewModel
        self.connection_status_changed.emit(connected)
    
    def _on_camera_error(self, error_msg: str):
        """处理摄像头错误"""
        self._display_message(error_msg, "错误")
    
    def _on_detection_status_changed(self, is_running: bool, message: str):
        """处理检测状态变化"""
        # 显示消息
        msg_type = "检测"
        self._display_message(message, msg_type)
        
        # 发射信号给ViewModel
        self.detection_status_changed.emit(is_running)
    
    def _on_detection_result_received(self, detection: dict):
        """处理检测结果"""
        # 格式化检测结果并显示
        try:
            angle = detection.get('angle', 0.0)
            depth = detection.get('depth', 0.0)
            real_depth = detection.get('real_depth', 0.0)
            central_center = detection.get('central_center', (0, 0))
        except Exception as e:
            self._display_message(f"检测结果格式错误: {e}", "错误")
    
    def _on_detection_error(self, error_msg: str):
        """处理检测错误"""
        self._display_message(error_msg, "错误")
        
    def _on_motion_detection_requested(self, current_position: list):
        """处理运动过程中的检测请求。
        
        当运动规划执行到"检测节点"时触发。
        流程：
        1. 获取最新检测结果
        2. 手眼标定计算目标位置
        3. 恢复运动规划（注入目标位置）
        
        Args:
            current_position (list): 机器人当前物理位置（关节角度）。
        """
        self._display_message("已到达检测点，正在进行识别...", "检测")
        
        # 1. 确保检测正在运行
        if not self.recognition_service.is_detection_running():
            # 如果没运行，尝试启动
            self._display_message("检测未运行，尝试自动启动...", "检测")
            if not self.recognition_service.start_detection():
                self._display_message("无法启动检测，流程中断", "错误")
                return
        
        # 2. 获取检测结果
        detection_result = self.recognition_service.get_latest_result()
        
        if not detection_result:
            self._display_message("未检测到目标，流程中断", "警告")
            return
            
        # 3. 手眼标定计算
        try:
            target_angles = self.hand_eye_service.calculate_target_joint_angles(
                central_center=detection_result['central_center'],
                depth=detection_result['depth'],
                real_center=detection_result['real_center'],
                real_depth=detection_result['real_depth'],
                angle=detection_result['angle'],
                current_joint_angles=current_position
            )
            
            if target_angles is None:
                self._display_message("逆运动学无解，无法到达目标位置", "错误")
                return
                
            self._display_message(
                f"识别成功，计算出目标位姿。角度: {np.degrees(detection_result['angle']):.1f}°",
                "检测"
            )
            
            # 4. 恢复运动规划
            self.motion_constructor.resume_after_detection(target_angles)
            
        except Exception as e:
            self._display_message(f"计算目标位姿失败: {str(e)}", "错误")
    
    def _display_message(self, message: str, msg_type: str = "摄像头"):
        """显示消息的统一接口"""
        self.message_display.display_message(message, msg_type)
    
    # ========== 查询接口（供ViewModel调用，避免直接访问Domain层）==========
    
    def is_camera_connected(self) -> bool:
        """检查摄像头是否连接。
        
        Returns:
            bool: True=已连接, False=未连接。
        """
        return self.camera_service.is_connected
    
    def is_color_available(self) -> bool:
        """检查彩色图像是否可用。
        
        Returns:
            bool: True=有数据, False=无数据。
        """
        return self.camera_service.is_color_available()
    
    def is_depth_available(self) -> bool:
        """检查深度图像是否可用。
        
        Returns:
            bool: True=有数据, False=无数据。
        """
        return self.camera_service.is_depth_available()
    
    def get_latest_color_image(self) -> Optional[np.ndarray]:
        """获取最新彩色图像。
        
        Returns:
            Optional[np.ndarray]: 彩色图像（BGR格式），如果无数据则返回None。
        """
        return self.camera_service.get_latest_color_image()
    
    def get_latest_depth_image(self) -> Optional[np.ndarray]:
        """获取最新深度图像。
        
        Returns:
            Optional[np.ndarray]: 深度图像（16位或32位浮点），如果无数据则返回None。
        """
        return self.camera_service.get_latest_depth_image()
    
    def visualize_depth_image(self, depth_image: np.ndarray) -> np.ndarray:
        """深度图可视化为伪彩色图。
        
        Args:
            depth_image (np.ndarray): 原始深度图（16位或32位浮点）。
            
        Returns:
            np.ndarray: 伪彩色深度图（BGR, uint8）。
        """
        return self.camera_service.visualize_depth_image(depth_image)
    
    def is_detection_running(self) -> bool:
        """检查检测是否正在运行。
        
        Returns:
            bool: True=运行中, False=未运行。
        """
        return self.recognition_service.is_detection_running()
    
    def get_latest_detection_result(self) -> Optional[Dict]:
        """获取最新检测结果。
        
        Returns:
            Optional[Dict]: 检测结果字典，如果无数据则返回None。
                包含字段：head_center, central_center, real_center, angle, depth, real_depth。
        """
        return self.recognition_service.get_latest_result()
    
    def get_image_with_detection(self, image: np.ndarray) -> np.ndarray:
        """获取叠加了检测结果的图像。
        
        封装了检测状态检查和图像绘制逻辑，提供高层业务接口。
        
        Args:
            image (np.ndarray): 原始图像（BGR格式）。
            
        Returns:
            np.ndarray: 如果检测正在运行且有结果，返回叠加了检测标注的图像；
                       否则返回原图像。
        """
        # 检查检测是否运行
        if self.recognition_service.is_detection_running():
            # 获取最新检测结果
            detection = self.recognition_service.get_latest_result()
            if detection:
                # 绘制检测结果到图像上
                return ImageDrawingUtils.draw_detection_result(image, detection)
        
        # 无检测或无结果，返回原图
        return image

