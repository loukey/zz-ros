"""
动力学视图模型 - Presentation层
订阅 RobotStateDomainService，实现力矩补偿和示教记录
"""
from typing import Optional, List
from PyQt5.QtCore import pyqtSignal
from .base_view_model import BaseViewModel
from controller.application import CommandHubService
from controller.domain import RobotStateDomainService, DynamicDomainService, TeachRecordDomainService
from controller.infrastructure import RecordRepository


class DynamicsViewModel(BaseViewModel):
    """
    动力学视图模型
    
    职责：
    1. 处理示教模式切换
    2. 订阅力矩补偿请求（数据驱动）
    3. 计算并发送力矩补偿
    4. 管理示教记录的录制、保存、播放
    """
    
    # 记录相关信号
    recording_state_changed = pyqtSignal(bool)  # 记录状态变化
    record_list_updated = pyqtSignal(list)  # 记录列表更新
    
    def __init__(
        self,
        command_hub_service: CommandHubService,
        robot_state_service: RobotStateDomainService,
        dynamic_service: DynamicDomainService,
        teach_record_service: TeachRecordDomainService,
        record_repository: RecordRepository,
        parent=None):
        super().__init__(parent)
        self.command_hub = command_hub_service
        self.robot_state = robot_state_service
        self.dynamic_service = dynamic_service
        self.teach_record_service = teach_record_service
        self.record_repository = record_repository
        
        # 订阅力矩补偿请求信号（数据驱动，不是定时器）
        self.robot_state.torque_compensation_requested.connect(
            self._on_torque_compensation_requested
        )
        
        # 订阅记录服务的信号
        self.teach_record_service.recording_state_changed.connect(
            self.recording_state_changed.emit
        )
        self.teach_record_service.record_added.connect(
            self._on_record_added
        )
        self.teach_record_service.record_deleted.connect(
            self._on_record_deleted
        )
        
        # 连接记录定时器到状态更新
        record_timer = self.teach_record_service.get_record_timer()
        record_timer.timeout.connect(self._on_record_timer_timeout)
        
        # 加载已存在的记录
        self._load_records_from_file()
    
    def toggle_teaching_mode(self, command_dict: dict):
        """
        切换示教模式
        
        Args:
            command_dict: 命令字典，包含 control 和 mode
        """
        # 1. 发送串口命令
        self.command_hub.command_distribution(command_dict)
        
        # 2. 更新状态服务的示教模式标志
        is_enable = (command_dict['control'] == 0x07)
        self.robot_state.set_teaching_mode(is_enable)
    
    def send_torque(self, torque_values: list):
        """
        发送力矩控制命令
        
        Args:
            torque_values: 力矩值列表（6个关节）
        """
        command_dict = {
            'control': 0x06,  # 力矩控制
            'mode': 0x0A,     # 周期力矩模式
            'torque': torque_values
        }
        self.command_hub.command_distribution(command_dict)
    
    def _on_torque_compensation_requested(self, joint_angles: list):
        """
        力矩补偿请求回调（数据驱动）
        
        由 RobotStateDomainService 在接收到 mode=0x0A 的数据时触发
        
        Args:
            joint_angles: 当前关节角度（已减去初始偏置）
        """
        try:
            # 1. 计算重力补偿
            gravity_torque = self.dynamic_service.compute_gravity_compensation(joint_angles)
            
            # 2. 获取静摩擦补偿
            friction_comp = self.robot_state.get_friction_compensation()
            
            # 3. 总力矩 = 重力补偿 + 静摩擦补偿
            total_torque = [
                g + f 
                for g, f in zip(gravity_torque, friction_comp)
            ]
            
            self.send_torque(total_torque)
            
        except Exception as e:
            pass
    
    # ════════════════════════════════════════════════════════
    # 示教记录功能
    # ════════════════════════════════════════════════════════
    
    def toggle_recording(self):
        """切换记录状态（开始/停止）"""
        if self.teach_record_service.is_recording():
            # 停止记录
            record_name = self.teach_record_service.stop_recording()
            if record_name:
                # 保存到文件
                self._save_records_to_file()
        else:
            # 开始记录
            self.teach_record_service.start_recording()
    
    def delete_record(self, record_name: str) -> bool:
        """
        删除指定记录
        
        Args:
            record_name: 记录名称
            
        Returns:
            是否删除成功
        """
        success = self.teach_record_service.delete_record(record_name)
        if success:
            self._save_records_to_file()
        return success
    
    def run_record(self, record_name: str):
        """
        运行指定记录（播放示教轨迹）
        
        流程：
        1. 获取示教记录的角度列表
        2. 委托给 CommandHubService 处理
        
        Args:
            record_name: 记录名称
        """
        angles_list = self.teach_record_service.get_record(record_name)
        if angles_list:
            self.command_hub.run_teach_record(angles_list)
    
    def reverse_record(self, record_name: str) -> Optional[str]:
        """
        反转指定记录
        
        Args:
            record_name: 记录名称
            
        Returns:
            新记录名称，失败返回 None
        """
        new_name = self.teach_record_service.reverse_record(record_name)
        if new_name:
            self._save_records_to_file()
        return new_name
    
    def get_record_names(self) -> List[str]:
        """获取所有记录名称"""
        return self.teach_record_service.get_record_names()
    
    def is_recording(self) -> bool:
        """查询是否正在记录"""
        return self.teach_record_service.is_recording()
    
    # ════════════════════════════════════════════════════════
    # 内部方法
    # ════════════════════════════════════════════════════════
    
    def _on_record_timer_timeout(self):
        """
        记录定时器超时回调
        从 RobotStateDomainService 获取当前角度并添加到记录
        """
        if self.teach_record_service.is_recording():
            current_angles = self.robot_state.get_current_angles()
            self.teach_record_service.add_angle_to_current_record(current_angles)
    
    def _on_record_added(self, record_name: str):
        """记录添加回调"""
        self._emit_record_list_update()
    
    def _on_record_deleted(self, record_name: str):
        """记录删除回调"""
        self._emit_record_list_update()
    
    def _emit_record_list_update(self):
        """发射记录列表更新信号"""
        record_names = self.teach_record_service.get_record_names()
        self.record_list_updated.emit(record_names)
    
    def _load_records_from_file(self):
        """从文件加载记录"""
        records = self.record_repository.load_records()
        self.teach_record_service.load_records(records)
        self._emit_record_list_update()
    
    def _save_records_to_file(self):
        """保存记录到文件"""
        records = self.teach_record_service.get_all_records()
        self.record_repository.save_records(records)

