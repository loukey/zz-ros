"""
机械臂状态领域服务 - Domain层
统一管理机械臂运行时状态
"""
from collections import deque
import threading
import time
from typing import Optional, List, Dict
from PyQt5.QtCore import QObject, pyqtSignal
from ...value_objects import RobotStateSnapshot


class RobotStateDomainService(QObject):
    """
    机械臂状态领域服务 - 单一数据源
    
    职责：
    1. 维护当前机械臂状态（最新快照）
    2. 维护历史状态缓冲（滑动窗口）
    3. 计算静摩擦补偿（基于历史数据）
    4. 发射信号通知订阅者
    
    设计原则：
    - Single Source of Truth: 所有状态只在这里维护
    - Pub-Sub Pattern: 支持多个订阅者
    - Thread Safe: 使用锁保护可变数据
    """
    
    # ════════════════════════════════════════════════════════
    # 信号定义
    # ════════════════════════════════════════════════════════
    
    # 主信号：状态更新（所有订阅者都监听）
    state_updated = pyqtSignal(object)  # 参数：RobotStateSnapshot
    
    # 可选信号：角度显著变化时触发（减少不必要的更新）
    angles_changed = pyqtSignal(list)   # 参数：[θ1, θ2, ..., θ6]
    
    # 力矩补偿请求信号（数据驱动）
    torque_compensation_requested = pyqtSignal(list)  # 参数：当前角度
    
    def __init__(self):
        super().__init__()
        
        # ════════════════════════════════════════════════════════
        # 状态存储
        # ════════════════════════════════════════════════════════
        
        # 当前状态（最新快照）
        self._current_state: Optional[RobotStateSnapshot] = None
        
        # 历史缓冲（滑动窗口，用于静摩擦计算）
        self._position_history = deque(maxlen=20)
        
        # ════════════════════════════════════════════════════════
        # 静摩擦计算相关（用于示教模式）
        # ════════════════════════════════════════════════════════
        
        # 静摩擦状态（每个关节：0=静止, 1=运动）
        self._friction_state = [0] * 6
        
        # 静摩擦补偿值（Nm）
        self._friction_compensation = [0.0] * 6
        
        # 静摩擦配置（阈值）
        self._friction_config = [4, 0, 3, 0.5, 0.5, 0.5]
        
        # ════════════════════════════════════════════════════════
        # 其他状态
        # ════════════════════════════════════════════════════════
        
        # 示教模式标志
        self._teaching_mode = False
        
        # 线程锁（保护可变数据）
        self._lock = threading.RLock()
        
        # 最后更新时间
        self._last_update_time = 0.0
    
    # ════════════════════════════════════════════════════════
    # 核心方法：更新状态
    # ════════════════════════════════════════════════════════
    
    def update_state(self, decoded_message):
        """
        更新机械臂状态（核心方法）
        
        由 MessageResponseService 调用，每次接收到串口数据并解码后调用。
        
        Args:
            decoded_message: 解码后的消息对象
        """
        new_snapshot = RobotStateSnapshot.from_decoded_message(decoded_message)
        old_snapshot = self._current_state
        self._current_state = new_snapshot
        self._last_update_time = new_snapshot.timestamp
        
        with self._lock:
            self._position_history.append({
                'timestamp': new_snapshot.timestamp,
                'angles': list(new_snapshot.joint_angles),
                'positions': list(new_snapshot.joint_positions)
            })
            
            if self._teaching_mode and len(self._position_history) >= 20:
                self._calculate_friction_compensation()
        
        self.state_updated.emit(new_snapshot)
        
        if old_snapshot and self._angles_changed_significantly(old_snapshot, new_snapshot):
            self.angles_changed.emit(list(new_snapshot.joint_angles))
        
        if new_snapshot.mode == 0x0A:
            if self._teaching_mode and new_snapshot.control in [0x06, 0x07]:
                from math import pi
                init_offset = [0.0, -pi/2, 0.0, pi/2, 0.0, 0.0]
                adjusted_angles = [
                    a - offset 
                    for a, offset in zip(new_snapshot.joint_angles, init_offset)
                ]
                self.torque_compensation_requested.emit(adjusted_angles)

    
    # ════════════════════════════════════════════════════════
    # 查询方法：获取状态
    # ════════════════════════════════════════════════════════
    
    def get_current_state(self) -> Optional[RobotStateSnapshot]:
        """获取当前状态快照（不可变，线程安全）"""
        return self._current_state
    
    def get_current_angles(self) -> List[float]:
        """获取当前关节角度（弧度）"""
        if self._current_state:
            return list(self._current_state.joint_angles)
        return [0.0] * 6
    
    def get_current_positions(self) -> List[int]:
        """获取当前关节位置（编码器值）"""
        if self._current_state:
            return list(self._current_state.joint_positions)
        return [0] * 6
    
    def get_position_history(self, count: int = 20) -> List[Dict]:
        """获取历史位置数据"""
        with self._lock:
            return list(self._position_history)[-count:]
    
    def get_friction_compensation(self) -> List[float]:
        """获取当前静摩擦补偿值"""
        with self._lock:
            return self._friction_compensation.copy()
    
    # ════════════════════════════════════════════════════════
    # 配置方法
    # ════════════════════════════════════════════════════════
    
    def set_teaching_mode(self, enabled: bool):
        """
        设置示教模式
        
        Args:
            enabled: True=开启, False=关闭
        """
        self._teaching_mode = enabled
        if enabled:
            # 开启示教时，清空历史
            with self._lock:
                self._position_history.clear()
                self._friction_state = [0] * 6
                self._friction_compensation = [0.0] * 6
    
    def set_friction_config(self, config: List[float]):
        """设置静摩擦配置"""
        if len(config) == 6:
            self._friction_config = config
    
    def clear_history(self):
        """清空历史数据"""
        with self._lock:
            self._position_history.clear()
            self._friction_state = [0] * 6
            self._friction_compensation = [0.0] * 6
    
    # ════════════════════════════════════════════════════════
    # 内部方法
    # ════════════════════════════════════════════════════════
    
    def _calculate_friction_compensation(self):
        """
        计算静摩擦补偿（内部方法）
        
        算法：状态机方式（与旧版GUI完全一致）
        - 状态0（静止）：检测运动开始
        - 状态1（运动）：检测运动停止
        
        注意：
        - 使用编码器位置值（整数）进行计算，不使用弧度
        - 阈值 5 表示 5 个编码器位置值（约 0.00006 弧度）
        - 这与旧版GUI的实现完全一致
        """
        if len(self._position_history) < 20:
            return
        
        # 运动检测阈值（编码器位置值）
        MOTION_THRESHOLD = 5  # 5 个编码器值 ≈ 0.00006 弧度 ≈ 0.0034 度
        
        # 获取最新值（使用编码器位置值，不是弧度）
        latest = self._position_history[-1]['positions']
        
        # 获取前19个值的最后5个（用于平均）
        recent_5 = list(self._position_history)[-6:-1]
        
        # 对每个关节维度计算
        for dim in range(6):
            # 计算最近5个点的平均值（编码器位置值）
            avg_5 = sum(data['positions'][dim] for data in recent_5) / 5
            
            # 计算差值（编码器位置值）
            difference = latest[dim] - avg_5
            
            # 状态机逻辑
            if self._friction_state[dim] == 0:
                # 状态0：检测运动开始
                if difference > MOTION_THRESHOLD:  # 正向运动
                    self._friction_compensation[dim] = self._friction_config[dim]
                    self._friction_state[dim] = 1
                elif difference < -MOTION_THRESHOLD:  # 负向运动
                    self._friction_compensation[dim] = -self._friction_config[dim]
                    self._friction_state[dim] = 1
            
            elif self._friction_state[dim] == 1:
                # 状态1：检测运动停止
                if abs(difference) < MOTION_THRESHOLD:  # 运动停止
                    self._friction_compensation[dim] = 0.0
                    self._friction_state[dim] = 0
    
    def _angles_changed_significantly(self, old_snapshot, new_snapshot, threshold=0.001):
        """
        检查角度是否显著变化
        
        Args:
            old_snapshot: 旧快照
            new_snapshot: 新快照
            threshold: 变化阈值（弧度）
            
        Returns:
            bool: 是否有显著变化
        """
        for old_a, new_a in zip(old_snapshot.joint_angles, new_snapshot.joint_angles):
            if abs(new_a - old_a) > threshold:
                return True
        return False

