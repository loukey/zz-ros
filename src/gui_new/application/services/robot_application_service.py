"""
机器人应用服务 - 提供机器人相关的业务用例
"""
from typing import List, Optional, Tuple
from datetime import datetime

from application.dto.robot_dto import RobotStateDTO, JointAnglesDTO, PoseDTO
from application.dto.command_dto import SendAnglesDTO, ControlCommandDTO, SerialConfigDTO
from domain.entities.robot import Robot
from domain.entities.trajectory import Trajectory
from domain.value_objects.pose import JointAngles, Pose
from domain.value_objects.command import SerialCommand, ControlMode, RunMode, EffectorMode
from domain.value_objects.trajectory import CurveType, VelocityProfile, ContourParameters
from domain.services.trajectory_planning_service import TrajectoryPlanningService
from domain.services.kinematic_service import KinematicService
from domain.services.command_formatting_service import CommandFormattingService
from shared.events.event_bus import EventBus
from shared.config.container import ServiceLocator


class RobotApplicationService:
    """机器人应用服务"""
    
    def __init__(self):
        self.event_bus = ServiceLocator.resolve(EventBus)
        self.trajectory_service = ServiceLocator.resolve(TrajectoryPlanningService)
        self.kinematic_service = ServiceLocator.resolve(KinematicService)
        self.command_service = ServiceLocator.resolve(CommandFormattingService)
        
        # 当前机器人实例
        self._robot: Optional[Robot] = None
        self._current_trajectory: Optional[Trajectory] = None
    
    def create_robot(self, model_name: str, serial_number: str) -> str:
        """创建机器人实例"""
        self._robot = Robot(
            model_name=model_name,
            serial_number=serial_number
        )
        return self._robot.id
    
    def connect_robot(self, config_dto: SerialConfigDTO) -> Tuple[bool, str]:
        """连接机器人"""
        if not self._robot:
            return False, "机器人实例未创建"
        
        try:
            from domain.value_objects.command import SerialConfig
            config = SerialConfig(
                port=config_dto.port,
                baud_rate=config_dto.baud_rate,
                data_bits=config_dto.data_bits,
                parity=config_dto.parity,
                stop_bits=config_dto.stop_bits,
                flow_control=config_dto.flow_control
            )
            
            self._robot.connect(config)
            
            # 发布领域事件
            for event in self._robot.get_domain_events():
                self.event_bus.publish(event)
            self._robot.clear_domain_events()
            
            return True, "连接成功"
            
        except Exception as e:
            return False, f"连接失败: {str(e)}"
    
    def disconnect_robot(self) -> Tuple[bool, str]:
        """断开机器人连接"""
        if not self._robot:
            return False, "机器人实例未创建"
        
        try:
            self._robot.disconnect()
            
            # 发布领域事件
            for event in self._robot.get_domain_events():
                self.event_bus.publish(event)
            self._robot.clear_domain_events()
            
            return True, "断开连接成功"
            
        except Exception as e:
            return False, f"断开连接失败: {str(e)}"
    
    def get_robot_state(self) -> Optional[RobotStateDTO]:
        """获取机器人状态"""
        if not self._robot:
            return None
        
        state = self._robot.current_state
        position = None
        orientation = None
        
        if state.current_pose:
            position = [
                state.current_pose.position.x,
                state.current_pose.position.y,
                state.current_pose.position.z
            ]
            orientation = [
                state.current_pose.orientation.x,
                state.current_pose.orientation.y,
                state.current_pose.orientation.z,
                state.current_pose.orientation.w
            ]
        
        return RobotStateDTO(
            joint_angles=list(state.current_angles.angles),
            position=position,
            orientation=orientation,
            velocities=list(state.velocities) if state.velocities else None,
            torques=list(state.torques) if state.torques else None,
            is_moving=state.is_moving,
            is_connected=self._robot.is_connected,
            last_updated=state.last_updated
        )
    
    def update_joint_angles(self, angles_dto: JointAnglesDTO) -> Tuple[bool, str]:
        """更新关节角度"""
        if not self._robot:
            return False, "机器人实例未创建"
        
        try:
            joint_angles = JointAngles.from_list(angles_dto.angles)
            
            # 验证关节限位
            is_valid, errors = self._robot.validate_joint_angles(joint_angles)
            if not is_valid:
                return False, f"关节角度超出限位: {'; '.join(errors)}"
            
            self._robot.update_joint_angles(joint_angles)
            
            # 发布领域事件
            for event in self._robot.get_domain_events():
                self.event_bus.publish(event)
            self._robot.clear_domain_events()
            
            return True, "关节角度更新成功"
            
        except Exception as e:
            return False, f"更新关节角度失败: {str(e)}"
    
    def send_angles(self, send_angles_dto: SendAnglesDTO) -> Tuple[bool, str, Optional[str]]:
        """发送角度命令"""
        if not self._robot:
            return False, "机器人实例未创建", None
        
        if not self._robot.is_connected:
            return False, "机器人未连接", None
        
        try:
            # 创建关节角度
            target_angles = JointAngles.from_list(send_angles_dto.target_angles)
            
            # 验证角度有效性
            is_valid, errors = self._robot.validate_joint_angles(target_angles)
            if not is_valid:
                return False, f"目标角度无效: {'; '.join(errors)}", None
            
            # 创建轮廓参数
            if send_angles_dto.contour_params:
                contour_params = ContourParameters(
                    speeds=tuple(send_angles_dto.contour_params[0]),
                    accelerations=tuple(send_angles_dto.contour_params[1]),
                    decelerations=tuple(send_angles_dto.contour_params[2])
                )
            else:
                contour_params = ContourParameters.zero()
            
            # 创建串口命令
            command = SerialCommand.position_command(
                joint_angles=target_angles,
                contour_params=contour_params
            )
            
            # 格式化命令
            formatted_command = self.command_service.format_command(
                command, send_angles_dto.encoding_type
            )
            
            # 验证命令
            is_valid, errors = self.command_service.validate_command(command)
            if not is_valid:
                return False, f"命令验证失败: {'; '.join(errors)}", None
            
            return True, "命令生成成功", formatted_command
            
        except Exception as e:
            return False, f"发送角度失败: {str(e)}", None
    
    def plan_trajectory(
        self, 
        start_angles: List[float],
        end_angles: List[float],
        curve_type: str = 'S型',
        dt: float = 0.01
    ) -> Tuple[bool, str, Optional[str]]:
        """规划轨迹"""
        try:
            start_joint_angles = JointAngles.from_list(start_angles)
            end_joint_angles = JointAngles.from_list(end_angles)
            
            # 转换曲线类型
            curve_type_enum = CurveType.S_CURVE
            if curve_type == '梯形':
                curve_type_enum = CurveType.TRAPEZOIDAL
            elif curve_type == '直线':
                curve_type_enum = CurveType.LINEAR
            
            # 获取速度配置
            from shared.config.container import ServiceLocator
            from shared.config.settings import AppSettings
            settings = ServiceLocator.resolve(AppSettings)
            velocity_profile = settings.get_robot_velocity_profile()
            
            # 规划轨迹
            trajectory_points = self.trajectory_service.plan_trajectory(
                start_joint_angles,
                end_joint_angles,
                curve_type_enum,
                velocity_profile,
                dt
            )
            
            # 创建轨迹实体
            trajectory = Trajectory(
                name=f"轨迹_{datetime.now().strftime('%H%M%S')}",
                description=f"从 {start_angles} 到 {end_angles}"
            )
            trajectory.set_points(trajectory_points)
            
            self._current_trajectory = trajectory
            
            return True, f"轨迹规划成功，共{len(trajectory_points)}个点", trajectory.id
            
        except Exception as e:
            return False, f"轨迹规划失败: {str(e)}", None
    
    def compute_forward_kinematics(
        self, joint_angles: List[float]
    ) -> Tuple[bool, str, Optional[PoseDTO]]:
        """计算正向运动学"""
        try:
            joint_angles_obj = JointAngles.from_list(joint_angles)
            pose = self.kinematic_service.forward_kinematics(joint_angles_obj)
            
            pose_dto = PoseDTO(
                position=[pose.position.x, pose.position.y, pose.position.z],
                orientation=[pose.orientation.x, pose.orientation.y, 
                           pose.orientation.z, pose.orientation.w]
            )
            
            return True, "正向运动学计算成功", pose_dto
            
        except Exception as e:
            return False, f"正向运动学计算失败: {str(e)}", None
    
    def compute_inverse_kinematics(
        self, 
        position: List[float], 
        orientation: List[float],
        initial_guess: Optional[List[float]] = None
    ) -> Tuple[bool, str, Optional[JointAnglesDTO]]:
        """计算逆向运动学"""
        try:
            from domain.value_objects.pose import Position, Orientation
            pose_obj = Pose(
                position=Position(position[0], position[1], position[2]),
                orientation=Orientation(orientation[0], orientation[1], 
                                      orientation[2], orientation[3])
            )
            
            guess = None
            if initial_guess:
                guess = JointAngles.from_list(initial_guess)
            
            joint_angles = self.kinematic_service.inverse_kinematics(pose_obj, guess)
            
            angles_dto = JointAnglesDTO(angles=joint_angles.to_list())
            
            return True, "逆向运动学计算成功", angles_dto
            
        except Exception as e:
            return False, f"逆向运动学计算失败: {str(e)}", None
    
    def start_movement(self) -> Tuple[bool, str]:
        """开始机器人运动"""
        if not self._robot:
            return False, "机器人实例未创建"
        
        try:
            self._robot.start_movement()
            
            # 发布领域事件
            for event in self._robot.get_domain_events():
                self.event_bus.publish(event)
            self._robot.clear_domain_events()
            
            return True, "机器人开始运动"
            
        except Exception as e:
            return False, f"启动运动失败: {str(e)}"
    
    def stop_movement(self) -> Tuple[bool, str]:
        """停止机器人运动"""
        if not self._robot:
            return False, "机器人实例未创建"
        
        try:
            self._robot.stop_movement()
            
            # 发布领域事件
            for event in self._robot.get_domain_events():
                self.event_bus.publish(event)
            self._robot.clear_domain_events()
            
            return True, "机器人停止运动"
            
        except Exception as e:
            return False, f"停止运动失败: {str(e)}" 