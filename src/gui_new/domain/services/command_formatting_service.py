"""
命令格式化服务 - 将业务指令转换为硬件通信格式
"""
from typing import List, Dict, Any, Optional
from domain.value_objects.pose import JointAngles
from domain.value_objects.command import SerialCommand, ControlMode, RunMode


class CommandFormattingService:
    """命令格式化服务"""
    
    def __init__(self):
        # 命令模板
        self.command_templates = {
            'move_joints': 'M {angles}',
            'set_speed': 'SP {speed}',
            'set_acceleration': 'AC {acceleration}',
            'emergency_stop': 'ES',
            'get_position': 'GP',
            'get_status': 'GS',
            'home': 'HM',
            'enable': 'EN',
            'disable': 'DI'
        }
    
    def format_joint_movement_command(self, joint_angles: JointAngles, 
                                    speed: float = 1.0) -> SerialCommand:
        """格式化关节运动命令"""
        try:
            # 将弧度转换为度
            angles_deg = [self._rad_to_deg(angle) for angle in joint_angles.angles]
            
            # 格式化角度字符串
            angles_str = ' '.join([f'{angle:.2f}' for angle in angles_deg])
            
            # 构建命令
            command_str = f"M {angles_str} SP{speed:.1f}"
            
            return SerialCommand(
                command=command_str,
                parameters={'angles': angles_deg, 'speed': speed},
                expected_response='OK',
                timeout=5.0
            )
            
        except Exception:
            return SerialCommand(
                command='',
                parameters={},
                expected_response='',
                timeout=1.0
            )
    
    def format_speed_command(self, speed_percentage: float) -> SerialCommand:
        """格式化速度设置命令"""
        try:
            # 限制速度范围 0-100%
            speed = max(0.0, min(100.0, speed_percentage))
            command_str = f"SP{speed:.1f}"
            
            return SerialCommand(
                command=command_str,
                parameters={'speed': speed},
                expected_response='OK',
                timeout=2.0
            )
            
        except Exception:
            return SerialCommand(command='', parameters={}, expected_response='', timeout=1.0)
    
    def format_acceleration_command(self, acceleration_percentage: float) -> SerialCommand:
        """格式化加速度设置命令"""
        try:
            # 限制加速度范围 0-100%
            accel = max(0.0, min(100.0, acceleration_percentage))
            command_str = f"AC{accel:.1f}"
            
            return SerialCommand(
                command=command_str,
                parameters={'acceleration': accel},
                expected_response='OK',
                timeout=2.0
            )
            
        except Exception:
            return SerialCommand(command='', parameters={}, expected_response='', timeout=1.0)
    
    def format_emergency_stop_command(self) -> SerialCommand:
        """格式化急停命令"""
        return SerialCommand(
            command='ES',
            parameters={},
            expected_response='STOP',
            timeout=1.0
        )
    
    def format_enable_command(self) -> SerialCommand:
        """格式化使能命令"""
        return SerialCommand(
            command='EN',
            parameters={},
            expected_response='OK',
            timeout=2.0
        )
    
    def format_disable_command(self) -> SerialCommand:
        """格式化失能命令"""
        return SerialCommand(
            command='DI',
            parameters={},
            expected_response='OK',
            timeout=2.0
        )
    
    def format_home_command(self) -> SerialCommand:
        """格式化回零命令"""
        return SerialCommand(
            command='HM',
            parameters={},
            expected_response='HOME_OK',
            timeout=30.0  # 回零需要更长时间
        )
    
    def format_position_query_command(self) -> SerialCommand:
        """格式化位置查询命令"""
        return SerialCommand(
            command='GP',
            parameters={},
            expected_response='POS',
            timeout=2.0
        )
    
    def format_status_query_command(self) -> SerialCommand:
        """格式化状态查询命令"""
        return SerialCommand(
            command='GS',
            parameters={},
            expected_response='STATUS',
            timeout=2.0
        )
    
    def format_custom_command(self, command: str, parameters: Dict[str, Any] = None,
                            expected_response: str = 'OK', timeout: float = 2.0) -> SerialCommand:
        """格式化自定义命令"""
        return SerialCommand(
            command=command,
            parameters=parameters or {},
            expected_response=expected_response,
            timeout=timeout
        )
    
    def parse_response(self, response: str, command_type: str) -> Dict[str, Any]:
        """解析机器人响应"""
        try:
            result = {'success': False, 'data': None, 'error': None}
            
            if not response:
                result['error'] = '空响应'
                return result
            
            response = response.strip()
            
            if command_type == 'position_query':
                # 解析位置响应: "POS 10.5 20.3 30.1 40.2 50.4 60.6"
                if response.startswith('POS'):
                    parts = response.split()
                    if len(parts) >= 7:
                        try:
                            angles_deg = [float(parts[i]) for i in range(1, 7)]
                            angles_rad = [self._deg_to_rad(angle) for angle in angles_deg]
                            result['success'] = True
                            result['data'] = {'joint_angles': angles_rad}
                        except ValueError:
                            result['error'] = '位置数据格式错误'
                    else:
                        result['error'] = '位置数据不完整'
                else:
                    result['error'] = '位置响应格式错误'
                    
            elif command_type == 'status_query':
                # 解析状态响应: "STATUS READY ENABLED"
                if response.startswith('STATUS'):
                    parts = response.split()
                    if len(parts) >= 2:
                        result['success'] = True
                        result['data'] = {
                            'status': parts[1] if len(parts) > 1 else 'UNKNOWN',
                            'enabled': 'ENABLED' in response,
                            'ready': 'READY' in response,
                            'error': 'ERROR' in response
                        }
                else:
                    result['error'] = '状态响应格式错误'
                    
            elif response == 'OK':
                result['success'] = True
                result['data'] = {'message': '命令执行成功'}
                
            elif response.startswith('ERROR'):
                result['error'] = response
                
            else:
                # 通用成功响应
                result['success'] = True
                result['data'] = {'response': response}
            
            return result
            
        except Exception as e:
            return {'success': False, 'data': None, 'error': f'响应解析失败: {str(e)}'}
    
    def _rad_to_deg(self, radians: float) -> float:
        """弧度转度"""
        try:
            return radians * 180.0 / 3.14159265359
        except Exception:
            return 0.0
    
    def _deg_to_rad(self, degrees: float) -> float:
        """度转弧度"""
        try:
            return degrees * 3.14159265359 / 180.0
        except Exception:
            return 0.0 