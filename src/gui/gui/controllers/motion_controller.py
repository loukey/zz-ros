"""
运动处理模块
"""
from PyQt5.QtCore import QTimer
from .utils import *


class MotionController:
    """运动处理类，处理机械臂运动相关功能"""
    
    def __init__(self, send_callback, display_callback, motion_model):
        self.send_callback = send_callback
        self.display_callback = display_callback
        self.motion_model = motion_model
        self.current_angles = None
        self.target_angles = None
        self.curve_params = None
        self.encoding_type = 'hex'
        self.run_mode = 0x08
    
    def send_control_command(self, control=0x01, run_mode=0x08, encoding_type='hex'):        
        if control in [0x05, 0x06]:
            if hasattr(self, 'trajectory_timer') and self.trajectory_timer.isActive():
                self.trajectory_timer.stop()
                self.display_callback("已停止轨迹发送", "控制")
        
        success, cmd_str = self.send_callback(
            control=control,
            encoding=encoding_type,
            mode=run_mode,
            return_cmd=True
        )
                
        if success:
            self.display_callback(f"{cmd_str}", "发送")
        else:
            self.display_callback(f"发送控制命令失败: {control}", "错误")
    
    def send_angles(self, 
                    target_angles, 
                    curve_type, 
                    frequency,
                    contour_params,
                    encoding_type='hex',
                    run_mode=0x08):
        try:
            # 记录目标角度和参数
            self.display_callback(f"目标角度: [{", ".join([f"{angle:.4f}" for angle in target_angles])}]", "控制")
            contour_speed, contour_acceleration, contour_deceleration = contour_params
            if run_mode == 0x01:
                success, cmd_str = self.send_callback(
                    joint_angles=target_angles,
                    control=0x06,
                    mode=run_mode,
                    contour_speed=contour_speed,
                    contour_acceleration=contour_acceleration,
                    contour_deceleration=contour_deceleration,
                    encoding=encoding_type,
                    return_cmd=True
                )
                if not success:
                    self.display_callback("轮廓位置模式位置发送失败", "错误")
                    return
                self.display_callback(f"{cmd_str}", "发送")
                self.display_callback("轮廓位置模式位置发送成功", "控制")
                return
            
            self.display_callback(f"曲线类型: {curve_type}, 频率: {frequency}秒", "参数")
            
            # 获取当前位置
            self.display_callback("正在获取当前位置...", "控制")
            success, cmd_str = self.send_callback(
                control=0x07, 
                mode=run_mode,
                encoding=encoding_type,
                return_cmd=True
            )
            
            if not success:
                self.display_callback("发送获取当前位置命令失败", "错误")
                return
                
            # 记录发送的命令
            self.display_callback(f"{cmd_str}", "发送")
            self.display_callback("等待位置数据响应...", "控制")
            
            # 保存参数等待位置响应
            self.target_angles = target_angles
            self.curve_params = (curve_type, frequency)
            self.encoding_type = encoding_type
            self.run_mode = run_mode
                
        except Exception as e:
            self.display_callback(f"发送角度命令异常: {str(e)}", "错误")
    
    def process_differential_motion(self, positions):
        """处理差分运动
        
        流程：获取当前位置 -> 调用轨迹计算 -> 发送轨迹点
        """
        self.current_position = positions
        if not self.current_position or not self.target_angles_pending:
            self.send_callback("缺少当前位置或目标位置数据", "错误")
            return
        
        try:
            # 获取存储的参数
            target_angles = self.target_angles_pending
            curve_type, duration, frequency = self.curve_params_pending
            encoding_type = self.encoding_type_pending
            run_mode = self.run_mode_pending
            
            # 将当前位置从整数值转换为弧度
            try:
                self.current_angles_rad = position_to_radian(self.current_position)
            except Exception as e:
                self.send_callback(f"位置数据转换异常: {str(e)}", "错误")
                return
            
            self.send_callback(f"当前角度(rad): {[round(a, 4) for a in self.current_angles_rad]}", "控制")
            self.send_callback(f"目标角度(rad): {[round(a, 4) for a in target_angles]}", "控制")
            self.send_callback("开始计算轨迹...", "控制")
            # 检查是否需要计算完整轨迹
            if duration > 0 and frequency > 0:
                trajectory_points, time_points = self.main_window.trajectory_controller.calculate_trajectory(
                    start_angles=self.current_angles_rad,
                    end_angles=target_angles,
                    duration=duration,
                    frequency=frequency,
                    curve_type=curve_type
                )
                self.send_callback(
                    f"轨迹计算完成: 共{len(trajectory_points)}个点, 总时长{time_points[-1] if len(time_points) > 0 else 0}秒", 
                    "控制"
                )
                self.send_trajectory_points(trajectory_points, time_points, encoding_type, run_mode, frequency)
            else:
                # 单点模式，直接发送目标角度
                self.send_callback("单点模式，直接发送目标角度", "控制")
                self.send_single_point(target_angles, encoding_type, run_mode)
            
            # 清除等待状态
            self.target_angles_pending = None
            self.curve_params_pending = None
            self.encoding_type_pending = None
            self.run_mode_pending = None
            
        except Exception as e:
            self.send_callback(f"差分运动处理异常: {str(e)}", "错误")
            import traceback
            self.send_callback(f"错误详情: {traceback.format_exc()}", "错误")
   
    def send_trajectory_points(self, trajectory_points, time_points, encoding_type, run_mode, frequency):
        """发送轨迹点序列
        
        Args:
            trajectory_points: 轨迹点列表
            time_points: 时间点列表
            encoding_type: 编码类型
            run_mode: 运行模式
            frequency: 频率
        """
        try:
            # 开始发送轨迹
            self.send_callback(f"开始发送轨迹: 共{len(trajectory_points)}个点", "控制")
            
            # 确保轨迹点和时间点是原生Python列表
            if hasattr(trajectory_points, 'tolist'):
                trajectory_points = [point.tolist() if hasattr(point, 'tolist') else list(point) for point in trajectory_points]
            else:
                # 深拷贝确保安全
                trajectory_points = [list(point) for point in trajectory_points]
                
            if hasattr(time_points, 'tolist'):
                time_points = time_points.tolist()
            else:
                time_points = list(time_points)
            
            # 创建轨迹发送定时器
            self.trajectory_timer = QTimer()
            self.current_trajectory_index = 0
            self.trajectory_points = trajectory_points
            self.time_points = time_points
            self.traj_encoding_type = encoding_type
            self.traj_run_mode = run_mode
            
            # 设置定时器间隔 - 直接使用界面设置的频率
            time_interval = int(frequency * 1000)  # 将秒转为毫秒
            
            self.send_callback(f"轨迹点发送间隔: {time_interval}ms (频率: {frequency}秒)", "控制")
            self.trajectory_timer.setInterval(time_interval)
            
            # 连接定时器信号
            self.trajectory_timer.timeout.connect(self.on_trajectory_timer_timeout)
            
            # 启动定时器
            self.trajectory_timer.start()
            
            return True
            
        except Exception as e:
            import traceback
            self.send_callback(f"启动轨迹发送异常: {str(e)}", "错误")
            self.send_callback(f"异常详情: {traceback.format_exc()}", "错误")
            return False
    
    def on_trajectory_timer_timeout(self):
        """轨迹定时器超时处理
        
        当定时器触发时，发送下一个轨迹点
        """
        try:
            # 检查是否已发送完所有点
            if self.current_trajectory_index >= len(self.trajectory_points):
                self.trajectory_timer.stop()
                self.send_callback("轨迹发送完成", "控制")
                return
            
            # 获取当前轨迹点
            current_point = self.trajectory_points[self.current_trajectory_index]
            point_time = self.time_points[self.current_trajectory_index]
            # 记录将要发送的数据
            self.send_callback(
                f"准备发送轨迹点 {self.current_trajectory_index+1}: {[round(a, 4) for a in current_point]}", 
                "调试"
            )
            
            # 发送当前轨迹点
            try:
                success, cmd_str = self.main_window.serial_handler.serial_controller.send_control_command(
                    joint_angles=current_point,
                    command_type='MOTION',
                    encoding=self.traj_encoding_type,
                    mode=self.traj_run_mode,
                    return_cmd=True
                )
            except Exception as cmd_error:
                # 捕获并记录发送命令中的具体异常
                self.send_callback(f"发送命令异常: {str(cmd_error)}", "错误")
                success = False
                cmd_str = ""
            
            if success:
                # 显示轨迹点信息
                joint_values_str = ", ".join([f"{angle:.4f}" for angle in current_point])
                self.send_callback(
                    f"轨迹点 {self.current_trajectory_index+1}/{len(self.trajectory_points)}, 时间: {point_time:.2f}s", 
                    "轨迹"
                )
                self.send_callback(f"关节值: [{joint_values_str}]", "轨迹")
                current_point_position = radian_to_position(current_point)
                self.send_callback(f"关节位置: [{current_point_position}]", "轨迹")
                self.send_callback(f"{cmd_str}", "发送")
                
                # 前进到下一个点
                self.current_trajectory_index += 1
            else:
                # 发送失败，记录更详细的错误并继续
                self.send_callback(
                    f"发送轨迹点 {self.current_trajectory_index+1} 失败，类型：{type(current_point)}", 
                    "错误"
                )
                self.current_trajectory_index += 1
                
        except Exception as e:
            import traceback
            self.send_callback(f"轨迹点发送异常: {str(e)}", "错误")
            self.send_callback(f"异常详情: {traceback.format_exc()}", "错误")
            # 发生异常时停止定时器
            self.trajectory_timer.stop()
    

