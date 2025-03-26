"""
运动处理模块
"""
from PyQt5.QtWidgets import QMessageBox
from PyQt5.QtCore import QTimer
from utils.command_utils import position_to_radian


class MotionHandler:
    """运动处理类，处理机械臂运动相关功能"""
    
    def __init__(self, main_window):
        self.main_window = main_window
        # 初始化状态变量
        self.current_position = None
        self.target_angles_pending = None
        self.curve_params_pending = None
        self.encoding_type_pending = None
        self.run_mode_pending = None
    
    def send_control_command(self, command_type):
        """发送控制命令"""
        if not self.main_window.serial_handler.is_connected():
            QMessageBox.warning(self.main_window, "警告", "请先连接串口")
            return
        
        encoding_type = self.main_window.control_buttons.get_encoding_type()
        run_mode = self.main_window.control_buttons.get_run_mode()
        command_desc = {
            "ENABLE": "使能",
            "DISABLE": "取消使能",
            "RELEASE": "释放刹车",
            "LOCK": "锁止刹车",
            "STOP": "立刻停止",
            "PAUSE": "暂停"
        }.get(command_type, f"执行 {command_type} 命令")
        
        # 命令为PAUSE或STOP时，我们可能需要停止正在进行的轨迹发送
        if command_type == "PAUSE" or command_type == "STOP":
            if hasattr(self, 'trajectory_timer') and self.trajectory_timer.isActive():
                self.trajectory_timer.stop()
                self.main_window.data_display.append_message("已停止轨迹发送", "控制")
        
        success, cmd_str = self.main_window.serial_handler.serial_controller.send_control_command(
            command_type=command_type,
            encoding=encoding_type,
            mode=run_mode,
            return_cmd=True
        )
        
        self.main_window.data_display.append_message(command_desc, "控制")
        
        if success:
            self.main_window.data_display.append_message(f"{cmd_str}", "发送")
        else:
            self.main_window.data_display.append_message(f"发送控制命令失败: {command_type}", "错误")
            QMessageBox.warning(self.main_window, "错误", f"发送{command_desc}命令失败")
    
    def send_angles(self):
        """发送角度命令
        
        流程：点击发送角度按钮 -> 获取当前位置 -> 计算轨迹 -> 发送轨迹
        """
        if not self.main_window.serial_handler.is_connected():
            QMessageBox.warning(self.main_window, "警告", "请先连接串口")
            return
        
        try:
            # 获取参数
            target_angles = self.main_window.angle_control.get_angles()            
            curve_type, duration, frequency = self.main_window.angle_control.get_curve_type()            
            encoding_type = self.main_window.control_buttons.get_encoding_type()            
            run_mode = self.main_window.control_buttons.get_run_mode()
            
            # 记录目标角度和参数
            angles_str = ", ".join([f"{angle:.4f}" for angle in target_angles])
            self.main_window.data_display.append_message(f"目标角度: [{angles_str}]", "控制")
            self.main_window.data_display.append_message(f"曲线类型: {curve_type}, 时长: {duration}秒, 频率: {frequency}秒", "参数")
            
            # 获取当前位置
            self.main_window.data_display.append_message("正在获取当前位置...", "控制")
            success, cmd_str = self.main_window.serial_handler.serial_controller.send_control_command(
                command_type="POSITION", 
                encoding=encoding_type,
                mode=run_mode,
                return_cmd=True
            )
            
            if not success:
                self.main_window.data_display.append_message("发送POSITION命令失败", "错误")
                return
                
            # 记录发送的命令
            self.main_window.data_display.append_message(f"{cmd_str}", "发送")
            self.main_window.data_display.append_message("等待位置数据响应...", "控制")
            
            # 保存参数等待位置响应
            self.current_position = None
            self.target_angles_pending = target_angles
            self.curve_params_pending = (curve_type, duration, frequency)
            self.encoding_type_pending = encoding_type
            self.run_mode_pending = run_mode
                
        except Exception as e:
            self.main_window.data_display.append_message(f"发送角度命令异常: {str(e)}", "错误")
    
    
    def process_differential_motion(self, positions):
        """处理差分运动
        
        流程：获取当前位置 -> 调用轨迹计算 -> 发送轨迹点
        """
        self.current_position = positions
        if not self.current_position or not self.target_angles_pending:
            self.main_window.data_display.append_message("缺少当前位置或目标位置数据", "错误")
            return
        
        try:
            # 获取存储的参数
            target_angles = self.target_angles_pending
            curve_type, duration, frequency = self.curve_params_pending
            encoding_type = self.encoding_type_pending
            run_mode = self.run_mode_pending
            
            # 将当前位置从整数值转换为弧度
            try:
                current_angles_rad = position_to_radian(self.current_position)
            except Exception as e:
                self.main_window.data_display.append_message(f"位置数据转换异常: {str(e)}", "错误")
                return
            
            self.main_window.data_display.append_message(f"当前角度(rad): {[round(a, 4) for a in current_angles_rad]}", "控制")
            self.main_window.data_display.append_message(f"目标角度(rad): {[round(a, 4) for a in target_angles]}", "控制")
            self.main_window.data_display.append_message("开始计算轨迹...", "控制")
            
            # 检查是否需要计算完整轨迹
            if duration > 0 and frequency > 0:
                trajectory_points, time_points = self.main_window.trajectory_controller.calculate_trajectory(
                    start_angles=current_angles_rad,
                    end_angles=target_angles,
                    duration=duration,
                    frequency=frequency,
                    curve_type=curve_type
                )
                self.main_window.data_display.append_message(
                    f"轨迹计算完成: 共{len(trajectory_points)}个点, 总时长{time_points[-1] if len(time_points) > 0 else 0}秒", 
                    "控制"
                )
                self.send_trajectory_points(trajectory_points, time_points, encoding_type, run_mode, frequency)
            else:
                # 单点模式，直接发送目标角度
                self.main_window.data_display.append_message("单点模式，直接发送目标角度", "控制")
                self.send_single_point(target_angles, encoding_type, run_mode)
            
            # 清除等待状态
            self.target_angles_pending = None
            self.curve_params_pending = None
            self.encoding_type_pending = None
            self.run_mode_pending = None
            
        except Exception as e:
            self.main_window.data_display.append_message(f"差分运动处理异常: {str(e)}", "错误")
            import traceback
            self.main_window.data_display.append_message(f"错误详情: {traceback.format_exc()}", "错误")
            # 重置状态
            self.waiting_for_position = False
            self.main_window.waiting_for_position = False
    
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
            self.main_window.data_display.append_message(f"开始发送轨迹: 共{len(trajectory_points)}个点", "控制")
            
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
            
            self.main_window.data_display.append_message(f"轨迹点发送间隔: {time_interval}ms (频率: {frequency}秒)", "控制")
            self.trajectory_timer.setInterval(time_interval)
            
            # 连接定时器信号
            self.trajectory_timer.timeout.connect(self.on_trajectory_timer_timeout)
            
            # 启动定时器
            self.trajectory_timer.start()
            
            return True
            
        except Exception as e:
            import traceback
            self.main_window.data_display.append_message(f"启动轨迹发送异常: {str(e)}", "错误")
            self.main_window.data_display.append_message(f"异常详情: {traceback.format_exc()}", "错误")
            return False
    
    def on_trajectory_timer_timeout(self):
        """轨迹定时器超时处理
        
        当定时器触发时，发送下一个轨迹点
        """
        try:
            # 检查是否已发送完所有点
            if self.current_trajectory_index >= len(self.trajectory_points):
                self.trajectory_timer.stop()
                self.main_window.data_display.append_message("轨迹发送完成", "控制")
                return
            
            # 获取当前轨迹点
            current_point = self.trajectory_points[self.current_trajectory_index]
            point_time = self.time_points[self.current_trajectory_index]
            for i in range(len(current_point)):
                current_point[i] += self.current_position[i]
            # 记录将要发送的数据
            self.main_window.data_display.append_message(
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
                self.main_window.data_display.append_message(f"发送命令异常: {str(cmd_error)}", "错误")
                success = False
                cmd_str = ""
            
            if success:
                # 显示轨迹点信息
                joint_values_str = ", ".join([f"{angle:.4f}" for angle in current_point])
                self.main_window.data_display.append_message(
                    f"轨迹点 {self.current_trajectory_index+1}/{len(self.trajectory_points)}, 时间: {point_time:.2f}s", 
                    "轨迹"
                )
                self.main_window.data_display.append_message(f"关节值: [{joint_values_str}]", "轨迹")
                self.main_window.data_display.append_message(f"{cmd_str}", "发送")
                
                # # 更新UI显示
                # for i, angle in enumerate(current_point):
                #     if i < len(self.main_window.angle_control.angle_vars):
                #         self.main_window.angle_control.angle_vars[i].setText(f"{angle:.4f}")
                
                # # 更新曲线图
                # if hasattr(self.main_window, 'curve_plot') and hasattr(self.main_window.curve_plot, 'plot_trajectory_point'):
                #     self.main_window.curve_plot.plot_trajectory_point(
                #         self.current_trajectory_index + 1, 
                #         point_time, 
                #         current_point
                #     )
                
                # 更新进度信息
                # progress = ((self.current_trajectory_index + 1) / len(self.trajectory_points)) * 100
                # if hasattr(self.main_window, 'status_bar'):
                #     self.main_window.status_bar.showMessage(f"轨迹进度: {progress:.1f}%")
                
                # if hasattr(self.main_window, 'progress_bar'):
                #     self.main_window.progress_bar.setValue(int(progress))
                
                # 前进到下一个点
                self.current_trajectory_index += 1
            else:
                # 发送失败，记录更详细的错误并继续
                self.main_window.data_display.append_message(
                    f"发送轨迹点 {self.current_trajectory_index+1} 失败，类型：{type(current_point)}", 
                    "错误"
                )
                # 尝试延长下一次发送的时间，可能是发送太快
                self.trajectory_timer.setInterval(self.trajectory_timer.interval() * 1.5)
                self.main_window.data_display.append_message(
                    f"增加发送间隔至 {self.trajectory_timer.interval()}ms", 
                    "控制"
                )
                self.current_trajectory_index += 1
                
        except Exception as e:
            import traceback
            self.main_window.data_display.append_message(f"轨迹点发送异常: {str(e)}", "错误")
            self.main_window.data_display.append_message(f"异常详情: {traceback.format_exc()}", "错误")
            # 发生异常时停止定时器
            self.trajectory_timer.stop()
    
    def convert_angles(self):
        """将角度值转换为弧度值"""
        try:
            # 获取当前所有输入框中的角度值（默认是度）
            angle_values = []
            for angle_var in self.main_window.angle_control.angle_vars:
                try:
                    angle_degrees = float(angle_var.text())
                    angle_values.append(angle_degrees)
                except ValueError:
                    # 转换失败时使用0.0作为默认值
                    angle_values.append(0.0)
            
            # 将角度转换为弧度 (角度 * pi / 180)
            from math import pi
            radian_values = [angle * (pi / 180) for angle in angle_values]
            
            # 更新输入框显示弧度值
            for i, radian in enumerate(radian_values):
                self.main_window.angle_control.angle_vars[i].setText(f"{radian:.4f}")
            
            # 显示转换信息
            angles_str = ", ".join([f"{angle:.2f}°" for angle in angle_values])
            radians_str = ", ".join([f"{radian:.4f}rad" for radian in radian_values])
            self.main_window.data_display.append_message(f"角度转换: [{angles_str}] → [{radians_str}]", "控制")
            
        except Exception as e:
            self.main_window.data_display.append_message(f"角度转换异常: {str(e)}", "错误")
            QMessageBox.warning(self.main_window, "警告", f"角度转换失败: {str(e)}")
    
    def zero_angles(self):
        """归零处理"""
        # 将所有角度值设置为0
        for angle_var in self.main_window.angle_control.angle_vars:
            angle_var.setText("0.0")
        
        # 显示归零信息
        self.main_window.data_display.append_message("角度值归零", "控制") 
