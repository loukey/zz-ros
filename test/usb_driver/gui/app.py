"""
USB串口通信界面主应用程序
"""

from tkinter import messagebox
from tkinter import ttk
import tkinter as tk
import math
import sys
import os
import matplotlib
import matplotlib.pyplot as plt
import platform
import atexit

# 配置matplotlib支持中文显示
if platform.system() == 'Windows':
    # Windows系统使用微软雅黑字体
    matplotlib.rcParams['font.family'] = ['Microsoft YaHei']
else:
    # 其他系统尝试使用其他中文字体
    matplotlib.rcParams['font.family'] = ['SimHei', 'WenQuanYi Micro Hei', 'STXihei', 'sans-serif']
matplotlib.rcParams['axes.unicode_minus'] = False  # 正确显示负号
# 设置matplotlib后端为Agg，避免交互式后端可能导致的进程残留问题
matplotlib.use('Agg')

# 添加项目根目录到Python路径，以便导入kinematic模块
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from kinematic.velocity_planning import trapezoidal_velocity_planning, s_curve_velocity_planning
from kinematic.kinematic_6dof import Kinematic6DOF

from .serial_manager import SerialManager
from .ui_components import (PortSelectionFrame, SerialConfigFrame, ControlButtonsFrame, 
                          AngleControlFrame, DataDisplayFrame, CurvePlotFrame, 
                          InverseKinematicFrame, EndPositionFrame)


class USBSerialApp:
    """USB串口通信应用程序类"""
    
    def __init__(self, root, title="镇中科技串口测试", version="v0.0.12"):
        """
        初始化应用程序
        
        参数:
            root: Tkinter根窗口
            title: 应用程序标题
            version: 应用程序版本
        """
        self.root = root
        self.root.title(f"{title}{version}")
        self.root.geometry("1400x750")  # 调整窗口大小，更紧凑
        
        # 创建串口管理器
        self.serial_manager = SerialManager(message_callback=self.handle_message)
        
        # 创建运动学求解器
        self.kinematic_solver = Kinematic6DOF()
        
        # 创建主标签页控件
        self.main_notebook = ttk.Notebook(self.root)
        self.main_notebook.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # 创建主界面标签页
        self.main_tab = ttk.Frame(self.main_notebook)
        self.main_notebook.add(self.main_tab, text="串口控制")
        
        # 创建曲线显示标签页
        self.curve_tab = ttk.Frame(self.main_notebook)
        self.main_notebook.add(self.curve_tab, text="曲线显示")
        
        # 创建逆运动学计算标签页
        self.ik_tab = ttk.Frame(self.main_notebook)
        self.main_notebook.add(self.ik_tab, text="逆运动学计算")
        
        # 创建UI组件
        self.create_ui_components()
        
        # 刷新串口列表
        self.refresh_ports()
        
        # 设置窗口关闭事件处理
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
    
    def create_ui_components(self):
        """创建所有UI组件"""
        # ===== 主界面标签页 =====
        # 创建滚动条容器
        main_canvas = tk.Canvas(self.main_tab)
        main_scrollbar = ttk.Scrollbar(self.main_tab, orient="vertical", command=main_canvas.yview)
        scrollable_frame = ttk.Frame(main_canvas)
        
        # 配置滚动区域
        scrollable_frame.bind(
            "<Configure>",
            lambda e: main_canvas.configure(scrollregion=main_canvas.bbox("all"))
        )
        
        # 在画布上创建窗口，显示滚动框架
        main_canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        main_canvas.configure(yscrollcommand=main_scrollbar.set)
        
        # 放置画布和滚动条
        main_canvas.pack(side="left", fill="both", expand=True)
        main_scrollbar.pack(side="right", fill="y")
        
        # 绑定鼠标滚轮事件
        def _on_main_mousewheel(event):
            # Windows系统
            if platform.system() == "Windows":
                main_canvas.yview_scroll(int(-1*(event.delta/120)), "units")
            # macOS系统
            elif platform.system() == "Darwin":
                main_canvas.yview_scroll(int(-1*event.delta), "units")
            # Linux系统
            else:
                if event.num == 4:
                    main_canvas.yview_scroll(-1, "units")
                elif event.num == 5:
                    main_canvas.yview_scroll(1, "units")
        
        # 存储主界面滚轮事件处理函数，以便后续绑定和解绑
        self.main_mousewheel_handler = _on_main_mousewheel
        
        # 创建控制面板框架
        control_panel = ttk.Frame(scrollable_frame)
        control_panel.pack(side=tk.TOP, fill=tk.X)  # 只在水平方向填充
        
        # 创建串口选择框架
        self.port_frame = PortSelectionFrame(control_panel, self.refresh_ports)
        
        # 创建串口配置框架
        self.config_frame = SerialConfigFrame(control_panel, self.toggle_connection)
        
        # 创建控制按钮框架
        self.control_frame = ControlButtonsFrame(control_panel, self.send_command)
        
        # 创建角度控制框架
        self.angle_frame = AngleControlFrame(
            control_panel, 
            self.send_angles, 
            self.convert_degrees_to_radians, 
            self.set_angles_to_zero
        )
        
        # 创建末端姿态显示框架
        self.end_position_frame = EndPositionFrame(scrollable_frame)
        self.end_position_frame.frame.pack(fill=tk.X, expand=False, padx=10)  # 添加水平内边距
        
        # 创建数据显示框架
        self.data_frame = DataDisplayFrame(
            scrollable_frame,
            lambda: self.clear_display("send"),
            lambda: self.clear_display("receive"),
            lambda: self.clear_display("all")
        )
        self.data_frame.frame.pack(fill=tk.BOTH, expand=True, padx=10)  # 添加水平内边距
        
        # ===== 曲线显示标签页 =====
        # 创建滚动条容器
        curve_canvas = tk.Canvas(self.curve_tab)
        curve_scrollbar = ttk.Scrollbar(self.curve_tab, orient="vertical", command=curve_canvas.yview)
        curve_scrollable_frame = ttk.Frame(curve_canvas)
        
        # 配置滚动区域
        curve_scrollable_frame.bind(
            "<Configure>",
            lambda e: curve_canvas.configure(scrollregion=curve_canvas.bbox("all"))
        )
        
        # 在画布上创建窗口，显示滚动框架
        curve_canvas.create_window((0, 0), window=curve_scrollable_frame, anchor="nw")
        curve_canvas.configure(yscrollcommand=curve_scrollbar.set)
        
        # 放置画布和滚动条
        curve_canvas.pack(side="left", fill="both", expand=True)
        curve_scrollbar.pack(side="right", fill="y")
        
        # 绑定鼠标滚轮事件
        def _on_curve_mousewheel(event):
            # Windows系统
            if platform.system() == "Windows":
                curve_canvas.yview_scroll(int(-1*(event.delta/120)), "units")
            # macOS系统
            elif platform.system() == "Darwin":
                curve_canvas.yview_scroll(int(-1*event.delta), "units")
            # Linux系统
            else:
                if event.num == 4:
                    curve_canvas.yview_scroll(-1, "units")
                elif event.num == 5:
                    curve_canvas.yview_scroll(1, "units")
        
        # 存储曲线界面滚轮事件处理函数，以便后续绑定和解绑
        self.curve_mousewheel_handler = _on_curve_mousewheel
        
        # 创建曲线绘制框架
        self.curve_frame = CurvePlotFrame(curve_scrollable_frame)
        
        # ===== 逆运动学计算标签页 =====
        # 创建滚动条容器
        ik_canvas = tk.Canvas(self.ik_tab)
        ik_scrollbar = ttk.Scrollbar(self.ik_tab, orient="vertical", command=ik_canvas.yview)
        ik_scrollable_frame = ttk.Frame(ik_canvas)
        
        # 配置滚动区域
        ik_scrollable_frame.bind(
            "<Configure>",
            lambda e: ik_canvas.configure(scrollregion=ik_canvas.bbox("all"))
        )
        
        # 在画布上创建窗口，显示滚动框架
        ik_canvas.create_window((0, 0), window=ik_scrollable_frame, anchor="nw")
        ik_canvas.configure(yscrollcommand=ik_scrollbar.set)
        
        # 放置画布和滚动条
        ik_canvas.pack(side="left", fill="both", expand=True)
        ik_scrollbar.pack(side="right", fill="y")
        
        # 绑定鼠标滚轮事件
        def _on_ik_mousewheel(event):
            # Windows系统
            if platform.system() == "Windows":
                ik_canvas.yview_scroll(int(-1*(event.delta/120)), "units")
            # macOS系统
            elif platform.system() == "Darwin":
                ik_canvas.yview_scroll(int(-1*event.delta), "units")
            # Linux系统
            else:
                if event.num == 4:
                    ik_canvas.yview_scroll(-1, "units")
                elif event.num == 5:
                    ik_canvas.yview_scroll(1, "units")
        
        # 存储逆运动学界面滚轮事件处理函数，以便后续绑定和解绑
        self.ik_mousewheel_handler = _on_ik_mousewheel
        
        # 创建逆运动学框架 - 放在逆运动学标签页中
        self.ik_frame = InverseKinematicFrame(
            ik_scrollable_frame,
            self.calculate_inverse_kinematics
        )
        # 使用pack布局，占据整个标签页
        self.ik_frame.frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # 设置应用按钮回调
        self.ik_frame.set_apply_callback(self.apply_ik_result_to_angles)
        
        # 存储Canvas对象，以便后续操作
        self.main_canvas = main_canvas
        self.curve_canvas = curve_canvas
        self.ik_canvas = ik_canvas
        
        # 设置标签页切换事件，用于处理滚轮事件的绑定
        self.main_notebook.bind("<<NotebookTabChanged>>", self._on_tab_changed)
        
        # 初始绑定主界面的滚轮事件
        self._bind_mousewheel_to_active_tab()
    
    def _on_tab_changed(self, event):
        """处理标签页切换事件"""
        self._bind_mousewheel_to_active_tab()
    
    def _bind_mousewheel_to_active_tab(self):
        """根据当前活动标签页绑定滚轮事件"""
        # 解绑所有滚轮事件
        self._unbind_all_mousewheel()
        
        # 获取当前选中的标签页索引
        current_tab = self.main_notebook.index("current")
        
        # 根据当前标签页绑定相应的滚轮事件
        if current_tab == 0:  # 主界面标签页
            self._bind_main_mousewheel()
        elif current_tab == 1:  # 曲线显示标签页
            self._bind_curve_mousewheel()
        elif current_tab == 2:  # 逆运动学计算标签页
            self._bind_ik_mousewheel()
    
    def _unbind_all_mousewheel(self):
        """解绑所有滚轮事件"""
        # Windows和macOS
        if platform.system() == "Windows" or platform.system() == "Darwin":
            self.root.unbind_all("<MouseWheel>")
        # Linux
        else:
            self.root.unbind_all("<Button-4>")
            self.root.unbind_all("<Button-5>")
    
    def _bind_main_mousewheel(self):
        """绑定主界面的滚轮事件"""
        # Windows和macOS
        if platform.system() == "Windows" or platform.system() == "Darwin":
            self.root.bind_all("<MouseWheel>", self.main_mousewheel_handler)
        # Linux
        else:
            self.root.bind_all("<Button-4>", self.main_mousewheel_handler)
            self.root.bind_all("<Button-5>", self.main_mousewheel_handler)
    
    def _bind_curve_mousewheel(self):
        """绑定曲线界面的滚轮事件"""
        # Windows和macOS
        if platform.system() == "Windows" or platform.system() == "Darwin":
            self.root.bind_all("<MouseWheel>", self.curve_mousewheel_handler)
        # Linux
        else:
            self.root.bind_all("<Button-4>", self.curve_mousewheel_handler)
            self.root.bind_all("<Button-5>", self.curve_mousewheel_handler)
    
    def _bind_ik_mousewheel(self):
        """绑定逆运动学界面的滚轮事件"""
        # Windows和macOS
        if platform.system() == "Windows" or platform.system() == "Darwin":
            self.root.bind_all("<MouseWheel>", self.ik_mousewheel_handler)
        # Linux
        else:
            self.root.bind_all("<Button-4>", self.ik_mousewheel_handler)
            self.root.bind_all("<Button-5>", self.ik_mousewheel_handler)
    
    def refresh_ports(self):
        """刷新可用串口列表"""
        ports = self.serial_manager.get_available_ports()
        self.port_frame.set_ports(ports)
    
    def toggle_connection(self):
        """切换串口连接状态"""
        if self.serial_manager.is_connected:
            # 断开连接
            success, message = self.serial_manager.disconnect()
            if success:
                self.config_frame.set_connect_button_state(False)
                self.angle_frame.set_send_button_state(False)
                self.control_frame.set_buttons_state(False)
                self.handle_message(message, "信息")
                messagebox.showinfo("信息", message)
            else:
                self.handle_message(message, "错误")
                messagebox.showerror("错误", message)
        else:
            # 获取选择的串口
            port = self.port_frame.get_selected_port()
            if not port:
                self.handle_message("请选择一个有效的串口设备", "错误")
                messagebox.showerror("错误", "请选择一个有效的串口设备")
                return
            
            # 获取配置参数
            config = self.config_frame.get_config()
            
            # 尝试重置连接
            if self.serial_manager.current_port == port:
                success, message = self.serial_manager.reset_connection()
            else:
                # 新的连接
                success, message = self.serial_manager.connect(
                    port=port,
                    baud_rate=config['baud_rate'],
                    data_bits=config['data_bits'],
                    parity=config['parity'],
                    stop_bits=config['stop_bits'],
                    flow_control=config['flow_control']
                )
            
            if success:
                info, details = message
                self.config_frame.set_connect_button_state(True)
                self.angle_frame.set_send_button_state(True)
                self.control_frame.set_buttons_state(True)
                
                # 显示连接信息
                self.handle_message(info, "信息")
                self.handle_message(details, "信息")
                
                # 显示消息对话框
                messagebox.showinfo("连接成功", f"{info}\n{details}")
            else:
                self.handle_message(message, "错误")
                messagebox.showerror("连接错误", message)
    
    def send_command(self, command, encoding_type='string'):
        """发送控制命令"""
        if not self.serial_manager.is_connected:
            self.handle_message("请先连接串口", "错误")
            messagebox.showerror("错误", "请先连接串口")
            return
        
        # 根据命令字符串获取对应的命令代码
        command_codes = {
            "ENABLE": 0x01,    # 使能
            "DISABLE": 0x02,   # 取消使能
            "RELEASE": 0x03,   # 释放刹车
            "LOCK": 0x04,      # 锁止刹车
            "STOP": 0x05,      # 立刻停止
            "MOTION": 0x06     # 运动状态
        }
        
        command_code = command_codes.get(command)
        if command_code is None:
            self.handle_message(f"未知命令: {command}", "错误")
            messagebox.showerror("发送错误", f"未知命令: {command}")
            return
        
        # 发送命令，使用全零角度
        angles = [0.0] * 6
        
        # 获取当前运行模式
        run_mode = self.control_frame.get_run_mode()
        
        # 更新末端位置显示（全零位置）
        try:
            px, py, pz, A, B, C = self.kinematic_solver.get_end_position(angles)
            self.end_position_frame.update_theoretical_position(px, py, pz, A, B, C)
        except Exception as e:
            self.handle_message(f"计算末端位置时出错: {str(e)}", "错误")
        
        if encoding_type == 'hex':
            # 使用二进制格式发送
            success, message = self.serial_manager.send_formatted_angles(angles, command_code, run_mode)
            if success:
                self.handle_message(f"命令: {command} (0x{command_code:02X})", "参数")
                self.handle_message(f"运行模式: {run_mode}", "参数")
                self.handle_message(f"发送的十六进制数据: {message}", "发送")
        else:
            # 使用字符串格式发送
            success, message = self.serial_manager.send_formatted_string(angles, command_code, run_mode)
            if success:
                self.handle_message(f"命令: {command} (编码: 字符串)", "参数")
                self.handle_message(f"运行模式: {run_mode}", "参数")
                self.handle_message(f"发送的字符串数据: {message}", "发送")
        
        if not success:
            self.handle_message(message, "错误")
            messagebox.showerror("发送错误", message)
            return
        
        # 如果是运动状态命令(0x06)，则继续按频率发送
        if command_code == 0x06:
            # 获取发送频率
            _, _, frequency = self.angle_frame.get_curve_type()
            
            # 设置定时器，每隔frequency秒发送一次
            self.root.after(int(frequency * 1000), 
                          lambda: self._send_motion_status(angles, encoding_type, run_mode, frequency))
    
    def _send_motion_status(self, angles, encoding_type, run_mode, frequency):
        """定期发送运动状态"""
        if not self.serial_manager.is_connected:
            return
            
        # 发送数据
        if encoding_type == 'hex':
            success, message = self.serial_manager.send_formatted_angles(angles, 0x06, run_mode)
            if success:
                self.handle_message(f"发送的十六进制数据: {message}", "发送")
        else:
            success, message = self.serial_manager.send_formatted_string(angles, 0x06, run_mode)
            if success:
                self.handle_message(f"发送的字符串数据: {message}", "发送")
        
        if not success:
            self.handle_message(message, "错误")
            return
        
        # 继续设置下一次发送
        self.root.after(int(frequency * 1000), 
                       lambda: self._send_motion_status(angles, encoding_type, run_mode, frequency))
    
    def send_angles(self):
        """发送角度数据并生成曲线"""
        angles = self.angle_frame.get_angles()
        
        # 获取选择的加减速曲线类型和时间参数
        curve_type, duration, frequency = self.angle_frame.get_curve_type()
        
        # 获取编码类型
        encoding_type = self.control_frame.get_encoding_type()
        
        # 使用MOTION命令代码(0x06)发送角度数据
        command_code = 0x06  # 运动状态
        
        try:
            # 计算时间点
            num_points = int(duration / frequency) + 1
            times = [i * frequency for i in range(num_points)]
            
            # 获取当前角度值（假设为0，如果需要可以从其他地方获取）
            start_angles = [0.0] * 6
            
            # 计算每个关节的步进值
            steps = []
            for i in range(6):
                total_diff = angles[i] - start_angles[i]
                step = total_diff / (num_points - 1) if num_points > 1 else 0
                steps.append(step)
            
            # 记录发送参数
            self.handle_message(f"总时长: {duration}s, 发送频率: {frequency}s", "参数")
            self.handle_message(f"目标角度: {','.join([f'{angle:.6f}' for angle in angles])}", "参数")
            
            # 开始发送第一个点
            self._send_next_point(0, times, start_angles, steps, encoding_type, command_code, frequency)
            
        except Exception as e:
            error_msg = f"发送角度数据时出错: {str(e)}"
            self.handle_message(error_msg, "错误")
            messagebox.showerror("发送错误", error_msg)
    
    def _send_next_point(self, i, times, start_angles, steps, encoding_type, command_code, frequency):
        """递归发送下一个点"""
        try:
            if i >= len(times):
                return
            
            t = times[i]
            # 计算当前时间点的角度值
            current_angles = [start_angles[j] + steps[j] * i for j in range(6)]
            
            # 获取当前运行模式
            run_mode = self.control_frame.get_run_mode()
            
            # 计算并更新末端位置
            try:
                px, py, pz, A, B, C = self.kinematic_solver.get_end_position(current_angles)
                self.end_position_frame.update_theoretical_position(px, py, pz, A, B, C)
            except Exception as e:
                self.handle_message(f"计算末端位置时出错: {str(e)}", "错误")
            
            # 根据编码类型发送数据
            if encoding_type == 'hex':
                success, message = self.serial_manager.send_formatted_angles(current_angles, command_code)
                if success:
                    self.handle_message(f"时间点 {t:.2f}s - 发送的十六进制数据: {message}", "发送")
            else:
                success, message = self.serial_manager.send_formatted_string(current_angles, command_code, run_mode)
                if success:
                    self.handle_message(f"时间点 {t:.2f}s - 发送的字符串数据: {message}", "发送")
            
            if not success:
                self.handle_message(f"发送失败: {message}", "错误")
                messagebox.showerror("发送错误", message)
                return
            
            # 如果还有下一个点，安排发送下一个点
            if i < len(times) - 1:
                self.root.after(int(frequency * 1000), 
                              lambda: self._send_next_point(i + 1, times, start_angles, steps, 
                                                          encoding_type, command_code, frequency))
                
        except Exception as e:
            error_msg = f"发送数据点时出错: {str(e)}"
            self.handle_message(error_msg, "错误")
            messagebox.showerror("发送错误", error_msg)
    
    def generate_and_plot_curves(self, angles, curve_type, duration):
        """
        生成并绘制速度、加速度和位置曲线
        
        参数:
            angles: 目标角度列表
            curve_type: 曲线类型，"trapezoidal" 或 "s_curve"
            duration: 运动总时长
        """
        try:
            # 根据曲线类型调用相应的函数
            if curve_type == "trapezoidal":
                times, velocities, accelerations, positions = trapezoidal_velocity_planning(angles, duration)
            else:  # s_curve
                times, velocities, accelerations, positions = s_curve_velocity_planning(angles, duration)
            
            # 绘制曲线
            self.curve_frame.plot_curves(times, velocities, accelerations, positions, curve_type)
            
            # 记录生成曲线的信息
            self.handle_message(f"已生成{curve_type}曲线，总时间: {duration:.2f}秒", "信息")
            
        except Exception as e:
            error_msg = f"生成曲线时出错: {str(e)}"
            self.handle_message(error_msg, "错误")
            messagebox.showerror("曲线生成错误", error_msg)
    
    def convert_degrees_to_radians(self):
        """将度数转换为弧度"""
        try:
            # 获取当前角度值（假设为度数）
            angles = self.angle_frame.get_angles()
            
            # 转换为弧度
            radians = [degrees * (math.pi / 180.0) for degrees in angles]
            
            # 设置新值
            self.angle_frame.set_angles(radians)
            
            messagebox.showinfo("转换完成", "已将所有角度从度数转换为弧度")
        except Exception as e:
            error_msg = f"转换过程中出错: {str(e)}"
            self.handle_message(error_msg, "错误")
            messagebox.showerror("转换错误", error_msg)
    
    def set_angles_to_zero(self):
        """将所有角度值设为零"""
        self.angle_frame.set_angles([0.0] * 6)
    
    def clear_display(self, display_type="all"):
        """清除数据显示区域"""
        self.data_frame.clear_display(display_type)
    
    def handle_message(self, message, message_type="信息"):
        """处理消息回调"""
        # 在UI线程中安全地更新显示
        self.root.after(0, lambda: self.data_frame.append_message(message, message_type))
        
        # 如果是接收到的数据，尝试解析并更新实际姿态
        if message_type == "接收" and message.startswith("AA55"):
            self.end_position_frame.parse_and_update_actual_position(message, self.kinematic_solver)
        
        # 处理串口状态变化
        if "检测到串口已断开" in message:
            self.root.after(0, lambda: self.update_ui_state(False))
        elif "串口连接已重置" in message:
            self.root.after(0, lambda: self.update_ui_state(True))
            messagebox.showinfo("连接已重置", "串口连接已自动重置")
    
    def update_ui_state(self, is_connected):
        """
        更新UI状态
        
        参数:
            is_connected: 是否已连接
        """
        # 更新连接按钮状态
        self.config_frame.set_connect_button_state(is_connected)
        
        # 更新发送按钮状态
        self.angle_frame.set_send_button_state(is_connected)
        
        # 更新控制按钮状态
        self.control_frame.set_buttons_state(is_connected)
        
        # 如果状态发生变化且断开连接，显示提示消息
        if not is_connected:
            # 检查是否是因为设备重启导致的断开
            if not self.serial_manager.is_connected:
                messagebox.showinfo("连接断开", "串口已断开连接，请手动重新连接或等待自动重置")
    
    def on_closing(self):
        """处理窗口关闭事件"""
        try:
            # 断开串口连接
            if self.serial_manager.is_connected:
                self.serial_manager.disconnect()
            
            # 关闭所有matplotlib图形
            plt.close('all')
            
            # 销毁所有tkinter窗口
            self.root.destroy()
            
            # 确保退出
            self.root.quit()
            
            # 如果是在Windows上，尝试终止进程
            if platform.system() == 'Windows':
                import os
                os._exit(0)
        except Exception as e:
            print(f"关闭应用程序时出错: {str(e)}")
            # 强制退出
            import os
            os._exit(0)
    
    def run(self):
        """运行应用程序"""
        # 注册退出处理函数
        atexit.register(plt.close, 'all')
        
        self.root.mainloop()
    
    def calculate_inverse_kinematics(self):
        """计算逆运动学"""
        try:
            # 获取位置和欧拉角输入
            position, euler = self.ik_frame.get_position_and_euler()
            
            # 解包位置和欧拉角
            px, py, pz = position
            A, B, C = euler
            
            # 记录计算信息
            self.handle_message(f"逆运动学计算 - 位置: [{px}, {py}, {pz}], 欧拉角: [{A}, {B}, {C}]", "信息")
            
            # 调用逆运动学求解
            try:
                angles = self.kinematic_solver.inverse_kinematic(A, B, C, px, py, pz)
                
                # 设置计算结果
                self.ik_frame.set_result(angles, success=True)
                
                # 记录计算结果
                self.handle_message(f"逆运动学计算成功 - 关节角度: {[round(angle, 6) for angle in angles]}", "信息")
                
            except Exception as e:
                self.ik_frame.set_result([], success=False)
                error_msg = f"逆运动学计算失败: {str(e)}"
                self.handle_message(error_msg, "错误")
                messagebox.showerror("计算错误", error_msg)
        
        except Exception as e:
            error_msg = f"逆运动学计算过程中出错: {str(e)}"
            self.handle_message(error_msg, "错误")
            messagebox.showerror("计算错误", error_msg)
    
    def apply_ik_result_to_angles(self, angles):
        """将逆运动学计算结果应用到角度控制"""
        try:
            # 设置角度控制框架中的角度值
            self.angle_frame.set_angles(angles)
            
            # 记录应用信息
            self.handle_message(f"已将逆运动学计算结果应用到角度控制: {[round(angle, 6) for angle in angles]}", "信息")
            
            messagebox.showinfo("应用成功", "已将逆运动学计算结果应用到角度控制")
        
        except Exception as e:
            error_msg = f"应用逆运动学结果时出错: {str(e)}"
            self.handle_message(error_msg, "错误")
            messagebox.showerror("应用错误", error_msg) 
            