"""
UI组件模块
包含各种UI元素的创建和管理
"""

import tkinter as tk
from tkinter import ttk, scrolledtext
from datetime import datetime
import matplotlib
# 设置matplotlib后端为Agg，避免交互式后端可能导致的进程残留问题
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
import platform

# 配置matplotlib支持中文显示
if platform.system() == 'Windows':
    # Windows系统使用微软雅黑字体
    matplotlib.rcParams['font.family'] = ['Microsoft YaHei']
else:
    # 其他系统尝试使用其他中文字体
    matplotlib.rcParams['font.family'] = ['SimHei', 'WenQuanYi Micro Hei', 'STXihei', 'sans-serif']
matplotlib.rcParams['axes.unicode_minus'] = False  # 正确显示负号


class PortSelectionFrame:
    """串口选择框架"""
    
    def __init__(self, parent, refresh_callback):
        """
        初始化串口选择框架
        
        参数:
            parent: 父级容器
            refresh_callback: 刷新按钮回调函数
        """
        self.frame = ttk.Frame(parent)
        self.frame.pack(fill=tk.X, padx=10, pady=10)
        
        # 创建串口选择下拉列表
        ttk.Label(self.frame, text="选择串口:").pack(side=tk.LEFT, padx=(0, 5))
        self.port_combobox = ttk.Combobox(self.frame, width=30)
        self.port_combobox.pack(side=tk.LEFT, padx=(0, 10))
        
        # 创建刷新按钮
        self.refresh_button = ttk.Button(self.frame, text="刷新", command=refresh_callback)
        self.refresh_button.pack(side=tk.LEFT)
    
    def set_ports(self, port_list):
        """
        设置可用串口列表
        
        参数:
            port_list: 包含(port, description)元组的列表
        """
        formatted_ports = []
        for port, desc in port_list:
            formatted_ports.append(f"{port} - {desc}")
        
        # 清空并更新下拉列表
        self.port_combobox['values'] = formatted_ports
        
        # 如果有可用串口，默认选择第一个
        if formatted_ports:
            self.port_combobox.current(0)
        else:
            self.port_combobox.set("未找到可用串口")
    
    def get_selected_port(self):
        """
        获取当前选择的串口设备名称
        
        返回:
            port: 串口设备名称，如果未选择则返回None
        """
        selection = self.port_combobox.get()
        if selection and " - " in selection:
            return selection.split(" - ")[0]
        return None


class SerialConfigFrame:
    """串口参数配置框架"""
    
    def __init__(self, parent, connect_callback):
        """
        初始化串口参数配置框架
        
        参数:
            parent: 父级容器
            connect_callback: 连接按钮回调函数
        """
        self.frame = ttk.LabelFrame(parent, text="串口通信参数配置")
        self.frame.pack(fill=tk.X, padx=10, pady=10)
        
        # 常用波特率列表
        self.baud_rates = ['9600', '19200', '38400', '57600', '115200', '230400', '460800', '921600']
        # 数据位选项
        self.data_bits = ['5', '6', '7', '8']
        # 校验位选项
        self.parity_bits = [('无', 'N'), ('奇校验', 'O'), ('偶校验', 'E'), ('标记', 'M'), ('空格', 'S')]
        # 停止位选项
        self.stop_bits = [('1', 1), ('1.5', 1.5), ('2', 2)]
        # 流控制选项
        self.flow_controls = [('无', ''), ('XON/XOFF (软件)', 'xonxoff'), 
                             ('RTS/CTS (硬件)', 'rtscts'), ('DSR/DTR (硬件)', 'dsrdtr')]
        
        # 创建参数配置控件
        self._create_config_widgets(connect_callback)
    
    def _create_config_widgets(self, connect_callback):
        """创建串口参数配置控件"""
        # 参数的行配置
        row = 0
        padding = 5
        
        # 波特率
        ttk.Label(self.frame, text="波特率:").grid(row=row, column=0, sticky=tk.W, padx=padding, pady=padding)
        self.baud_rate = tk.StringVar(value='9600')
        baud_combo = ttk.Combobox(self.frame, textvariable=self.baud_rate, values=self.baud_rates, width=15)
        baud_combo.grid(row=row, column=1, sticky=tk.W, padx=padding, pady=padding)
        
        # 数据位
        ttk.Label(self.frame, text="数据位:").grid(row=row, column=2, sticky=tk.W, padx=padding, pady=padding)
        self.data_bit = tk.StringVar(value='8')
        data_combo = ttk.Combobox(self.frame, textvariable=self.data_bit, values=self.data_bits, width=10)
        data_combo.grid(row=row, column=3, sticky=tk.W, padx=padding, pady=padding)
        
        row += 1
        
        # 校验位
        ttk.Label(self.frame, text="校验位:").grid(row=row, column=0, sticky=tk.W, padx=padding, pady=padding)
        self.parity = tk.StringVar(value='N')
        parity_frame = ttk.Frame(self.frame)
        parity_frame.grid(row=row, column=1, columnspan=3, sticky=tk.W, padx=padding, pady=padding)
        
        for i, (text, value) in enumerate(self.parity_bits):
            ttk.Radiobutton(parity_frame, text=text, variable=self.parity, value=value).grid(row=0, column=i, padx=5)
        
        row += 1
        
        # 停止位
        ttk.Label(self.frame, text="停止位:").grid(row=row, column=0, sticky=tk.W, padx=padding, pady=padding)
        self.stop_bit = tk.DoubleVar(value=1)
        stop_frame = ttk.Frame(self.frame)
        stop_frame.grid(row=row, column=1, columnspan=3, sticky=tk.W, padx=padding, pady=padding)
        
        for i, (text, value) in enumerate(self.stop_bits):
            ttk.Radiobutton(stop_frame, text=text, variable=self.stop_bit, value=value).grid(row=0, column=i, padx=5)
        
        row += 1
        
        # 流控制
        ttk.Label(self.frame, text="流控制:").grid(row=row, column=0, sticky=tk.W, padx=padding, pady=padding)
        self.flow_control_type = tk.StringVar(value='')
        flow_frame = ttk.Frame(self.frame)
        flow_frame.grid(row=row, column=1, columnspan=3, sticky=tk.W, padx=padding, pady=padding)
        
        for i, (text, value) in enumerate(self.flow_controls):
            ttk.Radiobutton(flow_frame, text=text, variable=self.flow_control_type, value=value).grid(row=i//2, column=i%2, sticky=tk.W, padx=5, pady=2)
        
        row += 1
        
        # 添加连接按钮
        self.connect_button = ttk.Button(self.frame, text="连接串口", command=connect_callback)
        self.connect_button.grid(row=row, column=0, columnspan=4, pady=15)
    
    def get_config(self):
        """
        获取当前配置参数
        
        返回:
            config: 包含所有配置参数的字典
        """
        return {
            'baud_rate': int(self.baud_rate.get()),
            'data_bits': int(self.data_bit.get()),
            'parity': self.parity.get(),
            'stop_bits': self.stop_bit.get(),
            'flow_control': self.flow_control_type.get()
        }
    
    def set_connect_button_state(self, is_connected):
        """
        设置连接按钮状态
        
        参数:
            is_connected: 是否已连接
        """
        if is_connected:
            self.connect_button.config(text="断开连接")
        else:
            self.connect_button.config(text="连接串口")


class ControlButtonsFrame:
    """控制按钮框架"""
    
    def __init__(self, parent, send_command_callback):
        """
        初始化控制按钮框架
        
        参数:
            parent: 父级容器
            send_command_callback: 发送命令回调函数
        """
        self.frame = ttk.LabelFrame(parent, text="控制命令")
        self.frame.pack(fill=tk.X, padx=10, pady=10)
        
        # 定义控制命令按钮
        self.commands = [
            ("使能", "ENABLE"),
            ("取消使能", "DISABLE"),
            ("释放刹车", "RELEASE"),
            ("锁止刹车", "LOCK"),
            ("立刻停止", "STOP")
        ]
        
        # 创建按钮
        self._create_control_buttons(send_command_callback)
    
    def _create_control_buttons(self, send_command_callback):
        """创建控制按钮"""
        button_frame = ttk.Frame(self.frame)
        button_frame.pack(fill=tk.X, padx=10, pady=10)
        
        # 创建每个命令按钮
        for i, (text, command) in enumerate(self.commands):
            # 创建一个闭包来保存当前命令值
            def create_command_callback(cmd=command):
                return lambda: send_command_callback(cmd)
            
            button = ttk.Button(
                button_frame, 
                text=text, 
                command=create_command_callback(),
                width=10
            )
            button.grid(row=0, column=i, padx=5, pady=5)
    
    def set_buttons_state(self, is_connected):
        """
        设置按钮状态
        
        参数:
            is_connected: 是否已连接
        """
        state = tk.NORMAL if is_connected else tk.DISABLED
        
        # 设置所有按钮的状态
        for child in self.frame.winfo_children():
            for button in child.winfo_children():
                if isinstance(button, ttk.Button):
                    button.config(state=state)


class AngleControlFrame:
    """角度控制框架"""
    
    def __init__(self, parent, send_callback, convert_callback, zero_callback):
        """
        初始化角度控制框架
        
        参数:
            parent: 父级容器
            send_callback: 发送按钮回调函数
            convert_callback: 转换按钮回调函数
            zero_callback: 归零按钮回调函数
        """
        self.frame = ttk.LabelFrame(parent, text="角度控制 (弧度值)")
        self.frame.pack(fill=tk.X, padx=10, pady=10)
        
        # 添加角度输入控件
        self.angle_vars = []
        # 加减速曲线类型
        self.curve_type = tk.StringVar(value="trapezoidal")
        self._create_angle_widgets(send_callback, convert_callback, zero_callback)
    
    def _create_angle_widgets(self, send_callback, convert_callback, zero_callback):
        """创建角度输入控件"""
        angle_frame_grid = ttk.Frame(self.frame)
        angle_frame_grid.pack(fill=tk.X, padx=10, pady=10)
        
        # 第一行标题
        ttk.Label(angle_frame_grid, text="角度1").grid(row=0, column=0, padx=5)
        ttk.Label(angle_frame_grid, text="角度2").grid(row=0, column=1, padx=5)
        ttk.Label(angle_frame_grid, text="角度3").grid(row=0, column=2, padx=5)
        
        # 第一行输入框
        for i in range(3):
            var = tk.DoubleVar(value=0.0)
            self.angle_vars.append(var)
            entry = ttk.Entry(angle_frame_grid, textvariable=var, width=10)
            entry.grid(row=1, column=i, padx=5, pady=5)
        
        # 第二行标题
        ttk.Label(angle_frame_grid, text="角度4").grid(row=2, column=0, padx=5)
        ttk.Label(angle_frame_grid, text="角度5").grid(row=2, column=1, padx=5)
        ttk.Label(angle_frame_grid, text="角度6").grid(row=2, column=2, padx=5)
        
        # 第二行输入框
        for i in range(3):
            var = tk.DoubleVar(value=0.0)
            self.angle_vars.append(var)
            entry = ttk.Entry(angle_frame_grid, textvariable=var, width=10)
            entry.grid(row=3, column=i, padx=5, pady=5)
        
        # 加减速曲线类型选择
        curve_frame = ttk.Frame(angle_frame_grid)
        curve_frame.grid(row=4, column=0, columnspan=3, sticky=tk.W, padx=5, pady=5)
        
        ttk.Label(curve_frame, text="加减速曲线:").pack(side=tk.LEFT, padx=(0, 5))
        
        # 梯形加速度曲线选项
        ttk.Radiobutton(
            curve_frame, 
            text="梯形加速度曲线", 
            variable=self.curve_type, 
            value="trapezoidal"
        ).pack(side=tk.LEFT, padx=5)
        
        # S形加减速曲线选项
        ttk.Radiobutton(
            curve_frame, 
            text="S形加减速曲线", 
            variable=self.curve_type, 
            value="s_curve"
        ).pack(side=tk.LEFT, padx=5)
        
        # 控制按钮 - 使用网格布局而不是pack布局，以便与上方输入框对齐
        # 发送按钮
        self.send_button = ttk.Button(angle_frame_grid, text="发送角度", command=send_callback, state=tk.DISABLED, width=10)
        self.send_button.grid(row=5, column=0, padx=5, pady=10)
        
        # 转换按钮 (度到弧度)
        self.convert_button = ttk.Button(angle_frame_grid, text="度数转弧度", command=convert_callback, width=10)
        self.convert_button.grid(row=5, column=1, padx=5, pady=10)
        
        # 归零按钮
        self.zero_button = ttk.Button(angle_frame_grid, text="全部归零", command=zero_callback, width=10)
        self.zero_button.grid(row=5, column=2, padx=5, pady=10)
    
    def get_angles(self):
        """
        获取当前角度值
        
        返回:
            angles: 角度值列表
        """
        return [var.get() for var in self.angle_vars]
    
    def get_curve_type(self):
        """
        获取当前选择的加减速曲线类型
        
        返回:
            curve_type: 曲线类型字符串，"trapezoidal" 或 "s_curve"
        """
        return self.curve_type.get()
    
    def set_angles(self, angles):
        """
        设置角度值
        
        参数:
            angles: 角度值列表
        """
        for i, angle in enumerate(angles):
            if i < len(self.angle_vars):
                self.angle_vars[i].set(angle)
    
    def set_send_button_state(self, is_connected):
        """
        设置发送按钮状态
        
        参数:
            is_connected: 是否已连接
        """
        if is_connected:
            self.send_button.config(state=tk.NORMAL)
        else:
            self.send_button.config(state=tk.DISABLED)


class DataDisplayFrame:
    """数据显示框架"""
    
    def __init__(self, parent, clear_send_callback, clear_receive_callback, clear_all_callback):
        """
        初始化数据显示框架
        
        参数:
            parent: 父级容器
            clear_send_callback: 清除发送显示回调函数
            clear_receive_callback: 清除接收显示回调函数
            clear_all_callback: 清除全部显示回调函数
        """
        self.frame = ttk.LabelFrame(parent, text="接收数据显示")
        self.frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # 添加数据显示控件
        self._create_data_display_widgets(clear_send_callback, clear_receive_callback, clear_all_callback)
    
    def _create_data_display_widgets(self, clear_send_callback, clear_receive_callback, clear_all_callback):
        """创建数据显示控件"""
        # 创建左右分栏框架
        display_pane = ttk.PanedWindow(self.frame, orient=tk.HORIZONTAL)
        display_pane.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # 左侧发送数据显示框架
        send_frame = ttk.LabelFrame(display_pane, text="发送数据")
        display_pane.add(send_frame, weight=1)
        
        # 右侧接收数据显示框架
        receive_frame = ttk.LabelFrame(display_pane, text="接收数据")
        display_pane.add(receive_frame, weight=1)
        
        # 创建发送数据显示区域
        self.send_display = scrolledtext.ScrolledText(send_frame, wrap=tk.WORD, height=10)
        self.send_display.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # 创建接收数据显示区域
        self.receive_display = scrolledtext.ScrolledText(receive_frame, wrap=tk.WORD, height=10)
        self.receive_display.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # 控制按钮框架
        control_frame = ttk.Frame(self.frame)
        control_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # 清除发送按钮
        self.clear_send_button = ttk.Button(control_frame, text="清除发送显示", command=clear_send_callback)
        self.clear_send_button.pack(side=tk.LEFT, padx=5)
        
        # 清除接收按钮
        self.clear_receive_button = ttk.Button(control_frame, text="清除接收显示", command=clear_receive_callback)
        self.clear_receive_button.pack(side=tk.LEFT, padx=5)
        
        # 清除全部按钮
        self.clear_all_button = ttk.Button(control_frame, text="清除全部", command=clear_all_callback)
        self.clear_all_button.pack(side=tk.LEFT, padx=5)
        
        # 自动滚动选项
        self.autoscroll_var = tk.BooleanVar(value=True)
        self.autoscroll_check = ttk.Checkbutton(control_frame, text="自动滚动", variable=self.autoscroll_var)
        self.autoscroll_check.pack(side=tk.LEFT, padx=5)
        
        # 显示时间戳选项
        self.timestamp_var = tk.BooleanVar(value=True)
        self.timestamp_check = ttk.Checkbutton(control_frame, text="显示时间戳", variable=self.timestamp_var)
        self.timestamp_check.pack(side=tk.LEFT, padx=5)
    
    def append_message(self, message, message_type="信息"):
        """
        向数据显示区域添加消息
        
        参数:
            message: 要显示的消息
            message_type: 消息类型，用于确定显示在哪个区域，可选值为 "发送", "接收", "信息", "错误"
        """
        # 如果需要时间戳
        if self.timestamp_var.get():
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            prefix = f"[{timestamp}] "
        else:
            prefix = ""
        
        # 根据消息类型选择显示区域
        if message_type.lower() in ["发送", "send"]:
            display = self.send_display
            prefix += "[发送] "
        elif message_type.lower() in ["接收", "receive"]:
            display = self.receive_display
            prefix += "[接收] "
        elif message_type.lower() in ["错误", "error"]:
            # 错误信息在两个区域都显示
            error_prefix = prefix + "[错误] "
            self.send_display.insert(tk.END, error_prefix + message + "\n")
            self.receive_display.insert(tk.END, error_prefix + message + "\n")
            
            # 如果启用了自动滚动
            if self.autoscroll_var.get():
                self.send_display.see(tk.END)
                self.receive_display.see(tk.END)
            return
        else:
            # 普通信息在两个区域都显示
            info_prefix = prefix + "[信息] "
            self.send_display.insert(tk.END, info_prefix + message + "\n")
            self.receive_display.insert(tk.END, info_prefix + message + "\n")
            
            # 如果启用了自动滚动
            if self.autoscroll_var.get():
                self.send_display.see(tk.END)
                self.receive_display.see(tk.END)
            return
        
        # 添加消息到选定的显示区域
        display.insert(tk.END, prefix + message + "\n")
        
        # 如果启用了自动滚动
        if self.autoscroll_var.get():
            display.see(tk.END)
    
    def clear_display(self, display_type="all"):
        """
        清除数据显示区域
        
        参数:
            display_type: 要清除的显示区域类型，可选值为 "send", "receive", "all"
        """
        if display_type in ["send", "all"]:
            self.send_display.delete(1.0, tk.END)
        
        if display_type in ["receive", "all"]:
            self.receive_display.delete(1.0, tk.END)


class CurvePlotFrame:
    """曲线绘制框架"""
    
    def __init__(self, parent):
        """
        初始化曲线绘制框架
        
        参数:
            parent: 父级容器
        """
        self.frame = ttk.Frame(parent)
        self.frame.pack(fill=tk.BOTH, expand=True)
        
        # 创建标签页控件
        self.notebook = ttk.Notebook(self.frame)
        self.notebook.pack(fill=tk.BOTH, expand=True)
        
        # 创建速度曲线标签页
        self.velocity_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.velocity_frame, text="速度曲线")
        
        # 创建位置曲线标签页
        self.position_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.position_frame, text="位置曲线")
        
        # 创建加速度曲线标签页
        self.acceleration_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.acceleration_frame, text="加速度曲线")
        
        # 初始化图表
        self._init_velocity_plot()
        self._init_position_plot()
        self._init_acceleration_plot()
    
    def _init_velocity_plot(self):
        """初始化速度曲线图表"""
        self.velocity_fig, self.velocity_ax = plt.subplots(figsize=(8, 5), dpi=100)
        self.velocity_canvas = FigureCanvasTkAgg(self.velocity_fig, master=self.velocity_frame)
        self.velocity_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.velocity_ax.set_title("速度曲线")
        self.velocity_ax.set_xlabel("时间 (秒)")
        self.velocity_ax.set_ylabel("速度 (弧度/秒)")
        self.velocity_ax.grid(True)
    
    def _init_position_plot(self):
        """初始化位置曲线图表"""
        self.position_fig, self.position_ax = plt.subplots(figsize=(8, 5), dpi=100)
        self.position_canvas = FigureCanvasTkAgg(self.position_fig, master=self.position_frame)
        self.position_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.position_ax.set_title("位置曲线")
        self.position_ax.set_xlabel("时间 (秒)")
        self.position_ax.set_ylabel("位置 (弧度)")
        self.position_ax.grid(True)
    
    def _init_acceleration_plot(self):
        """初始化加速度曲线图表"""
        self.acceleration_fig, self.acceleration_ax = plt.subplots(figsize=(8, 5), dpi=100)
        self.acceleration_canvas = FigureCanvasTkAgg(self.acceleration_fig, master=self.acceleration_frame)
        self.acceleration_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.acceleration_ax.set_title("加速度曲线")
        self.acceleration_ax.set_xlabel("时间 (秒)")
        self.acceleration_ax.set_ylabel("加速度 (弧度/秒²)")
        self.acceleration_ax.grid(True)
    
    def plot_curves(self, times, velocities, accelerations, positions, curve_type):
        """
        绘制曲线
        
        参数:
            times: 时间点序列
            velocities: 速度序列
            accelerations: 加速度序列
            positions: 位置序列
            curve_type: 曲线类型，"trapezoidal" 或 "s_curve"
        """
        # 清除旧图表
        self.velocity_ax.clear()
        self.position_ax.clear()
        self.acceleration_ax.clear()
        
        # 设置标题
        curve_name = "梯形加速度曲线" if curve_type == "trapezoidal" else "S形加减速曲线"
        self.velocity_ax.set_title(f"速度曲线 - {curve_name}")
        self.position_ax.set_title(f"位置曲线 - {curve_name}")
        self.acceleration_ax.set_title(f"加速度曲线 - {curve_name}")
        
        # 绘制速度曲线
        self.velocity_ax.plot(times, velocities, 'b-')
        self.velocity_ax.set_xlabel("时间 (秒)")
        self.velocity_ax.set_ylabel("速度 (弧度/秒)")
        self.velocity_ax.grid(True)
        
        # 绘制加速度曲线
        self.acceleration_ax.plot(times, accelerations, 'r-')
        self.acceleration_ax.set_xlabel("时间 (秒)")
        self.acceleration_ax.set_ylabel("加速度 (弧度/秒²)")
        self.acceleration_ax.grid(True)
        
        # 绘制位置曲线 - 为每个关节绘制一条曲线
        if len(positions.shape) > 1 and positions.shape[1] > 1:
            for i in range(positions.shape[1]):
                self.position_ax.plot(times, positions[:, i], label=f"关节{i+1}")
            self.position_ax.legend()
        else:
            self.position_ax.plot(times, positions, 'g-')
        
        self.position_ax.set_xlabel("时间 (秒)")
        self.position_ax.set_ylabel("位置 (弧度)")
        self.position_ax.grid(True)
        
        # 更新画布
        self.velocity_canvas.draw()
        self.position_canvas.draw()
        self.acceleration_canvas.draw()
        
        # 切换到速度曲线标签页
        self.notebook.select(0) 