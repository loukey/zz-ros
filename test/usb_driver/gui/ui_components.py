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
        self.frame.pack(fill=tk.X, padx=10, pady=5)
        
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
        self.frame.pack(fill=tk.X, padx=10, pady=5)
        
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
        # 创建主框架
        main_frame = ttk.Frame(self.frame)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # 第一行：波特率和数据位
        row1_frame = ttk.Frame(main_frame)
        row1_frame.pack(fill=tk.X, pady=2)
        
        # 波特率
        ttk.Label(row1_frame, text="波特率:").pack(side=tk.LEFT, padx=(0, 5))
        self.baud_rate = tk.StringVar(value='9600')
        baud_combo = ttk.Combobox(row1_frame, textvariable=self.baud_rate, values=self.baud_rates, width=10)
        baud_combo.pack(side=tk.LEFT, padx=(0, 15))
        
        # 数据位
        ttk.Label(row1_frame, text="数据位:").pack(side=tk.LEFT, padx=(0, 5))
        self.data_bit = tk.StringVar(value='8')
        data_combo = ttk.Combobox(row1_frame, textvariable=self.data_bit, values=self.data_bits, width=5)
        data_combo.pack(side=tk.LEFT)
        
        # 第二行：校验位
        row2_frame = ttk.Frame(main_frame)
        row2_frame.pack(fill=tk.X, pady=2)
        
        ttk.Label(row2_frame, text="校验位:").pack(side=tk.LEFT, padx=(0, 5))
        self.parity = tk.StringVar(value='N')
        
        for text, value in self.parity_bits:
            ttk.Radiobutton(row2_frame, text=text, variable=self.parity, value=value).pack(side=tk.LEFT, padx=5)
        
        # 第三行：停止位
        row3_frame = ttk.Frame(main_frame)
        row3_frame.pack(fill=tk.X, pady=2)
        
        ttk.Label(row3_frame, text="停止位:").pack(side=tk.LEFT, padx=(0, 5))
        self.stop_bit = tk.DoubleVar(value=1)
        
        for text, value in self.stop_bits:
            ttk.Radiobutton(row3_frame, text=text, variable=self.stop_bit, value=value).pack(side=tk.LEFT, padx=5)
        
        # 第四行：流控制
        row4_frame = ttk.Frame(main_frame)
        row4_frame.pack(fill=tk.X, pady=2)
        
        ttk.Label(row4_frame, text="流控制:").pack(side=tk.LEFT, padx=(0, 5))
        self.flow_control_type = tk.StringVar(value='')
        
        for i, (text, value) in enumerate(self.flow_controls[:2]):  # 只显示前两个选项在第一行
            ttk.Radiobutton(row4_frame, text=text, variable=self.flow_control_type, value=value).pack(side=tk.LEFT, padx=5)
        
        # 第五行：流控制（续）
        row5_frame = ttk.Frame(main_frame)
        row5_frame.pack(fill=tk.X, pady=2)
        
        for i, (text, value) in enumerate(self.flow_controls[2:]):  # 显示后两个选项在第二行
            ttk.Radiobutton(row5_frame, text=text, variable=self.flow_control_type, value=value).pack(side=tk.LEFT, padx=5)
        
        # 第六行：连接按钮
        row6_frame = ttk.Frame(main_frame)
        row6_frame.pack(fill=tk.X, pady=5)
        
        # 添加连接按钮
        self.connect_button = ttk.Button(row6_frame, text="连接串口", command=connect_callback)
        self.connect_button.pack(side=tk.LEFT, padx=5)
    
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
        self.frame.pack(fill=tk.X, padx=10, pady=5)
        
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
        button_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # 创建每个命令按钮
        for i, (text, command) in enumerate(self.commands):
            # 创建一个闭包来保存当前命令值
            def create_command_callback(cmd=command):
                return lambda: send_command_callback(cmd)
            
            button = ttk.Button(
                button_frame, 
                text=text, 
                command=create_command_callback()
            )
            button.pack(side=tk.LEFT, padx=2, pady=5, fill=tk.X)
    
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
        self.frame.pack(fill=tk.X, padx=10, pady=5)
        
        # 添加角度输入控件
        self.angle_vars = []
        # 加减速曲线类型
        self.curve_type = tk.StringVar(value="trapezoidal")
        self._create_angle_widgets(send_callback, convert_callback, zero_callback)
    
    def _create_angle_widgets(self, send_callback, convert_callback, zero_callback):
        """创建角度输入控件"""
        # 主框架
        main_frame = ttk.Frame(self.frame)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # 第一行：角度1-3
        row1_frame = ttk.Frame(main_frame)
        row1_frame.pack(fill=tk.X, pady=(0, 5))
        
        # 角度1
        angle1_frame = ttk.Frame(row1_frame)
        angle1_frame.pack(side=tk.LEFT, fill=tk.X)
        ttk.Label(angle1_frame, text="角度1").pack(anchor="w")
        var1 = tk.DoubleVar(value=0.0)
        self.angle_vars.append(var1)
        ttk.Entry(angle1_frame, textvariable=var1).pack(fill=tk.X, padx=(0, 5))
        
        # 角度2
        angle2_frame = ttk.Frame(row1_frame)
        angle2_frame.pack(side=tk.LEFT, fill=tk.X)
        ttk.Label(angle2_frame, text="角度2").pack(anchor="w")
        var2 = tk.DoubleVar(value=0.0)
        self.angle_vars.append(var2)
        ttk.Entry(angle2_frame, textvariable=var2).pack(fill=tk.X, padx=(0, 5))
        
        # 角度3
        angle3_frame = ttk.Frame(row1_frame)
        angle3_frame.pack(side=tk.LEFT, fill=tk.X)
        ttk.Label(angle3_frame, text="角度3").pack(anchor="w")
        var3 = tk.DoubleVar(value=0.0)
        self.angle_vars.append(var3)
        ttk.Entry(angle3_frame, textvariable=var3).pack(fill=tk.X)
        
        # 第二行：角度4-6
        row2_frame = ttk.Frame(main_frame)
        row2_frame.pack(fill=tk.X, pady=(0, 5))
        
        # 角度4
        angle4_frame = ttk.Frame(row2_frame)
        angle4_frame.pack(side=tk.LEFT, fill=tk.X)
        ttk.Label(angle4_frame, text="角度4").pack(anchor="w")
        var4 = tk.DoubleVar(value=0.0)
        self.angle_vars.append(var4)
        ttk.Entry(angle4_frame, textvariable=var4).pack(fill=tk.X, padx=(0, 5))
        
        # 角度5
        angle5_frame = ttk.Frame(row2_frame)
        angle5_frame.pack(side=tk.LEFT, fill=tk.X)
        ttk.Label(angle5_frame, text="角度5").pack(anchor="w")
        var5 = tk.DoubleVar(value=0.0)
        self.angle_vars.append(var5)
        ttk.Entry(angle5_frame, textvariable=var5).pack(fill=tk.X, padx=(0, 5))
        
        # 角度6
        angle6_frame = ttk.Frame(row2_frame)
        angle6_frame.pack(side=tk.LEFT, fill=tk.X)
        ttk.Label(angle6_frame, text="角度6").pack(anchor="w")
        var6 = tk.DoubleVar(value=0.0)
        self.angle_vars.append(var6)
        ttk.Entry(angle6_frame, textvariable=var6).pack(fill=tk.X)
        
        # 第三行：曲线类型和按钮
        row3_frame = ttk.Frame(main_frame)
        row3_frame.pack(fill=tk.X, pady=(5, 0))
        
        # 曲线类型选择
        curve_frame = ttk.Frame(row3_frame)
        curve_frame.pack(side=tk.LEFT, fill=tk.X, pady=2, anchor="w")
        
        ttk.Label(curve_frame, text="曲线:").pack(side=tk.LEFT, padx=(0, 2))
        ttk.Radiobutton(
            curve_frame, 
            text="梯形", 
            variable=self.curve_type, 
            value="trapezoidal"
        ).pack(side=tk.LEFT, padx=2)
        ttk.Radiobutton(
            curve_frame, 
            text="S形", 
            variable=self.curve_type, 
            value="s_curve"
        ).pack(side=tk.LEFT, padx=2)
        
        # 按钮区域
        button_frame = ttk.Frame(main_frame)
        button_frame.pack(fill=tk.X, pady=5)
        
        # 发送按钮
        self.send_button = ttk.Button(button_frame, text="发送角度", command=send_callback, state=tk.DISABLED)
        self.send_button.pack(side=tk.LEFT, padx=2, fill=tk.X)
        
        # 转换按钮 (度到弧度)
        self.convert_button = ttk.Button(button_frame, text="度数转弧度", command=convert_callback)
        self.convert_button.pack(side=tk.LEFT, padx=2, fill=tk.X)
        
        # 归零按钮
        self.zero_button = ttk.Button(button_frame, text="全部归零", command=zero_callback)
        self.zero_button.pack(side=tk.LEFT, padx=2, fill=tk.X)
    
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
        self.frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # 添加数据显示控件
        self._create_data_display_widgets(clear_send_callback, clear_receive_callback, clear_all_callback)
    
    def _create_data_display_widgets(self, clear_send_callback, clear_receive_callback, clear_all_callback):
        """创建数据显示控件"""
        # 创建左右分栏框架
        display_pane = ttk.PanedWindow(self.frame, orient=tk.HORIZONTAL)
        display_pane.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)
        
        # 左侧发送数据显示框架
        send_frame = ttk.LabelFrame(display_pane, text="发送数据")
        display_pane.add(send_frame, weight=1)
        
        # 右侧接收数据显示框架
        receive_frame = ttk.LabelFrame(display_pane, text="接收数据")
        display_pane.add(receive_frame, weight=1)
        
        # 创建发送数据显示区域
        self.send_display = scrolledtext.ScrolledText(send_frame, wrap=tk.WORD, height=10, width=50)
        self.send_display.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)
        
        # 创建接收数据显示区域
        self.receive_display = scrolledtext.ScrolledText(receive_frame, wrap=tk.WORD, height=10, width=50)
        self.receive_display.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)
        
        # 控制按钮框架
        control_frame = ttk.Frame(self.frame)
        control_frame.pack(fill=tk.X, padx=2, pady=2, anchor="w")
        
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
        self.velocity_fig, self.velocity_ax = plt.subplots(figsize=(10, 6), dpi=100)
        self.velocity_canvas = FigureCanvasTkAgg(self.velocity_fig, master=self.velocity_frame)
        self.velocity_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.velocity_ax.set_title("速度曲线")
        self.velocity_ax.set_xlabel("时间 (秒)")
        self.velocity_ax.set_ylabel("速度 (弧度/秒)")
        self.velocity_ax.grid(True)
    
    def _init_position_plot(self):
        """初始化位置曲线图表"""
        self.position_fig, self.position_ax = plt.subplots(figsize=(10, 6), dpi=100)
        self.position_canvas = FigureCanvasTkAgg(self.position_fig, master=self.position_frame)
        self.position_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.position_ax.set_title("位置曲线")
        self.position_ax.set_xlabel("时间 (秒)")
        self.position_ax.set_ylabel("位置 (弧度)")
        self.position_ax.grid(True)
    
    def _init_acceleration_plot(self):
        """初始化加速度曲线图表"""
        self.acceleration_fig, self.acceleration_ax = plt.subplots(figsize=(10, 6), dpi=100)
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
        self.velocity_ax.plot(times, velocities, 'b-', linewidth=2)
        self.velocity_ax.set_xlabel("时间 (秒)")
        self.velocity_ax.set_ylabel("速度 (弧度/秒)")
        self.velocity_ax.grid(True)
        
        # 绘制加速度曲线
        self.acceleration_ax.plot(times, accelerations, 'r-', linewidth=2)
        self.acceleration_ax.set_xlabel("时间 (秒)")
        self.acceleration_ax.set_ylabel("加速度 (弧度/秒²)")
        self.acceleration_ax.grid(True)
        
        # 绘制位置曲线 - 为每个关节绘制一条曲线
        if len(positions.shape) > 1 and positions.shape[1] > 1:
            for i in range(positions.shape[1]):
                self.position_ax.plot(times, positions[:, i], label=f"关节{i+1}", linewidth=2)
            self.position_ax.legend()
        else:
            self.position_ax.plot(times, positions, 'g-', linewidth=2)
        
        self.position_ax.set_xlabel("时间 (秒)")
        self.position_ax.set_ylabel("位置 (弧度)")
        self.position_ax.grid(True)
        
        # 更新画布
        self.velocity_canvas.draw()
        self.position_canvas.draw()
        self.acceleration_canvas.draw()
        
        # 切换到速度曲线标签页
        self.notebook.select(0)


class InverseKinematicFrame:
    """逆运动学输入框架"""
    
    def __init__(self, parent, inverse_callback):
        """
        初始化逆运动学输入框架
        
        参数:
            parent: 父级容器
            inverse_callback: 逆运动学计算回调函数
        """
        self.frame = ttk.Frame(parent)
        # 不再需要设置固定宽度和高度，因为现在有整个标签页的空间
        # 不再需要禁止pack_propagate，让框架自然扩展
        
        # 创建输入变量
        self.position_vars = []  # X, Y, Z
        self.euler_vars = []     # A, B, C (欧拉角)
        
        # 创建主框架，使用网格布局管理所有子框架
        main_frame = ttk.Frame(self.frame)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # 创建输入区域框架
        input_frame = ttk.LabelFrame(main_frame, text="输入参数")
        input_frame.grid(row=0, column=0, sticky="nsew", padx=10, pady=10)
        
        # 配置输入区域的网格布局
        ik_frame_grid = ttk.Frame(input_frame)
        ik_frame_grid.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # 配置列权重，使内容均匀分布
        for i in range(3):
            ik_frame_grid.columnconfigure(i, weight=1)
        
        # 第一行标签 - 位置
        ttk.Label(ik_frame_grid, text="X 位置").grid(row=0, column=0, padx=10)
        ttk.Label(ik_frame_grid, text="Y 位置").grid(row=0, column=1, padx=10)
        ttk.Label(ik_frame_grid, text="Z 位置").grid(row=0, column=2, padx=10)
        
        # 第一行输入框 - 位置 - 增加宽度
        for i in range(3):
            var = tk.DoubleVar(value=0.0)
            self.position_vars.append(var)
            entry = ttk.Entry(ik_frame_grid, textvariable=var, width=10)
            entry.grid(row=1, column=i, padx=10, pady=10, sticky="w")
        
        # 第二行标签 - 欧拉角
        ttk.Label(ik_frame_grid, text="A 角度").grid(row=2, column=0, padx=10)
        ttk.Label(ik_frame_grid, text="B 角度").grid(row=2, column=1, padx=10)
        ttk.Label(ik_frame_grid, text="C 角度").grid(row=2, column=2, padx=10)
        
        # 第二行输入框 - 欧拉角 - 增加宽度
        for i in range(3):
            var = tk.DoubleVar(value=0.0)
            self.euler_vars.append(var)
            entry = ttk.Entry(ik_frame_grid, textvariable=var, width=10)
            entry.grid(row=3, column=i, padx=10, pady=10, sticky="w")
        
        # 计算按钮 - 靠左显示
        button_frame = ttk.Frame(ik_frame_grid)
        button_frame.grid(row=4, column=0, columnspan=3, pady=15, sticky="w")
        
        self.calculate_button = ttk.Button(button_frame, text="计算逆运动学", command=inverse_callback, width=15)
        self.calculate_button.pack(side=tk.LEFT, pady=5)
        
        # 创建结果显示区域框架
        result_frame = ttk.LabelFrame(main_frame, text="计算结果")
        result_frame.grid(row=1, column=0, sticky="nsew", padx=10, pady=10)
        
        # 配置结果区域的内部布局
        result_inner_frame = ttk.Frame(result_frame)
        result_inner_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # 结果标签
        self.result_label = ttk.Label(result_inner_frame, text="请输入位置和欧拉角后点击计算", font=("Arial", 11))
        self.result_label.pack(padx=10, pady=10, anchor="w")
        
        # 结果值显示 - 两行显示
        self.result_values_frame = ttk.Frame(result_inner_frame)
        self.result_values_frame.pack(padx=10, pady=5, fill=tk.X)
        
        # 第一行角度值（角度1-3）
        self.result_values_row1 = ttk.Label(self.result_values_frame, text="", font=("Arial", 10))
        self.result_values_row1.pack(padx=10, pady=5, anchor="w")
        
        # 第二行角度值（角度4-6）
        self.result_values_row2 = ttk.Label(self.result_values_frame, text="", font=("Arial", 10))
        self.result_values_row2.pack(padx=10, pady=5, anchor="w")
        
        # 应用到角度控制按钮
        self.apply_button = ttk.Button(result_inner_frame, text="应用到角度控制", command=lambda: None, state=tk.DISABLED, width=15)
        self.apply_button.pack(padx=10, pady=10, anchor="w")
        
        # 配置主框架的行列权重，使两个区域大小一致
        main_frame.columnconfigure(0, weight=1)
        main_frame.rowconfigure(0, weight=1)
        main_frame.rowconfigure(1, weight=1)
    
    def get_position_and_euler(self):
        """
        获取当前位置和欧拉角值
        
        返回:
            position: 位置值列表 [X, Y, Z]
            euler: 欧拉角值列表 [A, B, C]
        """
        position = [var.get() for var in self.position_vars]
        euler = [var.get() for var in self.euler_vars]
        return position, euler
    
    def set_result(self, angles, success=True):
        """
        设置计算结果
        
        参数:
            angles: 计算得到的关节角度列表
            success: 计算是否成功
        """
        if success:
            self.result_label.config(text="计算成功，关节角度值(弧度):")
            
            # 将角度分为两行显示
            if len(angles) >= 6:
                # 第一行显示角度1-3
                row1_text = f"角度1: {angles[0]:.6f}, 角度2: {angles[1]:.6f}, 角度3: {angles[2]:.6f}"
                # 第二行显示角度4-6
                row2_text = f"角度4: {angles[3]:.6f}, 角度5: {angles[4]:.6f}, 角度6: {angles[5]:.6f}"
                
                self.result_values_row1.config(text=row1_text)
                self.result_values_row2.config(text=row2_text)
            else:
                # 如果角度数量不足6个，则全部显示在第一行
                self.result_values_row1.config(text=", ".join([f"{angle:.6f}" for angle in angles]))
                self.result_values_row2.config(text="")
            
            self.apply_button.config(state=tk.NORMAL)
            # 存储计算结果以便应用按钮使用
            self.calculated_angles = angles
        else:
            self.result_label.config(text="计算失败，请检查输入值")
            self.result_values_row1.config(text="")
            self.result_values_row2.config(text="")
            self.apply_button.config(state=tk.DISABLED)
    
    def set_apply_callback(self, callback):
        """
        设置应用按钮回调函数
        
        参数:
            callback: 应用按钮回调函数
        """
        self.apply_button.config(command=lambda: callback(self.calculated_angles)) 