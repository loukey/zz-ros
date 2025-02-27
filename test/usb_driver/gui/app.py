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

from .serial_manager import SerialManager
from .ui_components import PortSelectionFrame, SerialConfigFrame, ControlButtonsFrame, AngleControlFrame, DataDisplayFrame, CurvePlotFrame


class USBSerialApp:
    """USB串口通信应用程序类"""
    
    def __init__(self, root, title="镇中科技串口测试", version="v0.0.2"):
        """
        初始化应用程序
        
        参数:
            root: Tkinter根窗口
            title: 应用程序标题
            version: 应用程序版本
        """
        self.root = root
        self.root.title(f"{title}{version}")
        self.root.geometry("800x1200")  # 增加窗口大小以适应曲线显示
        
        # 创建串口管理器
        self.serial_manager = SerialManager(message_callback=self.handle_message)
        
        # 创建UI组件
        self.create_ui_components()
        
        # 刷新串口列表
        self.refresh_ports()
        
        # 设置窗口关闭事件处理
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
    
    def create_ui_components(self):
        """创建所有UI组件"""
        # 创建控制面板框架
        control_panel = ttk.Frame(self.root)
        control_panel.pack(side=tk.TOP, fill=tk.X)
        
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
        
        # 创建标签页控件
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # 创建数据显示标签页
        data_tab = ttk.Frame(self.notebook)
        self.notebook.add(data_tab, text="数据显示")
        
        # 创建曲线显示标签页
        curve_tab = ttk.Frame(self.notebook)
        self.notebook.add(curve_tab, text="曲线显示")
        
        # 在数据显示标签页中创建数据显示框架
        self.data_frame = DataDisplayFrame(
            data_tab,
            lambda: self.clear_display("send"),
            lambda: self.clear_display("receive"),
            lambda: self.clear_display("all")
        )
        
        # 在曲线显示标签页中创建曲线绘制框架
        self.curve_frame = CurvePlotFrame(curve_tab)
    
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
            
            # 连接串口
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
    
    def send_command(self, command):
        """
        发送控制命令
        
        参数:
            command: 要发送的命令字符串
        """
        if not self.serial_manager.is_connected:
            self.handle_message("请先连接串口", "错误")
            messagebox.showerror("错误", "请先连接串口")
            return
        
        # 发送命令
        success, message = self.serial_manager.send_data(command + "\n")
        
        if success:
            self.handle_message(f"命令: {command}", "发送")
        else:
            self.handle_message(message, "错误")
            messagebox.showerror("发送错误", message)
    
    def send_angles(self):
        """发送角度数据并生成曲线"""
        angles = self.angle_frame.get_angles()
        
        # 获取选择的加减速曲线类型
        curve_type = self.angle_frame.get_curve_type()
        
        # 构建发送数据，包含曲线类型
        data = f"{curve_type}:" + ",".join([f"{angle:.6f}" for angle in angles])
        
        # 发送数据
        success, message = self.serial_manager.send_data(data + "\n")
        
        if success:
            # 记录发送的内容
            self.handle_message(f"曲线类型: {curve_type}, 角度: {','.join([f'{angle:.6f}' for angle in angles])}", "发送")
            
            # 生成并显示曲线
            self.generate_and_plot_curves(angles, curve_type)
            
            # 切换到曲线显示标签页
            self.notebook.select(1)  # 索引1是曲线显示标签页
        else:
            self.handle_message(message, "错误")
            messagebox.showerror("发送错误", message)
    
    def generate_and_plot_curves(self, angles, curve_type):
        """
        生成并绘制速度、加速度和位置曲线
        
        参数:
            angles: 目标角度列表
            curve_type: 曲线类型，"trapezoidal" 或 "s_curve"
        """
        try:
            # 根据曲线类型调用相应的函数
            if curve_type == "trapezoidal":
                times, velocities, accelerations, positions = trapezoidal_velocity_planning(angles)
            else:  # s_curve
                times, velocities, accelerations, positions = s_curve_velocity_planning(angles)
            
            # 绘制曲线
            self.curve_frame.plot_curves(times, velocities, accelerations, positions, curve_type)
            
            # 记录生成曲线的信息
            self.handle_message(f"已生成{curve_type}曲线，总时间: {times[-1]:.2f}秒", "信息")
            
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