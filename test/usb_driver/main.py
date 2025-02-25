import serial
import serial.tools.list_ports
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import threading
import time
from datetime import datetime

class USBSerialUI:
    def __init__(self, root):
        self.root = root
        self.root.title("镇中科技串口测试v0.0.1")
        self.root.geometry("600x800")  # 增加窗口高度以容纳新控件
        
        # 创建顶部框架
        self.top_frame = ttk.Frame(root)
        self.top_frame.pack(fill=tk.X, padx=10, pady=10)
        
        # 创建串口选择下拉列表
        ttk.Label(self.top_frame, text="选择串口:").pack(side=tk.LEFT, padx=(0, 5))
        self.port_combobox = ttk.Combobox(self.top_frame, width=30)
        self.port_combobox.pack(side=tk.LEFT, padx=(0, 10))
        
        # 创建刷新按钮
        self.refresh_button = ttk.Button(self.top_frame, text="刷新", command=self.refresh_ports)
        self.refresh_button.pack(side=tk.LEFT)
        
        # 创建参数配置框架
        self.config_frame = ttk.LabelFrame(root, text="串口通信参数配置")
        self.config_frame.pack(fill=tk.X, padx=10, pady=10)
        
        # 创建参数配置网格布局
        self.create_config_widgets()
        
        # 创建角度输入和控制框架
        self.angle_frame = ttk.LabelFrame(root, text="角度控制 (弧度值)")
        self.angle_frame.pack(fill=tk.X, padx=10, pady=10)
        
        # 添加角度输入控件
        self.create_angle_widgets()
        
        # 创建数据显示框架
        self.data_frame = ttk.LabelFrame(root, text="接收数据显示")
        self.data_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # 添加数据显示控件
        self.create_data_display_widgets()
        
        # 初始化加载串口列表
        self.refresh_ports()
        
        # 添加串口连接状态变量
        self.serial = None
        self.is_connected = False
        self.receiver_thread = None
        self.receiving = False
    
    def create_config_widgets(self):
        """创建串口参数配置控件"""
        # 常用波特率列表
        baud_rates = ['9600', '19200', '38400', '57600', '115200', '230400', '460800', '921600']
        # 数据位选项
        data_bits = ['5', '6', '7', '8']
        # 校验位选项
        parity_bits = [('无', 'N'), ('奇校验', 'O'), ('偶校验', 'E'), ('标记', 'M'), ('空格', 'S')]
        # 停止位选项
        stop_bits = [('1', 1), ('1.5', 1.5), ('2', 2)]
        # 流控制选项
        flow_controls = [('无', ''), ('XON/XOFF (软件)', 'xonxoff'), 
                         ('RTS/CTS (硬件)', 'rtscts'), ('DSR/DTR (硬件)', 'dsrdtr')]
        
        # 参数的行配置
        row = 0
        padding = 5
        
        # 波特率
        ttk.Label(self.config_frame, text="波特率:").grid(row=row, column=0, sticky=tk.W, padx=padding, pady=padding)
        self.baud_rate = tk.StringVar(value='9600')
        baud_combo = ttk.Combobox(self.config_frame, textvariable=self.baud_rate, values=baud_rates, width=15)
        baud_combo.grid(row=row, column=1, sticky=tk.W, padx=padding, pady=padding)
        
        # 数据位
        ttk.Label(self.config_frame, text="数据位:").grid(row=row, column=2, sticky=tk.W, padx=padding, pady=padding)
        self.data_bit = tk.StringVar(value='8')
        data_combo = ttk.Combobox(self.config_frame, textvariable=self.data_bit, values=data_bits, width=10)
        data_combo.grid(row=row, column=3, sticky=tk.W, padx=padding, pady=padding)
        
        row += 1
        
        # 校验位
        ttk.Label(self.config_frame, text="校验位:").grid(row=row, column=0, sticky=tk.W, padx=padding, pady=padding)
        self.parity = tk.StringVar(value='N')
        parity_frame = ttk.Frame(self.config_frame)
        parity_frame.grid(row=row, column=1, columnspan=3, sticky=tk.W, padx=padding, pady=padding)
        
        for i, (text, value) in enumerate(parity_bits):
            ttk.Radiobutton(parity_frame, text=text, variable=self.parity, value=value).grid(row=0, column=i, padx=5)
        
        row += 1
        
        # 停止位
        ttk.Label(self.config_frame, text="停止位:").grid(row=row, column=0, sticky=tk.W, padx=padding, pady=padding)
        self.stop_bit = tk.DoubleVar(value=1)
        stop_frame = ttk.Frame(self.config_frame)
        stop_frame.grid(row=row, column=1, columnspan=3, sticky=tk.W, padx=padding, pady=padding)
        
        for i, (text, value) in enumerate(stop_bits):
            ttk.Radiobutton(stop_frame, text=text, variable=self.stop_bit, value=value).grid(row=0, column=i, padx=5)
        
        row += 1
        
        # 流控制
        ttk.Label(self.config_frame, text="流控制:").grid(row=row, column=0, sticky=tk.W, padx=padding, pady=padding)
        self.flow_control_type = tk.StringVar(value='')
        flow_frame = ttk.Frame(self.config_frame)
        flow_frame.grid(row=row, column=1, columnspan=3, sticky=tk.W, padx=padding, pady=padding)
        
        for i, (text, value) in enumerate(flow_controls):
            ttk.Radiobutton(flow_frame, text=text, variable=self.flow_control_type, value=value).grid(row=i//2, column=i%2, sticky=tk.W, padx=5, pady=2)
        
        row += 1
        
        # 添加连接按钮
        self.connect_button = ttk.Button(self.config_frame, text="连接串口", command=self.connect_serial)
        self.connect_button.grid(row=row, column=0, columnspan=4, pady=15)
    
    def connect_serial(self):
        """连接串口设备"""
        port = self.get_selected_port()
        if not port:
            self.append_to_display("请选择一个有效的串口设备", "错误")
            messagebox.showerror("错误", "请选择一个有效的串口设备")
            return
            
        # 如果已经连接，先断开
        if self.is_connected and self.serial:
            try:
                # 停止接收线程
                self.stop_receiver()
                
                self.serial.close()
                self.is_connected = False
                self.connect_button.config(text="连接串口")
                self.send_button.config(state=tk.DISABLED)
                
                # 在显示区域显示断开连接信息
                self.append_to_display("串口已断开连接", "信息")
                messagebox.showinfo("信息", "串口已断开连接")
                return
            except Exception as e:
                error_msg = f"断开连接时出错: {str(e)}"
                self.append_to_display(error_msg, "错误")
                messagebox.showerror("错误", error_msg)
                return
        
        # 获取配置参数
        try:
            baud_rate = int(self.baud_rate.get())
            data_bits = int(self.data_bit.get())
            parity = self.parity.get()
            stop_bits = self.stop_bit.get()
            
            # 设置流控制参数
            xonxoff = False
            rtscts = False
            dsrdtr = False
            
            flow_control = self.flow_control_type.get()
            if flow_control == 'xonxoff':
                xonxoff = True
            elif flow_control == 'rtscts':
                rtscts = True
            elif flow_control == 'dsrdtr':
                dsrdtr = True
                
            # 尝试连接串口
            try:
                self.serial = serial.Serial(
                    port=port,
                    baudrate=baud_rate,
                    bytesize=data_bits,
                    parity=parity,
                    stopbits=stop_bits,
                    xonxoff=xonxoff,
                    rtscts=rtscts,
                    dsrdtr=dsrdtr,
                    timeout=1
                )
                
                # 判断串口是否成功打开
                if self.serial.is_open:
                    self.is_connected = True
                    self.connect_button.config(text="断开连接")
                    
                    # 显示连接信息
                    info = f"已成功连接到串口 {port}"
                    details = f"波特率: {baud_rate}, 数据位: {data_bits}, 校验位: {parity}, 停止位: {stop_bits}"
                    
                    # 在显示区域显示连接信息
                    self.append_to_display(info, "信息")
                    self.append_to_display(details, "信息")
                    
                    # 同时也显示消息对话框
                    messagebox.showinfo("连接成功", f"{info}\n{details}")
                    
                    # 启用发送按钮
                    self.send_button.config(state=tk.NORMAL)
                    
                    # 启动接收线程
                    self.start_receiver()
                    
                    # 这里可以进行简单的通信测试
                    self.test_communication()
                else:
                    error_msg = "串口无法打开，请检查设备或参数"
                    self.append_to_display(error_msg, "错误")
                    messagebox.showerror("错误", error_msg)
                    
            except serial.SerialException as e:
                error_msg = f"无法连接到串口: {str(e)}"
                self.append_to_display(error_msg, "错误")
                messagebox.showerror("连接错误", error_msg)
            
        except Exception as e:
            error_msg = f"配置参数无效: {str(e)}"
            self.append_to_display(error_msg, "错误")
            messagebox.showerror("参数错误", error_msg)
    
    def test_communication(self):
        """测试串口通信"""
        if not self.is_connected or not self.serial:
            return
        
        try:
            # 测试写入数据
            test_data = b'\x00'  # 发送一个空字节作为测试
            self.serial.write(test_data)
            
            # 记录发送的测试数据
            self.append_to_display("发送测试数据: 0x00", "发送")
            
            # 设置短超时尝试读取响应
            old_timeout = self.serial.timeout
            self.serial.timeout = 0.5
            
            try:
                response = self.serial.read(1)  # 尝试读取一个字节
                if response:
                    # 在接收区域显示响应
                    hex_response = response.hex()
                    self.append_to_display(f"测试响应: 0x{hex_response}", "接收")
                else:
                    # 在接收区域显示无响应信息
                    self.append_to_display("测试无响应，设备未返回数据", "接收")
            except Exception as e:
                # 显示读取错误
                self.append_to_display(f"读取测试响应时出错: {str(e)}", "错误")
            
            # 恢复原来的超时设置
            self.serial.timeout = old_timeout
            
        except Exception as e:
            # 显示测试错误
            self.append_to_display(f"测试通信时出错: {str(e)}", "错误")
    
    def show_message(self, title, message):
        """显示消息对话框"""
        messagebox.showinfo(title, message)
    
    def refresh_ports(self):
        """获取所有可用串口并更新下拉列表"""
        ports = serial.tools.list_ports.comports()
        port_list = []
        
        for port in ports:
            port_str = f"{port.device} - {port.description}"
            port_list.append(port_str)
        
        # 清空并更新下拉列表
        self.port_combobox['values'] = port_list
        
        # 如果有可用串口，默认选择第一个
        if port_list:
            self.port_combobox.current(0)
        else:
            self.port_combobox.set("未找到可用串口")
    
    def get_selected_port(self):
        """获取当前选择的串口设备名称"""
        selection = self.port_combobox.get()
        if selection and " - " in selection:
            return selection.split(" - ")[0]
        return None

    def create_angle_widgets(self):
        """创建角度输入控件"""
        self.angle_vars = []
        angle_frame_grid = ttk.Frame(self.angle_frame)
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
        
        # 控制按钮
        control_frame = ttk.Frame(self.angle_frame)
        control_frame.pack(fill=tk.X, padx=10, pady=5)
        
        # 发送按钮
        self.send_button = ttk.Button(control_frame, text="发送角度", command=self.send_angles, state=tk.DISABLED)
        self.send_button.pack(side=tk.LEFT, padx=5)
        
        # 转换按钮 (度到弧度)
        self.convert_button = ttk.Button(control_frame, text="度数转弧度", command=self.convert_degrees_to_radians)
        self.convert_button.pack(side=tk.LEFT, padx=5)
        
        # 归零按钮
        self.zero_button = ttk.Button(control_frame, text="全部归零", command=self.set_angles_to_zero)
        self.zero_button.pack(side=tk.LEFT, padx=5)
    
    def create_data_display_widgets(self):
        """创建数据显示控件"""
        # 创建左右分栏框架
        display_pane = ttk.PanedWindow(self.data_frame, orient=tk.HORIZONTAL)
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
        control_frame = ttk.Frame(self.data_frame)
        control_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # 清除发送按钮
        self.clear_send_button = ttk.Button(control_frame, text="清除发送显示", 
                                           command=lambda: self.clear_display("send"))
        self.clear_send_button.pack(side=tk.LEFT, padx=5)
        
        # 清除接收按钮
        self.clear_receive_button = ttk.Button(control_frame, text="清除接收显示", 
                                              command=lambda: self.clear_display("receive"))
        self.clear_receive_button.pack(side=tk.LEFT, padx=5)
        
        # 清除全部按钮
        self.clear_all_button = ttk.Button(control_frame, text="清除全部", 
                                          command=lambda: self.clear_display("all"))
        self.clear_all_button.pack(side=tk.LEFT, padx=5)
        
        # 自动滚动选项
        self.autoscroll_var = tk.BooleanVar(value=True)
        self.autoscroll_check = ttk.Checkbutton(control_frame, text="自动滚动", variable=self.autoscroll_var)
        self.autoscroll_check.pack(side=tk.LEFT, padx=5)
        
        # 显示时间戳选项
        self.timestamp_var = tk.BooleanVar(value=True)
        self.timestamp_check = ttk.Checkbutton(control_frame, text="显示时间戳", variable=self.timestamp_var)
        self.timestamp_check.pack(side=tk.LEFT, padx=5)
    
    def send_angles(self):
        """发送角度数据到串口"""
        if not self.is_connected or not self.serial:
            messagebox.showerror("错误", "请先连接串口")
            return
        
        try:
            angles = [var.get() for var in self.angle_vars]
            # 构建要发送的数据格式 - 这里使用简单的格式，您可能需要根据实际协议调整
            data = ",".join([f"{angle:.6f}" for angle in angles]) + "\n"
            
            # 记录发送的内容
            self.append_to_display(f"角度: {data}", "发送")
            
            # 发送数据
            self.serial.write(data.encode())
            
        except Exception as e:
            messagebox.showerror("发送错误", f"发送角度数据时出错: {str(e)}")
    
    def convert_degrees_to_radians(self):
        """将度数转换为弧度"""
        try:
            for var in self.angle_vars:
                # 获取当前值（假设为度数）
                degrees = var.get()
                # 转换为弧度
                radians = degrees * (3.14159265359 / 180.0)
                # 设置新值
                var.set(radians)
            messagebox.showinfo("转换完成", "已将所有角度从度数转换为弧度")
        except Exception as e:
            messagebox.showerror("转换错误", f"转换过程中出错: {str(e)}")
    
    def set_angles_to_zero(self):
        """将所有角度值设为零"""
        for var in self.angle_vars:
            var.set(0.0)
    
    def clear_display(self, display_type="all"):
        """清除数据显示区域
        
        参数:
            display_type: 要清除的显示区域类型，可选值为 "send", "receive", "all"
        """
        if display_type in ["send", "all"]:
            self.send_display.delete(1.0, tk.END)
        
        if display_type in ["receive", "all"]:
            self.receive_display.delete(1.0, tk.END)
    
    def append_to_display(self, message, message_type="信息"):
        """向数据显示区域添加消息
        
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
    
    def start_receiver(self):
        """启动数据接收线程"""
        if self.receiving:
            return
            
        self.receiving = True
        self.receiver_thread = threading.Thread(target=self.receive_data, daemon=True)
        self.receiver_thread.start()
        self.append_to_display("已启动数据接收")
    
    def stop_receiver(self):
        """停止数据接收线程"""
        self.receiving = False
        if self.receiver_thread:
            # 等待线程结束，最多等待1秒
            if self.receiver_thread.is_alive():
                self.receiver_thread.join(1.0)
            self.receiver_thread = None
        self.append_to_display("已停止数据接收")
    
    def receive_data(self):
        """接收串口数据的线程函数"""
        buffer = b''
        
        while self.receiving and self.is_connected and self.serial:
            try:
                # 读取可用的数据
                if self.serial.in_waiting > 0:
                    data = self.serial.read(self.serial.in_waiting)
                    if data:
                        # 将数据添加到缓冲区
                        buffer += data
                        
                        # 处理可能的完整消息
                        while b'\n' in buffer:
                            # 找到一个完整的行
                            line, buffer = buffer.split(b'\n', 1)
                            try:
                                # 尝试解码为字符串
                                line_str = line.decode('utf-8').strip()
                                # 在UI线程中更新显示
                                self.root.after(0, lambda msg=line_str: self.append_to_display(msg, "接收"))
                            except UnicodeDecodeError:
                                # 如果无法解码为UTF-8，显示十六进制
                                hex_str = ' '.join([f'{b:02X}' for b in line])
                                self.root.after(0, lambda msg=hex_str: self.append_to_display(f"HEX: {msg}", "接收"))
                
                # 短暂休眠，避免CPU占用过高
                time.sleep(0.01)
                
            except Exception as e:
                # 在UI线程中显示错误
                self.root.after(0, lambda err=str(e): self.append_to_display(f"接收错误: {err}", "错误"))
                time.sleep(1)  # 出错后等待一段时间再继续

def main():
    root = tk.Tk()
    app = USBSerialUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()
