"""
串口相关UI组件
"""
from PyQt5.QtWidgets import (QWidget, QLabel, QPushButton, QComboBox, QVBoxLayout, 
                            QHBoxLayout, QGroupBox, QRadioButton, QButtonGroup)
from PyQt5.QtCore import pyqtSignal
from .base_component import BaseComponent, default_font, LabeledComboBox, RadioButtonGroup, ConfigRow


class PortSelectionFrame(BaseComponent):
    """串口选择框架"""
    # UI Command 信号 - 发送给ViewModel
    refresh_ports_requested = pyqtSignal()
    port_connect_requested = pyqtSignal(str)  # 只发送端口名
    port_disconnect_requested = pyqtSignal()
    
    def __init__(self, parent=None, view_model=None):
        super().__init__(parent, view_model)
    
    def setup_ui(self):
        """设置UI"""
        layout = QHBoxLayout(self)
        
        # 创建串口选择下拉列表 - 使用LabeledComboBox基础组件
        self.port_combo = LabeledComboBox("选择串口:")
        self.port_combo.combobox.setMinimumWidth(300)
        layout.addWidget(self.port_combo)
        
        # 创建刷新按钮
        self.refresh_button = QPushButton("刷新")
        self.refresh_button.setFont(default_font)
        self.refresh_button.clicked.connect(self.on_refresh_clicked)
        layout.addWidget(self.refresh_button)
        
        # 创建连接按钮
        self.connect_button = QPushButton("连接串口")
        self.connect_button.setFont(default_font)
        self.connect_button.clicked.connect(self.on_connect_clicked)
        layout.addWidget(self.connect_button)
        
        layout.addStretch()
    
    def connect_signals(self):
        """连接视图模型信号"""
        if self.view_model:
            self.view_model.connection_status_changed.connect(self.update_connection_status)
            self.view_model.port_list_updated.connect(self.set_ports)
            self.refresh_ports_requested.connect(self.view_model.refresh_ports)
            self.port_connect_requested.connect(self.view_model.connect_serial)
            self.port_disconnect_requested.connect(self.view_model.disconnect_serial)

    def on_connect_clicked(self):
        """连接按钮点击 - 发出信号而不是直接调用"""
        connect_button_text = self.connect_button.text()
        if connect_button_text == "连接串口":
            # 连接串口 - 发出信号 (只发送端口名，配置由SerialConfigViewModel单独获取)
            selected_port = self.get_selected_port()
            if selected_port:
                # ✅ 符合MVVM：通过信号发送命令
                self.port_connect_requested.emit(selected_port)
        else:
            # 断开连接 - 发出信号
            # ✅ 符合MVVM：通过信号发送命令
            self.port_disconnect_requested.emit()

    def on_refresh_clicked(self):
        """刷新按钮点击 - 发出信号而不是直接调用"""
        # ✅ 符合MVVM：通过信号发送命令
        self.refresh_ports_requested.emit()
    
    def on_data_received(self, data: str):
        """处理接收到的数据"""
        # 这里可以添加数据处理逻辑，比如显示在某个文本框中
        # 或者发送给其他组件处理
        pass
        
    def set_ports(self, port_list):
        """设置可用串口列表"""
        self.port_combo.combobox.clear()
        
        # 处理不同的数据格式
        if not port_list:
            self.port_combo.combobox.setCurrentText("未找到可用串口")
            return
            
        # 如果是字符串列表，直接使用
        if isinstance(port_list[0], str):
            self.port_combo.set_items(port_list)
        # 如果是(port, desc)元组列表，格式化显示
        elif isinstance(port_list[0], (tuple, list)) and len(port_list[0]) >= 2:
            formatted_ports = []
            for port, desc in port_list:
                formatted_ports.append(f"{port} - {desc}")
            self.port_combo.set_items(formatted_ports)
        else:
            # 其他情况，转换为字符串
            self.port_combo.set_items([str(item) for item in port_list])
        
        # 如果有可用串口，默认选择第一个
        if port_list:
            self.port_combo.set_current_index(0)
    
    def get_selected_port(self):
        """获取当前选择的串口设备名称"""
        selected_text = self.port_combo.current_text()
        if not selected_text or selected_text == "未找到可用串口" or not selected_text.strip():
            return None
        
        # 如果是格式化的文本（包含" - "），提取端口名
        if " - " in selected_text:
            return selected_text.split(" - ")[0]
        else:
            return selected_text
    
    def update_connection_status(self, is_connected):
        """更新连接状态"""
        if is_connected:
            self.connect_button.setText("断开连接")
            self.connect_button.setStyleSheet("QPushButton { background-color: #f44336; color: white; }")
            self.refresh_button.setEnabled(False)  # 连接时禁用刷新按钮
            self.port_combo.set_enabled(False)     # 连接时禁用端口选择
        else:
            self.connect_button.setText("连接串口")
            self.connect_button.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; }")
            self.refresh_button.setEnabled(True)   # 断开时启用刷新按钮
            self.port_combo.set_enabled(True)      # 断开时启用端口选择


class SerialConfigFrame(BaseComponent):
    """串口参数配置框架"""
    def __init__(self, parent=None, view_model=None):
        super().__init__(parent, view_model)
    
    def setup_ui(self):
        """设置UI"""
        # 创建分组框
        group_box = QGroupBox("串口通信参数配置")
        main_layout = QVBoxLayout(self)
        main_layout.addWidget(group_box)
        
        layout = QVBoxLayout(group_box)
        
        # 使用硬编码的默认值
        default_baud_rate = '115200'
        default_data_bits = '8'
        default_parity_ui = '无'
        default_stop_bits_ui = '1'
        default_flow_control_ui = '无'
        
        # 第一行：波特率和数据位 - 使用LabeledComboBox基础组件
        row1 = ConfigRow(None)
        
        self.baud_rate_combo = LabeledComboBox(
            "波特率:",
            items=['9600', '19200', '38400', '57600', '115200', '230400', '460800', '921600']
        )
        self.baud_rate_combo.set_current_text(default_baud_rate)
        row1.add_widget(self.baud_rate_combo)
        
        self.data_bit_combo = LabeledComboBox(
            "数据位:",
            items=['5', '6', '7', '8']
        )
        self.data_bit_combo.set_current_text(default_data_bits)
        row1.add_widget(self.data_bit_combo)
        
        row1.add_stretch()
        layout.addWidget(row1)
        
        # 第二行：校验位 - 使用RadioButtonGroup基础组件
        self.parity_group = RadioButtonGroup(
            "校验位:",
            options=['无', '奇校验', '偶校验', '标记', '空格'],
            default_option=default_parity_ui
        )
        layout.addWidget(self.parity_group)
        
        # 第三行：停止位 - 使用RadioButtonGroup基础组件
        self.stop_bit_group = RadioButtonGroup(
            "停止位:",
            options=['1', '1.5', '2'],
            default_option=default_stop_bits_ui
        )
        layout.addWidget(self.stop_bit_group)
        
        # 第四行：流控制 - 使用RadioButtonGroup基础组件
        self.flow_control_group = RadioButtonGroup(
            "流控制:",
            options=['无', 'XON/XOFF (软件)', 'RTS/CTS (硬件)', 'DSR/DTR (硬件)'],
            default_option=default_flow_control_ui
        )
        layout.addWidget(self.flow_control_group)
    
    def get_config(self):
        """获取当前配置参数 - 直接返回SerialAdapter期望的格式"""
        # 映射选项到pyserial标准参数名
        parity_map = {
            '无': 'N',
            '奇校验': 'O', 
            '偶校验': 'E',
            '标记': 'M',
            '空格': 'S'
        }
        
        # 流控制映射 - 直接映射为SerialAdapter期望的布尔值
        flow_control_selected = self.flow_control_group.get_selected()
        flow_control_settings = {
            'xonxoff': flow_control_selected == 'XON/XOFF (软件)',
            'rtscts': flow_control_selected == 'RTS/CTS (硬件)',
            'dsrdtr': flow_control_selected == 'DSR/DTR (硬件)'
        }
        
        # 返回SerialAdapter.connect()期望的精确格式
        return {
            'baudrate': int(self.baud_rate_combo.current_text()),           # PySerial标准参数名
            'bytesize': int(self.data_bit_combo.current_text()),            # PySerial标准参数名
            'parity': parity_map.get(self.parity_group.get_selected(), 'N'), # PySerial标准参数名
            'stopbits': float(self.stop_bit_group.get_selected()),          # PySerial标准参数名
            'timeout': 1,                                                   # 固定超时时间
            **flow_control_settings                                         # xonxoff, rtscts, dsrdtr布尔值
        }
    
    def update_connection_status(self, is_connected):
        """更新连接状态"""
        self.baud_rate_combo.set_enabled(not is_connected)
        self.data_bit_combo.set_enabled(not is_connected)
        
        # 禁用/启用单选按钮组
        for option, button in self.parity_group.buttons.items():
            button.setEnabled(not is_connected)
            
        for option, button in self.stop_bit_group.buttons.items():
            button.setEnabled(not is_connected)
            
        for option, button in self.flow_control_group.buttons.items():
            button.setEnabled(not is_connected) 