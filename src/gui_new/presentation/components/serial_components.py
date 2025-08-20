"""
串口相关UI组件
包含串口参数的默认配置
"""
from PyQt5.QtWidgets import (QWidget, QLabel, QPushButton, QComboBox, QVBoxLayout, 
                            QHBoxLayout, QGroupBox, QRadioButton, QButtonGroup)
from PyQt5.QtCore import pyqtSignal
from .base_component import BaseComponent, default_font, LabeledComboBox, RadioButtonGroup, ConfigRow


# ===== 串口参数配置常量 =====

# 默认串口连接参数
DEFAULT_BAUDRATE = 115200
DEFAULT_BYTESIZE = 8
DEFAULT_PARITY = 'N'  # 'N'=无, 'O'=奇校验, 'E'=偶校验, 'M'=标记, 'S'=空格
DEFAULT_STOPBITS = 1.0
DEFAULT_TIMEOUT = 1
DEFAULT_XONXOFF = False  # 软件流控制
DEFAULT_RTSCTS = False   # RTS/CTS硬件流控制  
DEFAULT_DSRDTR = False   # DSR/DTR硬件流控制

# UI显示用的默认值
DEFAULT_BAUDRATE_UI = '115200'
DEFAULT_BYTESIZE_UI = '8'
DEFAULT_PARITY_UI = '无'
DEFAULT_STOPBITS_UI = '1'
DEFAULT_FLOW_CONTROL_UI = '无'

# 串口参数选项列表
BAUDRATE_OPTIONS = ['9600', '19200', '38400', '57600', '115200', '230400', '460800', '921600']
BYTESIZE_OPTIONS = ['5', '6', '7', '8']
PARITY_OPTIONS = ['无', '奇校验', '偶校验', '标记', '空格']
STOPBITS_OPTIONS = ['1', '1.5', '2']
FLOW_CONTROL_OPTIONS = ['无', 'XON/XOFF (软件)', 'RTS/CTS (硬件)', 'DSR/DTR (硬件)']

# 校验位映射
PARITY_MAP = {
    '无': 'N',
    '奇校验': 'O',
    '偶校验': 'E', 
    '标记': 'M',
    '空格': 'S'
}

# ===== 工具函数 =====

def get_default_serial_config():
    """获取默认的串口连接配置字典"""
    return {
        'baudrate': DEFAULT_BAUDRATE,
        'bytesize': DEFAULT_BYTESIZE,
        'parity': DEFAULT_PARITY,
        'stopbits': DEFAULT_STOPBITS,
        'timeout': DEFAULT_TIMEOUT,
        'xonxoff': DEFAULT_XONXOFF,
        'rtscts': DEFAULT_RTSCTS,
        'dsrdtr': DEFAULT_DSRDTR
    }

def get_flow_control_settings(flow_control_ui_value):
    """根据UI选择值转换为流控制设置"""
    return {
        'xonxoff': flow_control_ui_value == 'XON/XOFF (软件)',
        'rtscts': flow_control_ui_value == 'RTS/CTS (硬件)',
        'dsrdtr': flow_control_ui_value == 'DSR/DTR (硬件)'
    }

def get_parity_value(parity_ui_value):
    """根据UI选择值获取校验位参数"""
    return PARITY_MAP.get(parity_ui_value, 'N')


class PortSelectionFrame(BaseComponent):
    """串口选择框架"""
    # UI Command 信号 - 发送给ViewModel
    refresh_ports_requested = pyqtSignal()
    port_connect_requested = pyqtSignal(str, dict)  # 发送端口名和配置
    port_disconnect_requested = pyqtSignal()
    
    def __init__(self, parent=None, view_model=None, get_serial_config=None):
        super().__init__(parent, view_model)
        self.get_serial_config = get_serial_config  # 获取配置的函数
    
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
            # 连接串口 - 获取端口名和配置
            selected_port = self.get_selected_port()
            if selected_port:
                config = self.get_serial_config()
                # ✅ 符合MVVM：通过信号发送命令
                self.port_connect_requested.emit(selected_port, config)
        else:
            # 断开连接 - 发出信号
            # ✅ 符合MVVM：通过信号发送命令
            self.port_disconnect_requested.emit()

    def on_refresh_clicked(self):
        self.refresh_ports_requested.emit()
        
    def set_ports(self, port_list):
        """设置可用串口列表"""
        self.port_combo.combobox.clear()
        if not port_list:
            self.port_combo.combobox.setCurrentText("未找到可用串口")
            return
        if isinstance(port_list[0], str):
            self.port_combo.set_items(port_list)
        elif isinstance(port_list[0], (tuple, list)) and len(port_list[0]) >= 2:
            formatted_ports = []
            for port, desc in port_list:
                formatted_ports.append(f"{port} - {desc}")
            self.port_combo.set_items(formatted_ports)
        else:
            self.port_combo.set_items([str(item) for item in port_list])
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
            self.refresh_button.setEnabled(False) 
            self.port_combo.set_enabled(False)    
        else:
            self.connect_button.setText("连接串口")
            self.connect_button.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; }")
            self.refresh_button.setEnabled(True)
            self.port_combo.set_enabled(True)


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
        
        # 使用本地配置的默认值
        default_baud_rate = DEFAULT_BAUDRATE_UI
        default_data_bits = DEFAULT_BYTESIZE_UI
        default_parity_ui = DEFAULT_PARITY_UI
        default_stop_bits_ui = DEFAULT_STOPBITS_UI
        default_flow_control_ui = DEFAULT_FLOW_CONTROL_UI
        
        # 第一行：波特率和数据位 - 使用LabeledComboBox基础组件
        row1 = ConfigRow(None)
        
        self.baud_rate_combo = LabeledComboBox(
            "波特率:",
            items=BAUDRATE_OPTIONS
        )
        self.baud_rate_combo.set_current_text(default_baud_rate)
        row1.add_widget(self.baud_rate_combo)
        
        self.data_bit_combo = LabeledComboBox(
            "数据位:",
            items=BYTESIZE_OPTIONS
        )
        self.data_bit_combo.set_current_text(default_data_bits)
        row1.add_widget(self.data_bit_combo)
        
        row1.add_stretch()
        layout.addWidget(row1)
        
        # 第二行：校验位 - 使用RadioButtonGroup基础组件
        self.parity_group = RadioButtonGroup(
            "校验位:",
            options=PARITY_OPTIONS,
            default_option=default_parity_ui
        )
        layout.addWidget(self.parity_group)
        
        # 第三行：停止位 - 使用RadioButtonGroup基础组件
        self.stop_bit_group = RadioButtonGroup(
            "停止位:",
            options=STOPBITS_OPTIONS,
            default_option=default_stop_bits_ui
        )
        layout.addWidget(self.stop_bit_group)
        
        # 第四行：流控制 - 使用RadioButtonGroup基础组件
        self.flow_control_group = RadioButtonGroup(
            "流控制:",
            options=FLOW_CONTROL_OPTIONS,
            default_option=default_flow_control_ui
        )
        layout.addWidget(self.flow_control_group)
    
    def get_config(self):
        """获取当前配置参数 - 使用本地转换方法"""
        # 获取UI选择的值
        flow_control_selected = self.flow_control_group.get_selected()
        parity_selected = self.parity_group.get_selected()
        
        # 使用本地转换方法
        flow_control_settings = get_flow_control_settings(flow_control_selected)
        parity_value = get_parity_value(parity_selected)
        
        # 返回SerialAdapter.connect()期望的精确格式
        return {
            'baudrate': int(self.baud_rate_combo.current_text()),
            'bytesize': int(self.data_bit_combo.current_text()),
            'parity': parity_value,
            'stopbits': float(self.stop_bit_group.get_selected()),
            'timeout': DEFAULT_TIMEOUT,  # 使用本地配置的超时时间
            **flow_control_settings
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
            