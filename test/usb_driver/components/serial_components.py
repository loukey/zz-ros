"""
串口相关组件
"""
from PyQt5.QtWidgets import (QWidget, QLabel, QPushButton, QComboBox, QVBoxLayout, 
                            QHBoxLayout, QGroupBox, QRadioButton, QButtonGroup)
from PyQt5.QtCore import Qt
from .base_components import default_font


class PortSelectionFrame(QWidget):
    """串口选择框架"""
    
    def __init__(self, parent=None, refresh_callback=None):
        super().__init__(parent)
        self.refresh_callback = refresh_callback
        self._init_ui()
    
    def _init_ui(self):
        """初始化UI"""
        layout = QHBoxLayout()
        
        # 创建串口选择下拉列表
        layout.addWidget(QLabel("选择串口:"))
        self.port_combobox = QComboBox()
        self.port_combobox.setFont(default_font)
        self.port_combobox.setMinimumWidth(300)
        layout.addWidget(self.port_combobox)
        
        # 创建刷新按钮
        self.refresh_button = QPushButton("刷新")
        self.refresh_button.setFont(default_font)
        self.refresh_button.clicked.connect(self.refresh_callback)
        layout.addWidget(self.refresh_button)
        
        layout.addStretch()
        self.setLayout(layout)
    
    def set_ports(self, port_list):
        """
        设置可用串口列表
        
        参数:
            port_list: 包含(port, description)元组的列表
        """
        self.port_combobox.clear()
        formatted_ports = []
        for port, desc in port_list:
            formatted_ports.append(f"{port} - {desc}")
        
        self.port_combobox.addItems(formatted_ports)
        
        # 如果有可用串口，默认选择第一个
        if formatted_ports:
            self.port_combobox.setCurrentIndex(0)
        else:
            self.port_combobox.setCurrentText("未找到可用串口")
    
    def get_selected_port(self):
        """
        获取当前选择的串口设备名称
        
        返回:
            port: 串口设备名称，如果未选择则返回None
        """
        selection = self.port_combobox.currentText()
        if selection and " - " in selection:
            return selection.split(" - ")[0]
        return None
    
    def update_connection_status(self, is_connected):
        """
        更新连接状态
        
        参数:
            is_connected: 是否已连接
        """
        self.refresh_button.setEnabled(not is_connected)
        self.port_combobox.setEnabled(not is_connected)


class SerialConfigFrame(QGroupBox):
    """串口参数配置框架"""
    
    def __init__(self, parent=None, connect_callback=None):
        super().__init__("串口通信参数配置", parent)
        self.connect_callback = connect_callback
        self._init_ui()
    
    def _init_ui(self):
        """初始化UI"""
        layout = QVBoxLayout()
        
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
        
        # 第一行：波特率和数据位
        row1_layout = QHBoxLayout()
        row1_layout.addWidget(QLabel("波特率:"))
        self.baud_rate = QComboBox()
        self.baud_rate.addItems(self.baud_rates)
        self.baud_rate.setCurrentText('115200')
        self.baud_rate.setFont(default_font)
        row1_layout.addWidget(self.baud_rate)
        
        row1_layout.addWidget(QLabel("数据位:"))
        self.data_bit = QComboBox()
        self.data_bit.addItems(self.data_bits)
        self.data_bit.setCurrentText('8')
        self.data_bit.setFont(default_font)
        row1_layout.addWidget(self.data_bit)
        row1_layout.addStretch()
        layout.addLayout(row1_layout)
        
        # 第二行：校验位
        row2_layout = QHBoxLayout()
        row2_layout.addWidget(QLabel("校验位:"))
        self.parity_group = QButtonGroup()
        for i, (text, value) in enumerate(self.parity_bits):
            radio = QRadioButton(text)
            radio.setFont(default_font)
            radio.setProperty('parity_value', value)  # 存储实际的校验位值
            if value == 'N':
                radio.setChecked(True)
            self.parity_group.addButton(radio, i)
            row2_layout.addWidget(radio)
        row2_layout.addStretch()
        layout.addLayout(row2_layout)
        
        # 第三行：停止位
        row3_layout = QHBoxLayout()
        row3_layout.addWidget(QLabel("停止位:"))
        self.stop_bit_group = QButtonGroup()
        for i, (text, value) in enumerate(self.stop_bits):
            radio = QRadioButton(text)
            radio.setFont(default_font)
            radio.setProperty('stop_bit_value', value)  # 存储实际的停止位值
            if value == 1:
                radio.setChecked(True)
            self.stop_bit_group.addButton(radio, i)
            row3_layout.addWidget(radio)
        row3_layout.addStretch()
        layout.addLayout(row3_layout)
        
        # 第四行：流控制
        row4_layout = QHBoxLayout()
        row4_layout.addWidget(QLabel("流控制:"))
        self.flow_control_group = QButtonGroup()
        for i, (text, value) in enumerate(self.flow_controls):
            radio = QRadioButton(text)
            radio.setFont(default_font)
            radio.setProperty('flow_control_value', value)  # 存储实际的流控制值
            if value == '':
                radio.setChecked(True)
            self.flow_control_group.addButton(radio, i)
            row4_layout.addWidget(radio)
        row4_layout.addStretch()
        layout.addLayout(row4_layout)
        
        # 连接按钮
        self.connect_button = QPushButton("连接串口")
        self.connect_button.setFont(default_font)
        self.connect_button.clicked.connect(self.connect_callback)
        layout.addWidget(self.connect_button)
        
        self.setLayout(layout)
    
    def get_config(self):
        """
        获取当前配置参数
        
        返回:
            config: 包含所有配置参数的字典
        """
        return {
            'baud_rate': int(self.baud_rate.currentText()),
            'data_bits': int(self.data_bit.currentText()),
            'parity': self.parity_group.checkedButton().property('parity_value'),
            'stop_bits': float(self.stop_bit_group.checkedButton().property('stop_bit_value')),
            'flow_control': self.flow_control_group.checkedButton().property('flow_control_value')
        }
    
    def set_connect_button_state(self, is_connected):
        """
        设置连接按钮状态
        
        参数:
            is_connected: 是否已连接
        """
        self.connect_button.setText("断开连接" if is_connected else "连接串口")
        
    def update_connection_status(self, is_connected):
        """
        更新连接状态
        
        参数:
            is_connected: 是否已连接
        """
        self.set_connect_button_state(is_connected)
        self.baud_rate.setEnabled(not is_connected)
        self.data_bit.setEnabled(not is_connected)
        
        # 禁用/启用单选按钮组
        for i in range(self.parity_group.buttons().__len__()):
            self.parity_group.button(i).setEnabled(not is_connected)
            
        for i in range(self.stop_bit_group.buttons().__len__()):
            self.stop_bit_group.button(i).setEnabled(not is_connected)
            
        for i in range(self.flow_control_group.buttons().__len__()):
            self.flow_control_group.button(i).setEnabled(not is_connected)
 