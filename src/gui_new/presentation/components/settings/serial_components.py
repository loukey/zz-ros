"""
Serial config component for Settings dialog
"""
from PyQt5.QtWidgets import (QWidget, QLabel, QPushButton, QComboBox, QVBoxLayout, 
                            QHBoxLayout, QGroupBox, QRadioButton, QButtonGroup)
from ..base_component import BaseComponent, default_font


class SerialConfigFrame(BaseComponent):
    """串口参数配置框架"""
    
    def __init__(self, parent=None, view_model=None):
        super().__init__(parent, view_model)
    
    def setup_ui(self):
        """设置UI"""
        group = QGroupBox("串口通信参数配置")
        main_layout = QVBoxLayout(self)
        main_layout.addWidget(group)
        layout = QVBoxLayout(group)
        
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
    
    def get_config(self):
        """获取当前配置参数"""
        return {
            'baudrate': int(self.baud_rate.currentText()),
            'bytesize': int(self.data_bit.currentText()),
            'parity': self.parity_group.checkedButton().property('parity_value'),
            'stopbits': float(self.stop_bit_group.checkedButton().property('stop_bit_value')),
            'timeout': 1,
            'xonxoff': self.flow_control_group.checkedButton().property('flow_control_value') == 'xonxoff',
            'rtscts': self.flow_control_group.checkedButton().property('flow_control_value') == 'rtscts',
            'dsrdtr': self.flow_control_group.checkedButton().property('flow_control_value') == 'dsrdtr',
        }


