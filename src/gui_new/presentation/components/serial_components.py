"""
串口相关UI组件
"""
from PyQt5.QtWidgets import (QWidget, QLabel, QPushButton, QComboBox, QVBoxLayout, 
                            QHBoxLayout, QGroupBox, QRadioButton, QButtonGroup)
from PyQt5.QtCore import pyqtSignal
from .base_component import BaseComponent, default_font, LabeledComboBox, RadioButtonGroup, ConfigRow


class PortSelectionFrame(BaseComponent):
    """串口选择框架"""
    refresh_ports_requested = pyqtSignal()
    port_connect_requested = pyqtSignal(str, dict)
    port_disconnect_requested = pyqtSignal()
    
    def __init__(self, parent=None, view_model=None, get_config=None):
        self.get_config = get_config
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

    def on_connect_clicked(self):
        """连接按钮点击"""
        connect_button_text = self.connect_button.text()
        if connect_button_text == "连接串口":
            selected_port = self.get_selected_port()
            if selected_port:
                config = self.get_config() if self.get_config else {}
                self.port_connect_requested.emit(selected_port, config)
        else:
            self.port_disconnect_requested.emit()

    def on_refresh_clicked(self):
        """刷新按钮点击"""
        self.refresh_ports_requested.emit()
        
    def set_ports(self, port_list):
        """设置可用串口列表"""
        self.port_combo.combobox.clear()
        formatted_ports = []
        for port, desc in port_list:
            formatted_ports.append(f"{port} - {desc}")
        
        self.port_combo.set_items(formatted_ports)
        
        # 如果有可用串口，默认选择第一个
        if formatted_ports:
            self.port_combo.set_current_index(0)
        else:
            self.port_combo.combobox.setCurrentText("未找到可用串口")
    
    def get_selected_port(self):
        """获取当前选择的串口设备名称"""
        selection = self.port_combo.current_text()
        if selection and " - " in selection:
            return selection.split(" - ")[0]
        return None
    
    def update_connection_status(self, is_connected):
        """更新连接状态"""
        self.refresh_button.setEnabled(not is_connected)
        self.port_combo.set_enabled(not is_connected)
        self.connect_button.setText("断开连接" if is_connected else "连接串口")


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
        
        # 第一行：波特率和数据位 - 使用LabeledComboBox基础组件
        row1 = ConfigRow(None)
        
        self.baud_rate_combo = LabeledComboBox(
            "波特率:",
            items=['9600', '19200', '38400', '57600', '115200', '230400', '460800', '921600']
        )
        self.baud_rate_combo.set_current_text('115200')
        row1.add_widget(self.baud_rate_combo)
        
        self.data_bit_combo = LabeledComboBox(
            "数据位:",
            items=['5', '6', '7', '8']
        )
        self.data_bit_combo.set_current_text('8')
        row1.add_widget(self.data_bit_combo)
        
        row1.add_stretch()
        layout.addWidget(row1)
        
        # 第二行：校验位 - 使用RadioButtonGroup基础组件
        self.parity_group = RadioButtonGroup(
            "校验位:",
            options=['无', '奇校验', '偶校验', '标记', '空格'],
            default_option='无'
        )
        layout.addWidget(self.parity_group)
        
        # 第三行：停止位 - 使用RadioButtonGroup基础组件
        self.stop_bit_group = RadioButtonGroup(
            "停止位:",
            options=['1', '1.5', '2'],
            default_option='1'
        )
        layout.addWidget(self.stop_bit_group)
        
        # 第四行：流控制 - 使用RadioButtonGroup基础组件
        self.flow_control_group = RadioButtonGroup(
            "流控制:",
            options=['无', 'XON/XOFF (软件)', 'RTS/CTS (硬件)', 'DSR/DTR (硬件)'],
            default_option='无'
        )
        layout.addWidget(self.flow_control_group)
    
    def connect_signals(self):
        """连接视图模型信号"""
        if self.view_model:
            # 当配置改变时通知视图模型
            self.baud_rate_combo.currentTextChanged.connect(self._on_config_changed)
            self.data_bit_combo.currentTextChanged.connect(self._on_config_changed)
            self.parity_group.selectionChanged.connect(self._on_config_changed)
            self.stop_bit_group.selectionChanged.connect(self._on_config_changed)
            self.flow_control_group.selectionChanged.connect(self._on_config_changed)
    
    def _on_config_changed(self):
        """配置改变时的回调"""
        if self.view_model:
            config = self.get_config()
            self.view_model.update_serial_config(config)
    
    def get_config(self):
        """获取当前配置参数"""
        # 映射选项到pyserial需要的值
        parity_map = {
            '无': 'N',
            '奇校验': 'O', 
            '偶校验': 'E',
            '标记': 'M',
            '空格': 'S'
        }
        
        flow_control_map = {
            '无': '',
            'XON/XOFF (软件)': 'xonxoff',
            'RTS/CTS (硬件)': 'rtscts',
            'DSR/DTR (硬件)': 'dsrdtr'
        }
        
        return {
            'baud_rate': int(self.baud_rate_combo.current_text()),
            'data_bits': int(self.data_bit_combo.current_text()),
            'parity': parity_map.get(self.parity_group.get_selected(), 'N'),
            'stop_bits': float(self.stop_bit_group.get_selected()),
            'flow_control': flow_control_map.get(self.flow_control_group.get_selected(), '')
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