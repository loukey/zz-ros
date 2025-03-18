"""
串口选择相关组件
"""
from PyQt5.QtWidgets import QComboBox, QPushButton, QLabel, QVBoxLayout, QHBoxLayout, QGroupBox
from components.base_components import default_font


class PortSelectFrame(QGroupBox):
    """串口选择框架"""
    
    def __init__(self, refresh_callback, connect_callback, parent=None):
        """
        初始化串口选择框架
        
        参数:
            refresh_callback: 刷新端口列表的回调函数
            connect_callback: 连接/断开端口的回调函数
            parent: 父控件
        """
        super().__init__("串口设置", parent)
        self.refresh_callback = refresh_callback
        self.connect_callback = connect_callback
        self._init_ui()
    
    def _init_ui(self):
        """初始化UI"""
        layout = QVBoxLayout()
        
        # 端口选择
        port_layout = QHBoxLayout()
        port_layout.addWidget(QLabel("端口:"))
        self.port_combo = QComboBox()
        self.port_combo.setFont(default_font)
        port_layout.addWidget(self.port_combo)
        
        self.refresh_button = QPushButton("刷新")
        self.refresh_button.setFont(default_font)
        self.refresh_button.clicked.connect(self.refresh_callback)
        port_layout.addWidget(self.refresh_button)
        layout.addLayout(port_layout)
        
        # 波特率选择
        baudrate_layout = QHBoxLayout()
        baudrate_layout.addWidget(QLabel("波特率:"))
        self.baudrate_combo = QComboBox()
        self.baudrate_combo.addItems(["9600", "19200", "38400", "57600", "115200", "230400", "460800", "921600"])
        self.baudrate_combo.setCurrentText("115200")
        self.baudrate_combo.setFont(default_font)
        baudrate_layout.addWidget(self.baudrate_combo)
        layout.addLayout(baudrate_layout)
        
        # 连接按钮
        self.connect_button = QPushButton("连接")
        self.connect_button.setFont(default_font)
        self.connect_button.clicked.connect(self.connect_callback)
        layout.addWidget(self.connect_button)
        
        self.setLayout(layout)
    
    def update_connection_status(self, connected):
        """
        更新连接状态
        
        参数:
            connected: 是否已连接
        """
        self.refresh_button.setEnabled(not connected)
        self.connect_button.setText("断开" if connected else "连接")
        # 禁用/启用端口选择下拉框
        self.port_combo.setEnabled(not connected)
        self.baudrate_combo.setEnabled(not connected)
    
    def set_ports(self, ports):
        """
        设置可用端口列表
        
        参数:
            ports: 端口列表，每个端口是一个元组，包含(port, description)
        """
        self.port_combo.clear()
        for port, description in ports:
            display_text = f"{port} - {description}" if description else port
            self.port_combo.addItem(display_text, port)
    
    def get_selected_port(self):
        """
        获取当前选择的端口
        
        返回:
            str: 端口名称，如果没有选择则返回None
        """
        if self.port_combo.count() == 0:
            return None
        return self.port_combo.currentData()
    
    def get_config(self):
        """
        获取串口配置
        
        返回:
            dict: 包含串口配置的字典
        """
        selected_port = self.get_selected_port()
        baudrate = int(self.baudrate_combo.currentText())
        
        return {
            'port': selected_port,
            'baud_rate': baudrate,
            'data_bits': 8,
            'parity': 'N',
            'stop_bits': 1,
            'flow_control': None
        } 