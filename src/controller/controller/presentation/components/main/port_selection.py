"""
Port selection component for Main tab
"""
from PyQt5.QtWidgets import QWidget, QPushButton, QHBoxLayout
from PyQt5.QtCore import pyqtSignal
from ..base_component import BaseComponent, default_font, LabeledComboBox


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
                # 通过信号发送命令
                self.port_connect_requested.emit(selected_port, config)
        else:
            # 断开连接 - 发出信号
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


