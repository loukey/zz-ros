#!/usr/bin/env python3
"""
串口模拟器
- 支持十六进制和文本格式数据收发
- 支持十六进制数据的显示和输入
- 支持文本格式数据的显示和输入
"""
import sys
import os
import time
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                            QHBoxLayout, QLabel, QPushButton, QTextEdit, 
                            QGroupBox, QLineEdit, QCheckBox)
from PyQt5.QtCore import Qt, pyqtSignal, QObject
from PyQt5.QtGui import QTextCursor, QFont

# 添加当前目录到Python路径
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, current_dir)

from virtual_serial import VirtualSerial
from crc import calculate_crc16


class LogSignals(QObject):
    """用于跨线程更新日志的信号类"""
    log_message = pyqtSignal(str, str, str)  # category, message, time_str

class SerialSimulator(QMainWindow):
    """串口模拟器窗口"""
    
    def __init__(self):
        super().__init__()
        
        # 初始化虚拟串口
        self.virtual_serial = VirtualSerial()
        
        # 初始化日志信号
        self.log_signals = LogSignals()
        self.log_signals.log_message.connect(self._update_log)
        
        self.setWindowTitle("串口模拟器")
        self.setMinimumSize(800, 600)
        
        self._init_ui()
    
    def _init_ui(self):
        """初始化UI"""
        # 创建中心部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 创建主布局
        main_layout = QVBoxLayout(central_widget)
        
        # 状态显示区域
        status_group = QGroupBox("串口状态")
        status_layout = QHBoxLayout()
        
        self.status_label = QLabel("未创建")
        status_layout.addWidget(QLabel("状态:"))
        status_layout.addWidget(self.status_label)
        
        self.device_name_label = QLabel("")
        status_layout.addWidget(QLabel("设备名:"))
        status_layout.addWidget(self.device_name_label)
        
        status_layout.addStretch()
        
        # 创建/关闭按钮
        self.create_button = QPushButton("创建串口")
        self.create_button.clicked.connect(self.toggle_virtual_serial)
        status_layout.addWidget(self.create_button)
        
        status_group.setLayout(status_layout)
        main_layout.addWidget(status_group)
        
        # 数据输入区域
        input_group = QGroupBox("数据输入")
        input_layout = QVBoxLayout()
        
        # 十六进制模式选择
        hex_mode_layout = QHBoxLayout()
        self.hex_mode_checkbox = QCheckBox("十六进制模式")
        self.hex_mode_checkbox.stateChanged.connect(self.toggle_hex_mode)
        hex_mode_layout.addWidget(self.hex_mode_checkbox)
        hex_mode_layout.addStretch()
        input_layout.addLayout(hex_mode_layout)
        
        # 数据输入框
        data_input_layout = QHBoxLayout()
        self.data_input = QLineEdit()
        self.data_input.setPlaceholderText("输入数据 (十六进制模式: 例如 '01 02 03 04', 文本模式: 例如 'cmd 01 08 78623 369707 83986 391414 508006 455123 B9FC')")
        self.data_input.returnPressed.connect(self.send_data)
        data_input_layout.addWidget(self.data_input)
        
        send_button = QPushButton("发送")
        send_button.clicked.connect(self.send_data)
        data_input_layout.addWidget(send_button)
        
        input_layout.addLayout(data_input_layout)
        input_group.setLayout(input_layout)
        main_layout.addWidget(input_group)
        
        # 通信日志显示区域
        log_group = QGroupBox("通信日志")
        log_layout = QVBoxLayout()
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        # 使用等宽字体
        self.log_text.setFont(QFont("Courier New", 10))
        log_layout.addWidget(self.log_text)
        
        # 清除按钮
        clear_button = QPushButton("清除日志")
        clear_button.clicked.connect(self.clear_log)
        log_layout.addWidget(clear_button)
        
        log_group.setLayout(log_layout)
        main_layout.addWidget(log_group)
    
    def toggle_hex_mode(self, state):
        """切换十六进制模式"""
        is_hex_mode = state == Qt.Checked
        self.virtual_serial.set_hex_mode(is_hex_mode)
        self.log_message("系统", f"十六进制模式: {'启用' if is_hex_mode else '禁用'}")
    
    def toggle_virtual_serial(self):
        """切换虚拟串口状态"""
        if not hasattr(self.virtual_serial, 'running') or not self.virtual_serial.running:
            # 创建虚拟串口
            success, result = self.virtual_serial.open()
            if success:
                # 设置回调函数
                self.virtual_serial.set_callback(self.handle_received_data)
                self.device_name_label.setText(result)
                self.status_label.setText("已创建")
                self.create_button.setText("关闭串口")
                self.log_message("系统", f"虚拟串口已创建，设备名: {result}")
            else:
                self.log_message("错误", f"创建虚拟串口失败: {result}")
        else:
            # 关闭虚拟串口
            self.virtual_serial.close()
            self.device_name_label.setText("")
            self.status_label.setText("未创建")
            self.create_button.setText("创建串口")
            self.log_message("系统", "虚拟串口已关闭")
    
    def send_data(self):
        """发送数据"""
        if not hasattr(self.virtual_serial, 'running') or not self.virtual_serial.running:
            self.log_message("错误", "虚拟串口未创建，无法发送")
            return
        
        # 获取输入数据
        data = self.data_input.text().strip()
        if not data:
            return
        
        # 发送数据
        if self.virtual_serial.hex_mode:
            # 十六进制模式
            try:
                # 移除所有空格
                hex_str = data.replace(" ", "")
                # 确保字符串长度为偶数
                if len(hex_str) % 2 != 0:
                    hex_str = "0" + hex_str
                # 转换为字节
                bytes_data = bytes.fromhex(hex_str)
                self.virtual_serial.write(bytes_data)
                self.log_message("发送", f"十六进制: {hex_str.upper()}")
            except ValueError as e:
                self.log_message("错误", f"无效的十六进制字符串: {str(e)}")
        else:
            # 文本模式
            self.virtual_serial.write(data)
            self.log_message("发送", f"文本: '{data}'")
        
        # 清空输入框
        self.data_input.clear()
    
    def handle_received_data(self, data):
        """处理接收到的数据"""
        if self.virtual_serial.hex_mode:
            # 十六进制模式：显示十六进制数据
            hex_data = data.hex().upper()
            # 每两个字符添加一个空格
            hex_data = ' '.join(hex_data[i:i+2] for i in range(0, len(hex_data), 2))
            self.log_signals.log_message.emit("接收", f"十六进制: {hex_data}", time.strftime('%H:%M:%S', time.localtime()))
            # 直接回显原始数据
            self.virtual_serial.write(data)
            self.log_signals.log_message.emit("回显", f"十六进制: {hex_data}", time.strftime('%H:%M:%S', time.localtime()))
        else:
            # 文本模式：显示文本数据
            try:
                text_data = data.decode('utf-8', errors='replace')
                self.log_signals.log_message.emit("接收", f"文本: '{text_data.strip()}'", time.strftime('%H:%M:%S', time.localtime()))
                # 直接回显原始数据
                self.virtual_serial.write(data)
                self.log_signals.log_message.emit("回显", f"文本: '{text_data.strip()}'", time.strftime('%H:%M:%S', time.localtime()))
            except Exception as e:
                self.log_signals.log_message.emit("错误", f"解码数据失败: {str(e)}", time.strftime('%H:%M:%S', time.localtime()))
    
    def _update_log(self, category, message, time_str):
        """安全地更新日志显示"""
        full_message = f"[{time_str}] [{category}] {message}"
        self.log_text.append(full_message)
        # 滚动到底部
        cursor = self.log_text.textCursor()
        cursor.movePosition(QTextCursor.End)
        self.log_text.setTextCursor(cursor)
    
    def log_message(self, category, message):
        """记录消息到日志区域"""
        time_str = time.strftime('%H:%M:%S', time.localtime())
        self.log_signals.log_message.emit(category, message, time_str)
    
    def clear_log(self):
        """清除日志"""
        self.log_text.clear()
    
    def closeEvent(self, event):
        """窗口关闭事件处理"""
        # 关闭虚拟串口
        if hasattr(self.virtual_serial, 'running') and self.virtual_serial.running:
            self.virtual_serial.close()
        
        super().closeEvent(event)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = SerialSimulator()
    window.show()
    sys.exit(app.exec_()) 