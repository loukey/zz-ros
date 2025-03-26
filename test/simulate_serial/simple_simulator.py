#!/usr/bin/env python3
"""
文本串口模拟器
- 创建虚拟串口
- 按行(\r\n)接收文本命令
- 按空格分割命令部分
- 使用UTF-8编码
- 收到消息后原样回显
"""
import sys
import os
import time
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                            QHBoxLayout, QLabel, QPushButton, QTextEdit, 
                            QGroupBox, QLineEdit)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QTextCursor, QFont

# 添加当前目录到Python路径
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, current_dir)

from virtual_serial import VirtualSerial
from crc import calculate_crc16


class TextCommandSimulator(QMainWindow):
    """文本命令串口模拟器窗口"""
    
    def __init__(self):
        super().__init__()
        
        # 初始化虚拟串口
        self.virtual_serial = VirtualSerial()
        
        self.setWindowTitle("文本串口模拟器")
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
        
        # 命令输入区域
        input_group = QGroupBox("命令输入")
        input_layout = QHBoxLayout()
        
        self.command_input = QLineEdit()
        self.command_input.setPlaceholderText("输入命令，按回车发送 (例如: cmd 01 08 78623 369707 83986 391414 508006 455123 B9FC)")
        self.command_input.returnPressed.connect(self.send_command)
        input_layout.addWidget(self.command_input)
        
        send_button = QPushButton("发送")
        send_button.clicked.connect(self.send_command)
        input_layout.addWidget(send_button)
        
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
    
    def send_command(self):
        """发送命令"""
        if not hasattr(self.virtual_serial, 'running') or not self.virtual_serial.running:
            self.log_message("错误", "虚拟串口未创建，无法发送")
            return
        
        # 获取命令文本
        command = self.command_input.text().strip()
        if not command:
            return
        
        # 发送命令
        self.virtual_serial.write(command)
        self.log_message("发送", f"'{command.strip()}'")
        
        # 清空输入框
        self.command_input.clear()
    
    def handle_received_data(self, data):
        """处理接收到的数据"""
        # 强制解码为UTF-8文本，出错时替换为问号
        text_data = data.decode('utf-8', errors='replace')
        self.log_message("接收", f"'{text_data.strip()}'")
        
        # 如果以cmd或msg开头，按空格分割解析命令
        if text_data.lower().startswith('cmd') or text_data.lower().startswith('msg'):
            self.parse_text_command(text_data)
            
            # 构造修改后的回显消息
            parts = text_data.strip().split()
            if len(parts) >= 9:  # cmd/msg + 控制 + 模式 + 6个位置值
                # 构建新消息：msg + 控制字节 + 模式字节 + 6个位置值 + 6个状态字 + 6个异常值 + CRC
                new_parts = ['msg', parts[1], parts[2]]  # 替换为msg，保留控制字节和模式字节
                new_parts.extend(parts[3:9])  # 添加6个位置值
                
                # 添加6个状态字，均为1250
                new_parts.extend(['1250'] * 6)
                
                # 添加6个异常值，均为0
                new_parts.extend(['0'] * 6)
                
                # 计算CRC - 使用纯文本字符串
                # 构造待计算CRC的消息字符串（不包含CRC部分）
                crc_message = ' '.join(new_parts)
                # 使用CRC模块计算CRC-16
                crc_value = calculate_crc16(crc_message)
                # 将CRC值转换为4位十六进制字符串，大写
                crc_hex = f"{crc_value:04X}"
                # 添加CRC到消息
                new_parts.append(crc_hex)
                
                # 构造新消息（确保使用正确的行尾符号）
                new_message = ' '.join(new_parts)
                # 直接添加实际的CR和LF字符
                message_with_crlf = new_message + '\r\n'
                
                # 简化日志，移除十六进制调试
                self.log_message("调试", f"构造的消息: '{new_message}'")
                
                # 回显修改后的消息 - 使用纯文本字符串
                self.virtual_serial.write(message_with_crlf.encode('utf-8'))
                self.log_message("回显", f"'{new_message}'")
                return  # 已经处理完毕，不执行原来的回显代码
        
        # 如果不是以cmd或msg开头，原样回显
        self.virtual_serial.write(data)
        self.log_message("回显", "已回显接收到的数据")
    
    def parse_text_command(self, command_text):
        """解析文本命令
        
        参数:
            command_text: 文本命令字符串
        """
        # 去除行尾，按空格分割
        parts = command_text.strip().split()
        if len(parts) < 3:
            self.log_message("解析", "命令格式不完整")
            return
        
        # 提取命令各部分
        prefix = parts[0]           # 命令前缀 (cmd 或 msg)
        control = parts[1]          # 控制字节
        mode = parts[2]             # 模式字节
        
        self.log_message("解析", f"命令: {prefix}")
        self.log_message("解析", f"控制字节: {control}")
        self.log_message("解析", f"模式字节: {mode}")
        
        # 提取位置数据（如果有）
        if len(parts) >= 9:  # cmd/msg + 控制 + 模式 + 6个位置值
            positions = parts[3:9]  # 直接提取原始字符串
            self.log_message("解析", f"位置数据: {positions}")
        
        # 提取状态字（如果有）
        if len(parts) >= 15:  # cmd/msg + 控制 + 模式 + 6个位置值 + 6个状态字
            status = parts[9:15]  # 提取状态字
            self.log_message("解析", f"状态字: {status}")
        
        # 提取异常值（如果有）
        if len(parts) >= 21:  # cmd/msg + 控制 + 模式 + 6个位置值 + 6个状态字 + 6个异常值
            errors = parts[15:21]  # 提取异常值
            self.log_message("解析", f"异常值: {errors}")
        
        # 提取CRC（如果有）
        if len(parts) >= 22:
            crc = parts[21]
            self.log_message("解析", f"CRC16: {crc}")
        elif len(parts) >= 10 and len(parts) < 15:
            crc = parts[9]
            self.log_message("解析", f"CRC16: {crc}")
    
    def log_message(self, category, message):
        """记录消息到日志区域"""
        time_str = time.strftime('%H:%M:%S', time.localtime())
        full_message = f"[{time_str}] [{category}] {message}"
        
        self.log_text.append(full_message)
        # 滚动到底部
        cursor = self.log_text.textCursor()
        cursor.movePosition(QTextCursor.End)
        self.log_text.setTextCursor(cursor)
    
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
    window = TextCommandSimulator()
    window.show()
    sys.exit(app.exec_()) 