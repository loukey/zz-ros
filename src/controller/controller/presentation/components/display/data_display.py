"""
Data display component for right panel
"""
from PyQt5.QtWidgets import (QWidget, QLabel, QTextEdit, QPushButton,
                           QVBoxLayout, QHBoxLayout, QSplitter, QGroupBox)
from PyQt5.QtCore import Qt, QDateTime, pyqtSignal
from PyQt5.QtGui import QTextCursor, QTextCharFormat, QColor
from ..base_component import BaseComponent, default_font, text_font
from controller.presentation.theme import MSG_COLOR_MAP, TEXT_PRIMARY


class DataDisplayFrame(BaseComponent):
    """数据显示区域"""
    
    # 定义停止命令信号
    stop_command_requested = pyqtSignal(dict)
    
    def __init__(self, parent=None, view_model=None, control_vm=None):
        self.control_vm = control_vm  # 控制视图模型（用于发送停止命令）
        super().__init__(parent, view_model)
    
    def setup_ui(self):
        """设置UI"""
        # 创建分组框
        group_box = QGroupBox("数据显示")
        group_box.setFont(default_font)
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.addWidget(group_box)

        layout = QVBoxLayout(group_box)
        layout.setContentsMargins(8, 6, 8, 6)
        layout.setSpacing(6)
        
        # 创建发送区
        send_group = QGroupBox("发送区")
        send_group.setFont(default_font)  # 设置为10号字体
        send_layout = QVBoxLayout()
        self.send_text = QTextEdit()
        self.send_text.setReadOnly(True)
        send_layout.addWidget(self.send_text)
        send_group.setLayout(send_layout)
        
        # 创建接收区
        receive_group = QGroupBox("接收区")
        receive_group.setFont(default_font)  # 设置为10号字体
        receive_layout = QVBoxLayout()
        self.receive_text = QTextEdit()
        self.receive_text.setReadOnly(True)
        receive_layout.addWidget(self.receive_text)
        receive_group.setLayout(receive_layout)
        
        # 创建按钮区
        button_layout = QHBoxLayout()
        
        # 立即停止按钮（红色）
        self.stop_btn = QPushButton("立即停止")
        self.stop_btn.setFont(default_font)
        self.stop_btn.setStyleSheet(
            "QPushButton { background-color: #FF4444; color: white; font-weight: bold; "
            "border-radius: 4px; padding: 5px 14px; }"
            "QPushButton:hover { background-color: #FF6666; }"
            "QPushButton:pressed { background-color: #CC0000; }"
            "QPushButton:disabled { background-color: #CCCCCC; color: #666666; }")
        self.stop_btn.clicked.connect(self.send_stop_command)
        self.stop_btn.setEnabled(False)  # 默认禁用，连接串口后启用
        button_layout.addWidget(self.stop_btn)
        
        self.clear_send_btn = QPushButton("清除发送区")
        self.clear_send_btn.setFont(default_font)
        self.clear_send_btn.clicked.connect(self.clear_send)
        self.clear_receive_btn = QPushButton("清除接收区")
        self.clear_receive_btn.setFont(default_font)
        self.clear_receive_btn.clicked.connect(self.clear_receive)
        self.clear_all_btn = QPushButton("清除所有")
        self.clear_all_btn.setFont(default_font)
        self.clear_all_btn.clicked.connect(self.clear_all)
        
        button_layout.addWidget(self.clear_send_btn)
        button_layout.addWidget(self.clear_receive_btn)
        button_layout.addWidget(self.clear_all_btn)
        
        # 添加所有组件到主布局
        layout.addWidget(send_group)
        layout.addWidget(receive_group)
        layout.addLayout(button_layout)
    
    def connect_signals(self):
        """连接视图模型信号"""
        if self.view_model:
            # 连接消息显示信号
            self.view_model.message_display_signal.connect(self.append_message)
            # 连接清除消息信号
            self.view_model.clear_requested.connect(self.clear_all)
        
        if self.control_vm:
            # 连接串口连接状态，控制停止按钮的启用/禁用
            self.control_vm.connection_status_changed.connect(self.update_stop_button_status)
            # 连接停止命令信号
            self.stop_command_requested.connect(self.control_vm.send_command)
    
    def _insert_colored(self, text_edit, text, color_hex):
        """使用 QTextCharFormat 带颜色插入文本"""
        cursor = text_edit.textCursor()
        cursor.movePosition(QTextCursor.End)
        fmt = QTextCharFormat()
        fmt.setForeground(QColor(color_hex))
        cursor.insertText(text, fmt)
        text_edit.setTextCursor(cursor)
        text_edit.moveCursor(QTextCursor.End)

    def append_message(self, message, message_type="接收"):
        """添加消息到显示区（带颜色区分）"""
        timestamp = QDateTime.currentDateTime().toString("HH:mm:ss.zzz")
        formatted_message = f"[{timestamp}][{message_type}] {message}\n"
        color = MSG_COLOR_MAP.get(message_type, TEXT_PRIMARY)

        if message_type in ["发送", "控制", "参数", "调试", "轨迹", "摄像头"]:
            self._insert_colored(self.send_text, formatted_message, color)
        else:
            self._insert_colored(self.receive_text, formatted_message, color)
    
    def clear_send(self):
        """清除发送区域"""
        self.send_text.clear()
    
    def clear_receive(self):
        """清除接收区域"""
        self.receive_text.clear()
    
    def clear_all(self):
        """清除所有区域"""
        self.send_text.clear()
        self.receive_text.clear()
    
    def send_stop_command(self):
        """发送立即停止命令"""
        # 发送 0x05 (立刻停止) 命令，模式使用默认的周期同步位置模式 0x08
        self.stop_command_requested.emit({
            'control': 0x05,
            'mode': 0x08
        })
    
    def update_stop_button_status(self, connected):
        """更新停止按钮的启用状态"""
        self.stop_btn.setEnabled(connected)


