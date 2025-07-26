"""
显示相关组件
"""
from PyQt5.QtWidgets import (QWidget, QLabel, QTextEdit, QPushButton, 
                           QVBoxLayout, QHBoxLayout, QSplitter, QGroupBox)
from PyQt5.QtCore import Qt, QDateTime
from .base_component import BaseComponent, default_font, text_font


class DataDisplayFrame(BaseComponent):
    """数据显示区域"""
    
    def __init__(self, parent=None, view_model=None):
        super().__init__(parent, view_model)
    
    def setup_ui(self):
        """设置UI"""
        # 创建分组框
        group_box = QGroupBox("数据显示")
        main_layout = QVBoxLayout(self)
        main_layout.addWidget(group_box)
        
        layout = QVBoxLayout(group_box)
        
        # 创建发送区
        send_group = QGroupBox("发送区")
        send_layout = QVBoxLayout()
        self.send_text = QTextEdit()
        self.send_text.setReadOnly(True)
        send_layout.addWidget(self.send_text)
        send_group.setLayout(send_layout)
        
        # 创建接收区
        receive_group = QGroupBox("接收区")
        receive_layout = QVBoxLayout()
        self.receive_text = QTextEdit()
        self.receive_text.setReadOnly(True)
        receive_layout.addWidget(self.receive_text)
        receive_group.setLayout(receive_layout)
        
        # 创建按钮区
        button_layout = QHBoxLayout()
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
            self.view_model.message_received.connect(self.append_message)
    
    def append_message(self, message, message_type="接收"):
        """添加消息到显示区"""
        timestamp = QDateTime.currentDateTime().toString("HH:mm:ss.zzz")
        
        # 使用[类型]格式显示不同类型的消息，每种类型对应不同颜色
        if message_type in ["发送", "控制", "参数", "调试", "轨迹", "摄像头"]:  # 这些类型显示在发送区
            # 根据消息类型设置颜色
            if message_type == "发送":
                color = "#007ACC"  # 蓝色
            elif message_type == "控制":
                color = "#E91E63"  # 粉色
            elif message_type == "调试":
                color = "#FF9800"  # 橙色 
            elif message_type == "轨迹":
                color = "#FF9800"  # 橙色
            else:  # 参数
                color = "#FF9800"  # 橙色
                
            formatted_message = f"<span style='color:{color};'>[{timestamp}][{message_type}]</span> {message}"
            self.send_text.append(formatted_message)
            self.send_text.verticalScrollBar().setValue(
                self.send_text.verticalScrollBar().maximum()
            )
        else:  # 接收、错误、信息、系统等类型显示在接收区
            if message_type == "接收":
                color = "#28A745"  # 绿色
            elif message_type == "错误":
                color = "#DC3545"  # 红色
            elif message_type == "信息":
                color = "#6C757D"  # 灰色
            elif message_type == "系统":
                color = "#9C27B0"  # 紫色
            else:
                color = "#6C757D"  # 默认灰色
                
            formatted_message = f"<span style='color:{color};'>[{timestamp}][{message_type}]</span> {message}"
            self.receive_text.append(formatted_message)
            self.receive_text.verticalScrollBar().setValue(
                self.receive_text.verticalScrollBar().maximum()
            )
    
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