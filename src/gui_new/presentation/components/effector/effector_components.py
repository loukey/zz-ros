"""
Effector frame for Main tab
"""
from PyQt5.QtWidgets import (QWidget, QLabel, QPushButton, QLineEdit, 
                           QVBoxLayout, QHBoxLayout, QGridLayout, QMessageBox, QGroupBox)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QDoubleValidator
from ..base_component import BaseComponent, default_font, LabeledComboBox


class EffectorFrame(BaseComponent):
    """执行器设置组件"""
    send_effector_command_requested = pyqtSignal(dict)
    
    def __init__(self, parent=None, view_model=None):
        super().__init__(parent, view_model)

    def setup_ui(self):
        """设置UI"""
        # 创建分组框
        group_box = QGroupBox("夹爪设置")
        main_layout = QVBoxLayout(self)
        main_layout.addWidget(group_box)
        
        # 创建主布局
        layout = QVBoxLayout(group_box)
        
        # 创建水平布局用于命令选择和参数值
        params_layout = QHBoxLayout()
        params_layout.setAlignment(Qt.AlignmentFlag.AlignLeft)  # 设置左对齐
        
        # 命令选择 - 使用LabeledComboBox基础组件
        self.command_combo = LabeledComboBox("命令选择:", [
            "00: 不进行任何操作",
            "01: 夹爪手动使能",
            "02: 设置夹爪目标位置",
            "03: 设置夹爪速度",
            "04: 设置夹爪电流",
            "05: 查询夹爪抓取状态",
            "06: 查询夹爪目前位置",
            "07: 查询夹爪电流"
        ])
        self.command_mode = {
            "00: 不进行任何操作": 0x00,
            "01: 夹爪手动使能": 0x01,
            "02: 设置夹爪目标位置": 0x02,
            "03: 设置夹爪速度": 0x03,
            "04: 设置夹爪电流": 0x04,
            "05: 查询夹爪抓取状态": 0x05,
            "06: 查询夹爪目前位置": 0x06,
            "07: 查询夹爪电流": 0x07
        }
        params_layout.addWidget(self.command_combo)
        
        # 参数输入
        param_layout = QHBoxLayout()
        param_layout.setAlignment(Qt.AlignmentFlag.AlignLeft)  # 设置左对齐
        param_layout.addWidget(QLabel("参数值:"))
        self.param_edit = QLineEdit("0.0")
        self.param_edit.setValidator(QDoubleValidator(-1000.0, 1000.0, 2))
        self.param_edit.setFont(default_font)
        self.param_edit.setMinimumWidth(100)
        param_layout.addWidget(self.param_edit)
        params_layout.addLayout(param_layout)
        
        # 添加伸缩项，让前面的内容向左对齐
        params_layout.addStretch(1)
        
        layout.addLayout(params_layout)
        
        # 控制按钮
        button_layout = QHBoxLayout()
        button_layout.setAlignment(Qt.AlignmentFlag.AlignLeft)  # 设置左对齐
        
        self.send_button = QPushButton("发送")
        self.send_button.setFont(default_font)
        self.send_button.clicked.connect(lambda: self.send_effector_command_requested.emit({
            'effector_params': self.get_effector_params()
        }))
        self.send_button.setEnabled(False)  # 初始状态为禁用
        button_layout.addWidget(self.send_button)
        
        # 添加伸缩项，让按钮向左对齐
        button_layout.addStretch(1)
        
        layout.addLayout(button_layout)
    
    def connect_signals(self):
        """连接视图模型信号"""
        if self.view_model:
            self.view_model.connection_status_changed.connect(self.update_connection_status)
    
    def get_effector_params(self):
        """获取执行器参数"""
        try:
            command_text = self.command_combo.current_text()
            command = self.command_mode[command_text]
            return [command, float(self.param_edit.text())]
        except (ValueError, KeyError):
            return None
            
    def update_connection_status(self, is_connected):
        """更新连接状态"""
        self.send_button.setEnabled(is_connected)


