"""
执行器设置相关组件
"""
from PyQt5.QtWidgets import (QWidget, QLabel, QPushButton, QLineEdit, 
                           QVBoxLayout, QHBoxLayout, QGridLayout, QMessageBox, QGroupBox)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QDoubleValidator
from .base_components import default_font, GroupFrame, LabeledComboBox, LabeledButton

class EffectorSettings(QWidget):
    """执行器设置组件"""
    
    def __init__(self, parent=None, send_effector_command_callback=None):
        super().__init__(parent)
        self.send_effector_command_callback = send_effector_command_callback
        self._init_ui()

    def _init_ui(self):
        """初始化UI"""
        # 创建主布局
        layout = QVBoxLayout()
        
        # 执行器参数设置组
        params_group = GroupFrame("执行器参数")
        params_layout = QHBoxLayout()
        
        # 命令选择
        params_layout.addWidget(QLabel("命令选择:"))

        self.command_combo = LabeledComboBox("", [
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
        params_layout.addWidget(QLabel("参数值:"))
        self.param_edit = QLineEdit("0.0")
        self.param_edit.setValidator(QDoubleValidator(-1000.0, 1000.0, 2))
        self.param_edit.setFont(default_font)
        params_layout.addWidget(self.param_edit)
        
        params_group.add_layout(params_layout)
        layout.addWidget(params_group)
        
        # 控制按钮
        button_layout = QHBoxLayout()
        self.send_button = QPushButton("发送")
        self.send_button.setFont(default_font)
        self.send_button.clicked.connect(self.send_effector_command_callback)
        button_layout.addWidget(self.send_button)
        
        layout.addLayout(button_layout)
        
        self.setLayout(layout)
    
    def get_effector_params(self):
        """获取执行器参数"""

        command_text = self.command_combo.combobox.currentText()
        command = self.command_mode[command_text]
        return command, float(self.param_edit.text())

