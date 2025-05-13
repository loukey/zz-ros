"""
执行器设置相关组件
"""
from PyQt5.QtWidgets import (QWidget, QLabel, QPushButton, QLineEdit, 
                           QVBoxLayout, QHBoxLayout, QGridLayout, QMessageBox, QGroupBox)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QDoubleValidator
from .base_components import default_font, GroupFrame, LabeledComboBox, LabeledButton

class EffectorFrame(QGroupBox):
    """执行器设置组件"""
    send_effector_command_requested = pyqtSignal(dict)
    
    def __init__(self, parent=None, get_encoding_type=None):
        super().__init__("夹爪设置", parent)
        self.get_encoding_type = get_encoding_type
        self._init_ui()

    def _init_ui(self):
        """初始化UI"""
        # 创建主布局
        layout = QVBoxLayout()
        
        # 创建水平布局用于命令选择和参数值
        params_layout = QHBoxLayout()
        params_layout.setAlignment(Qt.AlignmentFlag.AlignLeft)  # 设置左对齐
        
        # 命令选择
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
        self.send_button.clicked.connect(lambda: self.send_effector_command_requested.emit({'encoding_type': self.get_encoding_type(),
                                                                                            'effector_params': self.get_effector_params()}))
        self.send_button.setEnabled(False)  # 初始状态为禁用
        button_layout.addWidget(self.send_button)
        
        # 添加伸缩项，让按钮向左对齐
        button_layout.addStretch(1)
        
        layout.addLayout(button_layout)
        
        self.setLayout(layout)
    
    def get_effector_params(self):
        """获取执行器参数"""
        try:
            command_text = self.command_combo.get_selected_item()
            command = self.command_mode[command_text]
            return [command, float(self.param_edit.text())]
        except (ValueError, KeyError):
            return None
            
    def update_connection_status(self, is_connected):
        self.send_button.setEnabled(is_connected)
