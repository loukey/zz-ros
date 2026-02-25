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
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.addWidget(group_box)

        # 单行布局：命令选择 + 参数值 + 发送按钮
        row_layout = QHBoxLayout(group_box)
        row_layout.setContentsMargins(8, 6, 8, 6)
        row_layout.setSpacing(8)
        row_layout.setAlignment(Qt.AlignmentFlag.AlignLeft)

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
        row_layout.addWidget(self.command_combo)

        # 参数输入
        param_label = QLabel("参数值:")
        param_label.setFont(default_font)
        row_layout.addWidget(param_label)
        self.param_edit = QLineEdit("0.0")
        self.param_edit.setValidator(QDoubleValidator(-1000.0, 1000.0, 2))
        self.param_edit.setFont(default_font)
        self.param_edit.setMinimumWidth(100)
        row_layout.addWidget(self.param_edit)

        # 发送按钮
        self.send_button = QPushButton("发送")
        self.send_button.setFont(default_font)
        self.send_button.clicked.connect(self._on_send_clicked)
        self.send_button.setEnabled(False)
        row_layout.addWidget(self.send_button)

        row_layout.addStretch(1)
    
    def connect_signals(self):
        """连接视图模型信号"""
        if self.view_model:
            # 连接连接状态信号
            self.view_model.connection_status_changed.connect(self.update_connection_status)
            
            # 连接发送命令信号到 ViewModel
            self.send_effector_command_requested.connect(
                self.view_model.send_effector_command
            )
    
    def _on_send_clicked(self):
        """处理发送按钮点击"""
        params = self.get_effector_params()
        if params is None:
            QMessageBox.warning(self, "参数错误", "请输入有效的参数值")
            return
        
        effector_mode, effector_data = params
        
        # 构建完整的命令字典
        command_dict = {
            'control': 0x00,  # 系统指令（夹爪控制）
            'mode': 0x08,     # 默认模式
            'effector_mode': effector_mode,
            'effector_data': effector_data
        }
        
        # 发射信号给 ViewModel
        self.send_effector_command_requested.emit(command_dict)
    
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


