"""
轮廓设置相关组件
"""
from PyQt5.QtWidgets import (QWidget, QLabel, QPushButton, QLineEdit, 
                           QVBoxLayout, QHBoxLayout, QGridLayout, QMessageBox, QGroupBox)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QDoubleValidator
from .base_components import default_font, GroupFrame, LabeledComboBox, LabeledButton

class ContourSettings(QWidget):
    """轮廓设置组件"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self._init_ui()
    
    def _init_ui(self):
        """初始化UI"""
        # 创建主布局
        layout = QVBoxLayout()
        
        # 轮廓参数设置组
        params_group = GroupFrame("轮廓参数")
        params_layout = QVBoxLayout()
        
        # 速度设置行
        speed_layout = QHBoxLayout()
        speed_layout.addWidget(QLabel("速度:"))
        self.speed_edits = []
        for i in range(6):
            edit = QLineEdit("0.0667")
            edit.setValidator(QDoubleValidator(0.1, 10.0, 1))
            edit.setFont(default_font)
            edit.setMaximumWidth(80)
            self.speed_edits.append(edit)
            speed_layout.addWidget(edit)
            speed_layout.addWidget(QLabel("rad/s"))
        params_layout.addLayout(speed_layout)
        
        # 加速度设置行
        accel_layout = QHBoxLayout()
        accel_layout.addWidget(QLabel("加速度:"))
        self.accel_edits = []
        for i in range(6):
            edit = QLineEdit("0.0667")
            edit.setValidator(QDoubleValidator(0.1, 10.0, 1))
            edit.setFont(default_font)
            edit.setMaximumWidth(80)
            self.accel_edits.append(edit)
            accel_layout.addWidget(edit)
            accel_layout.addWidget(QLabel("rad/s²"))
        params_layout.addLayout(accel_layout)
        
        # 减速度设置行
        decel_layout = QHBoxLayout()
        decel_layout.addWidget(QLabel("减速度:"))
        self.decel_edits = []
        for i in range(6):
            edit = QLineEdit("0.0667")
            edit.setValidator(QDoubleValidator(0.1, 10.0, 1))
            edit.setFont(default_font)
            edit.setMaximumWidth(80)
            self.decel_edits.append(edit)
            decel_layout.addWidget(edit)
            decel_layout.addWidget(QLabel("rad/s²"))
        params_layout.addLayout(decel_layout)
        
        params_group.add_layout(params_layout)
        layout.addWidget(params_group)
        
        self.setLayout(layout)
    
    def get_contour_params(self):
        """获取轮廓参数"""
        try:
            return [float(edit.text()) for edit in self.speed_edits], [float(edit.text()) for edit in self.accel_edits], [float(edit.text()) for edit in self.decel_edits]
        except ValueError:
            return None
