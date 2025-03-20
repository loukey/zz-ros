"""
运动学相关组件
"""
from PyQt5.QtWidgets import (QWidget, QLabel, QPushButton, QLineEdit, 
                           QVBoxLayout, QHBoxLayout, QGridLayout, QMessageBox, QGroupBox)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QDoubleValidator
from gui.components.base_components import default_font, GroupFrame
from math import pi, degrees, radians


class InverseKinematicFrame(QWidget):
    """逆运动学计算框架"""
    
    def __init__(self, parent=None, inverse_callback=None):
        super().__init__(parent)
        self.inverse_callback = inverse_callback
        self.apply_callback = None
        self.calculated_angles = None
        self._init_ui()
    
    def _init_ui(self):
        """初始化UI"""
        # 创建主布局
        layout = QVBoxLayout()
        
        # 创建输入参数组
        input_group = QGroupBox("末端位置与姿态")
        input_layout = QGridLayout()
        
        # 位置输入
        input_layout.addWidget(QLabel("X 位置"), 0, 0)
        input_layout.addWidget(QLabel("Y 位置"), 0, 1)
        input_layout.addWidget(QLabel("Z 位置"), 0, 2)
        
        self.position_inputs = []
        for i in range(3):
            input_edit = QLineEdit("0.0")
            input_edit.setValidator(QDoubleValidator())
            input_edit.setFont(default_font)
            self.position_inputs.append(input_edit)
            input_layout.addWidget(input_edit, 1, i)
        
        # 姿态输入
        input_layout.addWidget(QLabel("A 角度"), 2, 0)
        input_layout.addWidget(QLabel("B 角度"), 2, 1)
        input_layout.addWidget(QLabel("C 角度"), 2, 2)
        
        self.euler_inputs = []
        for i in range(3):
            input_edit = QLineEdit("0.0")
            input_edit.setValidator(QDoubleValidator())
            input_edit.setFont(default_font)
            self.euler_inputs.append(input_edit)
            input_layout.addWidget(input_edit, 3, i)
        
        # 添加计算按钮
        self.calculate_button = QPushButton("计算逆运动学")
        self.calculate_button.setFont(default_font)
        self.calculate_button.clicked.connect(self.calculate_inverse_kinematics)
        input_layout.addWidget(self.calculate_button, 4, 0, 1, 3)
        
        input_group.setLayout(input_layout)
        layout.addWidget(input_group)
        
        # 创建结果显示组
        result_group = QGroupBox("计算结果")
        result_layout = QVBoxLayout()
        
        self.result_label = QLabel("请输入位置和姿态，然后点击计算")
        self.result_label.setFont(default_font)
        result_layout.addWidget(self.result_label)
        
        # 添加结果显示区域
        self.result_layout = QGridLayout()
        for i in range(6):
            col = i % 3
            row = i // 3
            self.result_layout.addWidget(QLabel(f"关节{i+1}:"), row, col*2)
            
            result_label = QLabel("0.0000")
            result_label.setFont(default_font)
            self.result_layout.addWidget(result_label, row, col*2+1)
            
        result_layout.addLayout(self.result_layout)
        
        # 添加应用按钮
        self.apply_button = QPushButton("应用到角度控制")
        self.apply_button.setFont(default_font)
        self.apply_button.setEnabled(False)
        self.apply_button.clicked.connect(self.apply_result)
        result_layout.addWidget(self.apply_button)
        
        result_group.setLayout(result_layout)
        layout.addWidget(result_group)
        
        # 设置布局
        self.setLayout(layout)
    
    def calculate_inverse_kinematics(self):
        """
        计算逆运动学
        """
        try:
            # 获取输入值
            position = [float(self.position_inputs[i].text()) for i in range(3)]
            euler = [float(self.euler_inputs[i].text()) for i in range(3)]
            
            # 调用回调函数计算逆运动学
            if self.inverse_callback:
                self.calculated_angles = self.inverse_callback(
                    position[0], position[1], position[2], 
                    euler[0], euler[1], euler[2]
                )
                
                if self.calculated_angles:
                    # 更新结果显示
                    self.update_result(self.calculated_angles)
                    self.apply_button.setEnabled(True)
                else:
                    self.apply_button.setEnabled(False)
                    self.result_label.setText("计算失败，请检查输入值")
            else:
                self.apply_button.setEnabled(False)
                self.result_label.setText("未配置逆运动学计算函数")
        except ValueError:
            self.apply_button.setEnabled(False)
            self.result_label.setText("请输入有效的数值")
    
    def update_result(self, angles):
        """
        更新计算结果显示
        
        参数:
            angles: 关节角度列表
        """
        # 更新结果标签
        self.result_label.setText("计算完成，得到的关节角度 (弧度)：")
        
        # 更新各关节角度值显示
        for i, angle in enumerate(angles):
            if i < 6:
                col = i % 3
                row = i // 3
                widget = self.result_layout.itemAtPosition(row, col*2+1).widget()
                if widget:
                    widget.setText(f"{angle:.6f}")
    
    def set_apply_callback(self, callback):
        """
        设置应用结果的回调函数
        
        参数:
            callback: 应用结果的回调函数
        """
        self.apply_callback = callback
    
    def apply_result(self):
        """应用计算结果"""
        if self.calculated_angles and self.apply_callback:
            self.apply_callback(self.calculated_angles)
    
    def get_result_angles(self):
        """
        获取计算结果
        
        返回:
            angles: 关节角度列表
        """
        return self.calculated_angles 