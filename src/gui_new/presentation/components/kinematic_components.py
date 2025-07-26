"""
运动学相关UI组件
"""
from PyQt5.QtWidgets import (QWidget, QLabel, QPushButton, QLineEdit, 
                           QVBoxLayout, QHBoxLayout, QGridLayout, QMessageBox, QGroupBox, QTabWidget)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QDoubleValidator
from .base_component import BaseComponent, default_font


class InverseKinematicFrame(BaseComponent):
    """逆运动学计算框架"""
    
    def __init__(self, parent=None, view_model=None, inverse_callback=None):
        self.inverse_callback = inverse_callback
        self.apply_callback = None
        self.calculated_angles = None
        super().__init__(parent, view_model)
    
    def setup_ui(self):
        """设置UI"""
        # 创建主布局
        layout = QVBoxLayout(self)
        
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
    
    def connect_signals(self):
        """连接视图模型信号"""
        if self.view_model:
            self.view_model.calculation_completed.connect(self.on_calculation_completed)
    
    def calculate_inverse_kinematics(self):
        """计算逆运动学"""
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
    
    def on_calculation_completed(self, result):
        """计算完成回调"""
        if result.get('success', False):
            angles = result.get('angles', [])
            self.calculated_angles = angles
            self.update_result(angles)
            self.apply_button.setEnabled(True)
        else:
            error_msg = result.get('error', '计算失败')
            self.result_label.setText(f"计算失败：{error_msg}")
            self.apply_button.setEnabled(False)
    
    def update_result(self, angles):
        """更新计算结果显示"""
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
        """设置应用结果的回调函数"""
        self.apply_callback = callback
    
    def apply_result(self):
        """应用计算结果"""
        if self.calculated_angles and self.apply_callback:
            self.apply_callback(self.calculated_angles)
    
    def get_result_angles(self):
        """获取计算结果"""
        return self.calculated_angles


class EndPositionFrame(BaseComponent):
    """末端姿态显示区域"""
    
    def __init__(self, parent=None, view_model=None):
        super().__init__(parent, view_model)
    
    def setup_ui(self):
        """设置UI"""
        # 创建分组框
        group_box = QGroupBox("末端姿态")
        main_layout = QVBoxLayout(self)
        main_layout.addWidget(group_box)
        
        layout = QGridLayout(group_box)
        
        # 理论位置
        layout.addWidget(QLabel("理论位置:"), 0, 0)
        self.theoretical_x = QLineEdit()
        self.theoretical_y = QLineEdit()
        self.theoretical_z = QLineEdit()
        
        for widget in [self.theoretical_x, self.theoretical_y, self.theoretical_z]:
            widget.setReadOnly(True)
            
        layout.addWidget(QLabel("X:"), 0, 1)
        layout.addWidget(self.theoretical_x, 0, 2)
        layout.addWidget(QLabel("Y:"), 0, 3)
        layout.addWidget(self.theoretical_y, 0, 4)
        layout.addWidget(QLabel("Z:"), 0, 5)
        layout.addWidget(self.theoretical_z, 0, 6)
        
        # 实际位置
        layout.addWidget(QLabel("实际位置:"), 1, 0)
        self.actual_x = QLineEdit()
        self.actual_y = QLineEdit()
        self.actual_z = QLineEdit()
        
        for widget in [self.actual_x, self.actual_y, self.actual_z]:
            widget.setReadOnly(True)
            
        layout.addWidget(QLabel("X:"), 1, 1)
        layout.addWidget(self.actual_x, 1, 2)
        layout.addWidget(QLabel("Y:"), 1, 3)
        layout.addWidget(self.actual_y, 1, 4)
        layout.addWidget(QLabel("Z:"), 1, 5)
        layout.addWidget(self.actual_z, 1, 6)
        
        # 理论姿态
        layout.addWidget(QLabel("理论姿态:"), 2, 0)
        self.theoretical_roll = QLineEdit()
        self.theoretical_pitch = QLineEdit()
        self.theoretical_yaw = QLineEdit()
        
        for widget in [self.theoretical_roll, self.theoretical_pitch, self.theoretical_yaw]:
            widget.setReadOnly(True)
            
        layout.addWidget(QLabel("Roll:"), 2, 1)
        layout.addWidget(self.theoretical_roll, 2, 2)
        layout.addWidget(QLabel("Pitch:"), 2, 3)
        layout.addWidget(self.theoretical_pitch, 2, 4)
        layout.addWidget(QLabel("Yaw:"), 2, 5)
        layout.addWidget(self.theoretical_yaw, 2, 6)
        
        # 实际姿态
        layout.addWidget(QLabel("实际姿态:"), 3, 0)
        self.actual_roll = QLineEdit()
        self.actual_pitch = QLineEdit()
        self.actual_yaw = QLineEdit()
        
        for widget in [self.actual_roll, self.actual_pitch, self.actual_yaw]:
            widget.setReadOnly(True)
            
        layout.addWidget(QLabel("Roll:"), 3, 1)
        layout.addWidget(self.actual_roll, 3, 2)
        layout.addWidget(QLabel("Pitch:"), 3, 3)
        layout.addWidget(self.actual_pitch, 3, 4)
        layout.addWidget(QLabel("Yaw:"), 3, 5)
        layout.addWidget(self.actual_yaw, 3, 6)
    
    def connect_signals(self):
        """连接视图模型信号"""
        if self.view_model:
            self.view_model.position_updated.connect(self.update_position_display)
    
    def update_position_display(self, position_data):
        """更新位置显示"""
        theoretical = position_data.get('theoretical', {})
        actual = position_data.get('actual', {})
        
        # 更新理论位置
        if 'position' in theoretical:
            pos = theoretical['position']
            self.theoretical_x.setText(f"{pos[0]:.6f}")
            self.theoretical_y.setText(f"{pos[1]:.6f}")
            self.theoretical_z.setText(f"{pos[2]:.6f}")
        
        if 'orientation' in theoretical:
            ori = theoretical['orientation']
            self.theoretical_roll.setText(f"{ori[0]:.6f}")
            self.theoretical_pitch.setText(f"{ori[1]:.6f}")
            self.theoretical_yaw.setText(f"{ori[2]:.6f}")
        
        # 更新实际位置
        if 'position' in actual:
            pos = actual['position']
            self.actual_x.setText(f"{pos[0]:.6f}")
            self.actual_y.setText(f"{pos[1]:.6f}")
            self.actual_z.setText(f"{pos[2]:.6f}")
        
        if 'orientation' in actual:
            ori = actual['orientation']
            self.actual_roll.setText(f"{ori[0]:.6f}")
            self.actual_pitch.setText(f"{ori[1]:.6f}")
            self.actual_yaw.setText(f"{ori[2]:.6f}")
    
    def update_theoretical_position(self, x, y, z):
        """更新理论位置"""
        self.theoretical_x.setText(f"{x:.6f}")
        self.theoretical_y.setText(f"{y:.6f}")
        self.theoretical_z.setText(f"{z:.6f}")
    
    def update_actual_position(self, x, y, z):
        """更新实际位置"""
        self.actual_x.setText(f"{x:.6f}")
        self.actual_y.setText(f"{y:.6f}")
        self.actual_z.setText(f"{z:.6f}")
    
    def update_theoretical_attitude(self, roll, pitch, yaw):
        """更新理论姿态"""
        self.theoretical_roll.setText(f"{roll:.6f}")
        self.theoretical_pitch.setText(f"{pitch:.6f}")
        self.theoretical_yaw.setText(f"{yaw:.6f}")
    
    def update_actual_attitude(self, roll, pitch, yaw):
        """更新实际姿态"""
        self.actual_roll.setText(f"{roll:.6f}")
        self.actual_pitch.setText(f"{pitch:.6f}")
        self.actual_yaw.setText(f"{yaw:.6f}")
    
    def update_position(self, x, y, z, roll, pitch, yaw):
        """更新所有位置和姿态"""
        self.update_theoretical_position(x, y, z)
        self.update_theoretical_attitude(roll, pitch, yaw) 