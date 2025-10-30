"""
工具组件
"""
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, 
                            QLabel, QDoubleSpinBox, QPushButton, QGridLayout,
                            QMessageBox, QFormLayout, QTextEdit)
from PyQt5.QtCore import Qt
from math import pi
from ..base_component import BaseComponent


class ToolsComponent(BaseComponent):
    """工具组件 - 正运动学计算器"""
    
    def __init__(self, parent=None, view_model=None):
        super().__init__(parent, view_model)
        # 保存当前的旋转矩阵和位置，用于逆解计算
        self.current_rotation_matrix = None
        self.current_position = None
    
    def connect_signals(self):
        """连接视图模型信号"""
        if self.view_model:
            self.view_model.calculation_result.connect(self.display_result)
            self.view_model.current_angles_received.connect(self.fill_current_angles)
            self.view_model.inverse_result.connect(self.display_inverse_result)
    
    def setup_ui(self):
        """设置UI"""
        # 主布局
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)
        
        # 1. 输入区域
        self._create_input_section(layout)
        
        # 2. 正运动学结果显示区域
        self._create_result_section(layout)
        
        # 3. 逆运动学区域
        self._create_inverse_section(layout)
        
        layout.addStretch()
    
    def _create_input_section(self, layout):
        """创建输入区域"""
        # 输入组
        group_box = QGroupBox("关节角度输入 (弧度)")
        group_layout = QVBoxLayout()
        
        # 6个角度输入框（2行×3列）
        grid_layout = QGridLayout()
        grid_layout.setSpacing(10)
        
        self.joint_spins = []
        for i in range(6):
            label = QLabel(f"关节{i+1}:")
            label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            
            spin = QDoubleSpinBox()
            spin.setRange(-2*pi, 2*pi)
            spin.setDecimals(6)
            spin.setSingleStep(0.1)
            spin.setValue(0.0)
            spin.setMinimumWidth(120)
            
            row = i // 3
            col = (i % 3) * 2
            grid_layout.addWidget(label, row, col)
            grid_layout.addWidget(spin, row, col + 1)
            
            self.joint_spins.append(spin)
        
        group_layout.addLayout(grid_layout)
        
        # 按钮行
        button_layout = QHBoxLayout()
        button_layout.addStretch()
        
        self.get_current_btn = QPushButton("获取当前位置")
        self.get_current_btn.setMinimumWidth(120)
        self.get_current_btn.clicked.connect(self.on_get_current_clicked)
        button_layout.addWidget(self.get_current_btn)
        
        self.generate_btn = QPushButton("生成")
        self.generate_btn.setMinimumWidth(100)
        self.generate_btn.clicked.connect(self.on_generate_clicked)
        button_layout.addWidget(self.generate_btn)
        
        self.clear_btn = QPushButton("清空")
        self.clear_btn.setMinimumWidth(100)
        self.clear_btn.clicked.connect(self.on_clear_clicked)
        button_layout.addWidget(self.clear_btn)
        
        group_layout.addLayout(button_layout)
        group_box.setLayout(group_layout)
        layout.addWidget(group_box)
    
    def _create_result_section(self, layout):
        """创建结果显示区域"""
        # 四元数显示
        self._create_quaternion_display(layout)
        
        # 位置显示
        self._create_position_display(layout)
        
        # 旋转矩阵显示
        self._create_rotation_matrix_display(layout)
    
    def _create_quaternion_display(self, layout):
        """创建四元数显示组"""
        group_box = QGroupBox("四元数 (Quaternion)")
        group_layout = QGridLayout()
        group_layout.setSpacing(10)
        
        # X, Y, Z, W 四个值
        self.quat_labels = {}
        labels = ['X', 'Y', 'Z', 'W']
        for i, key in enumerate(labels):
            label = QLabel(f"{key}:")
            label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            
            value_label = QLabel("0.000000")
            value_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
            value_label.setStyleSheet("QLabel { background-color: #f0f0f0; padding: 3px; }")
            value_label.setMinimumWidth(120)
            
            row = i // 2
            col = (i % 2) * 2
            group_layout.addWidget(label, row, col)
            group_layout.addWidget(value_label, row, col + 1)
            
            self.quat_labels[key] = value_label
        
        group_box.setLayout(group_layout)
        layout.addWidget(group_box)
    
    def _create_position_display(self, layout):
        """创建位置显示组"""
        group_box = QGroupBox("位置 (Position, 单位: 米)")
        group_layout = QGridLayout()
        group_layout.setSpacing(10)
        
        # X, Y, Z 三个值
        self.pos_labels = {}
        labels = ['X', 'Y', 'Z']
        for i, key in enumerate(labels):
            label = QLabel(f"{key}:")
            label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            
            value_label = QLabel("0.000000")
            value_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
            value_label.setStyleSheet("QLabel { background-color: #f0f0f0; padding: 3px; }")
            value_label.setMinimumWidth(120)
            
            row = i // 2
            col = (i % 2) * 2
            group_layout.addWidget(label, row, col)
            group_layout.addWidget(value_label, row, col + 1)
            
            self.pos_labels[key] = value_label
        
        group_box.setLayout(group_layout)
        layout.addWidget(group_box)
    
    def _create_rotation_matrix_display(self, layout):
        """创建旋转矩阵显示组"""
        group_box = QGroupBox("旋转矩阵 (Rotation Matrix 3×3)")
        group_layout = QVBoxLayout()
        
        # 使用只读文本框显示矩阵，更易于复制
        self.matrix_text = QTextEdit()
        self.matrix_text.setReadOnly(True)
        self.matrix_text.setMaximumHeight(100)
        self.matrix_text.setStyleSheet("""
            QTextEdit {
                background-color: #f0f0f0;
                font-family: 'Courier New', monospace;
                font-size: 10pt;
            }
        """)
        self.matrix_text.setText(
            "│  1.000000   0.000000   0.000000  │\n"
            "│  0.000000   1.000000   0.000000  │\n"
            "│  0.000000   0.000000   1.000000  │"
        )
        
        group_layout.addWidget(self.matrix_text)
        group_box.setLayout(group_layout)
        layout.addWidget(group_box)
    
    def _create_inverse_section(self, layout):
        """创建逆运动学区域"""
        group_box = QGroupBox("逆运动学求解")
        group_layout = QVBoxLayout()
        
        # 说明文字
        info_label = QLabel("根据上方显示的旋转矩阵和位置计算逆解")
        info_label.setStyleSheet("QLabel { color: #666; font-size: 10pt; }")
        group_layout.addWidget(info_label)
        
        # 按钮
        button_layout = QHBoxLayout()
        button_layout.addStretch()
        
        self.inverse_btn = QPushButton("生成逆解")
        self.inverse_btn.setMinimumWidth(120)
        self.inverse_btn.clicked.connect(self.on_inverse_clicked)
        button_layout.addWidget(self.inverse_btn)
        
        group_layout.addLayout(button_layout)
        
        # 结果显示 - 使用网格布局显示6个关节角度
        result_grid = QGridLayout()
        result_grid.setSpacing(10)
        
        self.inverse_labels = []
        for i in range(6):
            label = QLabel(f"关节{i+1}:")
            label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            
            value_label = QLabel("--")
            value_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
            value_label.setStyleSheet("QLabel { background-color: #f0f0f0; padding: 3px; }")
            value_label.setMinimumWidth(150)
            
            row = i // 3
            col = (i % 3) * 2
            result_grid.addWidget(label, row, col)
            result_grid.addWidget(value_label, row, col + 1)
            
            self.inverse_labels.append(value_label)
        
        group_layout.addLayout(result_grid)
        group_box.setLayout(group_layout)
        layout.addWidget(group_box)
    
    def on_get_current_clicked(self):
        """获取当前位置按钮点击事件"""
        if self.view_model:
            self.view_model.get_current_position()
    
    def on_generate_clicked(self):
        """生成按钮点击事件"""
        # 获取6个角度
        joint_angles = [spin.value() for spin in self.joint_spins]
        
        # 请求计算
        if self.view_model:
            self.view_model.calculate_kinematics(joint_angles)
    
    def on_clear_clicked(self):
        """清空按钮点击事件"""
        # 重置所有输入为0
        for spin in self.joint_spins:
            spin.setValue(0.0)
        
        # 重置显示
        for label in self.quat_labels.values():
            label.setText("0.000000")
        
        for label in self.pos_labels.values():
            label.setText("0.000000")
        
        self.matrix_text.setText(
            "│  1.000000   0.000000   0.000000  │\n"
            "│  0.000000   1.000000   0.000000  │\n"
            "│  0.000000   0.000000   1.000000  │"
        )
    
    def display_result(self, result: dict):
        """显示计算结果"""
        if "error" in result:
            QMessageBox.warning(self, "计算错误", result["error"])
            return
        
        # 更新四元数显示
        quat = result["quaternion"]
        self.quat_labels['X'].setText(f"{quat[0]:.6f}")
        self.quat_labels['Y'].setText(f"{quat[1]:.6f}")
        self.quat_labels['Z'].setText(f"{quat[2]:.6f}")
        self.quat_labels['W'].setText(f"{quat[3]:.6f}")
        
        # 更新位置显示
        pos = result["position"]
        self.pos_labels['X'].setText(f"{pos[0]:.6f}")
        self.pos_labels['Y'].setText(f"{pos[1]:.6f}")
        self.pos_labels['Z'].setText(f"{pos[2]:.6f}")
        
        # 更新旋转矩阵显示（3x3）
        rm = result["rotation_matrix"]
        matrix_str = ""
        for i in range(3):
            row_str = "│ " + "  ".join([f"{rm[i][j]:9.6f}" for j in range(3)]) + "  │"
            matrix_str += row_str
            if i < 2:
                matrix_str += "\n"
        
        self.matrix_text.setText(matrix_str)
        
        # 保存当前的旋转矩阵和位置，供逆解使用
        self.current_rotation_matrix = rm
        self.current_position = pos
    
    def fill_current_angles(self, angles: list):
        """填充当前关节角度到输入框"""
        if len(angles) >= 6:
            for i, spin in enumerate(self.joint_spins):
                spin.setValue(angles[i])
    
    def on_inverse_clicked(self):
        """逆解按钮点击事件"""
        if self.current_rotation_matrix is None or self.current_position is None:
            QMessageBox.warning(self, "错误", "请先生成正运动学结果后再计算逆解")
            return
        
        # 获取当前输入框的角度作为初始值
        initial_theta = [spin.value() for spin in self.joint_spins]
        
        # 请求计算逆解
        if self.view_model:
            self.view_model.calculate_inverse_kinematics(
                self.current_rotation_matrix,
                self.current_position,
                initial_theta
            )
    
    def display_inverse_result(self, result: dict):
        """显示逆解结果"""
        if "error" in result:
            QMessageBox.warning(self, "逆解计算错误", result["error"])
            # 重置显示
            for label in self.inverse_labels:
                label.setText("--")
            return
        
        # 更新逆解显示（显示弧度和角度）
        joint_angles = result["joint_angles"]
        joint_angles_deg = result["joint_angles_deg"]
        
        for i, label in enumerate(self.inverse_labels):
            if i < len(joint_angles):
                # 同时显示弧度和角度
                label.setText(f"{joint_angles[i]:.6f} rad ({joint_angles_deg[i]:.2f}°)")
