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
    
    def connect_signals(self):
        """连接视图模型信号"""
        if self.view_model:
            self.view_model.calculation_result.connect(self.display_result)
    
    def setup_ui(self):
        """设置UI"""
        # 主布局
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)
        
        # 1. 输入区域
        self._create_input_section(layout)
        
        # 2. 结果显示区域
        self._create_result_section(layout)
        
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
