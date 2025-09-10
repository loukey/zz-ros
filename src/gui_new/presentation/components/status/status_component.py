"""
Status display components for left bottom area
"""
from PyQt5.QtWidgets import (QWidget, QLabel, QVBoxLayout, QHBoxLayout, 
                           QGroupBox, QGridLayout, QFrame)
from PyQt5.QtCore import Qt
from ..base_component import BaseComponent, default_font, text_font


class StatusDisplayComponent(BaseComponent):
    """状态显示组件 - 显示解码后的消息各字段"""
    
    def __init__(self, parent=None, view_model=None):
        super().__init__(parent, view_model)
        self.setMaximumHeight(180)  # 限制高度，适应4行布局
    
    def setup_ui(self):
        """设置UI"""
        # 创建主分组框
        group_box = QGroupBox("机器人状态")
        group_box.setFont(default_font)
        main_layout = QVBoxLayout(self)
        main_layout.addWidget(group_box)
        
        # 使用网格布局显示各个状态字段
        grid_layout = QGridLayout(group_box)
        grid_layout.setHorizontalSpacing(4)  # 水平间距
        grid_layout.setVerticalSpacing(3)    # 垂直间距
        grid_layout.setContentsMargins(5, 5, 5, 5)
        
        # 设置所有列统一宽度
        total_cols = 16  # 总列数
        for i in range(total_cols):
            grid_layout.setColumnStretch(i, 1)  # 所有列均匀拉伸，统一宽度
        
        # 创建状态显示标签
        self.status_labels = {}
        
        # 第一行：状态、命令、模式、夹爪数据（单个显示框） + 位置（六个显示框）
        col = 0
        self._create_single_field("状态字", "init_status", 0, col, grid_layout)
        col += 2
        self._create_single_field("命令", "control", 0, col, grid_layout)
        col += 2
        self._create_single_field("模式", "mode", 0, col, grid_layout)
        col += 2
        self._create_single_field("夹爪", "effector_data", 0, col, grid_layout)
        col += 2
        
        # 位置（六个显示框）
        self._create_array_fields("位置", "positions", 0, col, grid_layout, 6)
        
        # 第二行：状态（六个框）+ 速度（六个框）
        self._create_array_fields("状态码", "status", 1, 0, grid_layout, 6)
        self._create_array_fields("速度", "speeds", 1, 8, grid_layout, 6)
        
        # 第三行：力矩（六个框） + 空白占位
        self._create_array_fields("力矩", "torques", 2, 0, grid_layout, 6)
        
        # 第四行：双编码器插值（六个框）+ 错误码（六个框）
        self._create_array_fields("双编码器", "double_encoder_interpolations", 3, 0, grid_layout, 6)
        self._create_array_fields("错误码", "errors", 3, 8, grid_layout, 6)
    
    def _create_single_field(self, label_text, field_name, row, col, layout):
        """创建单个状态字段显示"""
        # 标签
        label = QLabel(f"{label_text}:")
        label.setFont(text_font)
        label.setAlignment(Qt.AlignCenter | Qt.AlignVCenter)
        layout.addWidget(label, row, col)
        
        # 数值显示
        value_label = QLabel("--")
        value_label.setFont(default_font)
        value_label.setStyleSheet("""
            QLabel {
                background-color: #f0f0f0;
                border: 1px solid #ccc;
                padding: 2px 4px;
                border-radius: 3px;
            }
        """)
        value_label.setAlignment(Qt.AlignCenter | Qt.AlignVCenter)
        layout.addWidget(value_label, row, col + 1)
        
        # 保存引用
        self.status_labels[field_name] = value_label
    
    def _create_array_fields(self, label_text, field_name, row, col, layout, count):
        """创建数组字段显示（多个显示框）"""
        # 标签
        label = QLabel(f"{label_text}:")
        label.setFont(text_font)
        label.setAlignment(Qt.AlignCenter | Qt.AlignVCenter)
        layout.addWidget(label, row, col)
        
        # 创建多个数值显示框
        value_labels = []
        for i in range(count):
            value_label = QLabel("--")
            value_label.setFont(default_font)
            value_label.setStyleSheet("""
                QLabel {
                    background-color: #f0f0f0;
                    border: 1px solid #ccc;
                    padding: 2px 4px;
                    border-radius: 3px;
                }
            """)
            value_label.setAlignment(Qt.AlignCenter | Qt.AlignVCenter)
            layout.addWidget(value_label, row, col + 1 + i)
            value_labels.append(value_label)
        
        # 保存引用（数组形式）
        self.status_labels[field_name] = value_labels
    
    def connect_signals(self):
        """连接视图模型信号"""
        if self.view_model:
            self.view_model.status_updated.connect(self.update_status)
    
    def update_status(self, status_data):
        """更新状态显示"""
        for field_name, value in status_data.items():
            if field_name in self.status_labels:
                labels = self.status_labels[field_name]
                
                # 判断是单个标签还是标签列表
                if isinstance(labels, list):
                    # 数组字段，更新多个显示框
                    self._update_array_field(labels, value)
                else:
                    # 单个字段，更新单个显示框
                    formatted_value = self._format_single_value(field_name, value)
                    labels.setText(formatted_value)
    
    def _update_array_field(self, labels, value):
        """更新数组字段显示"""
        if value is None or not isinstance(value, (list, tuple)):
            # 没有数据时显示 "--"
            for label in labels:
                label.setText("--")
        else:
            # 有数据时逐个显示
            for i, label in enumerate(labels):
                if i < len(value):
                    # 格式化数值
                    if isinstance(value[i], float):
                        label.setText(f"{value[i]:.2f}")
                    elif isinstance(value[i], int):
                        label.setText(str(value[i]))
                    else:
                        label.setText(str(value[i]))
                else:
                    # 超出数据范围显示 "--"
                    label.setText("--")
    
    def _format_single_value(self, field_name, value):
        """格式化单个字段显示值"""
        if value is None:
            return "--"
        
        if field_name in ["control", "mode", "init_status"]:
            # 十六进制显示
            if isinstance(value, int):
                return f"0x{value:02X}"
            return str(value)
        elif field_name == "effector_data":
            # 夹爪数据
            if isinstance(value, (list, tuple)) and len(value) > 0:
                return f"{value[0]}"  # 显示第一个值
            elif isinstance(value, (int, float)):
                return str(value)
            return str(value)
        else:
            # 其他类型直接显示
            return str(value)


class StatusSeparator(QFrame):
    """状态区域分隔线"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFrameShape(QFrame.HLine)
        self.setFrameShadow(QFrame.Sunken)
        self.setMaximumHeight(2)


