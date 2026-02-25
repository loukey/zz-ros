"""
Status display components for left bottom area
"""
from PyQt5.QtWidgets import (QWidget, QLabel, QVBoxLayout, QHBoxLayout,
                           QGroupBox, QGridLayout, QFrame)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont
from ..base_component import BaseComponent, default_font, text_font
from controller.presentation.theme import MONO_FAMILY

# 等宽数值字体
_mono_font = QFont(MONO_FAMILY, 9)


class StatusDisplayComponent(BaseComponent):
    """状态显示组件 - 显示解码后的消息各字段"""

    def __init__(self, parent=None, view_model=None):
        super().__init__(parent, view_model)
        self.setMaximumHeight(260)

    def setup_ui(self):
        """设置UI"""
        group_box = QGroupBox("机器人状态")
        group_box.setFont(default_font)
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.addWidget(group_box)

        inner_layout = QVBoxLayout(group_box)
        inner_layout.setContentsMargins(8, 14, 8, 8)
        inner_layout.setSpacing(4)

        self.status_labels = {}

        # --- 顶部单行：状态字、命令、模式、夹爪 ---
        top_row = QHBoxLayout()
        top_row.setSpacing(12)
        for label_text, field_name in [("状态字", "init_status"),
                                        ("命令", "control"),
                                        ("模式", "mode"),
                                        ("夹爪", "effector_data")]:
            lbl = QLabel(f"{label_text}:")
            lbl.setFont(text_font)
            top_row.addWidget(lbl)

            val = QLabel("--")
            val.setFont(_mono_font)
            val.setMinimumWidth(55)
            val.setStyleSheet(
                "QLabel { background-color: #f0f0f0; border: 1px solid #ccc; "
                "padding: 2px 4px; border-radius: 3px; }")
            val.setAlignment(Qt.AlignCenter)
            top_row.addWidget(val)
            self.status_labels[field_name] = val
        top_row.addStretch()
        inner_layout.addLayout(top_row)

        # --- 下方 grid：7 列 (1 标签 + 6 数值)，6 行 ---
        grid = QGridLayout()
        grid.setHorizontalSpacing(4)
        grid.setVerticalSpacing(3)
        # 标签列固定，数值列均匀拉伸
        grid.setColumnStretch(0, 0)
        for c in range(1, 7):
            grid.setColumnStretch(c, 1)

        row_defs = [
            ("位置", "positions"),
            ("速度", "speeds"),
            ("状态码", "status"),
            ("力矩", "torques"),
            ("双编码器", "double_encoder_interpolations"),
            ("错误码", "errors"),
        ]
        for r, (label_text, field_name) in enumerate(row_defs):
            lbl = QLabel(f"{label_text}:")
            lbl.setFont(text_font)
            lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            grid.addWidget(lbl, r, 0)

            value_labels = []
            for c in range(6):
                val = QLabel("--")
                val.setFont(_mono_font)
                val.setMinimumWidth(55)
                val.setStyleSheet(
                    "QLabel { background-color: #f0f0f0; border: 1px solid #ccc; "
                    "padding: 2px 4px; border-radius: 3px; }")
                val.setAlignment(Qt.AlignCenter)
                grid.addWidget(val, r, c + 1)
                value_labels.append(val)
            self.status_labels[field_name] = value_labels

        inner_layout.addLayout(grid)

    def connect_signals(self):
        """连接视图模型信号"""
        if self.view_model:
            self.view_model.status_updated.connect(self.update_status)

    def update_status(self, status_data):
        """更新状态显示"""
        for field_name, value in status_data.items():
            if field_name in self.status_labels:
                labels = self.status_labels[field_name]

                if isinstance(labels, list):
                    self._update_array_field(labels, value)
                else:
                    formatted_value = self._format_single_value(field_name, value)
                    labels.setText(formatted_value)

    def _update_array_field(self, labels, value):
        """更新数组字段显示"""
        if value is None or not isinstance(value, (list, tuple)):
            for label in labels:
                label.setText("--")
        else:
            for i, label in enumerate(labels):
                if i < len(value):
                    if isinstance(value[i], float):
                        label.setText(f"{value[i]:.2f}")
                    elif isinstance(value[i], int):
                        label.setText(str(value[i]))
                    else:
                        label.setText(str(value[i]))
                else:
                    label.setText("--")

    def _format_single_value(self, field_name, value):
        """格式化单个字段显示值"""
        if value is None:
            return "--"

        if field_name in ["control", "mode", "init_status"]:
            if isinstance(value, int):
                return f"0x{value:02X}"
            return str(value)
        elif field_name == "effector_data":
            if isinstance(value, (list, tuple)) and len(value) > 0:
                return f"{value[0]}"
            elif isinstance(value, (int, float)):
                return str(value)
            return str(value)
        else:
            return str(value)


class StatusSeparator(QFrame):
    """状态区域分隔线"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFrameShape(QFrame.HLine)
        self.setFrameShadow(QFrame.Sunken)
        self.setMaximumHeight(2)
