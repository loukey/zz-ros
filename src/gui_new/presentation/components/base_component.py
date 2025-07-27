"""
基础UI组件
"""
from PyQt5.QtWidgets import (QFrame, QWidget, QLabel, QComboBox, QPushButton, 
                           QVBoxLayout, QHBoxLayout, QGroupBox, QSizePolicy,
                           QLineEdit, QDoubleSpinBox, QSpinBox, QGridLayout,
                           QRadioButton, QButtonGroup, QSlider, QCheckBox)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QFont, QDoubleValidator, QIntValidator

# 设置全局字体
default_font = QFont('SimHei', 10)
text_font = QFont('SimHei', 9)


class BaseComponent(QFrame):
    """基础组件类"""
    
    def __init__(self, parent=None, view_model=None):
        super().__init__(parent)
        self.view_model = view_model
        self.setup_ui()
        if view_model:
            self.connect_signals()
    
    def setup_ui(self):
        """设置UI - 子类实现"""
        pass
    
    def connect_signals(self):
        """连接信号 - 子类实现"""
        pass


class LabeledLineEdit(QWidget):
    """带标签的文本输入框"""
    
    textChanged = pyqtSignal(str)
    
    def __init__(self, label_text, default_text="", validator=None, parent=None):
        super().__init__(parent)
        self.label = QLabel(label_text)
        self.label.setFont(default_font)
        
        self.line_edit = QLineEdit(default_text)
        self.line_edit.setFont(default_font)
        self.line_edit.textChanged.connect(self.textChanged.emit)
        
        if validator:
            self.line_edit.setValidator(validator)
        
        layout = QHBoxLayout()
        layout.addWidget(self.label)
        layout.addWidget(self.line_edit)
        layout.setStretchFactor(self.line_edit, 1)
        
        self.setLayout(layout)
    
    def text(self):
        """获取文本"""
        return self.line_edit.text()
    
    def set_text(self, text):
        """设置文本"""
        self.line_edit.setText(text)
    
    def set_enabled(self, enabled):
        """设置是否启用"""
        self.line_edit.setEnabled(enabled)
    
    def set_validator(self, validator):
        """设置验证器"""
        self.line_edit.setValidator(validator)


class LabeledDoubleSpinBox(QWidget):
    """带标签的双精度数字输入框"""
    
    valueChanged = pyqtSignal(float)
    
    def __init__(self, label_text, minimum=0.0, maximum=100.0, value=0.0, 
                 decimals=2, single_step=1.0, parent=None):
        super().__init__(parent)
        self.label = QLabel(label_text)
        self.label.setFont(default_font)
        
        self.spin_box = QDoubleSpinBox()
        self.spin_box.setFont(default_font)
        self.spin_box.setMinimum(minimum)
        self.spin_box.setMaximum(maximum)
        self.spin_box.setValue(value)
        self.spin_box.setDecimals(decimals)
        self.spin_box.setSingleStep(single_step)
        self.spin_box.valueChanged.connect(self.valueChanged.emit)
        
        layout = QHBoxLayout()
        layout.addWidget(self.label)
        layout.addWidget(self.spin_box)
        layout.setStretchFactor(self.spin_box, 1)
        
        self.setLayout(layout)
    
    def value(self):
        """获取值"""
        return self.spin_box.value()
    
    def set_value(self, value):
        """设置值"""
        self.spin_box.setValue(value)
    
    def set_enabled(self, enabled):
        """设置是否启用"""
        self.spin_box.setEnabled(enabled)


class LabeledSpinBox(QWidget):
    """带标签的整数输入框"""
    
    valueChanged = pyqtSignal(int)
    
    def __init__(self, label_text, minimum=0, maximum=100, value=0, parent=None):
        super().__init__(parent)
        self.label = QLabel(label_text)
        self.label.setFont(default_font)
        
        self.spin_box = QSpinBox()
        self.spin_box.setFont(default_font)
        self.spin_box.setMinimum(minimum)
        self.spin_box.setMaximum(maximum)
        self.spin_box.setValue(value)
        self.spin_box.valueChanged.connect(self.valueChanged.emit)
        
        layout = QHBoxLayout()
        layout.addWidget(self.label)
        layout.addWidget(self.spin_box)
        layout.setStretchFactor(self.spin_box, 1)
        
        self.setLayout(layout)
    
    def value(self):
        """获取值"""
        return self.spin_box.value()
    
    def set_value(self, value):
        """设置值"""
        self.spin_box.setValue(value)
    
    def set_enabled(self, enabled):
        """设置是否启用"""
        self.spin_box.setEnabled(enabled)


class LabeledComboBox(QWidget):
    """带标签的下拉列表框"""
    
    currentTextChanged = pyqtSignal(str)
    currentIndexChanged = pyqtSignal(int)
    
    def __init__(self, label_text, items=None, parent=None):
        super().__init__(parent)
        self.label = QLabel(label_text)
        self.label.setFont(default_font)
        
        self.combobox = QComboBox()
        self.combobox.setFont(default_font)
        
        if items:
            self.combobox.addItems(items)
        
        self.combobox.currentTextChanged.connect(self.currentTextChanged.emit)
        self.combobox.currentIndexChanged.connect(self.currentIndexChanged.emit)
        
        layout = QHBoxLayout()
        layout.addWidget(self.label)
        layout.addWidget(self.combobox)
        layout.setStretchFactor(self.combobox, 1)
        
        self.setLayout(layout)
    
    def current_text(self):
        """获取当前选中的项"""
        return self.combobox.currentText()
    
    def current_index(self):
        """获取当前选中项的索引"""
        return self.combobox.currentIndex()
    
    def set_items(self, items):
        """设置项列表"""
        self.combobox.clear()
        self.combobox.addItems(items)
    
    def set_current_index(self, index):
        """设置当前选中项的索引"""
        if 0 <= index < self.combobox.count():
            self.combobox.setCurrentIndex(index)
    
    def set_current_text(self, text):
        """设置当前选中的项"""
        index = self.combobox.findText(text)
        if index >= 0:
            self.combobox.setCurrentIndex(index)
    
    def set_enabled(self, enabled):
        """设置是否启用"""
        self.combobox.setEnabled(enabled)


class LabeledButton(QWidget):
    """带标签的按钮"""
    
    clicked = pyqtSignal()
    
    def __init__(self, label_text, button_text, on_click=None, parent=None):
        super().__init__(parent)
        self.label = QLabel(label_text)
        self.label.setFont(default_font)
        
        self.button = QPushButton(button_text)
        self.button.setFont(default_font)
        self.button.clicked.connect(self.clicked.emit)
        
        if on_click:
            self.button.clicked.connect(on_click)
        
        layout = QHBoxLayout()
        layout.addWidget(self.label)
        layout.addWidget(self.button)
        layout.setStretchFactor(self.button, 1)
        
        self.setLayout(layout)
    
    def set_enabled(self, enabled):
        """设置按钮是否启用"""
        self.button.setEnabled(enabled)
    
    def set_text(self, text):
        """设置按钮文本"""
        self.button.setText(text)


class InputGrid(QWidget):
    """输入网格组件"""
    
    def __init__(self, labels, rows=2, cols=3, default_value="0.0", validator=None, parent=None):
        super().__init__(parent)
        self.inputs = []
        
        layout = QGridLayout()
        
        # 添加标签
        for i, label_text in enumerate(labels):
            row = i // cols
            col = i % cols
            layout.addWidget(QLabel(label_text), row * 2, col)
            
            # 添加输入框
            line_edit = QLineEdit(default_value)
            line_edit.setFont(default_font)
            if validator:
                line_edit.setValidator(validator)
            
            layout.addWidget(line_edit, row * 2 + 1, col)
            self.inputs.append(line_edit)
        
        self.setLayout(layout)
    
    def get_values(self):
        """获取所有输入值"""
        return [input_widget.text() for input_widget in self.inputs]
    
    def set_values(self, values):
        """设置所有输入值"""
        for i, value in enumerate(values):
            if i < len(self.inputs):
                self.inputs[i].setText(str(value))
    
    def get_float_values(self):
        """获取所有输入值（浮点数）"""
        values = []
        for input_widget in self.inputs:
            try:
                values.append(float(input_widget.text()))
            except ValueError:
                values.append(0.0)
        return values
    
    def set_enabled(self, enabled):
        """设置是否启用"""
        for input_widget in self.inputs:
            input_widget.setEnabled(enabled)


class RadioButtonGroup(QWidget):
    """单选按钮组"""
    
    selectionChanged = pyqtSignal(str)
    
    def __init__(self, label_text, options, default_option=None, parent=None):
        super().__init__(parent)
        
        layout = QHBoxLayout()
        
        if label_text:
            label = QLabel(label_text)
            label.setFont(default_font)
            layout.addWidget(label)
        
        self.button_group = QButtonGroup()
        self.buttons = {}
        
        for i, option in enumerate(options):
            radio_button = QRadioButton(option)
            radio_button.setFont(default_font)
            self.button_group.addButton(radio_button, i)
            self.buttons[option] = radio_button
            layout.addWidget(radio_button)
            
            if default_option == option or (default_option is None and i == 0):
                radio_button.setChecked(True)
        
        self.button_group.buttonClicked.connect(self._on_selection_changed)
        
        layout.addStretch()
        self.setLayout(layout)
    
    def _on_selection_changed(self, button):
        """选择改变时的回调"""
        self.selectionChanged.emit(button.text())
    
    def get_selected(self):
        """获取当前选中的选项"""
        for option, button in self.buttons.items():
            if button.isChecked():
                return option
        return None
    
    def set_selected(self, option):
        """设置选中的选项"""
        if option in self.buttons:
            self.buttons[option].setChecked(True)


class ButtonRow(QWidget):
    """按钮行组件"""
    
    def __init__(self, button_configs, parent=None):
        """
        button_configs: [(text, callback, enabled), ...]
        """
        super().__init__(parent)
        self.buttons = []
        
        layout = QHBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)  # 移除边距
        layout.setSpacing(4)  # 减少按钮间距
        
        for config in button_configs:
            text = config[0]
            callback = config[1] if len(config) > 1 else None
            enabled = config[2] if len(config) > 2 else True
            
            button = QPushButton(text)
            button.setFont(default_font)
            button.setEnabled(enabled)
            
            if callback:
                button.clicked.connect(callback)
            
            layout.addWidget(button)
            self.buttons.append(button)
        
        self.setLayout(layout)
    
    def set_button_enabled(self, index, enabled):
        """设置指定按钮的启用状态"""
        if 0 <= index < len(self.buttons):
            self.buttons[index].setEnabled(enabled)
    
    def set_all_buttons_enabled(self, enabled):
        """设置所有按钮的启用状态"""
        for button in self.buttons:
            button.setEnabled(enabled)
    
    def get_button(self, index):
        """获取指定索引的按钮"""
        if 0 <= index < len(self.buttons):
            return self.buttons[index]
        return None


class GroupFrame(QGroupBox):
    """自定义分组框架"""
    
    def __init__(self, title, parent=None):
        super().__init__(title, parent)
        self.setFont(default_font)
        self.layout = QVBoxLayout()
        self.layout.setContentsMargins(6, 6, 6, 6)  # 减少内部边距
        self.layout.setSpacing(2)  # 减少组件间距
        self.setLayout(self.layout)
    
    def add_widget(self, widget):
        """添加部件"""
        self.layout.addWidget(widget)
    
    def add_layout(self, layout):
        """添加布局"""
        self.layout.addLayout(layout)
    
    def add_stretch(self):
        """添加伸缩空间"""
        self.layout.addStretch()


class HorizontalLine(QFrame):
    """水平分割线"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFrameShape(QFrame.HLine)
        self.setFrameShadow(QFrame.Sunken)


class ConfigRow(QWidget):
    """配置行组件 - 用于创建标签+多个控件的配置行"""
    
    def __init__(self, label_text, parent=None):
        super().__init__(parent)
        self.layout = QHBoxLayout()
        self.layout.setContentsMargins(0, 0, 0, 0)  # 移除边距
        self.layout.setSpacing(4)  # 减少组件间距
        
        if label_text:
            label = QLabel(label_text)
            label.setFont(default_font)
            self.layout.addWidget(label)
        
        self.setLayout(self.layout)
    
    def add_widget(self, widget):
        """添加控件"""
        self.layout.addWidget(widget)
    
    def add_stretch(self):
        """添加伸缩空间"""
        self.layout.addStretch()
    
    def add_spacing(self, spacing):
        """添加间距"""
        self.layout.addSpacing(spacing)


class LabeledSlider(QWidget):
    """带标签的滑动条"""
    
    valueChanged = pyqtSignal(int)
    
    def __init__(self, label_text, minimum=0, maximum=100, value=0, 
                 orientation=Qt.Horizontal, parent=None):
        super().__init__(parent)
        
        self.label = QLabel(label_text)
        self.label.setFont(default_font)
        
        self.slider = QSlider(orientation)
        self.slider.setMinimum(minimum)
        self.slider.setMaximum(maximum)
        self.slider.setValue(value)
        self.slider.valueChanged.connect(self.valueChanged.emit)
        
        self.value_label = QLabel(str(value))
        self.value_label.setFont(default_font)
        self.slider.valueChanged.connect(lambda v: self.value_label.setText(str(v)))
        
        layout = QHBoxLayout()
        layout.addWidget(self.label)
        layout.addWidget(self.slider)
        layout.addWidget(self.value_label)
        layout.setStretchFactor(self.slider, 1)
        
        self.setLayout(layout)
    
    def value(self):
        """获取当前值"""
        return self.slider.value()
    
    def set_value(self, value):
        """设置值"""
        self.slider.setValue(value)
    
    def set_range(self, minimum, maximum):
        """设置范围"""
        self.slider.setRange(minimum, maximum)
    
    def set_enabled(self, enabled):
        """设置是否启用"""
        self.slider.setEnabled(enabled) 
        