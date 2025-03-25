"""
基础UI组件模块
"""
from PyQt5.QtWidgets import (QWidget, QLabel, QComboBox, QPushButton, QFrame, 
                           QVBoxLayout, QHBoxLayout, QGroupBox)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont

# 设置全局字体
default_font = QFont('WenQuanYi Micro Hei', 12)
text_font = QFont('WenQuanYi Micro Hei', 11)


class LabeledComboBox(QWidget):
    """带标签的下拉列表框"""
    
    def __init__(self, label_text, items=None, parent=None):
        super().__init__(parent)
        self.label = QLabel(label_text)
        self.label.setFont(default_font)
        
        self.combobox = QComboBox()
        self.combobox.setFont(default_font)
        
        if items:
            self.combobox.addItems(items)
        
        layout = QHBoxLayout()
        layout.addWidget(self.label)
        layout.addWidget(self.combobox)
        layout.setStretchFactor(self.combobox, 1)
        
        self.setLayout(layout)
    
    def get_selected_item(self):
        """获取当前选中的项"""
        return self.combobox.currentText()
    
    def get_selected_index(self):
        """获取当前选中项的索引"""
        return self.combobox.currentIndex()
    
    def set_items(self, items):
        """设置项列表"""
        self.combobox.clear()
        self.combobox.addItems(items)
    
    def set_selected_index(self, index):
        """设置当前选中项的索引"""
        if 0 <= index < self.combobox.count():
            self.combobox.setCurrentIndex(index)
    
    def set_selected_item(self, item):
        """设置当前选中的项"""
        index = self.combobox.findText(item)
        if index >= 0:
            self.combobox.setCurrentIndex(index)


class LabeledButton(QWidget):
    """带标签的按钮"""
    
    def __init__(self, label_text, button_text, on_click=None, parent=None):
        super().__init__(parent)
        self.label = QLabel(label_text)
        self.label.setFont(default_font)
        
        self.button = QPushButton(button_text)
        self.button.setFont(default_font)
        
        if on_click:
            self.button.clicked.connect(on_click)
        
        layout = QHBoxLayout()
        layout.addWidget(self.label)
        layout.addWidget(self.button)
        layout.setStretchFactor(self.button, 1)
        
        self.setLayout(layout)
    
    def set_button_enabled(self, enabled):
        """设置按钮是否启用"""
        self.button.setEnabled(enabled)
    
    def set_button_text(self, text):
        """设置按钮文本"""
        self.button.setText(text)


class HorizontalLine(QFrame):
    """水平分割线"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFrameShape(QFrame.HLine)
        self.setFrameShadow(QFrame.Sunken)


class GroupFrame(QGroupBox):
    """
    自定义分组框架，用于包含相关UI元素
    """
    
    def __init__(self, title, parent=None):
        """
        初始化分组框架
        
        参数:
            title: 分组标题
            parent: 父控件
        """
        super().__init__(title, parent)
        self.setFont(default_font)
        self.layout = QVBoxLayout()
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