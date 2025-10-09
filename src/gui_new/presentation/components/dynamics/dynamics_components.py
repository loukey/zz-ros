"""
Dynamics tab components
"""
from PyQt5.QtWidgets import (QWidget, QPushButton, QVBoxLayout, QHBoxLayout, 
                           QGroupBox, QFrame, QLabel, QLineEdit, QGridLayout, 
                           QListWidget, QListWidgetItem, QMessageBox)
from PyQt5.QtCore import pyqtSignal, QTimer
from PyQt5.QtGui import QDoubleValidator
from ..base_component import BaseComponent, default_font, InputGrid, ButtonRow, HorizontalLine
import json
import os


class DynamicsFrame(BaseComponent):
    """动力学控制框架"""
    
    # 定义信号
    teaching_mode_toggle_requested = pyqtSignal(dict)  # 发送示教模式命令
    send_torque_requested = pyqtSignal(list)  # 发送力矩信号
    
    def __init__(self, parent=None, view_model=None):
        self.teaching_mode_enabled = False
        self.buttons = []
        self.torque_inputs = []
        super().__init__(parent, view_model)
    
    def setup_ui(self):
        """设置UI"""
        # 创建分组框
        group_box = QGroupBox("动力学控制")
        main_layout = QVBoxLayout(self)
        main_layout.addWidget(group_box)
        
        layout = QVBoxLayout(group_box)
        
        # 创建基本控制按钮区域
        basic_control_layout = QHBoxLayout()
        
        # 示教模式按钮
        self.teaching_mode_button = QPushButton("开启示教模式")
        self.teaching_mode_button.setFont(default_font)
        self.teaching_mode_button.clicked.connect(self._toggle_teaching_mode)
        self.teaching_mode_button.setEnabled(False)
        basic_control_layout.addWidget(self.teaching_mode_button)
        self.buttons.append(self.teaching_mode_button)
        
        # 添加伸缩项让按钮居左
        basic_control_layout.addStretch()
        
        layout.addLayout(basic_control_layout)
        
        # 添加分隔线
        layout.addWidget(HorizontalLine())
        
        # 创建力矩输入区域 - 使用InputGrid基础组件
        torque_group = QGroupBox("力矩输入 (Nm)")
        torque_layout = QVBoxLayout(torque_group)
        
        # 使用InputGrid创建力矩输入
        torque_labels = [f"力矩{i+1}" for i in range(6)]
        self.torque_grid = InputGrid(
            labels=torque_labels,
            rows=2, cols=3,
            default_value="0.0",
            validator=QDoubleValidator(-1000.0, 1000.0, 3)
        )
        # 设置输入框宽度
        for input_widget in self.torque_grid.inputs:
            input_widget.setMaximumWidth(80)
            input_widget.setMinimumWidth(80)
        
        torque_layout.addWidget(self.torque_grid)
        self.torque_inputs = self.torque_grid.inputs  # 保存引用以保持兼容性
        
        # 力矩控制按钮区域 - 使用ButtonRow基础组件
        torque_button_configs = [
            ("发送力矩", self._send_torque, False),
            ("清零", self._clear_torque_inputs, True)
        ]
        
        self.torque_button_row = ButtonRow(torque_button_configs)
        self.send_torque_button = self.torque_button_row.get_button(0)  # 保存引用
        self.buttons.append(self.send_torque_button)
        
        torque_layout.addWidget(self.torque_button_row)
        layout.addWidget(torque_group)
        
        # 添加分隔线
        layout.addWidget(HorizontalLine())
        
        # 创建记录功能区域
        record_group = QGroupBox("示教记录")
        record_layout = QVBoxLayout(record_group)
        
        # 记录控制按钮区域
        record_button_layout = QHBoxLayout()
        
        # 开启记录按钮
        self.record_button = QPushButton("开启记录")
        self.record_button.setFont(default_font)
        self.record_button.clicked.connect(self._toggle_recording)
        self.record_button.setEnabled(False)
        record_button_layout.addWidget(self.record_button)
        self.buttons.append(self.record_button)
        
        # 删除记录按钮
        self.delete_record_button = QPushButton("删除记录")
        self.delete_record_button.setFont(default_font)
        self.delete_record_button.clicked.connect(self._delete_record)
        record_button_layout.addWidget(self.delete_record_button)
        
        # 运行记录按钮
        self.run_record_button = QPushButton("运行记录")
        self.run_record_button.setFont(default_font)
        self.run_record_button.clicked.connect(self._run_record)
        record_button_layout.addWidget(self.run_record_button)
        
        # 反转记录按钮
        self.reverse_record_button = QPushButton("反转记录")
        self.reverse_record_button.setFont(default_font)
        self.reverse_record_button.clicked.connect(self._reverse_record)
        record_button_layout.addWidget(self.reverse_record_button)
        
        # 添加伸缩项
        record_button_layout.addStretch()
        
        record_layout.addLayout(record_button_layout)
        
        # 记录列表
        self.record_list = QListWidget()
        self.record_list.setFont(default_font)
        self.record_list.setMaximumHeight(150)
        record_layout.addWidget(self.record_list)
        
        layout.addWidget(record_group)
    
    def connect_signals(self):
        """连接视图模型信号"""
        if self.view_model:
            # 连接连接状态信号
            self.view_model.connection_status_changed.connect(self.update_connection_status)
            
            # 连接示教模式信号到 ViewModel
            self.teaching_mode_toggle_requested.connect(
                self.view_model.toggle_teaching_mode
            )
            
            # 连接力矩发送信号到 ViewModel
            self.send_torque_requested.connect(
                self.view_model.send_torque
            )
            
            # 订阅记录状态变化信号
            self.view_model.recording_state_changed.connect(
                self._on_recording_state_changed
            )
            
            # 订阅记录列表更新信号
            self.view_model.record_list_updated.connect(
                self._on_record_list_updated
            )
    
    def _toggle_teaching_mode(self):
        """切换示教模式状态"""
        self.teaching_mode_enabled = not self.teaching_mode_enabled
        
        if self.teaching_mode_enabled:
            self.teaching_mode_button.setText("关闭示教模式")
            # 开启示教模式命令
            command_dict = {
                'control': 0x07,  # 开启示教
                'mode': 0x0A      # 周期力矩模式
            }
        else:
            self.teaching_mode_button.setText("开启示教模式")
            # 关闭示教模式命令
            command_dict = {
                'control': 0x05,  # 关闭示教
                'mode': 0x0A      # 周期力矩模式
            }
        
        # 发射信号，传递命令字典
        self.teaching_mode_toggle_requested.emit(command_dict)
    
    def _send_torque(self):
        """发送力矩数据"""
        try:
            torque_values = self.torque_grid.get_float_values()
            # 发射信号
            self.send_torque_requested.emit(torque_values)
            
        except ValueError:
            # 如果输入不是有效数字，可以在这里处理错误
            print("力矩输入格式错误")
    
    def _clear_torque_inputs(self):
        """清零所有力矩输入框"""
        self.torque_grid.set_values(["0.0"] * 6)
    
    def _toggle_recording(self):
        """切换记录状态（委托给 ViewModel）"""
        if self.view_model:
            self.view_model.toggle_recording()
    
    def _delete_record(self):
        """删除选中的记录（委托给 ViewModel）"""
        current_item = self.record_list.currentItem()
        if current_item:
            record_name = current_item.text()
            
            # 确认删除
            reply = QMessageBox.question(
                self, '确认删除', 
                f'确定要删除记录 "{record_name}" 吗？',
                QMessageBox.Yes | QMessageBox.No, 
                QMessageBox.No
            )
            
            if reply == QMessageBox.Yes and self.view_model:
                self.view_model.delete_record(record_name)
    
    def _run_record(self):
        """运行选中的记录（委托给 ViewModel）"""
        current_item = self.record_list.currentItem()
        if current_item:
            record_name = current_item.text()
            if self.view_model:
                self.view_model.run_record(record_name)
        else:
            QMessageBox.warning(self, '警告', '请先选择一个记录！')
    
    def _reverse_record(self):
        """反转选中的记录（委托给 ViewModel）"""
        current_item = self.record_list.currentItem()
        if current_item:
            record_name = current_item.text()
            if self.view_model:
                new_name = self.view_model.reverse_record(record_name)
                if new_name:
                    QMessageBox.information(self, '完成', f'已生成反转记录 "{new_name}"')
                else:
                    QMessageBox.warning(self, '失败', f'反转记录失败')
        else:
            QMessageBox.warning(self, '警告', '请先选择一个记录！')
    
    # ════════════════════════════════════════════════════════
    # ViewModel 信号响应
    # ════════════════════════════════════════════════════════
    
    def _on_recording_state_changed(self, is_recording: bool):
        """记录状态变化回调"""
        if is_recording:
            self.record_button.setText("关闭记录")
        else:
            self.record_button.setText("开启记录")
    
    def _on_record_list_updated(self, record_names: list):
        """记录列表更新回调"""
        self.record_list.clear()
        for name in record_names:
            self.record_list.addItem(name)
    
    def get_torque_values(self):
        """获取当前力矩值"""
        return self.torque_grid.get_float_values()
    
    def set_torque_values(self, torque_values):
        """设置力矩值"""
        if len(torque_values) == 6:
            value_strings = [f"{value:.3f}" for value in torque_values]
            self.torque_grid.set_values(value_strings)
    
    def update_connection_status(self, connected):
        """更新连接状态"""
        for button in self.buttons:
            button.setEnabled(connected)
    
    def set_teaching_mode_enabled(self, enabled):
        """设置示教模式状态（外部调用）"""
        self.teaching_mode_enabled = enabled
        if enabled:
            self.teaching_mode_button.setText("关闭示教模式")
        else:
            self.teaching_mode_button.setText("开启示教模式")


