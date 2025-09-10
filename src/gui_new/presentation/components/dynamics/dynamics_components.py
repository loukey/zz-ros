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
    start_cyclic_torque_requested = pyqtSignal()
    teaching_mode_toggle_requested = pyqtSignal(bool)  # True为开启，False为关闭
    send_torque_requested = pyqtSignal(list)  # 发送力矩信号
    # 添加记录相关信号
    start_recording_requested = pyqtSignal()
    stop_recording_requested = pyqtSignal()
    delete_record_requested = pyqtSignal(str)  # 删除记录信号，传递记录名称
    run_record_requested = pyqtSignal(list)  # 运行记录信号，传递角度数组
    
    def __init__(self, parent=None, view_model=None):
        self.teaching_mode_enabled = False
        self.buttons = []
        self.torque_inputs = []
        self.recording_enabled = False
        self.record_data = {}  # 存储记录数据
        self.current_record_angles = []  # 当前记录的角度数组
        self.record_timer = QTimer()
        self.record_timer.setInterval(10)  # 10ms间隔
        self.record_timer.timeout.connect(self._on_record_timer_timeout)
        self.record_counter = 0  # 记录计数器
        self.teach_record_file = "teach_record.json"  # 记录文件名
        super().__init__(parent, view_model)
        self._load_records()  # 加载已存在的记录
    
    def setup_ui(self):
        """设置UI"""
        # 创建分组框
        group_box = QGroupBox("动力学控制")
        main_layout = QVBoxLayout(self)
        main_layout.addWidget(group_box)
        
        layout = QVBoxLayout(group_box)
        
        # 创建基本控制按钮区域
        basic_control_layout = QHBoxLayout()
        
        # 启动周期力矩模式按钮
        self.cyclic_torque_button = QPushButton("启动周期力矩模式")
        self.cyclic_torque_button.setFont(default_font)
        self.cyclic_torque_button.clicked.connect(self.start_cyclic_torque_requested.emit)
        self.cyclic_torque_button.setEnabled(False)
        basic_control_layout.addWidget(self.cyclic_torque_button)
        self.buttons.append(self.cyclic_torque_button)
        
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
            self.view_model.connection_status_changed.connect(self.update_connection_status)
    
    def _toggle_teaching_mode(self):
        """切换示教模式状态"""
        self.teaching_mode_enabled = not self.teaching_mode_enabled
        
        if self.teaching_mode_enabled:
            self.teaching_mode_button.setText("关闭示教模式")
        else:
            self.teaching_mode_button.setText("开启示教模式")
        
        # 发射信号
        self.teaching_mode_toggle_requested.emit(self.teaching_mode_enabled)
    
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
        """切换记录状态"""
        if not self.recording_enabled:
            # 开始记录
            self.recording_enabled = True
            self.record_button.setText("关闭记录")
            self.current_record_angles = []
            self.record_counter = 0
            self.record_timer.start()
            self.start_recording_requested.emit()
        else:
            # 停止记录
            self.recording_enabled = False
            self.record_button.setText("开启记录")
            self.record_timer.stop()
            self.stop_recording_requested.emit()
            self._save_current_record()
    
    def _delete_record(self):
        """删除选中的记录"""
        current_item = self.record_list.currentItem()
        if current_item:
            record_name = current_item.text()
            
            # 确认删除
            reply = QMessageBox.question(self, '确认删除', 
                                       f'确定要删除记录 "{record_name}" 吗？',
                                       QMessageBox.Yes | QMessageBox.No, 
                                       QMessageBox.No)
            
            if reply == QMessageBox.Yes:
                # 从数据中删除
                if record_name in self.record_data:
                    del self.record_data[record_name]
                
                # 从列表中删除
                self.record_list.takeItem(self.record_list.row(current_item))
                
                # 保存到文件
                self._save_records()
                
                # 发射信号
                self.delete_record_requested.emit(record_name)
    
    def _run_record(self):
        """运行选中的记录"""
        current_item = self.record_list.currentItem()
        if current_item:
            record_name = current_item.text()
            if record_name in self.record_data:
                angles_list = self.record_data[record_name]
                
                # 直接传递角度数组列表
                self.run_record_requested.emit(angles_list)
        else:
            QMessageBox.warning(self, '警告', '请先选择一个记录！')
    
    def _on_record_timer_timeout(self):
        """记录定时器超时处理"""
        # 这个方法将在dynamic_controller中被连接到获取角度的功能
        pass
    
    def add_angle_to_current_record(self, angles):
        """添加角度到当前记录"""
        if self.recording_enabled:
            self.current_record_angles.append(angles)
            self.record_counter += 1
    
    def _save_current_record(self):
        """保存当前记录"""
        if self.current_record_angles:
            # 生成记录名称
            record_name = f"record{len(self.record_data) + 1}"
            
            # 保存到数据字典
            self.record_data[record_name] = self.current_record_angles
            
            # 添加到列表显示
            self.record_list.addItem(record_name)
            
            # 保存到文件
            self._save_records()
            
            print(f"记录 {record_name} 已保存，共 {len(self.current_record_angles)} 个数据点")
    
    def _load_records(self):
        """加载记录文件"""
        try:
            if os.path.exists(self.teach_record_file):
                with open(self.teach_record_file, 'r', encoding='utf-8') as f:
                    self.record_data = json.load(f)
                
                # 更新列表显示
                self.record_list.clear()
                for record_name in self.record_data.keys():
                    self.record_list.addItem(record_name)
                
                print(f"已加载 {len(self.record_data)} 个记录")
        except Exception as e:
            print(f"加载记录文件失败: {e}")
            self.record_data = {}
    
    def _save_records(self):
        """保存记录到文件"""
        try:
            with open(self.teach_record_file, 'w', encoding='utf-8') as f:
                json.dump(self.record_data, f, ensure_ascii=False, indent=2)
            print("记录已保存到文件")
        except Exception as e:
            print(f"保存记录文件失败: {e}")
    
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
    
    def get_record_timer(self):
        """获取记录定时器"""
        return self.record_timer


