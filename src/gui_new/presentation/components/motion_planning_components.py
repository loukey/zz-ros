"""
运动规划组件
"""
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QPushButton, 
                            QTableWidget, QTableWidgetItem, QDialog, QLabel, 
                            QDoubleSpinBox, QComboBox, QFormLayout, QDialogButtonBox,
                            QHeaderView, QMessageBox, QLineEdit, QCheckBox, QRadioButton, QButtonGroup)
from PyQt5.QtCore import Qt, pyqtSignal
from .base_component import BaseComponent
import math  # 用于角度转弧度计算
import json
import os


class MotionPlanningFrame(BaseComponent):
    """运动规划框架"""
    
    motion_start_signal = pyqtSignal(list)
    get_current_position_signal = pyqtSignal()  # 添加获取当前位置的信号
    
    def __init__(self, parent=None, view_model=None):
        self.data_file_path = "./motion_planning_data.json"
        self.current_dialog = None  # 保存当前打开的对话框引用
        super().__init__(parent, view_model)
        self.load_motion_data()
    
    def setup_ui(self):
        """设置UI"""
        layout = QVBoxLayout(self)
        
        # 创建表格
        self.motion_table = MotionPlanningTable(self)
        self.motion_table.data_changed.connect(self.save_motion_data)  # 连接数据变更信号
        layout.addWidget(self.motion_table)
        
        # 添加操作按钮布局
        button_layout = QHBoxLayout()
        
        # 添加关节点按钮
        self.add_point_btn = QPushButton("添加关节点")
        self.add_point_btn.clicked.connect(self.add_motion_point)
        button_layout.addWidget(self.add_point_btn)
        
        # 删除选中点按钮
        self.delete_point_btn = QPushButton("删除选中点")
        self.delete_point_btn.clicked.connect(self.delete_motion_point)
        button_layout.addWidget(self.delete_point_btn)
        
        # 运行按钮
        self.run_btn = QPushButton("运行规划")
        self.run_btn.clicked.connect(self.run_motion_plan)
        button_layout.addWidget(self.run_btn)
        
        # 添加按钮布局
        layout.addLayout(button_layout)
    
    def add_motion_point(self):
        """添加运动点"""
        dialog = MotionPointDialog(self)
        self.current_dialog = dialog  # 保存当前对话框的引用
        dialog.only_get_current_position_send_signal.connect(self.on_get_current_position)
        if dialog.exec_() == QDialog.Accepted:
            self.motion_table.add_motion_point(dialog.get_motion_data())
            self.save_motion_data()
        self.current_dialog = None  # 对话框关闭后清除引用
    
    def delete_motion_point(self):
        """删除选中的运动点"""
        selected_rows = self.motion_table.selectionModel().selectedRows()
        if not selected_rows:
            QMessageBox.warning(self, "警告", "请先选择要删除的行")
            return
        
        for index in sorted(selected_rows, reverse=True):
            self.motion_table.removeRow(index.row())
        
        self.save_motion_data()
    
    def run_motion_plan(self):
        """执行运动规划"""
        motion_data = self.motion_table.get_all_motion_data()
        if not motion_data:
            QMessageBox.warning(self, "警告", "没有可执行的运动规划数据")
            return
        self.motion_start_signal.emit(motion_data)
    
    def save_motion_data(self):
        """保存运动规划数据到本地文件"""
        try:
            motion_data = self.motion_table.get_all_motion_data()
            with open(self.data_file_path, 'w', encoding='utf-8') as f:
                json.dump(motion_data, f, indent=2, ensure_ascii=False)
        except Exception as e:
            QMessageBox.warning(self, "保存失败", f"保存运动规划数据失败: {str(e)}")
    
    def load_motion_data(self):
        """从本地文件加载运动规划数据"""
        try:
            if os.path.exists(self.data_file_path):
                with open(self.data_file_path, 'r', encoding='utf-8') as f:
                    motion_data = json.load(f)
                    
                # 清空当前表格
                self.motion_table.setRowCount(0)
                
                # 添加加载的数据
                for point in motion_data:
                    self.motion_table.add_motion_point(point)
        except Exception as e:
            QMessageBox.warning(self, "加载失败", f"加载运动规划数据失败: {str(e)}")

    def on_get_current_position(self):
        """当对话框请求获取当前位置时触发"""
        self.get_current_position_signal.emit()
        
    def set_current_position(self, positions):
        """设置当前位置数据到对话框"""
        if self.current_dialog is not None:
            self.current_dialog.display_current_position(positions)
    
    def edit_table_motion_point(self, row, current_data):
        """编辑表格中的运动点"""
        # 创建编辑对话框
        dialog = MotionPointDialog(self, current_data)
        self.current_dialog = dialog  # 保存当前对话框的引用
        dialog.only_get_current_position_send_signal.connect(self.on_get_current_position)
        
        if dialog.exec_() == QDialog.Accepted:
            new_data = dialog.get_motion_data()
            self.motion_table.update_motion_point(row, new_data)
            self.save_motion_data()
            
        self.current_dialog = None  # 对话框关闭后清除引用


class MotionPlanningTable(QTableWidget):
    """运动规划表格组件"""
    
    data_changed = pyqtSignal()  # 数据变更信号
    
    def __init__(self, parent=None):
        super(MotionPlanningTable, self).__init__(parent)
        
        # 设置列数和列标题
        self.setColumnCount(12)
        self.setHorizontalHeaderLabels([
            "模式", "关节1", "关节2", "关节3", "关节4", "关节5", "关节6", 
            "频率", "曲线类型", "夹爪命令", "夹爪参数", "备注"
        ])
        
        # 设置表格属性
        self.setSelectionBehavior(QTableWidget.SelectRows)
        self.setSelectionMode(QTableWidget.SingleSelection)
        self.setAlternatingRowColors(True)
        self.setEditTriggers(QTableWidget.NoEditTriggers)  # 禁止直接编辑单元格
        
        # 设置列宽
        header = self.horizontalHeader()
        
        # 设置固定宽度的列
        self.setColumnWidth(0, 80)   # 模式列
        header.setSectionResizeMode(0, QHeaderView.Fixed)
        
        # 关节角度列 - 使用适中的固定宽度
        for i in range(1, 7):
            self.setColumnWidth(i, 100)
            header.setSectionResizeMode(i, QHeaderView.Fixed)
        
        # 频率列
        self.setColumnWidth(7, 80)
        header.setSectionResizeMode(7, QHeaderView.Fixed)
        
        # 曲线类型列
        self.setColumnWidth(8, 100)
        header.setSectionResizeMode(8, QHeaderView.Fixed)
        
        # 夹爪命令列 - 需要更宽以显示完整命令
        self.setColumnWidth(9, 180)
        header.setSectionResizeMode(9, QHeaderView.Fixed)
        
        # 夹爪参数列
        self.setColumnWidth(10, 100)
        header.setSectionResizeMode(10, QHeaderView.Fixed)
        
        # 备注列 - 自适应剩余空间
        header.setSectionResizeMode(11, QHeaderView.Stretch)
        
        # 连接双击信号
        self.cellDoubleClicked.connect(self.edit_motion_point)
        
        # 也响应单击回车键编辑
        self.keyPressEvent = self.handle_key_press
    
    def handle_key_press(self, event):
        """处理按键事件"""
        if event.key() == Qt.Key_Return or event.key() == Qt.Key_Enter:
            selected_rows = self.selectionModel().selectedRows()
            if selected_rows:
                self.edit_motion_point(selected_rows[0].row(), 0)
        else:
            # 调用父类的keyPressEvent以保持默认行为
            super(MotionPlanningTable, self).keyPressEvent(event)
    
    def add_motion_point(self, motion_data):
        """添加运动点到表格"""
        row_position = self.rowCount()
        self.insertRow(row_position)
        
        # 模式 (第0列) - 支持旧数据格式的兼容性
        mode = motion_data.get("mode")
        if mode is None:
            # 兼容旧数据格式
            if motion_data.get("enable_gripper", False):
                mode = "夹爪"
            else:
                mode = "运动"
        mode_item = QTableWidgetItem(mode)
        mode_item.setTextAlignment(Qt.AlignCenter)
        self.setItem(row_position, 0, mode_item)
        
        # 设置表格项 (关节角度从第1列开始)
        for col, key in enumerate(["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]):
            item = QTableWidgetItem(str(motion_data[key]))
            item.setTextAlignment(Qt.AlignCenter)
            self.setItem(row_position, col + 1, item)
        
        # 频率 (第7列)
        freq_item = QTableWidgetItem(str(motion_data["frequency"]))
        freq_item.setTextAlignment(Qt.AlignCenter)
        self.setItem(row_position, 7, freq_item)
        
        # 曲线类型 (第8列)
        curve_item = QTableWidgetItem(motion_data["curve_type"])
        curve_item.setTextAlignment(Qt.AlignCenter)
        self.setItem(row_position, 8, curve_item)
        
        # 夹爪命令 (第9列)
        gripper_cmd_item = QTableWidgetItem(motion_data.get("gripper_command", "00: 不进行任何操作"))
        gripper_cmd_item.setTextAlignment(Qt.AlignCenter)
        self.setItem(row_position, 9, gripper_cmd_item)
        
        # 夹爪参数 (第10列)
        gripper_param_item = QTableWidgetItem(str(motion_data.get("gripper_param", 0.0)))
        gripper_param_item.setTextAlignment(Qt.AlignCenter)
        self.setItem(row_position, 10, gripper_param_item)
        
        # 备注 (第11列)
        note_item = QTableWidgetItem(motion_data.get("note", ""))
        self.setItem(row_position, 11, note_item)
    
    def edit_motion_point(self, row, column):
        """编辑运动点"""
        # 获取当前行的数据
        current_data = {}
        
        # 模式 (第0列)
        mode_item = self.item(row, 0)
        current_data["mode"] = mode_item.text() if mode_item else "运动"
        
        # 关节角度 (第1-6列)
        for col, key in enumerate(["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]):
            item = self.item(row, col + 1)
            current_data[key] = float(item.text()) if item else 0.0
        
        # 频率 (第7列)
        freq_item = self.item(row, 7)
        current_data["frequency"] = float(freq_item.text()) if freq_item else 0.01
        
        # 曲线类型 (第8列)
        curve_item = self.item(row, 8)
        current_data["curve_type"] = curve_item.text() if curve_item else "直线"
        
        # 夹爪命令 (第9列)
        gripper_cmd_item = self.item(row, 9)
        current_data["gripper_command"] = gripper_cmd_item.text() if gripper_cmd_item else "00: 不进行任何操作"
        
        # 夹爪参数 (第10列)
        gripper_param_item = self.item(row, 10)
        current_data["gripper_param"] = float(gripper_param_item.text()) if gripper_param_item else 0.0
        
        # 备注 (第11列)
        note_item = self.item(row, 11)
        current_data["note"] = note_item.text() if note_item else ""
        
        # 获取父对象(MotionPlanningFrame)
        parent = self.parent()
        if hasattr(parent, 'edit_table_motion_point'):
            # 调用父对象的编辑方法
            parent.edit_table_motion_point(row, current_data)
        else:
            # 如果父对象没有edit_table_motion_point方法，则直接处理
            dialog = MotionPointDialog(self, current_data)
            if dialog.exec_() == QDialog.Accepted:
                self.update_motion_point(row, dialog.get_motion_data())
    
    def get_all_motion_data(self):
        """获取所有运动数据"""
        motion_data = []
        for row in range(self.rowCount()):
            point_data = {}
            
            # 模式 (第0列)
            mode_item = self.item(row, 0)
            point_data["mode"] = mode_item.text() if mode_item else "运动"
            
            # 关节角度 (第1-6列)
            for col, key in enumerate(["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]):
                item = self.item(row, col + 1)
                point_data[key] = float(item.text()) if item else 0.0
            
            # 频率 (第7列)
            freq_item = self.item(row, 7)
            point_data["frequency"] = float(freq_item.text()) if freq_item else 0.01
            
            # 曲线类型 (第8列)
            curve_item = self.item(row, 8)
            point_data["curve_type"] = curve_item.text() if curve_item else "直线"
            
            # 夹爪命令 (第9列)
            gripper_cmd_item = self.item(row, 9)
            point_data["gripper_command"] = gripper_cmd_item.text() if gripper_cmd_item else "00: 不进行任何操作"
            
            # 夹爪参数 (第10列)
            gripper_param_item = self.item(row, 10)
            point_data["gripper_param"] = float(gripper_param_item.text()) if gripper_param_item else 0.0
            
            # 备注 (第11列)
            note_item = self.item(row, 11)
            point_data["note"] = note_item.text() if note_item else ""
            
            motion_data.append(point_data)
        
        return motion_data

    def update_motion_point(self, row, new_data):
        """更新表格中的运动点数据"""
        # 模式 (第0列)
        mode_item = QTableWidgetItem(new_data.get("mode", "运动"))
        mode_item.setTextAlignment(Qt.AlignCenter)
        self.setItem(row, 0, mode_item)
        
        # 更新表格数据 (关节角度从第1列开始)
        for col, key in enumerate(["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]):
            item = QTableWidgetItem(str(new_data[key]))
            item.setTextAlignment(Qt.AlignCenter)
            self.setItem(row, col + 1, item)
        
        # 频率 (第7列)
        freq_item = QTableWidgetItem(str(new_data["frequency"]))
        freq_item.setTextAlignment(Qt.AlignCenter)
        self.setItem(row, 7, freq_item)
        
        # 曲线类型 (第8列)
        curve_item = QTableWidgetItem(new_data["curve_type"])
        curve_item.setTextAlignment(Qt.AlignCenter)
        self.setItem(row, 8, curve_item)
        
        # 夹爪命令 (第9列)
        gripper_cmd_item = QTableWidgetItem(new_data.get("gripper_command", "00: 不进行任何操作"))
        gripper_cmd_item.setTextAlignment(Qt.AlignCenter)
        self.setItem(row, 9, gripper_cmd_item)
        
        # 夹爪参数 (第10列)
        gripper_param_item = QTableWidgetItem(str(new_data.get("gripper_param", 0.0)))
        gripper_param_item.setTextAlignment(Qt.AlignCenter)
        self.setItem(row, 10, gripper_param_item)
        
        # 备注 (第11列)
        note_item = QTableWidgetItem(new_data.get("note", ""))
        self.setItem(row, 11, note_item)
        
        # 发出数据变更信号
        self.data_changed.emit()


class MotionPointDialog(QDialog):
    """运动点编辑对话框"""
    only_get_current_position_send_signal = pyqtSignal()
    
    def __init__(self, parent=None, data=None):
        super(MotionPointDialog, self).__init__(parent)
        self.setWindowTitle("编辑运动点")
        self.resize(400, 300)
        self.init_ui()
        
        # 如果提供了数据，则填充到界面
        if data:
            self.fill_data(data)
    
    def init_ui(self):
        """初始化UI"""
        layout = QVBoxLayout(self)
        
        # 最顶部的单选框布局
        radio_layout = QHBoxLayout()
        radio_layout.addWidget(QLabel("模式选择:"))
        
        self.mode_group = QButtonGroup()
        
        self.motion_radio = QRadioButton("运动")
        self.motion_radio.setChecked(True)  # 默认选中运动
        self.mode_group.addButton(self.motion_radio, 0)
        radio_layout.addWidget(self.motion_radio)
        
        self.gripper_radio = QRadioButton("夹爪")
        self.mode_group.addButton(self.gripper_radio, 1)
        radio_layout.addWidget(self.gripper_radio)
        
        radio_layout.addStretch()
        layout.addLayout(radio_layout)
        
        # 创建表单布局
        form_layout = QFormLayout()
        
        # 角度输入
        self.joint_spins = []
        for i in range(1, 7):
            spin = QDoubleSpinBox()
            spin.setRange(-180, 180)
            spin.setDecimals(12)
            spin.setSingleStep(1.0)
            form_layout.addRow(f"关节{i}角度:", spin)
            self.joint_spins.append(spin)
        
        # 频率输入
        self.frequency_spin = QDoubleSpinBox()
        self.frequency_spin.setRange(0.01, 0.1)
        self.frequency_spin.setDecimals(2)
        self.frequency_spin.setSingleStep(0.01)
        self.frequency_spin.setValue(0.01)
        form_layout.addRow("频率:", self.frequency_spin)
        
        # 曲线类型选择
        self.curve_type_combo = QComboBox()
        self.curve_type_combo.addItems(["S曲线", "直线"])
        form_layout.addRow("曲线类型:", self.curve_type_combo)
        
        # 夹爪命令选择
        self.gripper_command_combo = QComboBox()
        self.gripper_command_combo.addItems([
            "00: 不进行任何操作",
            "01: 夹爪手动使能",
            "02: 设置夹爪目标位置",
            "03: 设置夹爪速度",
            "04: 设置夹爪电流",
            "05: 查询夹爪抓取状态",
            "06: 查询夹爪目前位置",
            "07: 查询夹爪电流"
        ])
        form_layout.addRow("夹爪命令:", self.gripper_command_combo)
        
        # 夹爪参数输入
        self.gripper_param_spin = QDoubleSpinBox()
        self.gripper_param_spin.setRange(-1000.0, 1000.0)
        self.gripper_param_spin.setDecimals(2)
        self.gripper_param_spin.setSingleStep(1.0)
        self.gripper_param_spin.setValue(0.0)
        form_layout.addRow("夹爪参数:", self.gripper_param_spin)
        
        # 备注输入
        self.note_input = QLineEdit()
        form_layout.addRow("备注:", self.note_input)
        
        layout.addLayout(form_layout)
        
        # 添加功能按钮
        button_row = QHBoxLayout()
        
        # 转为弧度按钮
        self.convert_btn = QPushButton("转为弧度")
        self.convert_btn.clicked.connect(self.convert_to_radians)
        button_row.addWidget(self.convert_btn)
        
        # 获取位置按钮
        self.get_position_btn = QPushButton("获取位置")
        self.get_position_btn.clicked.connect(self.get_current_position)
        button_row.addWidget(self.get_position_btn)
        
        layout.addLayout(button_row)
        
        # 对话框按钮
        self.button_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        self.button_box.accepted.connect(self.accept)
        self.button_box.rejected.connect(self.reject)
        layout.addWidget(self.button_box)
    
    def fill_data(self, data):
        """使用提供的数据填充界面"""
        # 填充模式选择
        mode = data.get("mode", "运动")
        if mode == "夹爪":
            self.gripper_radio.setChecked(True)
        else:
            self.motion_radio.setChecked(True)
        
        # 填充关节角度
        for i, key in enumerate(["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]):
            if key in data:
                self.joint_spins[i].setValue(data[key])
        
        # 填充频率
        if "frequency" in data:
            self.frequency_spin.setValue(data["frequency"])
        
        # 填充曲线类型
        if "curve_type" in data:
            index = self.curve_type_combo.findText(data["curve_type"])
            if index >= 0:
                self.curve_type_combo.setCurrentIndex(index)
        
        # 填充夹爪命令
        if "gripper_command" in data:
            index = self.gripper_command_combo.findText(data["gripper_command"])
            if index >= 0:
                self.gripper_command_combo.setCurrentIndex(index)
        
        # 填充夹爪参数
        if "gripper_param" in data:
            self.gripper_param_spin.setValue(data["gripper_param"])
        
        # 填充备注
        if "note" in data:
            self.note_input.setText(data["note"])
    
    def get_motion_data(self):
        """获取运动点数据"""
        data = {}
        
        # 获取模式选择
        if self.gripper_radio.isChecked():
            data["mode"] = "夹爪"
        else:
            data["mode"] = "运动"
        
        # 获取关节角度
        for i, key in enumerate(["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]):
            data[key] = self.joint_spins[i].value()
        
        # 获取频率
        data["frequency"] = self.frequency_spin.value()
        
        # 获取曲线类型
        data["curve_type"] = self.curve_type_combo.currentText()
        
        # 获取夹爪命令
        data["gripper_command"] = self.gripper_command_combo.currentText()
        
        # 获取夹爪参数
        data["gripper_param"] = self.gripper_param_spin.value()
        
        # 获取备注
        data["note"] = self.note_input.text()
        
        return data
    
    def get_current_position(self):
        """获取当前机械臂位置"""
        self.only_get_current_position_send_signal.emit()
        
    def display_current_position(self, positions):
        """显示当前机械臂位置"""
        for i in range(6):
            self.joint_spins[i].setValue(positions[i])

    def convert_to_radians(self):
        """将角度值转换为弧度值"""
        for i, spin in enumerate(self.joint_spins):
            angle_value = spin.value()
            radian_value = math.radians(angle_value)
            spin.setValue(round(radian_value, 4)) 