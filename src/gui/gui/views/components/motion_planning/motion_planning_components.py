"""
运动规划组件
"""
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QPushButton, 
                            QTableWidget, QTableWidgetItem, QDialog, QLabel, 
                            QDoubleSpinBox, QComboBox, QFormLayout, QDialogButtonBox,
                            QHeaderView, QMessageBox, QLineEdit)
from PyQt5.QtCore import Qt, pyqtSignal
import math  # 用于角度转弧度计算
import json
import os


class MotionPlanningFrame(QWidget):
    motion_start_signal = pyqtSignal(list)
    get_current_position_signal = pyqtSignal()  # 添加获取当前位置的信号
    
    def __init__(self, parent=None):
        super(MotionPlanningFrame, self).__init__(parent)
        self.setObjectName("motion_planning_frame")
        self.data_file_path = "./motion_planning_data.json"
        self.current_dialog = None  # 保存当前打开的对话框引用
        self.init_ui()
        self.load_motion_data()
        
    def init_ui(self):
        """初始化UI"""
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
        """设置当前位置数据到对话框
        
        Args:
            positions: 包含当前关节位置的列表
        """
        if self.current_dialog is not None:
            self.current_dialog.display_current_position(positions)
    
    def edit_table_motion_point(self, row, current_data):
        """编辑表格中的运动点
        
        Args:
            row: 行索引
            current_data: 当前运动点数据
        """
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
        self.setColumnCount(9)
        self.setHorizontalHeaderLabels([
            "关节1", "关节2", "关节3", "关节4", "关节5", "关节6", 
            "频率", "曲线类型", "备注"
        ])
        
        # 设置表格属性
        self.setSelectionBehavior(QTableWidget.SelectRows)
        self.setSelectionMode(QTableWidget.SingleSelection)
        self.setAlternatingRowColors(True)
        self.setEditTriggers(QTableWidget.NoEditTriggers)  # 禁止直接编辑单元格
        
        # 设置列宽
        header = self.horizontalHeader()
        for i in range(6):  # 关节角度列
            header.setSectionResizeMode(i, QHeaderView.Stretch)
        header.setSectionResizeMode(6, QHeaderView.ResizeToContents)  # 频率列
        header.setSectionResizeMode(7, QHeaderView.ResizeToContents)  # 曲线类型列
        header.setSectionResizeMode(8, QHeaderView.Stretch)  # 备注列
        
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
        """添加运动点到表格
        
        Args:
            motion_data: 包含运动点数据的字典
        """
        row_position = self.rowCount()
        self.insertRow(row_position)
        
        # 设置表格项
        for col, key in enumerate(["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]):
            item = QTableWidgetItem(str(motion_data[key]))
            item.setTextAlignment(Qt.AlignCenter)
            self.setItem(row_position, col, item)
        
        # 频率
        freq_item = QTableWidgetItem(str(motion_data["frequency"]))
        freq_item.setTextAlignment(Qt.AlignCenter)
        self.setItem(row_position, 6, freq_item)
        
        # 曲线类型
        curve_item = QTableWidgetItem(motion_data["curve_type"])
        curve_item.setTextAlignment(Qt.AlignCenter)
        self.setItem(row_position, 7, curve_item)
        
        # 备注
        note_item = QTableWidgetItem(motion_data.get("note", ""))
        self.setItem(row_position, 8, note_item)
    
    def edit_motion_point(self, row, column):
        """编辑运动点
        
        Args:
            row: 行索引
            column: 列索引
        """
        # 获取当前行的数据
        current_data = {}
        for col, key in enumerate(["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]):
            item = self.item(row, col)
            current_data[key] = float(item.text()) if item else 0.0
        
        # 频率
        freq_item = self.item(row, 6)
        current_data["frequency"] = float(freq_item.text()) if freq_item else 1.0
        
        # 曲线类型
        curve_item = self.item(row, 7)
        current_data["curve_type"] = curve_item.text() if curve_item else "线性"
        
        # 备注
        note_item = self.item(row, 8)
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
        """获取所有运动数据
        
        Returns:
            list: 包含所有运动点数据的列表
        """
        motion_data = []
        for row in range(self.rowCount()):
            point_data = {}
            for col, key in enumerate(["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]):
                item = self.item(row, col)
                point_data[key] = float(item.text()) if item else 0.0
            
            # 频率
            freq_item = self.item(row, 6)
            point_data["frequency"] = float(freq_item.text()) if freq_item else 1.0
            
            # 曲线类型
            curve_item = self.item(row, 7)
            point_data["curve_type"] = curve_item.text() if curve_item else "线性"
            
            # 备注
            note_item = self.item(row, 8)
            point_data["note"] = note_item.text() if note_item else ""
            
            motion_data.append(point_data)
        
        return motion_data

    def update_motion_point(self, row, new_data):
        """更新表格中的运动点数据
        
        Args:
            row: 行索引
            new_data: 新的运动点数据
        """
        # 更新表格数据
        for col, key in enumerate(["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]):
            item = QTableWidgetItem(str(new_data[key]))
            item.setTextAlignment(Qt.AlignCenter)
            self.setItem(row, col, item)
        
        # 频率
        freq_item = QTableWidgetItem(str(new_data["frequency"]))
        freq_item.setTextAlignment(Qt.AlignCenter)
        self.setItem(row, 6, freq_item)
        
        # 曲线类型
        curve_item = QTableWidgetItem(new_data["curve_type"])
        curve_item.setTextAlignment(Qt.AlignCenter)
        self.setItem(row, 7, curve_item)
        
        # 备注
        note_item = QTableWidgetItem(new_data.get("note", ""))
        self.setItem(row, 8, note_item)
        
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
        """使用提供的数据填充界面
        
        Args:
            data: 包含运动点数据的字典
        """
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
        
        # 填充备注
        if "note" in data:
            self.note_input.setText(data["note"])
    
    def get_motion_data(self):
        """获取运动点数据
        
        Returns:
            dict: 包含运动点数据的字典
        """
        data = {}
        
        # 获取关节角度
        for i, key in enumerate(["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]):
            data[key] = self.joint_spins[i].value()
        
        # 获取频率
        data["frequency"] = self.frequency_spin.value()
        
        # 获取曲线类型
        data["curve_type"] = self.curve_type_combo.currentText()
        
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
