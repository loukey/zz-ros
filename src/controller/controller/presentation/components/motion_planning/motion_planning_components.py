"""
Motion planning tab components
"""
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QPushButton, 
                            QTableWidget, QTableWidgetItem, QDialog, QLabel, 
                            QDoubleSpinBox, QComboBox, QFormLayout, QDialogButtonBox,
                            QHeaderView, QMessageBox, QLineEdit, QCheckBox, QRadioButton, 
                            QButtonGroup, QInputDialog, QListWidget, QSpinBox)
from PyQt5.QtCore import Qt, pyqtSignal
from ..base_component import BaseComponent
from .trajectory_plot_dialog import TrajectoryPlotDialog
import math


class MotionPlanningFrame(BaseComponent):
    """运动规划框架"""
    
    motion_start_signal = pyqtSignal(list)
    get_current_position_signal = pyqtSignal()  # 添加获取当前位置的信号
    
    def __init__(self, parent=None, view_model=None):
        self.current_dialog = None  # 保存当前打开的对话框引用
        super().__init__(parent, view_model)
    
    def connect_signals(self):
        """连接视图模型信号"""
        if self.view_model:
            # 连接方案管理信号
            self.view_model.plan_list_changed.connect(self.refresh_plan_list)
            self.view_model.current_plan_changed.connect(self.refresh_point_list)
            self.view_model.point_list_changed.connect(self.refresh_point_list)
            
            # 连接"获取位置"信号流
            # UI请求 → ViewModel
            self.get_current_position_signal.connect(
                self.view_model.request_current_position
            )
            # ViewModel响应 → UI显示
            self.view_model.current_position_received.connect(
                self.set_current_position
            )
            
            # 连接"轨迹预览"信号
            self.view_model.trajectory_preview_signal.connect(
                self.show_trajectory_plot
            )
    
    def setup_ui(self):
        """设置UI"""
        layout = QVBoxLayout(self)
        
        # ========== 1. 方案选择区 ==========
        plan_selector_layout = QHBoxLayout()
        
        plan_selector_layout.addWidget(QLabel("运动方案:"))
        
        self.plan_combo = QComboBox()
        self.plan_combo.currentIndexChanged.connect(self.on_plan_switched)
        plan_selector_layout.addWidget(self.plan_combo)
        
        self.new_plan_btn = QPushButton("新建方案")
        self.new_plan_btn.clicked.connect(self.create_plan)
        plan_selector_layout.addWidget(self.new_plan_btn)
        
        self.delete_plan_btn = QPushButton("删除方案")
        self.delete_plan_btn.clicked.connect(self.delete_plan)
        plan_selector_layout.addWidget(self.delete_plan_btn)
        
        self.rename_plan_btn = QPushButton("重命名")
        self.rename_plan_btn.clicked.connect(self.rename_plan)
        plan_selector_layout.addWidget(self.rename_plan_btn)
        
        plan_selector_layout.addStretch()
        layout.addLayout(plan_selector_layout)
        
        # ========== 2. 节点表格 ==========
        self.motion_table = MotionPlanningTable(self)
        layout.addWidget(self.motion_table)
        
        # ========== 3. 操作按钮区（第一行） ==========
        button_layout = QHBoxLayout()
        
        # 添加关节点按钮
        self.add_point_btn = QPushButton("添加节点")
        self.add_point_btn.clicked.connect(self.add_motion_point)
        button_layout.addWidget(self.add_point_btn)
        
        # 加载示教记录按钮
        self.load_teach_btn = QPushButton("加载示教记录")
        self.load_teach_btn.clicked.connect(self.load_teach_record)
        button_layout.addWidget(self.load_teach_btn)
        
        # 加载本地轨迹按钮
        self.load_trajectory_btn = QPushButton("加载本地轨迹")
        self.load_trajectory_btn.clicked.connect(self.load_local_trajectory)
        button_layout.addWidget(self.load_trajectory_btn)
        
        # 删除选中点按钮
        self.delete_point_btn = QPushButton("删除节点")
        self.delete_point_btn.clicked.connect(self.delete_motion_point)
        button_layout.addWidget(self.delete_point_btn)
        
        # 上移按钮
        self.move_up_btn = QPushButton("上移↑")
        self.move_up_btn.clicked.connect(self.move_point_up)
        button_layout.addWidget(self.move_up_btn)
        
        # 下移按钮
        self.move_down_btn = QPushButton("下移↓")
        self.move_down_btn.clicked.connect(self.move_point_down)
        button_layout.addWidget(self.move_down_btn)
        
        # 编辑按钮
        self.edit_btn = QPushButton("编辑")
        self.edit_btn.clicked.connect(self.edit_motion_point)
        button_layout.addWidget(self.edit_btn)
        
        button_layout.addStretch()
        
        # 运行选中节点按钮
        self.run_single_btn = QPushButton("运行选中节点")
        self.run_single_btn.clicked.connect(self.run_single_point)
        button_layout.addWidget(self.run_single_btn)
        
        # 运行整个方案按钮
        self.run_btn = QPushButton("运行整个方案")
        self.run_btn.clicked.connect(self.run_motion_plan)
        button_layout.addWidget(self.run_btn)
        
        # 添加第一行按钮布局
        layout.addLayout(button_layout)
        
        # ========== 4. 轨迹操作按钮区（第二行） ==========
        button_layout2 = QHBoxLayout()
        
        # 保存节点轨迹按钮
        self.save_node_btn = QPushButton("保存节点轨迹")
        self.save_node_btn.clicked.connect(self.save_node_trajectory)
        button_layout2.addWidget(self.save_node_btn)
        
        # 保存方案轨迹按钮
        self.save_plan_btn = QPushButton("保存方案轨迹")
        self.save_plan_btn.clicked.connect(self.save_plan_trajectory)
        button_layout2.addWidget(self.save_plan_btn)
        
        # 显示节点曲线按钮
        self.preview_node_btn = QPushButton("显示节点曲线")
        self.preview_node_btn.clicked.connect(self.preview_node_trajectory)
        button_layout2.addWidget(self.preview_node_btn)
        
        # 显示方案曲线按钮
        self.preview_plan_btn = QPushButton("显示方案曲线")
        self.preview_plan_btn.clicked.connect(self.preview_plan_trajectory)
        button_layout2.addWidget(self.preview_plan_btn)
        
        # 合并曲线按钮
        self.merge_curves_btn = QPushButton("合并曲线")
        self.merge_curves_btn.clicked.connect(self.merge_selected_curves)
        button_layout2.addWidget(self.merge_curves_btn)
        
        button_layout2.addStretch()
        
        # 添加第二行按钮布局
        layout.addLayout(button_layout2)
        
        # ========== 初始化加载 ==========
        self.refresh_plan_list()
        self.refresh_point_list()
    
    # ========== 方案操作 ==========
    
    def create_plan(self):
        """创建新方案"""
        if not self.view_model:
            return
        
        name, ok = QInputDialog.getText(self, "新建方案", "方案名称:")
        if ok and name:
            self.view_model.create_plan(name)
    
    def delete_plan(self):
        """删除方案"""
        if not self.view_model:
            return
        
        current_index = self.plan_combo.currentIndex()
        if current_index >= 0:
            reply = QMessageBox.question(
                self, 
                "确认删除", 
                "确定删除该方案？",
                QMessageBox.Yes | QMessageBox.No
            )
            if reply == QMessageBox.Yes:
                success = self.view_model.delete_plan(current_index)
                if not success:
                    QMessageBox.warning(self, "警告", "至少需要保留一个方案！")
    
    def rename_plan(self):
        """重命名方案"""
        if not self.view_model:
            return
        
        current_index = self.plan_combo.currentIndex()
        if current_index >= 0:
            old_name = self.plan_combo.currentText()
            new_name, ok = QInputDialog.getText(
                self, 
                "重命名方案", 
                "新方案名称:", 
                text=old_name
            )
            if ok and new_name and new_name != old_name:
                self.view_model.rename_plan(current_index, new_name)
    
    def on_plan_switched(self, index: int):
        """方案切换（下拉框变化）"""
        if not self.view_model or index < 0:
            return
        # 只有用户手动切换时才调用
        if self.plan_combo.signalsBlocked():
            return
        self.view_model.switch_plan(index)
    
    def refresh_plan_list(self):
        """刷新方案列表"""
        if not self.view_model:
            return
        
        self.plan_combo.blockSignals(True)
        self.plan_combo.clear()
        names = self.view_model.get_plan_names()
        self.plan_combo.addItems(names)
        current_index = self.view_model.get_current_plan_index()
        if current_index >= 0:
            self.plan_combo.setCurrentIndex(current_index)
        self.plan_combo.blockSignals(False)
    
    def refresh_current_plan(self, index: int):
        """刷新当前方案选中状态"""
        self.plan_combo.blockSignals(True)
        if index >= 0:
            self.plan_combo.setCurrentIndex(index)
        self.plan_combo.blockSignals(False)
    
    # ========== 节点操作 ==========
    
    def add_motion_point(self):
        """添加运动点"""
        dialog = MotionPointDialog(self)
        self.current_dialog = dialog
        dialog.only_get_current_position_send_signal.connect(self.on_get_current_position)
        if dialog.exec_() == QDialog.Accepted:
            point_data = dialog.get_motion_data()
            if self.view_model:
                self.view_model.add_point(point_data)
        self.current_dialog = None
    
    def delete_motion_point(self):
        """删除选中的运动点"""
        if not self.view_model:
            return
        
        current_row = self.motion_table.currentRow()
        if current_row < 0:
            QMessageBox.warning(self, "警告", "请先选择要删除的节点")
            return
        
        self.view_model.delete_point(current_row)
    
    def move_point_up(self):
        """上移节点"""
        if not self.view_model:
            return
        
        current_row = self.motion_table.currentRow()
        if current_row <= 0:
            return
        
        self.view_model.move_point_up(current_row)
        # 保持选中状态（上移后选中上一行）
        self.motion_table.setCurrentCell(current_row - 1, 0)
    
    def move_point_down(self):
        """下移节点"""
        if not self.view_model:
            return
        
        current_row = self.motion_table.currentRow()
        if current_row < 0 or current_row >= self.motion_table.rowCount() - 1:
            return
        
        self.view_model.move_point_down(current_row)
        # 保持选中状态（下移后选中下一行）
        self.motion_table.setCurrentCell(current_row + 1, 0)
    
    def edit_motion_point(self):
        """编辑选中的运动点"""
        current_row = self.motion_table.currentRow()
        if current_row >= 0:
            self.edit_table_motion_point(current_row)
    
    def load_teach_record(self):
        """加载示教记录"""
        if not self.view_model:
            return
        
        # 获取所有示教记录名称
        record_names = self.view_model.get_teach_record_names()
        if not record_names:
            QMessageBox.information(self, "提示", "当前没有示教记录")
            return
        
        # 创建选择对话框
        from PyQt5.QtWidgets import QInputDialog
        record_name, ok = QInputDialog.getItem(
            self,
            "选择示教记录",
            "请选择要加载的示教记录:",
            record_names,
            0,
            False
        )
        
        if ok and record_name:
            # 获取示教记录数据
            teach_angles = self.view_model.get_teach_record(record_name)
            if not teach_angles:
                QMessageBox.warning(self, "警告", "获取示教记录失败")
                return
            
            # 创建示教节点数据（模式为"示教-记录名"）
            point_data = {
                "mode": f"示教-{record_name}",
                "joint1": 0.0,
                "joint2": 0.0,
                "joint3": 0.0,
                "joint4": 0.0,
                "joint5": 0.0,
                "joint6": 0.0,
                "frequency": 0.01,
                "curve_type": "S曲线",
                "gripper_command": "00: 不进行任何操作",
                "gripper_param": 0.0,
                "note": f"示教记录，共{len(teach_angles)}个点",
                "teach_record_name": record_name,  # 保存原始记录名
                "teach_data": teach_angles  # ⚠️ 修正：统一使用 teach_data 字段名
            }
            
            # 添加到当前方案
            self.view_model.add_point(point_data)
            QMessageBox.information(self, "成功", f"已加载示教记录：{record_name}")
    
    def load_local_trajectory(self):
        """从 plans 目录加载本地轨迹文件"""
        if not self.view_model:
            return
        
        # 使用文件选择对话框
        from PyQt5.QtWidgets import QFileDialog
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "选择轨迹文件",
            "./plans",  # 默认目录
            "轨迹文件 (*.json);;所有文件 (*)"
        )
        
        if not file_path:
            return
        
        # 加载文件
        success = self.view_model.load_local_trajectory(file_path)
        
        if success:
            from pathlib import Path
            filename = Path(file_path).stem
            QMessageBox.information(self, "成功", f"已加载轨迹文件：{filename}")
        else:
            QMessageBox.warning(self, "失败", "轨迹加载失败，请检查文件格式")
    
    def run_motion_plan(self):
        """执行整个运动方案"""
        if not self.view_model:
            return
        
        motion_data = self.view_model.get_all_points()
        if not motion_data:
            QMessageBox.warning(self, "警告", "当前方案没有运动节点")
            return
        
        # 调用 ViewModel 的执行方法
        self.view_model.execute_motion_plan()
    
    def run_single_point(self):
        """执行选中的单个节点"""
        if not self.view_model:
            return
        
        current_row = self.motion_table.currentRow()
        if current_row < 0:
            QMessageBox.warning(self, "警告", "请先选择要运行的节点")
            return
        
        # 调用 ViewModel 的执行方法
        self.view_model.execute_single_point(current_row)
    
    # ========== 保存轨迹功能 ==========
    
    def save_node_trajectory(self):
        """保存选中节点的轨迹"""
        if not self.view_model:
            return
        
        current_row = self.motion_table.currentRow()
        if current_row < 0:
            QMessageBox.warning(self, "警告", "请先选择要保存的节点")
            return
        
        # 调用 ViewModel 的保存方法
        success = self.view_model.save_node_trajectory(current_row)
        
        if success:
            QMessageBox.information(
                self,
                "提示",
                "正在获取当前位置并保存轨迹...\n请查看消息显示区域查看保存结果"
            )
        else:
            QMessageBox.warning(self, "错误", "准备保存失败，请检查节点数据")
    
    def save_plan_trajectory(self):
        """保存整个方案的轨迹"""
        if not self.view_model:
            return
        
        motion_data = self.view_model.get_all_points()
        if not motion_data:
            QMessageBox.warning(self, "警告", "当前方案没有运动节点")
            return
        
        # 确认对话框
        reply = QMessageBox.question(
            self,
            "确认保存",
            "是否保存整个方案的轨迹？",
            QMessageBox.Yes | QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            success = self.view_model.save_plan_trajectory()
            
            if success:
                QMessageBox.information(
                    self,
                    "提示",
                    "正在获取当前位置并保存轨迹...\n请查看消息显示区域查看保存结果"
                )
            else:
                QMessageBox.warning(self, "错误", "准备保存失败，请检查方案数据")
    
    # ========== 预览轨迹功能 ==========
    
    def preview_node_trajectory(self):
        """显示选中节点的轨迹曲线"""
        if not self.view_model:
            return
        
        current_row = self.motion_table.currentRow()
        if current_row < 0:
            QMessageBox.warning(self, "警告", "请先选择要预览的节点")
            return
        
        # 调用 ViewModel 的预览方法
        success = self.view_model.preview_node_trajectory(current_row)
        
        if success:
            QMessageBox.information(
                self,
                "提示",
                "正在获取当前位置并生成轨迹曲线...\n请稍候"
            )
        else:
            QMessageBox.warning(self, "错误", "准备预览失败，请检查节点数据")
    
    def preview_plan_trajectory(self):
        """显示整个方案的轨迹曲线"""
        if not self.view_model:
            return
        
        motion_data = self.view_model.get_all_points()
        if not motion_data:
            QMessageBox.warning(self, "警告", "当前方案没有运动节点")
            return
        
        # 调用 ViewModel 的预览方法
        success = self.view_model.preview_plan_trajectory()
        
        if success:
            QMessageBox.information(
                self,
                "提示",
                "正在获取当前位置并生成轨迹曲线...\n请稍候"
            )
        else:
            QMessageBox.warning(self, "错误", "准备预览失败，请检查方案数据")
    
    # ========== 合并曲线功能 ==========
    
    def merge_selected_curves(self):
        """合并选中的多个节点为示教节点"""
        if not self.view_model:
            return
        
        # 获取选中的行
        selected_rows = self.motion_table.get_selected_rows()
        
        # 校验：至少选中2个节点
        if len(selected_rows) < 2:
            QMessageBox.warning(self, "警告", "请至少选中2个节点进行合并\n（按住 Ctrl 键可多选）")
            return
        
        # 校验：选中的节点必须是连续的
        for i in range(1, len(selected_rows)):
            if selected_rows[i] - selected_rows[i - 1] != 1:
                QMessageBox.warning(self, "警告", "选中的节点必须是连续的")
                return
        
        # 确认对话框
        reply = QMessageBox.question(
            self,
            "确认合并",
            f"是否将选中的 {len(selected_rows)} 个节点（第 {selected_rows[0]+1} 至 {selected_rows[-1]+1} 行）合并为一个示教节点？\n\n"
            "合并后将无法撤销。",
            QMessageBox.Yes | QMessageBox.No
        )
        
        if reply != QMessageBox.Yes:
            return
        
        # 调用 ViewModel 合并方法
        success, message = self.view_model.merge_selected_nodes(selected_rows)
        
        if success:
            QMessageBox.information(self, "成功", message)
        else:
            QMessageBox.warning(self, "合并失败", message)
    
    def show_trajectory_plot(self, trajectory_data: dict, context: dict):
        """
        显示轨迹曲线对话框
        
        Args:
            trajectory_data: 轨迹数据 {
                "time": [...],
                "positions": [[...], ...],
                "velocities": [[...], ...],
                "accelerations": [[...], ...]
            }
            context: 上下文信息 {"type": "node|plan", ...}
        """
        dialog = TrajectoryPlotDialog(self, trajectory_data, context)
        dialog.exec_()
    
    def refresh_point_list(self):
        """刷新节点列表"""
        if not self.view_model:
            return
        
        # 保存当前选中行
        current_row = self.motion_table.currentRow()
        
        # 清空表格
        self.motion_table.setRowCount(0)
        
        # 加载节点
        points = self.view_model.get_all_points()
        for point in points:
            self.motion_table.add_motion_point(point)
        
        # 恢复选中状态
        if current_row >= 0 and current_row < self.motion_table.rowCount():
            self.motion_table.setCurrentCell(current_row, 0)

    def on_get_current_position(self):
        """当对话框请求获取当前位置时触发"""
        self.get_current_position_signal.emit()
        
    def set_current_position(self, positions):
        """设置当前位置数据到对话框"""
        if self.current_dialog is not None:
            self.current_dialog.display_current_position(positions)
    
    def edit_table_motion_point(self, row):
        """编辑表格中的运动点"""
        if not self.view_model:
            return
        
        # 获取当前数据
        points = self.view_model.get_all_points()
        if row < 0 or row >= len(points):
            return
        
        current_data = points[row]
        
        # 检查是否为示教节点（模式以"示教-"开头）
        if current_data.get("mode", "").startswith("示教-"):
            QMessageBox.warning(
                self, 
                "无法编辑", 
                "示教记录节点不可编辑，只能删除。\n如需修改，请删除后重新加载。"
            )
            return
        
        # 创建编辑对话框
        dialog = MotionPointDialog(self, current_data)
        self.current_dialog = dialog
        dialog.only_get_current_position_send_signal.connect(self.on_get_current_position)
        
        if dialog.exec_() == QDialog.Accepted:
            new_data = dialog.get_motion_data()
            self.view_model.update_point(row, new_data)
        
        self.current_dialog = None


class MotionPlanningTable(QTableWidget):
    """运动规划表格组件"""
    
    def __init__(self, parent=None):
        super(MotionPlanningTable, self).__init__(parent)
        
        # 设置列数和列标题
        self.setColumnCount(16)
        self.setHorizontalHeaderLabels([
            "模式", "关节1", "关节2", "关节3", "关节4", "关节5", "关节6", 
            "频率", "曲线类型", "夹爪命令", "夹爪参数", "其他指令", "指令值",
            "前置等待", "后置等待", "备注"
        ])
        
        # 设置表格属性
        self.setSelectionBehavior(QTableWidget.SelectRows)
        self.setSelectionMode(QTableWidget.ExtendedSelection)  # 支持 Ctrl 多选
        self.setAlternatingRowColors(True)
        self.setEditTriggers(QTableWidget.NoEditTriggers)  # 禁止直接编辑单元格
        
        # 设置列宽（用户可以手动调整）
        header = self.horizontalHeader()
        
        # 设置初始列宽，并允许用户手动调整
        self.setColumnWidth(0, 180)   # 模式列
        header.setSectionResizeMode(0, QHeaderView.Interactive)
        
        # 关节角度列
        for i in range(1, 7):
            self.setColumnWidth(i, 100)
            header.setSectionResizeMode(i, QHeaderView.Interactive)
        
        # 频率列
        self.setColumnWidth(7, 80)
        header.setSectionResizeMode(7, QHeaderView.Interactive)
        
        # 曲线类型列
        self.setColumnWidth(8, 100)
        header.setSectionResizeMode(8, QHeaderView.Interactive)
        
        # 夹爪命令列
        self.setColumnWidth(9, 180)
        header.setSectionResizeMode(9, QHeaderView.Interactive)
        
        # 夹爪参数列
        self.setColumnWidth(10, 100)
        header.setSectionResizeMode(10, QHeaderView.Interactive)
        
        # 其他指令列
        self.setColumnWidth(11, 80)
        header.setSectionResizeMode(11, QHeaderView.Interactive)
        
        # 指令值列
        self.setColumnWidth(12, 60)
        header.setSectionResizeMode(12, QHeaderView.Interactive)
        
        # 前置等待列
        self.setColumnWidth(13, 80)
        header.setSectionResizeMode(13, QHeaderView.Interactive)
        
        # 后置等待列
        self.setColumnWidth(14, 80)
        header.setSectionResizeMode(14, QHeaderView.Interactive)
        
        # 备注列 - 自适应剩余空间（也可手动调整）
        header.setSectionResizeMode(15, QHeaderView.Stretch)
        
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
        
        # 模式 (第0列)
        mode = motion_data.get("mode", "运动点")
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
        if "blend_points" in motion_data:
            curve_item.setData(Qt.UserRole, motion_data["blend_points"])
        self.setItem(row_position, 8, curve_item)
        
        # 夹爪命令 (第9列)
        gripper_cmd_item = QTableWidgetItem(motion_data.get("gripper_command", "00: 不进行任何操作"))
        gripper_cmd_item.setTextAlignment(Qt.AlignCenter)
        self.setItem(row_position, 9, gripper_cmd_item)
        
        # 夹爪参数 (第10列)
        gripper_param_item = QTableWidgetItem(str(motion_data.get("gripper_param", 0.0)))
        gripper_param_item.setTextAlignment(Qt.AlignCenter)
        self.setItem(row_position, 10, gripper_param_item)
        
        # 其他指令 (第11列) - 清扫/压机
        other_cmd_type = motion_data.get("other_command_type", "-")
        other_cmd_item = QTableWidgetItem(other_cmd_type)
        other_cmd_item.setTextAlignment(Qt.AlignCenter)
        self.setItem(row_position, 11, other_cmd_item)
        
        # 指令值 (第12列)
        other_cmd_value = motion_data.get("other_command_value", 0)
        other_value_item = QTableWidgetItem(str(other_cmd_value) if other_cmd_type != "-" else "-")
        other_value_item.setTextAlignment(Qt.AlignCenter)
        self.setItem(row_position, 12, other_value_item)
        
        # 前置等待 (第13列) - 根据模式选择正确的字段
        if mode == "其他":
            pre_delay = motion_data.get("other_pre_delay", 0.0)
        else:
            pre_delay = motion_data.get("gripper_pre_delay", 0.0)
        pre_delay_item = QTableWidgetItem(str(pre_delay) + "s")
        pre_delay_item.setTextAlignment(Qt.AlignCenter)
        self.setItem(row_position, 13, pre_delay_item)
        
        # 后置等待 (第14列) - 根据模式选择正确的字段
        if mode == "其他":
            post_delay = motion_data.get("other_post_delay", 1.0)
        else:
            post_delay = motion_data.get("gripper_post_delay", 1.0)
        post_delay_item = QTableWidgetItem(str(post_delay) + "s")
        post_delay_item.setTextAlignment(Qt.AlignCenter)
        self.setItem(row_position, 14, post_delay_item)
        
        # 备注 (第15列)
        note_item = QTableWidgetItem(motion_data.get("note", ""))
        self.setItem(row_position, 15, note_item)
    
    def edit_motion_point(self, row, column):
        """编辑运动点（双击时触发）"""
        # 检查是否为示教节点
        mode_item = self.item(row, 0)
        if mode_item and mode_item.text().startswith("示教-"):
            QMessageBox.warning(
                self, 
                "无法编辑", 
                "示教记录节点不可编辑，只能删除。\n如需修改，请删除后重新加载。"
            )
            return
        
        # 获取父对象(MotionPlanningFrame)
        parent = self.parent()
        if hasattr(parent, 'edit_table_motion_point'):
            parent.edit_table_motion_point(row)
    
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
            if point_data["curve_type"] == "混合" and curve_item:
                blend_points = curve_item.data(Qt.UserRole)
                if blend_points:
                    point_data["blend_points"] = blend_points
            
            # 夹爪命令 (第9列)
            gripper_cmd_item = self.item(row, 9)
            point_data["gripper_command"] = gripper_cmd_item.text() if gripper_cmd_item else "00: 不进行任何操作"
            
            # 夹爪参数 (第10列)
            gripper_param_item = self.item(row, 10)
            point_data["gripper_param"] = float(gripper_param_item.text()) if gripper_param_item else 0.0
            
            # 其他指令 (第11列)
            other_cmd_item = self.item(row, 11)
            other_cmd_type = other_cmd_item.text() if other_cmd_item else "-"
            if other_cmd_type != "-":
                point_data["other_command_type"] = other_cmd_type
            
            # 指令值 (第12列)
            other_value_item = self.item(row, 12)
            if other_value_item and other_value_item.text() != "-":
                try:
                    point_data["other_command_value"] = int(other_value_item.text())
                except ValueError:
                    point_data["other_command_value"] = 0
            
            # 前置等待 (第13列) - 移除"s"后缀，根据模式保存到正确的字段
            pre_delay_item = self.item(row, 13)
            pre_delay_value = 0.0
            if pre_delay_item:
                pre_delay_text = pre_delay_item.text().replace("s", "").strip()
                pre_delay_value = float(pre_delay_text) if pre_delay_text else 0.0
            
            # 后置等待 (第14列) - 移除"s"后缀，根据模式保存到正确的字段
            post_delay_item = self.item(row, 14)
            post_delay_value = 1.0
            if post_delay_item:
                post_delay_text = post_delay_item.text().replace("s", "").strip()
                post_delay_value = float(post_delay_text) if post_delay_text else 1.0
            
            # 根据模式保存到正确的字段
            if point_data["mode"] == "其他":
                point_data["other_pre_delay"] = pre_delay_value
                point_data["other_post_delay"] = post_delay_value
            else:
                point_data["gripper_pre_delay"] = pre_delay_value
                point_data["gripper_post_delay"] = post_delay_value
            
            # 备注 (第15列)
            note_item = self.item(row, 15)
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
        if "blend_points" in new_data:
            curve_item.setData(Qt.UserRole, new_data["blend_points"])
        self.setItem(row, 8, curve_item)
        
        # 夹爪命令 (第9列)
        gripper_cmd_item = QTableWidgetItem(new_data.get("gripper_command", "00: 不进行任何操作"))
        gripper_cmd_item.setTextAlignment(Qt.AlignCenter)
        self.setItem(row, 9, gripper_cmd_item)
        
        # 夹爪参数 (第10列)
        gripper_param_item = QTableWidgetItem(str(new_data.get("gripper_param", 0.0)))
        gripper_param_item.setTextAlignment(Qt.AlignCenter)
        self.setItem(row, 10, gripper_param_item)
        
        # 其他指令 (第11列)
        other_cmd_type = new_data.get("other_command_type", "-")
        other_cmd_item = QTableWidgetItem(other_cmd_type)
        other_cmd_item.setTextAlignment(Qt.AlignCenter)
        self.setItem(row, 11, other_cmd_item)
        
        # 指令值 (第12列)
        other_cmd_value = new_data.get("other_command_value", 0)
        other_value_item = QTableWidgetItem(str(other_cmd_value) if other_cmd_type != "-" else "-")
        other_value_item.setTextAlignment(Qt.AlignCenter)
        self.setItem(row, 12, other_value_item)
        
        # 前置等待 (第13列) - 根据模式选择正确的字段
        mode = new_data.get("mode", "运动")
        if mode == "其他":
            pre_delay = new_data.get("other_pre_delay", 0.0)
        else:
            pre_delay = new_data.get("gripper_pre_delay", 0.0)
        pre_delay_item = QTableWidgetItem(str(pre_delay) + "s")
        pre_delay_item.setTextAlignment(Qt.AlignCenter)
        self.setItem(row, 13, pre_delay_item)
        
        # 后置等待 (第14列) - 根据模式选择正确的字段
        if mode == "其他":
            post_delay = new_data.get("other_post_delay", 1.0)
        else:
            post_delay = new_data.get("gripper_post_delay", 1.0)
        post_delay_item = QTableWidgetItem(str(post_delay) + "s")
        post_delay_item.setTextAlignment(Qt.AlignCenter)
        self.setItem(row, 14, post_delay_item)
        
        # 备注 (第15列)
        note_item = QTableWidgetItem(new_data.get("note", ""))
        self.setItem(row, 15, note_item)
    
    def get_selected_rows(self) -> list:
        """获取所有选中行的索引列表。
        
        Returns:
            list: 选中行的索引列表（已排序）。
        """
        selected_indexes = self.selectionModel().selectedRows()
        return sorted([index.row() for index in selected_indexes])


class MotionPointDialog(QDialog):
    """运动点编辑对话框"""
    only_get_current_position_send_signal = pyqtSignal()
    
    def __init__(self, parent=None, data=None):
        super(MotionPointDialog, self).__init__(parent)
        self.setWindowTitle("编辑运动点")
        self.resize(400, 300)
        self.blend_points_data = []  # 存储混合运动点
        self._is_adding_blend_point = False  # 标记是否正在添加混合点
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
        
        self.gripper2_radio = QRadioButton("夹爪二")
        self.mode_group.addButton(self.gripper2_radio, 2)
        radio_layout.addWidget(self.gripper2_radio)
        
        self.other_radio = QRadioButton("其他")
        self.mode_group.addButton(self.other_radio, 3)
        radio_layout.addWidget(self.other_radio)
        
        self.detect_radio = QRadioButton("检测")
        self.mode_group.addButton(self.detect_radio, 4)
        radio_layout.addWidget(self.detect_radio)
        
        radio_layout.addStretch()
        layout.addLayout(radio_layout)
        
        # 其他指令设置容器（默认隐藏）
        self.other_params_widget = QWidget()
        other_layout = QFormLayout(self.other_params_widget)
        
        # 指令类型选择
        other_type_layout = QHBoxLayout()
        self.other_type_group = QButtonGroup()
        self.sweep_radio = QRadioButton("清扫指令")
        self.sweep_radio.setChecked(True)
        self.other_type_group.addButton(self.sweep_radio, 0)
        other_type_layout.addWidget(self.sweep_radio)
        
        self.press_radio = QRadioButton("压机指令")
        self.other_type_group.addButton(self.press_radio, 1)
        other_type_layout.addWidget(self.press_radio)
        other_type_layout.addStretch()
        other_layout.addRow("指令类型:", other_type_layout)
        
        # 指令值输入
        self.other_value_spin = QSpinBox()
        self.other_value_spin.setRange(0, 255)
        self.other_value_spin.setValue(0)
        other_layout.addRow("指令值 (0-255):", self.other_value_spin)
        
        # 其他模式专用前置等待
        self.other_pre_delay_spin = QDoubleSpinBox()
        self.other_pre_delay_spin.setRange(0.0, 10.0)
        self.other_pre_delay_spin.setDecimals(2)
        self.other_pre_delay_spin.setSingleStep(0.1)
        self.other_pre_delay_spin.setValue(0.0)
        self.other_pre_delay_spin.setSuffix(" 秒")
        other_layout.addRow("前置等待:", self.other_pre_delay_spin)
        
        # 其他模式专用后置等待
        self.other_post_delay_spin = QDoubleSpinBox()
        self.other_post_delay_spin.setRange(0.0, 10.0)
        self.other_post_delay_spin.setDecimals(2)
        self.other_post_delay_spin.setSingleStep(0.1)
        self.other_post_delay_spin.setValue(1.0)
        self.other_post_delay_spin.setSuffix(" 秒")
        other_layout.addRow("后置等待:", self.other_post_delay_spin)
        
        # 其他模式专用备注
        self.other_note_input = QLineEdit()
        other_layout.addRow("备注:", self.other_note_input)
        
        self.other_params_widget.setVisible(False)
        layout.addWidget(self.other_params_widget)
        
        # 监听模式切换，显示/隐藏其他指令参数
        self.mode_group.buttonClicked.connect(self._on_mode_changed)
        
        # ========== 运动参数容器（运动模式专用） ==========
        self.motion_params_widget = QWidget()
        motion_layout = QFormLayout(self.motion_params_widget)
        
        # 角度输入
        self.joint_spins = []
        for i in range(1, 7):
            spin = QDoubleSpinBox()
            spin.setRange(-180, 180)
            spin.setDecimals(12)
            spin.setSingleStep(1.0)
            motion_layout.addRow(f"关节{i}角度:", spin)
            self.joint_spins.append(spin)
        
        # 频率输入
        self.frequency_spin = QDoubleSpinBox()
        self.frequency_spin.setRange(0.01, 0.1)
        self.frequency_spin.setDecimals(2)
        self.frequency_spin.setSingleStep(0.01)
        self.frequency_spin.setValue(0.01)
        motion_layout.addRow("频率:", self.frequency_spin)
        
        # 曲线类型选择
        self.curve_type_combo = QComboBox()
        self.curve_type_combo.addItems(["S曲线", "直线", "向量", "曲线", "混合"])
        self.curve_type_combo.currentTextChanged.connect(self._on_curve_type_changed)
        motion_layout.addRow("曲线类型:", self.curve_type_combo)
        
        # 向量运动参数容器（默认隐藏）
        self.vector_params_widget = QWidget()
        vector_layout = QFormLayout(self.vector_params_widget)
        
        # 距离输入
        self.distance_spin = QDoubleSpinBox()
        self.distance_spin.setRange(-10.0, 10.0)
        self.distance_spin.setDecimals(4)
        self.distance_spin.setSingleStep(0.01)
        self.distance_spin.setValue(0.1)
        vector_layout.addRow("移动距离(m):", self.distance_spin)
        
        # 方向向量 X
        self.direction_x_spin = QDoubleSpinBox()
        self.direction_x_spin.setRange(-1.0, 1.0)
        self.direction_x_spin.setDecimals(4)
        self.direction_x_spin.setSingleStep(0.1)
        self.direction_x_spin.setValue(0.0)
        vector_layout.addRow("方向X:", self.direction_x_spin)
        
        # 方向向量 Y
        self.direction_y_spin = QDoubleSpinBox()
        self.direction_y_spin.setRange(-1.0, 1.0)
        self.direction_y_spin.setDecimals(4)
        self.direction_y_spin.setSingleStep(0.1)
        self.direction_y_spin.setValue(0.0)
        vector_layout.addRow("方向Y:", self.direction_y_spin)
        
        # 方向向量 Z
        self.direction_z_spin = QDoubleSpinBox()
        self.direction_z_spin.setRange(-1.0, 1.0)
        self.direction_z_spin.setDecimals(4)
        self.direction_z_spin.setSingleStep(0.1)
        self.direction_z_spin.setValue(1.0)
        vector_layout.addRow("方向Z:", self.direction_z_spin)
        
        self.vector_params_widget.setVisible(False)
        motion_layout.addRow(self.vector_params_widget)
        
        # 曲线运动参数容器（默认隐藏）
        self.curve_params_widget = QWidget()
        curve_layout = QFormLayout(self.curve_params_widget)
        
        # 中间点1 - X
        self.mid_point1_x_spin = QDoubleSpinBox()
        self.mid_point1_x_spin.setRange(-2.0, 2.0)
        self.mid_point1_x_spin.setDecimals(4)
        self.mid_point1_x_spin.setSingleStep(0.01)
        self.mid_point1_x_spin.setValue(0.0)
        curve_layout.addRow("中间点1-X(m):", self.mid_point1_x_spin)
        
        # 中间点1 - Y
        self.mid_point1_y_spin = QDoubleSpinBox()
        self.mid_point1_y_spin.setRange(-2.0, 2.0)
        self.mid_point1_y_spin.setDecimals(4)
        self.mid_point1_y_spin.setSingleStep(0.01)
        self.mid_point1_y_spin.setValue(0.0)
        curve_layout.addRow("中间点1-Y(m):", self.mid_point1_y_spin)
        
        # 中间点1 - Z
        self.mid_point1_z_spin = QDoubleSpinBox()
        self.mid_point1_z_spin.setRange(-2.0, 2.0)
        self.mid_point1_z_spin.setDecimals(4)
        self.mid_point1_z_spin.setSingleStep(0.01)
        self.mid_point1_z_spin.setValue(0.0)
        curve_layout.addRow("中间点1-Z(m):", self.mid_point1_z_spin)
        
        # 中间点2 - X
        self.mid_point2_x_spin = QDoubleSpinBox()
        self.mid_point2_x_spin.setRange(-2.0, 2.0)
        self.mid_point2_x_spin.setDecimals(4)
        self.mid_point2_x_spin.setSingleStep(0.01)
        self.mid_point2_x_spin.setValue(0.0)
        curve_layout.addRow("中间点2-X(m):", self.mid_point2_x_spin)
        
        # 中间点2 - Y
        self.mid_point2_y_spin = QDoubleSpinBox()
        self.mid_point2_y_spin.setRange(-2.0, 2.0)
        self.mid_point2_y_spin.setDecimals(4)
        self.mid_point2_y_spin.setSingleStep(0.01)
        self.mid_point2_y_spin.setValue(0.0)
        curve_layout.addRow("中间点2-Y(m):", self.mid_point2_y_spin)
        
        # 中间点2 - Z
        self.mid_point2_z_spin = QDoubleSpinBox()
        self.mid_point2_z_spin.setRange(-2.0, 2.0)
        self.mid_point2_z_spin.setDecimals(4)
        self.mid_point2_z_spin.setSingleStep(0.01)
        self.mid_point2_z_spin.setValue(0.0)
        curve_layout.addRow("中间点2-Z(m):", self.mid_point2_z_spin)
        
        self.curve_params_widget.setVisible(False)
        motion_layout.addRow(self.curve_params_widget)
        
        # 混合运动参数容器（默认隐藏）
        self.blend_params_widget = QWidget()
        blend_layout = QVBoxLayout(self.blend_params_widget)

        # 按钮区
        blend_btn_layout = QHBoxLayout()
        self.add_blend_point_btn = QPushButton("添加当前位置")
        self.add_blend_point_btn.clicked.connect(self.add_blend_point)
        blend_btn_layout.addWidget(self.add_blend_point_btn)

        self.clear_blend_points_btn = QPushButton("清空")
        self.clear_blend_points_btn.clicked.connect(self.clear_blend_points)
        blend_btn_layout.addWidget(self.clear_blend_points_btn)
        
        blend_layout.addLayout(blend_btn_layout)

        # 列表区
        self.blend_point_list = QListWidget()
        self.blend_point_list.setMaximumHeight(150)
        blend_layout.addWidget(self.blend_point_list)

        self.blend_params_widget.setVisible(False)
        motion_layout.addRow(self.blend_params_widget)
        
        # 运动模式备注
        self.note_input = QLineEdit()
        motion_layout.addRow("备注:", self.note_input)
        
        layout.addWidget(self.motion_params_widget)
        
        # ========== 夹爪参数容器（夹爪/夹爪二模式专用） ==========
        self.gripper_params_widget = QWidget()
        gripper_layout = QFormLayout(self.gripper_params_widget)
        
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
        gripper_layout.addRow("夹爪命令:", self.gripper_command_combo)
        
        # 夹爪参数输入
        self.gripper_param_spin = QDoubleSpinBox()
        self.gripper_param_spin.setRange(-1000.0, 1000.0)
        self.gripper_param_spin.setDecimals(2)
        self.gripper_param_spin.setSingleStep(1.0)
        self.gripper_param_spin.setValue(0.0)
        gripper_layout.addRow("夹爪参数:", self.gripper_param_spin)
        
        # 夹爪前置等待时间
        self.gripper_pre_delay_spin = QDoubleSpinBox()
        self.gripper_pre_delay_spin.setRange(0.0, 10.0)
        self.gripper_pre_delay_spin.setDecimals(2)
        self.gripper_pre_delay_spin.setSingleStep(0.1)
        self.gripper_pre_delay_spin.setValue(0.0)
        self.gripper_pre_delay_spin.setSuffix(" 秒")
        gripper_layout.addRow("前置等待:", self.gripper_pre_delay_spin)
        
        # 夹爪后置等待时间
        self.gripper_post_delay_spin = QDoubleSpinBox()
        self.gripper_post_delay_spin.setRange(0.0, 10.0)
        self.gripper_post_delay_spin.setDecimals(2)
        self.gripper_post_delay_spin.setSingleStep(0.1)
        self.gripper_post_delay_spin.setValue(1.0)
        self.gripper_post_delay_spin.setSuffix(" 秒")
        gripper_layout.addRow("后置等待:", self.gripper_post_delay_spin)
        
        # 夹爪模式备注
        self.gripper_note_input = QLineEdit()
        gripper_layout.addRow("备注:", self.gripper_note_input)
        
        self.gripper_params_widget.setVisible(False)
        layout.addWidget(self.gripper_params_widget)
        
        # ========== 检测模式容器（检测模式专用） ==========
        self.detect_params_widget = QWidget()
        detect_layout = QFormLayout(self.detect_params_widget)
        self.detect_note_input = QLineEdit()
        detect_layout.addRow("备注:", self.detect_note_input)
        self.detect_params_widget.setVisible(False)
        layout.addWidget(self.detect_params_widget)
        
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
    
    def _on_mode_changed(self, button):
        """模式切换时，显示/隐藏相应的参数区域"""
        is_motion = (button == self.motion_radio)
        is_gripper = (button == self.gripper_radio) or (button == self.gripper2_radio)
        is_other = (button == self.other_radio)
        is_detect = (button == self.detect_radio)
        
        # 运动模式：显示 motion_params_widget
        self.motion_params_widget.setVisible(is_motion)
        # 夹爪/夹爪二模式：显示 gripper_params_widget
        self.gripper_params_widget.setVisible(is_gripper)
        # 其他模式：显示 other_params_widget
        self.other_params_widget.setVisible(is_other)
        # 检测模式：显示 detect_params_widget
        self.detect_params_widget.setVisible(is_detect)
    
    def _on_curve_type_changed(self, curve_type: str):
        """曲线类型变化时，显示/隐藏向量参数和曲线参数"""
        is_vector = (curve_type == "向量")
        is_curve = (curve_type == "曲线")
        is_blend = (curve_type == "混合")
        self.vector_params_widget.setVisible(is_vector)
        self.curve_params_widget.setVisible(is_curve)
        self.blend_params_widget.setVisible(is_blend)
    
    def add_blend_point(self):
        """添加混合运动点请求"""
        self._is_adding_blend_point = True
        self.only_get_current_position_send_signal.emit()

    def clear_blend_points(self):
        """清空混合运动点"""
        self.blend_points_data = []
        self.blend_point_list.clear()

    def fill_data(self, data):
        """使用提供的数据填充界面"""
        # 填充模式选择并控制容器可见性
        mode = data.get("mode", "运动")
        
        # 先隐藏所有参数容器
        self.motion_params_widget.setVisible(False)
        self.gripper_params_widget.setVisible(False)
        self.other_params_widget.setVisible(False)
        self.detect_params_widget.setVisible(False)
        
        if mode == "夹爪":
            self.gripper_radio.setChecked(True)
            self.gripper_params_widget.setVisible(True)
            # 填充夹爪参数
            if "gripper_command" in data:
                index = self.gripper_command_combo.findText(data["gripper_command"])
                if index >= 0:
                    self.gripper_command_combo.setCurrentIndex(index)
            if "gripper_param" in data:
                self.gripper_param_spin.setValue(data["gripper_param"])
            if "gripper_pre_delay" in data:
                self.gripper_pre_delay_spin.setValue(data["gripper_pre_delay"])
            if "gripper_post_delay" in data:
                self.gripper_post_delay_spin.setValue(data["gripper_post_delay"])
            self.gripper_note_input.setText(data.get("note", ""))
            
        elif mode == "夹爪二":
            self.gripper2_radio.setChecked(True)
            self.gripper_params_widget.setVisible(True)
            # 填充夹爪参数
            if "gripper_command" in data:
                index = self.gripper_command_combo.findText(data["gripper_command"])
                if index >= 0:
                    self.gripper_command_combo.setCurrentIndex(index)
            if "gripper_param" in data:
                self.gripper_param_spin.setValue(data["gripper_param"])
            if "gripper_pre_delay" in data:
                self.gripper_pre_delay_spin.setValue(data["gripper_pre_delay"])
            if "gripper_post_delay" in data:
                self.gripper_post_delay_spin.setValue(data["gripper_post_delay"])
            self.gripper_note_input.setText(data.get("note", ""))
            
        elif mode == "其他":
            self.other_radio.setChecked(True)
            self.other_params_widget.setVisible(True)
            # 填充其他指令参数
            other_command_type = data.get("other_command_type", "清扫")
            if other_command_type == "压机":
                self.press_radio.setChecked(True)
            else:
                self.sweep_radio.setChecked(True)
            self.other_value_spin.setValue(data.get("other_command_value", 0))
            self.other_pre_delay_spin.setValue(data.get("other_pre_delay", 0.0))
            self.other_post_delay_spin.setValue(data.get("other_post_delay", 1.0))
            self.other_note_input.setText(data.get("note", ""))
            
        elif mode == "检测":
            self.detect_radio.setChecked(True)
            self.detect_params_widget.setVisible(True)
            self.detect_note_input.setText(data.get("note", ""))
            
        else:
            # 运动模式（默认）
            self.motion_radio.setChecked(True)
            self.motion_params_widget.setVisible(True)
            
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
            
            # 填充向量参数
            if data.get("curve_type") == "向量":
                if "distance" in data:
                    self.distance_spin.setValue(data["distance"])
                if "direction_x" in data:
                    self.direction_x_spin.setValue(data["direction_x"])
                if "direction_y" in data:
                    self.direction_y_spin.setValue(data["direction_y"])
                if "direction_z" in data:
                    self.direction_z_spin.setValue(data["direction_z"])
            
            # 填充曲线参数
            if data.get("curve_type") == "曲线":
                if "mid_point1_x" in data:
                    self.mid_point1_x_spin.setValue(data["mid_point1_x"])
                if "mid_point1_y" in data:
                    self.mid_point1_y_spin.setValue(data["mid_point1_y"])
                if "mid_point1_z" in data:
                    self.mid_point1_z_spin.setValue(data["mid_point1_z"])
                if "mid_point2_x" in data:
                    self.mid_point2_x_spin.setValue(data["mid_point2_x"])
                if "mid_point2_y" in data:
                    self.mid_point2_y_spin.setValue(data["mid_point2_y"])
                if "mid_point2_z" in data:
                    self.mid_point2_z_spin.setValue(data["mid_point2_z"])
            
            # 填充混合参数
            if data.get("curve_type") == "混合" and "blend_points" in data:
                self.blend_points_data = data["blend_points"]
                self.blend_point_list.clear()
                for i, pos in enumerate(self.blend_points_data):
                    point_str = f"点 {i+1}: {[round(p, 2) for p in pos]}"
                    self.blend_point_list.addItem(point_str)
            
            # 填充备注
            self.note_input.setText(data.get("note", ""))
    
    def get_motion_data(self):
        """获取运动点数据"""
        data = {}
        
        # 获取模式选择
        if self.gripper_radio.isChecked() or self.gripper2_radio.isChecked():
            # 夹爪/夹爪二模式
            data["mode"] = "夹爪" if self.gripper_radio.isChecked() else "夹爪二"
            # 从夹爪参数容器获取数据
            data["gripper_command"] = self.gripper_command_combo.currentText()
            data["gripper_param"] = self.gripper_param_spin.value()
            data["gripper_pre_delay"] = self.gripper_pre_delay_spin.value()
            data["gripper_post_delay"] = self.gripper_post_delay_spin.value()
            data["note"] = self.gripper_note_input.text()
            # 其他字段使用默认值
            for key in ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]:
                data[key] = 0.0
            data["frequency"] = 0.01
            data["curve_type"] = "S曲线"
            return data
            
        elif self.other_radio.isChecked():
            # 其他模式（清扫/压机）
            data["mode"] = "其他"
            data["other_command_type"] = "清扫" if self.sweep_radio.isChecked() else "压机"
            data["other_command_value"] = self.other_value_spin.value()
            data["other_pre_delay"] = self.other_pre_delay_spin.value()
            data["other_post_delay"] = self.other_post_delay_spin.value()
            data["note"] = self.other_note_input.text()
            # 其他字段使用默认值
            for key in ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]:
                data[key] = 0.0
            data["frequency"] = 0.01
            data["curve_type"] = "S曲线"
            data["gripper_command"] = "00: 不进行任何操作"
            data["gripper_param"] = 0.0
            return data
            
        elif self.detect_radio.isChecked():
            # 检测模式
            data["mode"] = "检测"
            data["note"] = self.detect_note_input.text()
            # 其他字段使用默认值
            for key in ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]:
                data[key] = 0.0
            data["frequency"] = 0.01
            data["curve_type"] = "S曲线"
            data["gripper_command"] = "00: 不进行任何操作"
            data["gripper_param"] = 0.0
            data["gripper_pre_delay"] = 0.0
            data["gripper_post_delay"] = 1.0
            return data
        
        # 运动模式（默认）
        data["mode"] = "运动"
        
        # 获取关节角度
        for i, key in enumerate(["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]):
            data[key] = self.joint_spins[i].value()
        
        # 获取频率
        data["frequency"] = self.frequency_spin.value()
        
        # 获取曲线类型
        data["curve_type"] = self.curve_type_combo.currentText()
        
        # 如果是向量类型，添加向量参数
        if data["curve_type"] == "向量":
            data["distance"] = self.distance_spin.value()
            data["direction_x"] = self.direction_x_spin.value()
            data["direction_y"] = self.direction_y_spin.value()
            data["direction_z"] = self.direction_z_spin.value()
        
        # 如果是曲线类型，添加曲线参数
        if data["curve_type"] == "曲线":
            data["mid_point1_x"] = self.mid_point1_x_spin.value()
            data["mid_point1_y"] = self.mid_point1_y_spin.value()
            data["mid_point1_z"] = self.mid_point1_z_spin.value()
            data["mid_point2_x"] = self.mid_point2_x_spin.value()
            data["mid_point2_y"] = self.mid_point2_y_spin.value()
            data["mid_point2_z"] = self.mid_point2_z_spin.value()
        
        # 如果是混合类型，添加点列表
        if data["curve_type"] == "混合":
            data["blend_points"] = self.blend_points_data
            # 如果备注为空，自动生成
            if not self.note_input.text():
                 data["note"] = f"混合路径: 共{len(self.blend_points_data)}个路点"

        # 运动模式的默认夹爪参数
        data["gripper_command"] = "00: 不进行任何操作"
        data["gripper_param"] = 0.0
        data["gripper_pre_delay"] = 0.0
        data["gripper_post_delay"] = 1.0
        
        # 获取备注
        data["note"] = self.note_input.text()
        
        return data
    
    def get_current_position(self):
        """获取当前机械臂位置"""
        self._is_adding_blend_point = False # 确保不是混合添加模式
        self.only_get_current_position_send_signal.emit()
        
    def display_current_position(self, positions):
        """显示当前机械臂位置"""
        if self._is_adding_blend_point:
            # 添加到混合点列表
            self.blend_points_data.append(positions)
            point_str = f"点 {len(self.blend_points_data)}: {[round(p, 2) for p in positions]}"
            self.blend_point_list.addItem(point_str)
            self._is_adding_blend_point = False # 重置标志
            
            # 同时更新主界面的坐标，方便用户看到最后一个点作为参考
            for i in range(6):
                self.joint_spins[i].setValue(positions[i])
        else:
            for i in range(6):
                self.joint_spins[i].setValue(positions[i])

    def convert_to_radians(self):
        """将角度值转换为弧度值"""
        for i, spin in enumerate(self.joint_spins):
            angle_value = spin.value()
            radian_value = math.radians(angle_value)
            spin.setValue(round(radian_value, 4))


