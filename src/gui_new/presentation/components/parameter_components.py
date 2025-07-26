"""
参数相关UI组件
"""
from PyQt5.QtWidgets import (QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout, 
                           QGroupBox, QFrame)
from PyQt5.QtCore import Qt, pyqtSignal
from .base_component import (BaseComponent, default_font, GroupFrame, 
                           ButtonRow, ConfigRow)


class VelocityCalibrationFrame(BaseComponent):
    """速度标定框架"""
    
    # 定义信号
    start_velocity_collection_requested = pyqtSignal()  # 开始速度收集请求
    stop_velocity_collection_requested = pyqtSignal()   # 停止速度收集请求
    save_velocity_requested = pyqtSignal()              # 保存速度请求
    
    def __init__(self, parent=None, view_model=None):
        self.is_collecting = False
        super().__init__(parent, view_model)
    
    def setup_ui(self):
        """设置UI"""
        # 创建分组框
        group_box = GroupFrame("速度标定")
        main_layout = QVBoxLayout(self)
        main_layout.addWidget(group_box)
        
        # 创建水平布局
        main_row = ConfigRow(None)
        
        # 左侧按钮区域
        button_configs = [
            ("开启速度收集", self._on_velocity_button_clicked, True),
            ("保存", self._on_save_button_clicked, True)
        ]
        
        self.button_row = ButtonRow(button_configs)
        self.velocity_button = self.button_row.get_button(0)
        self.save_button = self.button_row.get_button(1)
        
        # 创建垂直布局包装按钮
        button_widget = QWidget()
        button_layout = QVBoxLayout(button_widget)
        button_layout.addWidget(self.button_row)
        button_layout.addStretch()
        
        main_row.add_widget(button_widget)
        
        # 添加分隔线
        separator = QFrame()
        separator.setFrameShape(QFrame.VLine)
        separator.setFrameShadow(QFrame.Sunken)
        main_row.add_widget(separator)
        
        # 实时速度显示区域
        realtime_velocity_widget = self._create_velocity_display_widget(
            "测量速度:",
            "realtime"
        )
        main_row.add_widget(realtime_velocity_widget)
        
        # 添加分隔线
        separator2 = QFrame()
        separator2.setFrameShape(QFrame.VLine)
        separator2.setFrameShadow(QFrame.Sunken)
        main_row.add_widget(separator2)
        
        # 历史速度显示区域
        history_velocity_widget = self._create_velocity_display_widget(
            "历史速度:",
            "history"
        )
        main_row.add_widget(history_velocity_widget)
        
        # 添加弹性空间
        main_row.add_stretch()
        
        group_box.add_widget(main_row)
    
    def _create_velocity_display_widget(self, title, prefix):
        """创建速度显示组件"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # 标题标签
        title_label = QLabel(title)
        title_label.setFont(default_font)
        layout.addWidget(title_label)
        
        # X方向速度标签
        x_label = QLabel("Vx: 0.00 px/s")
        x_label.setFont(default_font)
        layout.addWidget(x_label)
        
        # Y方向速度标签
        y_label = QLabel("Vy: 0.00 px/s")
        y_label.setFont(default_font)
        layout.addWidget(y_label)
        
        # 根据前缀保存标签引用
        if prefix == "realtime":
            self.velocity_x_label = x_label
            self.velocity_y_label = y_label
        elif prefix == "history":
            self.history_velocity_x_label = x_label
            self.history_velocity_y_label = y_label
        
        return widget
    
    def connect_signals(self):
        """连接视图模型信号"""
        if self.view_model:
            self.view_model.velocity_updated.connect(self.update_velocity_display)
            self.view_model.history_velocity_updated.connect(self.update_history_velocity_display)
    
    def _on_velocity_button_clicked(self):
        """速度收集按钮点击处理"""
        if not self.is_collecting:
            # 开始收集
            self.start_velocity_collection_requested.emit()
            self.velocity_button.setText("结束速度收集")
            self.is_collecting = True
            # 在测量过程中显示等待状态
            self.velocity_x_label.setText("Vx: 测量中...")
            self.velocity_y_label.setText("Vy: 测量中...")
        else:
            # 停止收集
            self.stop_velocity_collection_requested.emit()
            self.velocity_button.setText("开启速度收集")
            self.is_collecting = False
    
    def _on_save_button_clicked(self):
        """保存按钮点击处理"""
        self.save_velocity_requested.emit()
    
    def update_velocity_display(self, vx, vy):
        """更新测量速度显示"""
        if self.is_collecting:
            # 如果正在收集，这意味着测量完成了
            self.velocity_x_label.setText(f"Vx: {vx:.2f} px/s")
            self.velocity_y_label.setText(f"Vy: {vy:.2f} px/s")
        else:
            # 如果不在收集状态，显示最终结果
            self.velocity_x_label.setText(f"Vx: {vx:.2f} px/s")
            self.velocity_y_label.setText(f"Vy: {vy:.2f} px/s")
    
    def update_history_velocity_display(self, vx, vy):
        """更新历史速度显示"""
        self.history_velocity_x_label.setText(f"Vx: {vx:.2f} px/s")
        self.history_velocity_y_label.setText(f"Vy: {vy:.2f} px/s")
    
    def reset_velocity_display(self):
        """重置速度显示"""
        self.velocity_x_label.setText("Vx: 0.00 px/s")
        self.velocity_y_label.setText("Vy: 0.00 px/s")
        self.velocity_button.setText("开启速度收集")
        self.is_collecting = False
    
    def set_collecting_state(self, collecting):
        """设置收集状态"""
        self.is_collecting = collecting
        if collecting:
            self.velocity_button.setText("结束速度收集")
        else:
            self.velocity_button.setText("开启速度收集")


class ParameterFrame(BaseComponent):
    """参数框架主容器"""
    
    def __init__(self, parent=None, view_model=None):
        super().__init__(parent, view_model)
    
    def setup_ui(self):
        """设置UI"""
        layout = QVBoxLayout()
        self.setLayout(layout)
        
        # 添加速度标定区域
        self.velocity_calibration_frame = VelocityCalibrationFrame(self, self.view_model)
        layout.addWidget(self.velocity_calibration_frame)
        
        # 添加弹性空间，将组件推到顶部
        layout.addStretch()
        
        # 加载历史速度
        self._load_history_velocity()
    
    def connect_signals(self):
        """连接视图模型信号"""
        if self.view_model:
            # 连接速度标定相关信号
            self.velocity_calibration_frame.start_velocity_collection_requested.connect(
                self.view_model.start_velocity_collection
            )
            self.velocity_calibration_frame.stop_velocity_collection_requested.connect(
                self.view_model.stop_velocity_collection
            )
            self.velocity_calibration_frame.save_velocity_requested.connect(
                self.view_model.save_velocity_to_config
            )
    
    def _load_history_velocity(self):
        """加载并显示历史速度"""
        # 历史速度加载已经在ParameterViewModel初始化时通过_load_parameters_from_settings自动处理
        # 并通过history_velocity_updated信号发出，所以这里不需要额外的加载逻辑
        if not self.view_model:
            # 如果没有view_model，显示默认值
            self.velocity_calibration_frame.update_history_velocity_display(0.0, 0.0)
    
    def get_velocity_calibration_frame(self):
        """获取速度标定框架"""
        return self.velocity_calibration_frame 