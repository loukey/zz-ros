"""
参数相关UI组件
"""
from PyQt5.QtWidgets import (QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout, 
                           QGroupBox, QFrame)
from PyQt5.QtCore import Qt, pyqtSignal
from .base_components import default_font
from gui.config.params import ParamConfig


class VelocityCalibrationFrame(QGroupBox):
    """速度标定框架"""
    
    # 定义信号
    start_velocity_collection_requested = pyqtSignal()  # 开始速度收集请求
    stop_velocity_collection_requested = pyqtSignal()   # 停止速度收集请求
    save_velocity_requested = pyqtSignal()              # 保存速度请求
    
    def __init__(self, parent=None):
        super().__init__("速度标定", parent)
        self.is_collecting = False
        self._init_ui()
    
    def _init_ui(self):
        """初始化UI"""
        layout = QHBoxLayout()
        
        # 左侧按钮区域
        button_layout = QVBoxLayout()
        
        # 开启速度收集按钮
        self.velocity_button = QPushButton("开启速度收集")
        self.velocity_button.setFont(default_font)
        self.velocity_button.clicked.connect(self._on_velocity_button_clicked)
        button_layout.addWidget(self.velocity_button)
        
        # 保存按钮
        self.save_button = QPushButton("保存")
        self.save_button.setFont(default_font)
        self.save_button.clicked.connect(self._on_save_button_clicked)
        button_layout.addWidget(self.save_button)
        
        button_widget = QWidget()
        button_widget.setLayout(button_layout)
        layout.addWidget(button_widget)
        
        # 添加分隔线
        separator = QFrame()
        separator.setFrameShape(QFrame.VLine)
        separator.setFrameShadow(QFrame.Sunken)
        layout.addWidget(separator)
        
        # 实时速度显示区域
        realtime_velocity_layout = QVBoxLayout()
        
        # 实时速度标签
        realtime_velocity_title = QLabel("测量速度:")
        realtime_velocity_title.setFont(default_font)
        realtime_velocity_layout.addWidget(realtime_velocity_title)
        
        # X方向速度
        self.velocity_x_label = QLabel("Vx: 0.00 px/s")
        self.velocity_x_label.setFont(default_font)
        realtime_velocity_layout.addWidget(self.velocity_x_label)
        
        # Y方向速度
        self.velocity_y_label = QLabel("Vy: 0.00 px/s")
        self.velocity_y_label.setFont(default_font)
        realtime_velocity_layout.addWidget(self.velocity_y_label)
        
        realtime_velocity_widget = QWidget()
        realtime_velocity_widget.setLayout(realtime_velocity_layout)
        layout.addWidget(realtime_velocity_widget)
        
        # 添加分隔线
        separator2 = QFrame()
        separator2.setFrameShape(QFrame.VLine)
        separator2.setFrameShadow(QFrame.Sunken)
        layout.addWidget(separator2)
        
        # 历史速度显示区域
        history_velocity_layout = QVBoxLayout()
        
        # 历史速度标签
        history_velocity_title = QLabel("历史速度:")
        history_velocity_title.setFont(default_font)
        history_velocity_layout.addWidget(history_velocity_title)
        
        # 历史X方向速度
        self.history_velocity_x_label = QLabel("Vx: 0.00 px/s")
        self.history_velocity_x_label.setFont(default_font)
        history_velocity_layout.addWidget(self.history_velocity_x_label)
        
        # 历史Y方向速度
        self.history_velocity_y_label = QLabel("Vy: 0.00 px/s")
        self.history_velocity_y_label.setFont(default_font)
        history_velocity_layout.addWidget(self.history_velocity_y_label)
        
        history_velocity_widget = QWidget()
        history_velocity_widget.setLayout(history_velocity_layout)
        layout.addWidget(history_velocity_widget)
        
        # 添加弹性空间
        layout.addStretch()
        
        self.setLayout(layout)
    
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


class ParameterFrame(QWidget):
    """参数框架主容器"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self._init_ui()
        self._load_history_velocity()
    
    def _init_ui(self):
        """初始化UI"""
        layout = QVBoxLayout()
        
        # 添加速度标定区域
        self.velocity_calibration_frame = VelocityCalibrationFrame(self)
        layout.addWidget(self.velocity_calibration_frame)
        
        # 添加弹性空间，将组件推到顶部
        layout.addStretch()
        
        self.setLayout(layout)
    
    def _load_history_velocity(self):
        """加载并显示历史速度"""
        try:
            # 从配置中获取历史速度
            vx, vy = ParamConfig.get_velocity()
            
            # 更新历史速度显示
            self.velocity_calibration_frame.update_history_velocity_display(vx, vy)
            
        except Exception as e:
            print(f"加载历史速度失败: {e}")
            # 如果加载失败，显示默认值
            self.velocity_calibration_frame.update_history_velocity_display(0.0, 0.0)
    
    def get_velocity_calibration_frame(self):
        """获取速度标定框架"""
        return self.velocity_calibration_frame 