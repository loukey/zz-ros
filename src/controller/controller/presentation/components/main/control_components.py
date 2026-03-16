"""
Control and angle components for Main tab
"""
from PyQt5.QtWidgets import (QWidget, QLabel, QLineEdit, QPushButton, QRadioButton,
                           QVBoxLayout, QHBoxLayout, QGridLayout, QButtonGroup, QComboBox,
                           QGroupBox, QFrame, QMessageBox)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QDoubleValidator
from ..base_component import (BaseComponent, default_font, LabeledComboBox, 
                           InputGrid, RadioButtonGroup, ConfigRow, HorizontalLine)
from math import pi


class ControlButtonsFrame(BaseComponent):
    """控制按钮框架"""
    send_command_requested = pyqtSignal(dict)
    
    def __init__(self, parent=None, view_model=None):
        self.buttons = []
        super().__init__(parent, view_model)
    
    def setup_ui(self):
        """设置UI"""
        # 创建分组框
        group_box = QGroupBox("参数配置和控制命令")
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.addWidget(group_box)

        layout = QVBoxLayout(group_box)
        layout.setContentsMargins(8, 6, 8, 6)
        layout.setSpacing(6)
        
        # 创建参数配置区域
        # 注释：编码格式固定为hex模式，无需用户选择
        
        # 运行模式选择 - 使用LabeledComboBox基础组件
        self.run_modes = [
            ('轮廓位置模式', 0x01),
            ('轮廓速度模式', 0x03),
            ('轮廓扭矩模式', 0x04),
            ('回零模式(暂不支持)', 0x06),
            ('位置插补模式(暂不支持)', 0x07),
            ('周期同步位置模式', 0x08),
            ('周期同步速度模式', 0x09),
            ('周期同步扭矩模式', 0x0A)
        ]
        
        mode_items = [f"{mode[0]} ({mode[1]:02X})" for mode in self.run_modes]
        self.run_mode_combo = LabeledComboBox(
            "运行模式:",
            items=mode_items
        )
        self.run_mode_combo.set_current_text('周期同步位置模式 (08)')
        
        mode_row = ConfigRow(None)
        mode_row.add_widget(self.run_mode_combo)
        mode_row.add_stretch()
        layout.addWidget(mode_row)
        
        # 添加分隔线
        layout.addWidget(HorizontalLine())
        
        # 创建控制按钮
        button_layout = QHBoxLayout()
        button_layout.setSpacing(6)
        commands = [
            ("使能", 0x01),
            ("取消使能", 0x02),
            ("释放刹车", 0x03),
            ("锁止刹车", 0x04),
            ("立刻停止", 0x05),
            ("暂停", 0x08)
        ]
        
        for text, command in commands:
            button = QPushButton(text)
            button.setFont(default_font)
            button.clicked.connect(lambda checked, cmd=command: self.send_command_requested.emit({
                'control': cmd, 
                'mode': self.get_run_mode()
            }))
            button.setEnabled(False)
            button_layout.addWidget(button)
            self.buttons.append(button)
        
        layout.addLayout(button_layout)
    
    def connect_signals(self):
        """连接视图模型信号"""
        self.view_model.connection_status_changed.connect(self.update_connection_status)
        self.send_command_requested.connect(self.view_model.send_command)
    
    def update_connection_status(self, connected):
        """更新连接状态"""
        for button in self.buttons:
            button.setEnabled(connected)
    
    def get_run_mode(self):
        """获取当前选择的运行模式"""
        selected = self.run_mode_combo.current_text()
        for mode_name, mode_code in self.run_modes:
            if mode_name in selected:
                return mode_code
        return 0x01  # 默认返回轮廓位置模式


class AngleControlFrame(BaseComponent):
    """角度控制框架"""
    send_angles_requested = pyqtSignal(dict)
    
    def __init__(self, parent=None, view_model=None,
                 get_contour=None,
                 get_run_mode=None):
        self.get_contour = get_contour
        self.get_run_mode = get_run_mode
        super().__init__(parent, view_model)
    
    def setup_ui(self):
        """设置UI"""
        # 创建分组框
        group_box = QGroupBox("角度控制 (弧度值)")
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.addWidget(group_box)

        layout = QVBoxLayout(group_box)
        layout.setContentsMargins(8, 6, 8, 6)
        layout.setSpacing(6)
        
        # 创建角度输入区域 - 使用InputGrid基础组件
        angle_labels = [f"角度{i+1}" for i in range(6)]
        self.angle_grid = InputGrid(
            labels=angle_labels,
            rows=2, cols=3,
            default_value="0.0",
            validator=QDoubleValidator()
        )
        layout.addWidget(self.angle_grid)
        
        # 注意：曲线类型和频率参数已经在 CommandHubService 中默认设置为 s_curve 和 0.01
        # 因此这里不再提供 UI 配置选项
        
        # 按钮区域
        button_layout = QHBoxLayout()
        button_layout.setSpacing(6)

        self.send_button = QPushButton("发送角度")
        self.send_button.setFont(default_font)
        self.send_button.clicked.connect(lambda: self.send_angles_requested.emit({
            'control': 0x06,
            'mode': self.get_run_mode() if self.get_run_mode else 0x08,
            'target_angles': self.get_angles(),
            'contour_params': self.get_contour() if self.get_contour else None
        }))
        self.send_button.setEnabled(False)
        button_layout.addWidget(self.send_button)
        
        convert_button = QPushButton("度数转弧度")
        convert_button.setFont(default_font)
        convert_button.clicked.connect(self.convert_angles)
        button_layout.addWidget(convert_button)
        
        zero_button = QPushButton("全部归零")
        zero_button.setFont(default_font)
        zero_button.clicked.connect(self.zero_angles)
        button_layout.addWidget(zero_button)
        
        layout.addLayout(button_layout)
    
    def connect_signals(self):
        """连接视图模型信号"""
        self.view_model.connection_status_changed.connect(self.update_connection_status)
        self.send_angles_requested.connect(self.view_model.send_command)
    
    def get_angles(self):
        """获取当前角度值"""
        return self.angle_grid.get_float_values()

    def set_angles(self, angles):
        """设置角度值"""
        angle_strings = [f"{angle:.6f}" for angle in angles[:6]]
        self.angle_grid.set_values(angle_strings)
        
    def update_connection_status(self, connected):
        """更新连接状态"""
        self.send_button.setEnabled(connected)
    
    def convert_angles(self):
        """度数转弧度"""
        angle_values = self.angle_grid.get_float_values()
        
        # 将角度转换为弧度 (角度 * pi / 180)
        radian_values = [angle * (pi / 180) for angle in angle_values]
        
        # 更新输入框显示弧度值
        radian_strings = [f"{radian:.4f}" for radian in radian_values]
        self.angle_grid.set_values(radian_strings)
    
    def zero_angles(self):
        """归零处理"""
        # 将所有角度值设置为0
        zero_angles = [0.0, -pi/2, 0.0, pi/2, 0.0, 0.0]
        zero_strings = [f"{angle:.4f}" for angle in zero_angles]
        self.angle_grid.set_values(zero_strings)


class PoseControlFrame(BaseComponent):
    """姿态控制框架 - 通过欧拉角+位置控制机械臂"""

    def __init__(self, parent=None, view_model=None,
                 get_contour=None, get_run_mode=None):
        self.get_contour = get_contour
        self.get_run_mode = get_run_mode
        super().__init__(parent, view_model)

    def setup_ui(self):
        """设置UI"""
        group_box = QGroupBox("姿态控制")
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.addWidget(group_box)

        layout = QVBoxLayout(group_box)
        layout.setContentsMargins(8, 6, 8, 6)
        layout.setSpacing(6)

        # 欧拉角输入 (xyz内旋，弧度)
        euler_labels = ["欧拉角X", "欧拉角Y", "欧拉角Z"]
        self.euler_grid = InputGrid(
            labels=euler_labels,
            rows=1, cols=3,
            default_value="0.0",
            validator=QDoubleValidator()
        )
        layout.addWidget(self.euler_grid)

        # 位置输入 (xyz，米)
        pos_labels = ["位置X", "位置Y", "位置Z"]
        self.pos_grid = InputGrid(
            labels=pos_labels,
            rows=1, cols=3,
            default_value="0.0",
            validator=QDoubleValidator()
        )
        layout.addWidget(self.pos_grid)

        # 按钮区域
        button_layout = QHBoxLayout()
        button_layout.setSpacing(6)

        self.get_pose_button = QPushButton("获取当前位姿")
        self.get_pose_button.setFont(default_font)
        self.get_pose_button.clicked.connect(self._on_get_pose)
        self.get_pose_button.setEnabled(False)
        button_layout.addWidget(self.get_pose_button)

        self.send_pose_button = QPushButton("发送姿态")
        self.send_pose_button.setFont(default_font)
        self.send_pose_button.clicked.connect(self._on_send_pose)
        self.send_pose_button.setEnabled(False)
        button_layout.addWidget(self.send_pose_button)

        layout.addLayout(button_layout)

    def connect_signals(self):
        """连接视图模型信号"""
        self.view_model.connection_status_changed.connect(self._update_connection_status)
        self.view_model.pose_error.connect(self._on_pose_error)

    def _update_connection_status(self, connected):
        """更新连接状态"""
        self.get_pose_button.setEnabled(connected)
        self.send_pose_button.setEnabled(connected)

    def _on_get_pose(self):
        """获取当前位姿并填入输入栏"""
        try:
            euler, pos = self.view_model.get_current_pose()
            euler_strings = [f"{v:.6f}" for v in euler]
            pos_strings = [f"{v:.6f}" for v in pos]
            self.euler_grid.set_values(euler_strings)
            self.pos_grid.set_values(pos_strings)
        except Exception as e:
            QMessageBox.warning(self, "获取位姿失败", str(e))

    def _on_send_pose(self):
        """发送姿态命令"""
        euler = self.euler_grid.get_float_values()
        pos = self.pos_grid.get_float_values()
        run_mode = self.get_run_mode() if self.get_run_mode else 0x08
        contour_params = self.get_contour() if self.get_contour else None
        self.view_model.send_pose_command(euler, pos, run_mode, contour_params)

    def _on_pose_error(self, message):
        """显示姿态控制错误"""
        QMessageBox.warning(self, "姿态控制错误", message)


class RelativePoseControlFrame(BaseComponent):
    """相对姿态控制框架 - 在末端执行器坐标系下通过增量欧拉角+位置控制机械臂"""

    def __init__(self, parent=None, view_model=None,
                 get_contour=None, get_run_mode=None):
        self.get_contour = get_contour
        self.get_run_mode = get_run_mode
        super().__init__(parent, view_model)

    def setup_ui(self):
        """设置UI"""
        group_box = QGroupBox("相对姿态控制（末端坐标系）")
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.addWidget(group_box)

        layout = QVBoxLayout(group_box)
        layout.setContentsMargins(8, 6, 8, 6)
        layout.setSpacing(6)

        # 欧拉角增量输入 (xyz内旋，弧度)
        euler_labels = ["旋转X", "旋转Y", "旋转Z"]
        self.euler_grid = InputGrid(
            labels=euler_labels,
            rows=1, cols=3,
            default_value="0.0",
            validator=QDoubleValidator()
        )
        layout.addWidget(self.euler_grid)

        # 位置增量输入 (xyz，米)
        pos_labels = ["平移X", "平移Y", "平移Z"]
        self.pos_grid = InputGrid(
            labels=pos_labels,
            rows=1, cols=3,
            default_value="0.0",
            validator=QDoubleValidator()
        )
        layout.addWidget(self.pos_grid)

        # 按钮区域
        button_layout = QHBoxLayout()
        button_layout.setSpacing(6)

        self.send_button = QPushButton("发送相对姿态")
        self.send_button.setFont(default_font)
        self.send_button.clicked.connect(self._on_send_relative_pose)
        self.send_button.setEnabled(False)
        button_layout.addWidget(self.send_button)

        layout.addLayout(button_layout)

    def connect_signals(self):
        """连接视图模型信号"""
        self.view_model.connection_status_changed.connect(self._update_connection_status)
        self.view_model.pose_error.connect(self._on_pose_error)

    def _update_connection_status(self, connected):
        """更新连接状态"""
        self.send_button.setEnabled(connected)

    def _on_send_relative_pose(self):
        """发送相对姿态命令"""
        euler = self.euler_grid.get_float_values()
        pos = self.pos_grid.get_float_values()
        run_mode = self.get_run_mode() if self.get_run_mode else 0x08
        contour_params = self.get_contour() if self.get_contour else None
        self.view_model.send_relative_pose_command(euler, pos, run_mode, contour_params)

    def _on_pose_error(self, message):
        """显示姿态控制错误"""
        QMessageBox.warning(self, "姿态控制错误", message)


