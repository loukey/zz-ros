"""
主窗口视图 - 新架构实现
基于MVVM模式，与原有界面保持一致的外观和功能
"""
from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QTabWidget, QMessageBox, 
                           QHBoxLayout, QStatusBar, QProgressBar, QToolBar, QApplication,
                           QMenu, QAction, QPushButton, QLabel, QSplitter)
from PyQt5.QtCore import Qt
from ..components import *
from ..view_models.main_view_model import MainViewModel


class MainWindow(QMainWindow):
    """主窗口类"""
    
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        
        # 初始化视图模型
        self.view_model = MainViewModel()
        
        # 设置窗口标题和大小（与原版保持一致）
        self.setWindowTitle("镇中科技机械臂控制工具v0.5")
        self.resize(1400, 1000)
        
        # 初始化界面
        self.init_ui()
        self.init_signals()
    
        # 添加窗口关闭事件处理
        app = QApplication.instance()
        if app:
            app.aboutToQuit.connect(self.cleanup)
    
    def init_ui(self):
        """初始化用户界面"""
        # 创建中央部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 创建主要的水平分割布局
        main_layout = QHBoxLayout(central_widget)
        
        # 创建水平分割器
        splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(splitter)
        
        # 左侧区域（40%宽度）
        self._create_left_panel(splitter)
        
        # 右侧区域（60%宽度）
        self._create_right_panel(splitter)
        
        # 设置分割器的初始比例
        splitter.setStretchFactor(0, 2)  # 左侧40%
        splitter.setStretchFactor(1, 3)  # 右侧60%
        
        # 创建状态栏
        self._create_status_bar()
        
        # 创建菜单栏
        self._create_menu_bar()
    
    def _create_left_panel(self, parent):
        """创建左侧面板"""
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        
        # 创建左侧标签页
        self.left_tab_widget = QTabWidget()
        left_layout.addWidget(self.left_tab_widget)
        
        # 主页标签
        self._create_main_tab()
        
        # 运动规划标签
        self._create_motion_planning_tab()
        
        # 动力学标签
        self._create_dynamics_tab()
        
        # 摄像头标签
        self._create_camera_tab()
        
        # 参数标签
        self._create_parameter_tab()
        
        parent.addWidget(left_widget)
    
    def _create_right_panel(self, parent):
        """创建右侧面板"""
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        
        # 创建右侧标签页
        self.right_tab_widget = QTabWidget()
        right_layout.addWidget(self.right_tab_widget)
        
        # 数据显示标签
        self._create_data_display_tab()
        
        # 逆运动学标签
        self._create_inverse_kinematic_tab()
        
        parent.addWidget(right_widget)
    
    def _create_main_tab(self):
        """创建主页标签"""
        main_tab = QWidget()
        layout = QVBoxLayout(main_tab)
        
        # 串口选择区域
        self.port_frame = PortSelectionFrame(
            parent=main_tab,
            view_model=self.view_model.serial_vm
        )
        layout.addWidget(self.port_frame)
        
        # 控制按钮区域
        self.control_frame = ControlButtonsFrame(
            parent=main_tab,
            view_model=self.view_model.control_vm
        )
        layout.addWidget(self.control_frame)
        
        # 角度控制区域
        self.angle_control_frame = AngleControlFrame(
            parent=main_tab,
            view_model=self.view_model.motion_vm
        )
        layout.addWidget(self.angle_control_frame)
        
        # 末端位置显示区域
        self.end_position_frame = EndPositionFrame(
            parent=main_tab,
            view_model=self.view_model.kinematic_vm
        )
        layout.addWidget(self.end_position_frame)
        
        # 末端执行器区域
        self.effector_frame = EffectorFrame(
            parent=main_tab,
            view_model=self.view_model.effector_vm,
            get_encoding_type=self.control_frame.get_encoding_type
        )
        layout.addWidget(self.effector_frame)
        
        # 添加伸缩空间
        layout.addStretch()
        
        self.left_tab_widget.addTab(main_tab, "主页")
    
    def _create_motion_planning_tab(self):
        """创建运动规划标签"""
        motion_tab = QWidget()
        layout = QVBoxLayout(motion_tab)
        
        # 运动规划框架
        self.motion_planning_frame = MotionPlanningFrame(
            parent=motion_tab,
            view_model=self.view_model.trajectory_vm
        )
        layout.addWidget(self.motion_planning_frame)
        
        self.left_tab_widget.addTab(motion_tab, "运动规划")
    
    def _create_dynamics_tab(self):
        """创建动力学标签"""
        dynamics_tab = QWidget()
        layout = QVBoxLayout(dynamics_tab)
        
        # 动力学控制框架
        self.dynamics_frame = DynamicsFrame(
            parent=dynamics_tab,
            view_model=self.view_model.dynamics_vm
        )
        layout.addWidget(self.dynamics_frame)
        
        # 添加伸缩空间
        layout.addStretch()
        
        self.left_tab_widget.addTab(dynamics_tab, "动力学")
    
    def _create_camera_tab(self):
        """创建摄像头标签"""
        camera_tab = QWidget()
        layout = QVBoxLayout(camera_tab)
        
        # 摄像头控制框架
        self.camera_frame = CameraFrame(
            parent=camera_tab,
            view_model=self.view_model.camera_vm
        )
        layout.addWidget(self.camera_frame)
        
        # 添加伸缩空间
        layout.addStretch()
        
        self.left_tab_widget.addTab(camera_tab, "摄像头")
    
    def _create_parameter_tab(self):
        """创建参数标签"""
        parameter_tab = QWidget()
        layout = QVBoxLayout(parameter_tab)
        
        # 参数设置框架
        self.parameter_frame = ParameterFrame(
            parent=parameter_tab,
            view_model=self.view_model.parameter_vm
        )
        layout.addWidget(self.parameter_frame)
        
        # 添加伸缩空间
        layout.addStretch()
        
        self.left_tab_widget.addTab(parameter_tab, "参数")
    
    def _create_data_display_tab(self):
        """创建数据显示标签"""
        display_tab = QWidget()
        layout = QVBoxLayout(display_tab)
        
        # 数据显示区域
        self.data_display_frame = DataDisplayFrame(
            parent=display_tab,
            view_model=self.view_model.display_vm
        )
        layout.addWidget(self.data_display_frame)
        
        self.right_tab_widget.addTab(display_tab, "数据显示")
    
    def _create_inverse_kinematic_tab(self):
        """创建逆运动学标签"""
        kinematic_tab = QWidget()
        layout = QVBoxLayout(kinematic_tab)
        
        # 逆运动学计算区域
        self.inverse_kinematic_frame = InverseKinematicFrame(
            parent=kinematic_tab,
            view_model=self.view_model.kinematic_vm
        )
        layout.addWidget(self.inverse_kinematic_frame)
        
        # 添加伸缩空间
        layout.addStretch()
        
        self.right_tab_widget.addTab(kinematic_tab, "逆运动学")
    
    def _create_status_bar(self):
        """创建状态栏"""
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        
        # 连接状态标签
        self.connection_status_label = QLabel("未连接")
        self.status_bar.addPermanentWidget(self.connection_status_label)
        
        # 进度条
        self.progress_bar = QProgressBar()
        self.progress_bar.setVisible(False)
        self.status_bar.addPermanentWidget(self.progress_bar)
        
        # 默认状态消息
        self.status_bar.showMessage("就绪")
    
    def _create_menu_bar(self):
        """创建菜单栏"""
        menubar = self.menuBar()
        
        # 设置菜单
        settings_menu = menubar.addMenu('设置')
        
        # 串口参数配置
        serial_config_action = QAction('串口参数配置', self)
        serial_config_action.triggered.connect(self._show_serial_config)
        settings_menu.addAction(serial_config_action)
        
        # 轮廓模式参数配置
        contour_config_action = QAction('轮廓模式参数配置', self)
        contour_config_action.triggered.connect(self._show_contour_config)
        settings_menu.addAction(contour_config_action)
    
    def _show_serial_config(self):
        """显示串口配置对话框"""
        # TODO: 实现串口配置对话框
        QMessageBox.information(self, "提示", "串口配置功能待实现")
    
    def _show_contour_config(self):
        """显示轮廓配置对话框"""
        # TODO: 实现轮廓配置对话框
        QMessageBox.information(self, "提示", "轮廓配置功能待实现")
    
    def init_signals(self):
        """初始化信号连接"""
        # 连接主视图模型的信号
        self.view_model.status_message_changed.connect(self.status_bar.showMessage)
        self.view_model.connection_status_changed.connect(self._update_connection_status)
        self.view_model.progress_changed.connect(self._update_progress)
    
    def _update_connection_status(self, connected):
        """更新连接状态"""
        if connected:
            self.connection_status_label.setText("已连接")
            self.connection_status_label.setStyleSheet("color: green")
        else:
            self.connection_status_label.setText("未连接")
            self.connection_status_label.setStyleSheet("color: red")
    
    def _update_progress(self, value, visible):
        """更新进度条"""
        self.progress_bar.setValue(value)
        self.progress_bar.setVisible(visible)
    
    def closeEvent(self, event):
        """窗口关闭事件"""
        reply = QMessageBox.question(self, '确认退出', 
                                   '确定要退出程序吗？',
                                   QMessageBox.Yes | QMessageBox.No, 
                                   QMessageBox.No)
        
        if reply == QMessageBox.Yes:
            self.cleanup()
            event.accept()
        else:
            event.ignore()
    
    def cleanup(self):
        """清理资源"""
        if hasattr(self, 'view_model') and self.view_model:
            self.view_model.cleanup() 