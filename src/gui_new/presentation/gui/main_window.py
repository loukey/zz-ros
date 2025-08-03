"""
主窗口视图 - 新架构实现
基于MVVM模式，与原有界面保持一致的外观和功能
"""
from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QTabWidget, QMessageBox, 
                           QHBoxLayout, QStatusBar, QProgressBar, QToolBar, QApplication,
                           QMenu, QAction, QPushButton, QLabel, QSplitter)
from PyQt5.QtCore import Qt
from presentation.components import *


class MainWindow(QMainWindow):
    """主窗口类"""
    
    def __init__(self, view_model, parent=None):
        super(MainWindow, self).__init__(parent)
        
        # 通过依赖注入接收视图模型（必需参数）
        self.view_model = view_model
        
        # 设置窗口标题和大小（与原版保持一致）
        self.setWindowTitle("镇中科技机械臂控制工具v0.6")
        self.resize(1800, 1000)
        
        # 初始化设置对话框
        self._init_serial_config()
        self._init_contour_settings()
        
        # 初始化界面
        self.init_ui()
    
        # 添加窗口关闭事件处理
        app = QApplication.instance()
        if app:
            app.aboutToQuit.connect(self.cleanup)
    
    def init_ui(self):
        """初始化用户界面"""
        # 创建中央部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 创建主要的水平分割布局，设置为贴边
        main_layout = QHBoxLayout(central_widget)
        main_layout.setContentsMargins(0, 0, 0, 0)  # 移除外边距
        main_layout.setSpacing(0)  # 移除间距
        
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
        
        
        # 创建菜单栏
        self._create_menu_bar()
    
    def _create_left_panel(self, parent):
        """创建左侧面板"""
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(0, 0, 0, 0)  # 移除边距
        left_layout.setSpacing(0)  # 移除间距
        
        # 创建左侧标签页
        self.left_tab_widget = QTabWidget()
        self.left_tab_widget.setContentsMargins(0, 0, 0, 0)  # TabWidget本身无边距
        left_layout.addWidget(self.left_tab_widget)
        
        # 主页标签
        self._create_main_tab()
        
        # 运动规划标签
        self._create_motion_planning_tab()
        
        # 动力学标签
        self._create_dynamics_tab()
        
        # 摄像头标签
        self._create_camera_tab()
        
        parent.addWidget(left_widget)
    
    def _create_right_panel(self, parent):
        """创建右侧面板"""
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(0, 0, 0, 0)  # 移除边距
        right_layout.setSpacing(0)  # 移除间距
        
        # 直接创建数据显示区域，无需TabWidget嵌套
        # 使用display_vm来统一处理所有消息显示
        self.data_display_frame = DataDisplayFrame(
            parent=right_widget,
            view_model=self.view_model.display_vm  # 使用DisplayViewModel统一管理消息
        )
        right_layout.addWidget(self.data_display_frame)
        
        parent.addWidget(right_widget)
    
    def _create_main_tab(self):
        """创建主页标签"""
        main_tab = QWidget()
        layout = QVBoxLayout(main_tab)
        layout.setContentsMargins(6, 6, 6, 6)  # 保留少量内边距
        layout.setSpacing(2)  # 减少组件间距
        
        # 串口选择区域
        self.port_frame = PortSelectionFrame(
            parent=main_tab,
            view_model=self.view_model.serial_vm,
            get_serial_config=self.serial_config.get_config
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
        
        # 末端执行器区域
        self.effector_frame = EffectorFrame(
            parent=main_tab,
            view_model=self.view_model.effector_vm
        )
        layout.addWidget(self.effector_frame)
        
        # 添加伸缩空间
        layout.addStretch()
        
        self.left_tab_widget.addTab(main_tab, "主页")
    
    def _create_motion_planning_tab(self):
        """创建运动规划标签"""
        motion_tab = QWidget()
        layout = QVBoxLayout(motion_tab)
        layout.setContentsMargins(6, 6, 6, 6)  # 保留少量内边距
        layout.setSpacing(2)  # 减少组件间距
        
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
        layout.setContentsMargins(6, 6, 6, 6)  # 保留少量内边距
        layout.setSpacing(2)  # 减少组件间距
        
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
        layout.setContentsMargins(6, 6, 6, 6)  # 保留少量内边距
        layout.setSpacing(2)  # 减少组件间距
        
        # 摄像头控制框架
        self.camera_frame = CameraFrame(
            parent=camera_tab,
            view_model=self.view_model.camera_vm
        )
        layout.addWidget(self.camera_frame)
        
        # 添加伸缩空间
        layout.addStretch()
        
        self.left_tab_widget.addTab(camera_tab, "摄像头")
    
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
    
    def _init_serial_config(self):
        """初始化串口配置对话框"""
        self.serial_config_dialog = QWidget()
        self.serial_config_dialog.setWindowTitle("串口参数配置")
        layout = QVBoxLayout()
        # 使用默认配置创建串口配置框架
        self.serial_config = SerialConfigFrame()
        layout.addWidget(self.serial_config)
        self.serial_config_dialog.setLayout(layout)
        self.serial_config_dialog.resize(400, 300)
        self.serial_config_dialog.hide()

    def _init_contour_settings(self):
        """初始化轮廓设置对话框"""
        self.contour_settings_dialog = QWidget()
        self.contour_settings_dialog.setWindowTitle("轮廓模式参数配置")
        layout = QVBoxLayout()
        self.contour_settings = ContourSettings(self.contour_settings_dialog)
        layout.addWidget(self.contour_settings)
        self.contour_settings_dialog.setLayout(layout)
        self.contour_settings_dialog.resize(600, 400)
        self.contour_settings_dialog.hide()
    
    def _show_serial_config(self):
        """显示串口配置对话框"""
        self.serial_config_dialog.show()
    
    def _show_contour_config(self):
        """显示轮廓配置对话框"""
        self.contour_settings_dialog.show()
    
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
