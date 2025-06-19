"""
主窗口视图
"""
from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QTabWidget, QMessageBox, 
                           QHBoxLayout, QStatusBar, QProgressBar, QToolBar, QApplication,
                           QMenu, QAction, QPushButton)
from gui.views.components import *
from gui.controllers import *
from gui.models import *


class MainWindow(QMainWindow):
    """主窗口类"""
    
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        
        # 设置窗口标题和大小
        self.setWindowTitle("镇中科技机械臂控制工具v0.2.25")
        self.resize(1200, 1400)
        self.init_models()
        self.init_controllers()
        self.init_ui()
        self.init_signals()
    
        # 添加窗口关闭事件处理
        app = QApplication.instance()
        if app:
            app.aboutToQuit.connect(self.closeEvent)
    
    def init_ui(self):
        """初始化用户界面"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        self._init_serial_config()
        self._init_contour_settings()
        self._create_toolbar()

        # ===== 上半部分 =====
        top_panel = QVBoxLayout()
        top_panel.setContentsMargins(0, 0, 0, 0)  # 设置外部布局的边距为0
        
        # 创建上半部分的标签页
        self.top_tab_widget = QTabWidget()
        
        # 主页标签内容
        main_tab = QWidget()
        main_tab_layout = QVBoxLayout(main_tab)
        main_tab_layout.setContentsMargins(6, 6, 6, 6)  # 设置适当的内部边距
        
        first_row = QHBoxLayout()
        self.port_frame = PortSelectionFrame(
            parent=self,
            get_config=self.serial_config.get_config,
        )
        first_row.addWidget(self.port_frame)
        main_tab_layout.addLayout(first_row)
        
        second_row = QHBoxLayout()
        self.control_frame = ControlButtonsFrame(
            parent=self,
        )
        second_row.addWidget(self.control_frame)
        main_tab_layout.addLayout(second_row)
        
        third_row = QHBoxLayout()
        self.angle_control_frame = AngleControlFrame(
            parent=self,
            get_contour=self.contour_settings.get_contour_params,
            get_encoding_type=self.control_frame.get_encoding_type,
            get_run_mode=self.control_frame.get_run_mode
        )
        third_row.addWidget(self.angle_control_frame)
        self.end_position = EndPositionFrame(self)
        third_row.addWidget(self.end_position)
        main_tab_layout.addLayout(third_row)
        
        fourth_row = QHBoxLayout()
        self.effector_frame = EffectorFrame(
            self,
            get_encoding_type=self.control_frame.get_encoding_type,
        )
        fourth_row.addWidget(self.effector_frame)
        main_tab_layout.addLayout(fourth_row)
        
        # 添加主页标签
        self.top_tab_widget.addTab(main_tab, "主页")
        
        # 创建运动规划标签
        motion_planning_tab = QWidget()
        motion_planning_layout = QVBoxLayout(motion_planning_tab)
        motion_planning_layout.setContentsMargins(6, 6, 6, 6)  # 设置适当的内部边距
        # 添加运动规划相关的组件
        self.motion_planning_frame = MotionPlanningFrame(self)
        motion_planning_layout.addWidget(self.motion_planning_frame)
        
        # 添加运动规划标签
        self.top_tab_widget.addTab(motion_planning_tab, "运动规划")

        # 创建动力学标签
        dynamics_tab = QWidget()
        dynamics_layout = QVBoxLayout(dynamics_tab)
        dynamics_layout.setContentsMargins(6, 6, 6, 6)  # 设置适当的内部边距
        # 添加动力学相关的组件
        self.dynamics_frame = DynamicsFrame(self)
        dynamics_layout.addWidget(self.dynamics_frame)
        
        # 添加动力学标签
        self.top_tab_widget.addTab(dynamics_tab, "动力学")

        # 创建摄像头标签
        camera_tab = QWidget()
        camera_layout = QVBoxLayout(camera_tab)
        camera_layout.setContentsMargins(6, 6, 6, 6)  # 设置适当的内部边距
        # 添加摄像头相关的组件
        self.camera_display = CameraDisplayWidget(self)
        camera_layout.addWidget(self.camera_display)
        
        # 添加摄像头标签
        self.top_tab_widget.addTab(camera_tab, "摄像头")

        # 将标签页添加到上半部分
        top_panel.addWidget(self.top_tab_widget)
        
        top_widget = QWidget()
        top_widget.setLayout(top_panel)
        top_widget.setContentsMargins(0, 0, 0, 0)  # 设置容器边距为0
        main_layout.addWidget(top_widget, 4)
        
        # ===== 下半部分（Tab区域）=====
        self.tab_widget = QTabWidget()
        main_layout.setContentsMargins(0, 0, 0, 0)  # 设置主布局的边距为0

        self.data_display = DataDisplayFrame(self)
        self.tab_widget.addTab(self.data_display, "数据显示")
        
        self.inverse_kinematic = InverseKinematicFrame(self)
        self.tab_widget.addTab(self.inverse_kinematic, "逆运动学")
        
        main_layout.addWidget(self.tab_widget, 6)

    def init_models(self):
        self.serial_model = SerialModel()
        self.motion_model = MotionModel(self.serial_model)
        self.camera_model = CameraModel()
        self.detection_model = DetectionModel(self.camera_model)
        self.robot_model = RobotModel()

    def init_controllers(self):
        self.serial_controller = SerialController(serial_model=self.serial_model,
                                                  motion_model=self.motion_model)
        self.motion_controller = MotionController(serial_model=self.serial_model,
                                                  motion_model=self.motion_model,
                                                  robot_model=self.robot_model)
        self.dynamic_controller = DynamicController(serial_model=self.serial_model)
        self.effector_controller = EffectorController(serial_model=self.serial_model)
        self.camera_controller = CameraController(camera_model=self.camera_model, detection_model=self.detection_model, serial_model=self.serial_model, robot_model=self.robot_model)
        self.detection_controller = DetectionController(detection_model=self.detection_model, 
                                                       camera_model=self.camera_model)

    def init_signals(self):
        self.port_frame.port_connect_requested.connect(self.serial_controller.connect)
        self.port_frame.port_disconnect_requested.connect(self.serial_controller.disconnect)
        self.port_frame.refresh_ports_requested.connect(self.serial_controller.refresh_ports)

        self.control_frame.send_command_requested.connect(self.motion_controller.handle_command_requested)
        self.angle_control_frame.send_angles_requested.connect(self.motion_controller.handle_angles_requested)
        self.effector_frame.send_effector_command_requested.connect(self.effector_controller.handle_effector_command_requested)

        self.motion_planning_frame.motion_start_signal.connect(self.motion_controller.multiple_motion_setting)
        self.motion_planning_frame.get_current_position_signal.connect(self.motion_controller.get_current_position)
        self.motion_controller.only_get_current_position_signal.connect(self.motion_planning_frame.set_current_position)

        self.motion_controller.display_requested.connect(self.data_display.append_message)
        self.effector_controller.display_requested.connect(self.data_display.append_message)
        self.dynamic_controller.display_requested.connect(self.data_display.append_message)

        # 添加动力学组件信号连接
        self.dynamics_frame.start_cyclic_torque_requested.connect(self.dynamic_controller.handle_enable_dynamic_command_requested)
        self.dynamics_frame.teaching_mode_toggle_requested.connect(self.dynamic_controller.handle_dynamic_command_requested)
        self.dynamics_frame.send_torque_requested.connect(self.dynamic_controller.send_dynamic_torque_command)
        self.motion_controller.torque_calculation_signal.connect(self.dynamic_controller.handle_dynamic_torque_calculation_command_requested)

        # 添加摄像头组件信号连接
        self.camera_display.connect_camera_requested.connect(self.camera_controller.connect_camera)
        self.camera_display.disconnect_camera_requested.connect(self.camera_controller.disconnect_camera)
        self.camera_display.color_display_requested.connect(self.camera_controller.start_color_display)
        self.camera_display.depth_display_requested.connect(self.camera_controller.start_depth_display)
        self.camera_display.stop_display_requested.connect(self.camera_controller.stop_display)
        self.camera_display.start_detection_requested.connect(self.camera_controller.start_detection)
        self.camera_display.stop_detection_requested.connect(self.camera_controller.stop_detection)
        self.camera_display.start_pose_publish_requested.connect(self.camera_controller.start_pose_publish)
        self.camera_display.stop_pose_publish_requested.connect(self.camera_controller.stop_pose_publish)
        
        # 连接摄像头控制器信号到显示组件
        self.camera_controller.image_display_requested.connect(self.camera_display.display_image)
        self.camera_controller.status_update_requested.connect(self.camera_display.update_status)
        self.camera_controller.image_info_updated.connect(self.camera_display.update_image_info)
        self.camera_controller.connection_status_changed.connect(self.camera_display.update_connection_status)
        self.camera_controller.display_requested.connect(self.data_display.append_message)
        
        # 添加检测控制器信号连接
        self.detection_controller.display_requested.connect(self.data_display.append_message)

        self.serial_controller.display_requested.connect(self.data_display.append_message)
        self.serial_controller.connection_changed.connect(self.handle_connection_changed)
        self.serial_controller.ports_updated.connect(self.port_frame.set_ports)

    def _create_toolbar(self):
        toolbar = QToolBar()
        toolbar.setMovable(False)
        self.addToolBar(toolbar)
        
        settings_menu = QMenu("设置", self)
        
        serial_action = QAction("串口参数配置", self)
        serial_action.triggered.connect(self.show_serial_config)
        settings_menu.addAction(serial_action)
        
        contour_action = QAction("轮廓模式参数配置", self)
        contour_action.triggered.connect(self.show_contour_settings)
        settings_menu.addAction(contour_action)
        
        settings_button = QPushButton("设置")
        settings_button.setMenu(settings_menu)
        toolbar.addWidget(settings_button)
    
    def _init_serial_config(self):
        self.serial_config_dialog = QWidget()
        self.serial_config_dialog.setWindowTitle("串口参数配置")
        layout = QVBoxLayout()
        self.serial_config = SerialConfigFrame()
        layout.addWidget(self.serial_config)
        self.serial_config_dialog.setLayout(layout)
        self.serial_config_dialog.resize(400, 300)
        self.serial_config_dialog.hide()

    def _init_contour_settings(self):
        self.contour_settings_dialog = QWidget()
        self.contour_settings_dialog.setWindowTitle("轮廓模式参数配置")
        layout = QVBoxLayout()
        self.contour_settings = ContourSettings(self.contour_settings_dialog)
        layout.addWidget(self.contour_settings)
        self.contour_settings_dialog.setLayout(layout)
        self.contour_settings_dialog.resize(600, 400)
        self.contour_settings_dialog.hide()
    
    def show_serial_config(self):
        """显示串口参数配置对话框"""
        self.serial_config_dialog.show()
    
    def show_contour_settings(self):
        """显示轮廓模式参数配置对话框"""
        self.contour_settings_dialog.show()

    def handle_connection_changed(self, connected):
        self.port_frame.update_connection_status(connected)
        self.control_frame.update_connection_status(connected)
        self.effector_frame.update_connection_status(connected)
        self.serial_config.update_connection_status(connected)
        self.angle_control_frame.update_connection_status(connected)
        self.dynamics_frame.update_connection_status(connected)

    def closeEvent(self, event=None):
        """关闭窗口事件处理"""
        # 断开串口连接
        if hasattr(self, 'serial_controller') and self.serial_controller:
            self.serial_controller.disconnect()
        
        # 清理摄像头资源
        if hasattr(self, 'camera_controller') and self.camera_controller:
            self.camera_controller.cleanup()
            
        # 允许默认关闭行为
        if event:
            event.accept()
