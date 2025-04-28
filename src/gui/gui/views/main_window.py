"""
主窗口视图
"""
from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QTabWidget, QMessageBox, 
                           QHBoxLayout, QStatusBar, QProgressBar, QToolBar, QApplication,
                           QMenu, QAction, QPushButton)
from .components import *
from .main_view import *
from gui.controllers import *


class MainWindow(QMainWindow):
    """主窗口类"""
    
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        
        # 设置窗口标题和大小
        self.setWindowTitle("镇中科技机械臂控制工具v0.2.0")
        self.resize(1200, 1400)
        self.init_controllers()
        self.init_ui()
        self.init_handlers()
        self._connect_controllers()
    
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

        first_row = QHBoxLayout()
        self.port_frame = PortSelectionFrame(
            parent=self,
            refresh_callback=self.refresh_ports,
            connect_callback=self.toggle_connection
        )
        first_row.addWidget(self.port_frame)
        top_panel.addLayout(first_row)
        
        second_row = QHBoxLayout()
        self.control_buttons = ControlButtonsFrame(
            send_control_command_callback=self.send_control_command,
            parent=self
        )
        second_row.addWidget(self.control_buttons)
        top_panel.addLayout(second_row)
        
        third_row = QHBoxLayout()
        self.angle_control = AngleControlFrame(
            parent=self,
            send_callback=self.send_angles,
            convert_callback=self.convert_angles,
            zero_callback=self.zero_angles
        )
        third_row.addWidget(self.angle_control)
        self.end_position = EndPositionFrame(self)
        third_row.addWidget(self.end_position)
        top_panel.addLayout(third_row)
        
        fourth_row = QHBoxLayout()
        self.effector_settings = EffectorSettings(
            self,
            send_callback=self.send_effector_command
        )
        fourth_row.addWidget(self.effector_settings)
        top_panel.addLayout(fourth_row)
        
        top_widget = QWidget()
        top_widget.setLayout(top_panel)
        main_layout.addWidget(top_widget, 4)
        
        # ===== 下半部分（Tab区域）=====
        self.tab_widget = QTabWidget()

        self.data_display = DataDisplayFrame(self)
        self.tab_widget.addTab(self.data_display, "数据显示")
        
        self.inverse_kinematic = InverseKinematicFrame(self)
        self.tab_widget.addTab(self.inverse_kinematic, "逆运动学")
        
        main_layout.addWidget(self.tab_widget, 6) 
    
    def _connect_controllers(self):
        self.serial_controller.register_data_received_callback(self.handle_data_received)
        self.serial_controller.register_connection_changed_callback(self.handle_connection_changed)
        self.serial_controller.register_error_occurred_callback(self.handle_error_occurred)
        self.trajectory_controller.calculation_error.connect(self.handle_calculation_error)
        self.data_display.clear_send_btn.clicked.connect(self.clear_send)
        self.data_display.clear_receive_btn.clicked.connect(self.clear_receive)
        self.data_display.clear_all_btn.clicked.connect(self.clear_all)
        # 设置逆运动学回调函数
        # self.inverse_kinematic.set_apply_callback(self.kinematic_handler.apply_inverse_kinematics_result)
        
    
    def init_controllers(self):
        self.serial_controller = SerialController()
        self.trajectory_controller = TrajectoryController()

    def init_handlers(self):
        self.serial_handler = SerialHandler(self)
        self.motion_handler = MotionHandler(self)
        self.kinematic_handler = KinematicHandler(self)
        self.effector_handler = EffectorHandler(self)

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
    
    def refresh_ports(self):
        """刷新串口列表"""
        self.serial_handler.refresh_ports()
    
    def toggle_connection(self):
        """切换串口连接状态"""
        self.serial_handler.toggle_connection()
    
    def send_control_command(self, command_type):
        """发送控制命令"""
        self.motion_handler.send_control_command(command_type)
    
    def send_angles(self):
        """发送角度命令"""
        self.motion_handler.send_angles()
    
    def convert_angles(self):
        """将角度值转换为弧度值"""
        self.motion_handler.convert_angles()
    
    def zero_angles(self):
        """归零处理"""
        self.motion_handler.zero_angles()

    def send_effector_command(self):
        """发送执行器命令"""
        self.effector_handler.handle_send_clicked()

    def clear_send(self):
        """清空发送区域"""
        self.data_display.clear_send()
    
    def clear_receive(self):
        """清空接收区域"""
        self.data_display.clear_receive()
    
    def clear_all(self):
        """清空所有区域"""
        self.data_display.clear_all()

    def closeEvent(self, event=None):
        """关闭窗口事件处理"""
        # 断开串口连接
        if hasattr(self, 'serial_controller') and self.serial_controller:
            self.serial_controller.disconnect()
        
        # 关闭ROS控制器
        if hasattr(self, 'ros_controller') and self.ros_controller:
            self.ros_controller.shutdown()
        
        # 停止定时器
        if hasattr(self, 'refresh_timer') and self.refresh_timer:
            self.refresh_timer.stop()
            
        # 允许默认关闭行为
        if event:
            event.accept()

    def handle_data_received(self, data):
        """处理接收到的数据"""
        self.serial_handler.handle_data_received(data)
    
    def handle_connection_changed(self, connected):
        """处理连接状态变化"""
        self.serial_handler.handle_connection_changed(connected)
    
    def handle_error_occurred(self, error):
        """处理错误事件"""
        self.serial_handler.handle_error_occurred(error)

    def handle_calculation_error(self, error_message):
        """处理轨迹计算错误事件"""
        self.serial_handler.handle_calculation_error(error_message)
