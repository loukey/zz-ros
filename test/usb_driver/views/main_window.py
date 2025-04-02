"""
主窗口视图
"""
from PyQt5.QtWidgets import QMainWindow, QWidget, QVBoxLayout, QTabWidget, QMessageBox, QHBoxLayout, QStatusBar, QProgressBar, QToolBar, QApplication
from PyQt5.QtCore import QTimer
from components import *
from controllers import *
from .main_view import *
from components.effector_components import EffectorSettings
from components.contour_components import ContourSettings
from views.main_view.effector_handler import EffectorHandler


class MainWindow(QMainWindow):
    """主窗口类"""
    
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        
        # 设置窗口标题和大小
        self.setWindowTitle("镇中科技机械臂控制工具v0.1.22")
        self.resize(1200, 1000)
        
        # 初始化状态变量
        self.waiting_for_position = False
        self.current_position = None
        self.ros_initialized = False
        
        # 创建功能类实例
        self.serial_manager = None
        self.motion_controller = None
        self.kinematic_handler = None
        self.trajectory_handler = None
        
        # 创建状态栏
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("就绪")
        
        # 创建进度条
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        self.status_bar.addPermanentWidget(self.progress_bar)
        
        # 初始化串口控制器
        try:
            self.serial_controller = SerialController()
        except Exception as e:
            QMessageBox.critical(self, "错误", f"初始化串口控制器失败: {str(e)}")
        
        # 初始化轨迹控制器 - 简化为只做轨迹计算
        try:
            self.trajectory_controller = TrajectoryController()
        except Exception as e:
            QMessageBox.critical(self, "错误", f"初始化轨迹控制器失败: {str(e)}")
            
        # 初始化界面
        self._init_ui()
        
        # 初始化ROS控制器
        try:
            self.ros_controller = ROSController()
            self.ros_initialized = self.ros_controller.initialize()
        except Exception as e:
            self.ros_initialized = False
        
        # 初始化各功能管理器
        self.init_managers()
        
        # 连接信号（确保在初始化管理器后进行）
        self.connect_signals()
        
        # 设置控制器信号连接
        self._connect_controllers()
        
        # 开启定时刷新
        self.refresh_timer = QTimer()
        self.refresh_timer.timeout.connect(self.serial_handler.refresh_ports)
        self.refresh_timer.start(2000)  # 2秒刷新一次
        
        # 添加窗口关闭事件处理
        # 使用 QApplication.instance() 获取应用程序实例，避免直接使用 qApp
        app = QApplication.instance()
        if app:
            app.aboutToQuit.connect(self.closeEvent)
        
        # 初始化成功日志
        if hasattr(self, 'data_display') and self.data_display:
            if self.ros_initialized:
                self.data_display.append_message("ROS控制器初始化成功", "ROS")
            else:
                self.data_display.append_message("ROS控制器初始化失败或不可用", "ROS")
    
    def _init_ui(self):
        """初始化用户界面"""
        # 创建主窗口部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 创建主布局
        main_layout = QVBoxLayout(central_widget)
        
        # 创建顶部工具栏
        self._create_toolbar()
        
        # ===== 上半部分 =====
        top_panel = QVBoxLayout()
        
        # 第一行：串口选择区域
        first_row = QHBoxLayout()
        
        # 创建串口选择区域
        self.port_frame = PortSelectionFrame(
            parent=self,
            refresh_callback=self.refresh_ports
        )
        first_row.addWidget(self.port_frame)
        
        top_panel.addLayout(first_row)
        
        # 第二行：串口通信参数配置
        second_row = QHBoxLayout()
        
        # 添加串口配置区域
        self.serial_config = SerialConfigFrame(
            parent=self,
            connect_callback=self.toggle_connection
        )
        second_row.addWidget(self.serial_config)
        
        top_panel.addLayout(second_row)
        
        # 第三行：控制命令按钮区域
        third_row = QHBoxLayout()
        
        # 添加控制按钮面板
        self.control_buttons = ControlButtonsFrame(
            send_control_command_callback=self.send_control_command,
            parent=self
        )
        third_row.addWidget(self.control_buttons)
        
        top_panel.addLayout(third_row)
        
        # 第四行：角度控制和末端姿态并排
        fourth_row = QHBoxLayout()
        
        # 添加角度控制面板
        self.angle_control = AngleControlFrame(
            parent=self,
            send_callback=self.send_angles,
            convert_callback=self.convert_angles,
            zero_callback=self.zero_angles
        )
        fourth_row.addWidget(self.angle_control)
        
        # 添加位置控制面板
        self.end_position = EndPositionFrame(self)
        fourth_row.addWidget(self.end_position)
        
        top_panel.addLayout(fourth_row)
        
        top_widget = QWidget()
        top_widget.setLayout(top_panel)
        main_layout.addWidget(top_widget, 4)  # 权重从5减小到4（40%）
        
        # ===== 下半部分（Tab区域）=====
        
        # 创建一个Tab控件
        self.tab_widget = QTabWidget()
        
        # 创建数据显示Tab
        self.data_display = DataDisplayFrame(self)
        self.tab_widget.addTab(self.data_display, "数据显示")
        
        # 创建曲线显示Tab
        self.curve_plot = CurvePlotFrame(self)
        self.tab_widget.addTab(self.curve_plot, "曲线显示")
        
        # 创建逆运动学Tab
        self.inverse_kinematic = InverseKinematicFrame(
            parent=self
        )
        self.tab_widget.addTab(self.inverse_kinematic, "逆运动学")
        
        # 创建轮廓设置Tab
        self.contour_settings = ContourSettings(self)
        self.tab_widget.addTab(self.contour_settings, "轮廓模式参数")
        
        # 创建执行器设置Tab
        self.effector_settings = EffectorSettings(
            self,
            send_effector_command_callback=self.send_effector_command
        )
        self.tab_widget.addTab(self.effector_settings, "夹爪设置")
        
        # 添加Tab控件到主布局（占60%的空间）
        main_layout.addWidget(self.tab_widget, 6)  # 权重从5增加到6（60%）
    
    def _connect_controllers(self):
        """连接控制器信号"""
        # 连接串口控制器信号
        if hasattr(self, 'serial_controller'):
            # 使用 register_ 方法注册回调函数
            self.serial_controller.register_data_received_callback(self.handle_data_received)
            self.serial_controller.register_connection_changed_callback(self.handle_connection_changed)
            self.serial_controller.register_error_occurred_callback(self.handle_error_occurred)
            
        # 连接轨迹控制器的信号 - 简化后只有错误信号
        if hasattr(self, 'trajectory_controller'):
            # 轨迹控制器只有计算错误信号
            self.trajectory_controller.calculation_error.connect(self.handle_calculation_error)
    
    def init_managers(self):
        """初始化各功能管理器"""
        try:
            # 创建串口处理器
            self.serial_handler = SerialHandler(self)
            
            # 创建运动处理器
            self.motion_handler = MotionHandler(self)
            
            # 创建运动学处理器
            self.kinematic_handler = KinematicHandler(self)
            
            # 创建执行器处理器
            self.effector_handler = EffectorHandler(self)
            
            # 初始化成功消息
            self.data_display.append_message("功能处理器初始化成功", "系统")
        except Exception as e:
            self.data_display.append_message(f"功能处理器初始化异常: {str(e)}", "错误")
    
    def _create_toolbar(self):
        """创建工具栏"""
        # 创建工具栏
        toolbar = QToolBar("主工具栏")
        self.addToolBar(toolbar)
    
    def connect_signals(self):
        """连接控件信号"""
        # 数据显示面板信号已经在初始化时连接
        
        # 连接数据显示面板信号
        self.data_display.clear_send_btn.clicked.connect(self.clear_send)
        self.data_display.clear_receive_btn.clicked.connect(self.clear_receive)
        self.data_display.clear_all_btn.clicked.connect(self.clear_all)
        
        # 设置逆运动学回调函数
        # self.inverse_kinematic.set_apply_callback(self.kinematic_handler.apply_inverse_kinematics_result)
    
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
    
    def handle_connection_changed(self, connected, port=None):
        """处理连接状态变化"""
        self.serial_handler.handle_connection_changed(connected, port)
    
    def handle_error_occurred(self, error):
        """处理错误事件"""
        self.serial_handler.handle_error_occurred(error)

    def handle_calculation_error(self, error_message):
        """处理轨迹计算错误事件"""
        self.serial_handler.handle_calculation_error(error_message)
