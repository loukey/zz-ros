 #!/usr/bin/env python3
"""
QTimer精度测试 + 串口通信
测试10ms间隔的QTimer精度，并通过串口发送命令
"""

import sys
import time
import serial
import serial.tools.list_ports
from statistics import mean, stdev
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                           QLabel, QPushButton, QTextEdit, QComboBox, 
                           QSpinBox, QCheckBox, QLineEdit, QGroupBox,
                           QDoubleSpinBox, QFormLayout)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QThread
from PyQt5.QtGui import QFont

# 导入命令格式化函数
from test import format_command


class SerialTimerTest(QWidget):
    precision_update = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self.init_ui()
        
        # Timer设置
        self.precision_timer = QTimer()
        self.precision_timer.timeout.connect(self.on_precision_timer)
        
        # 串口对象
        self.serial_port = None
        
        # 精度测试数据
        self.timing_data = []
        self.last_time = 0
        self.test_count = 0
        self.max_test_count = 1000
        
        # 命令计数
        self.cmd_count = 0
        
    def init_ui(self):
        self.setWindowTitle("QTimer精度测试 + 串口通信")
        self.setGeometry(100, 100, 900, 700)
        
        layout = QVBoxLayout()
        
        # 标题
        title = QLabel("QTimer (10ms) 精度测试 + 串口通信")
        title.setFont(QFont("Arial", 16, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        # 串口配置区域
        serial_group = self.create_serial_config()
        layout.addWidget(serial_group)
        
        # 测试控制区域
        control_group = self.create_control_panel()
        layout.addWidget(control_group)
        
        # 机器人参数设置区域
        robot_params_group = self.create_robot_params()
        layout.addWidget(robot_params_group)
        
        # 精度显示区域
        precision_group = self.create_precision_display()
        layout.addWidget(precision_group)
        
        # 日志显示区域
        log_group = self.create_log_display()
        layout.addWidget(log_group)
        
        self.setLayout(layout)
        
        # 连接信号
        self.precision_update.connect(self.update_precision_display)
        
        # 刷新串口列表
        self.refresh_ports()
    
    def create_serial_config(self):
        """创建串口配置区域"""
        group = QWidget()
        layout = QHBoxLayout()
        
        # 串口选择
        layout.addWidget(QLabel("串口:"))
        self.port_combo = QComboBox()
        layout.addWidget(self.port_combo)
        
        # 波特率
        layout.addWidget(QLabel("波特率:"))
        self.baudrate_combo = QComboBox()
        self.baudrate_combo.addItems(['9600', '19200', '38400', '57600', '115200', '230400'])
        self.baudrate_combo.setCurrentText('115200')
        layout.addWidget(self.baudrate_combo)
        
        # 连接按钮
        self.connect_btn = QPushButton("连接")
        self.connect_btn.clicked.connect(self.toggle_serial_connection)
        layout.addWidget(self.connect_btn)
        
        # 刷新按钮
        refresh_btn = QPushButton("刷新")
        refresh_btn.clicked.connect(self.refresh_ports)
        layout.addWidget(refresh_btn)
        
        # 连接状态
        self.connection_status = QLabel("未连接")
        self.connection_status.setStyleSheet("color: red;")
        layout.addWidget(self.connection_status)
        
        group.setLayout(layout)
        return group
    
    def create_control_panel(self):
        """创建控制面板"""
        group = QWidget()
        layout = QHBoxLayout()
        
        # 定时器间隔设置（固定为10ms）
        layout.addWidget(QLabel("定时器间隔:"))
        self.interval_spin = QSpinBox()
        self.interval_spin.setRange(1, 1000)
        self.interval_spin.setValue(10)  # 10ms = 0.01s
        self.interval_spin.setSuffix(" ms")
        layout.addWidget(self.interval_spin)
        
        # 测试次数
        layout.addWidget(QLabel("测试次数:"))
        self.test_count_spin = QSpinBox()
        self.test_count_spin.setRange(100, 10000)
        self.test_count_spin.setValue(1000)
        layout.addWidget(self.test_count_spin)
        
        # 发送命令选项
        self.send_cmd_check = QCheckBox("发送机器人命令")
        self.send_cmd_check.setChecked(True)
        layout.addWidget(self.send_cmd_check)
        
        # 命令类型选择
        layout.addWidget(QLabel("命令类型:"))
        self.cmd_type_combo = QComboBox()
        self.cmd_type_combo.addItems(["格式化命令", "自定义命令"])
        self.cmd_type_combo.currentTextChanged.connect(self.on_cmd_type_changed)
        layout.addWidget(self.cmd_type_combo)
        
        # 自定义命令输入
        self.custom_cmd_input = QLineEdit()
        self.custom_cmd_input.setText("AA550608")  # 默认简单命令
        self.custom_cmd_input.setPlaceholderText("输入自定义命令(hex)")
        self.custom_cmd_input.setVisible(False)
        layout.addWidget(self.custom_cmd_input)
        
        # 开始测试按钮
        self.start_btn = QPushButton("开始测试")
        self.start_btn.clicked.connect(self.start_test)
        layout.addWidget(self.start_btn)
        
        # 停止测试按钮
        self.stop_btn = QPushButton("停止测试")
        self.stop_btn.clicked.connect(self.stop_test)
        self.stop_btn.setEnabled(False)
        layout.addWidget(self.stop_btn)
        
        group.setLayout(layout)
        return group
    
    def create_robot_params(self):
        """创建机器人参数设置区域"""
        group = QGroupBox("机器人参数 (format_command)")
        layout = QFormLayout()
        
        # 关节角度输入 (6个关节)
        self.joint_angles = []
        for i in range(6):
            spin = QDoubleSpinBox()
            spin.setRange(-3.14159, 3.14159)  # 弧度范围
            spin.setDecimals(4)
            spin.setSingleStep(0.1)
            spin.setValue(0.0)
            spin.setSuffix(" rad")
            layout.addRow(f"关节{i+1}角度:", spin)
            self.joint_angles.append(spin)
        
        # 控制和模式参数
        self.control_spin = QSpinBox()
        self.control_spin.setRange(0, 255)
        self.control_spin.setValue(0x06)
        self.control_spin.setDisplayIntegerBase(16)
        self.control_spin.setPrefix("0x")
        layout.addRow("控制字节:", self.control_spin)
        
        self.mode_spin = QSpinBox()
        self.mode_spin.setRange(0, 255)
        self.mode_spin.setValue(0x08)
        self.mode_spin.setDisplayIntegerBase(16)
        self.mode_spin.setPrefix("0x")
        layout.addRow("模式字节:", self.mode_spin)
        
        group.setLayout(layout)
        group.setVisible(True)  # 默认显示格式化命令参数
        self.robot_params_widget = group
        return group
    
    def on_cmd_type_changed(self, cmd_type):
        """命令类型改变回调"""
        if cmd_type == "格式化命令":
            self.robot_params_widget.setVisible(True)
            self.custom_cmd_input.setVisible(False)
        else:
            self.robot_params_widget.setVisible(False)
            self.custom_cmd_input.setVisible(True)
    
    def create_precision_display(self):
        """创建精度显示区域"""
        group = QWidget()
        layout = QVBoxLayout()
        
        title = QLabel("定时器精度统计")
        title.setFont(QFont("Arial", 12, QFont.Bold))
        layout.addWidget(title)
        
        # 实时精度显示
        stats_layout = QHBoxLayout()
        
        self.avg_interval_label = QLabel("平均间隔: -- ms")
        stats_layout.addWidget(self.avg_interval_label)
        
        self.std_dev_label = QLabel("标准偏差: -- ms")
        stats_layout.addWidget(self.std_dev_label)
        
        self.accuracy_label = QLabel("精度: -- %")
        stats_layout.addWidget(self.accuracy_label)
        
        self.cmd_count_label = QLabel("命令发送: 0")
        stats_layout.addWidget(self.cmd_count_label)
        
        layout.addLayout(stats_layout)
        
        # 测试进度
        self.progress_label = QLabel("进度: 0/0")
        layout.addWidget(self.progress_label)
        
        group.setLayout(layout)
        return group
    
    def create_log_display(self):
        """创建日志显示区域"""
        group = QWidget()
        layout = QVBoxLayout()
        
        title = QLabel("测试日志")
        title.setFont(QFont("Arial", 12, QFont.Bold))
        layout.addWidget(title)
        
        self.log_text = QTextEdit()
        self.log_text.setFont(QFont("Consolas", 9))
        self.log_text.setMaximumHeight(200)
        layout.addWidget(self.log_text)
        
        group.setLayout(layout)
        return group
    
    def refresh_ports(self):
        """刷新串口列表"""
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.port_combo.addItem(f"{port.device} - {port.description}")
        
        if not ports:
            self.port_combo.addItem("无可用串口")
    
    def toggle_serial_connection(self):
        """切换串口连接状态"""
        if self.serial_port and self.serial_port.is_open:
            self.disconnect_serial()
        else:
            self.connect_serial()
    
    def connect_serial(self):
        """连接串口"""
        try:
            if self.port_combo.count() == 0 or self.port_combo.currentText() == "无可用串口":
                self.log("没有可用的串口")
                return
            
            port_text = self.port_combo.currentText()
            port_name = port_text.split(" - ")[0]
            baudrate = int(self.baudrate_combo.currentText())
            
            self.serial_port = serial.Serial(
                port=port_name,
                baudrate=baudrate,
                timeout=0.1
            )
            
            self.connect_btn.setText("断开")
            self.connection_status.setText("已连接")
            self.connection_status.setStyleSheet("color: green;")
            self.log(f"串口连接成功: {port_name} @ {baudrate}")
            
        except Exception as e:
            self.log(f"串口连接失败: {str(e)}")
    
    def disconnect_serial(self):
        """断开串口连接"""
        try:
            if self.serial_port:
                self.serial_port.close()
                self.serial_port = None
            
            self.connect_btn.setText("连接")
            self.connection_status.setText("未连接")
            self.connection_status.setStyleSheet("color: red;")
            self.log("串口已断开")
            
        except Exception as e:
            self.log(f"断开串口时出错: {str(e)}")
    
    def start_test(self):
        """开始测试"""
        # 重置数据
        self.timing_data = []
        self.test_count = 0
        self.cmd_count = 0
        self.max_test_count = self.test_count_spin.value()
        
        # 更新UI状态
        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        
        # 记录开始时间
        self.last_time = time.perf_counter()
        
        # 启动定时器
        interval = self.interval_spin.value()
        self.precision_timer.start(interval)
        
        self.log(f"开始测试: 间隔={interval}ms, 次数={self.max_test_count}")
    
    def stop_test(self):
        """停止测试"""
        self.precision_timer.stop()
        
        # 更新UI状态
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        
        # 显示最终结果
        if self.timing_data:
            self.show_final_results()
        
        self.log("测试已停止")
    
    def on_precision_timer(self):
        """定时器回调函数"""
        current_time = time.perf_counter()
        
        # 计算实际间隔（跳过第一次）
        if self.test_count > 0:
            actual_interval = (current_time - self.last_time) * 1000  # 转换为毫秒
            self.timing_data.append(actual_interval)
        
        self.last_time = current_time
        self.test_count += 1
        
        # 发送串口命令（如果启用）
        if self.send_cmd_check.isChecked() and self.serial_port and self.serial_port.is_open:
            self.send_serial_command()
        
        # 更新精度显示（每10次更新一次以提高性能）
        if self.test_count % 10 == 0:
            self.update_realtime_precision()
        
        # 检查是否完成
        if self.test_count >= self.max_test_count:
            self.stop_test()
    
    def send_serial_command(self):
        """发送串口命令"""
        try:
            if self.cmd_type_combo.currentText() == "格式化命令":
                # 使用format_command函数生成命令
                joint_angles = [spin.value() for spin in self.joint_angles]
                control = self.control_spin.value()
                mode = self.mode_spin.value()
                
                # 调用format_command生成命令
                cmd_hex = format_command(
                    joint_angles=joint_angles,
                    control=control,
                    mode=mode
                )
                
                # 转换hex字符串为字节
                cmd_bytes = bytes.fromhex(cmd_hex)
                self.serial_port.write(cmd_bytes)
                self.cmd_count += 1
                
                # 记录发送的命令（只显示前部分，避免日志过长）
                if self.test_count % 100 == 0:  # 每100次记录一次
                    self.log(f"发送格式化命令: {cmd_hex[:20]}... ({len(cmd_hex)} hex chars)")
                
            else:
                # 自定义命令
                cmd_hex = self.custom_cmd_input.text().strip()
                if cmd_hex:
                    cmd_bytes = bytes.fromhex(cmd_hex)
                    self.serial_port.write(cmd_bytes)
                    self.cmd_count += 1
                    
                    if self.test_count % 100 == 0:
                        self.log(f"发送自定义命令: {cmd_hex}")
            
            # 尝试读取响应（非阻塞）
            if self.serial_port.in_waiting > 0:
                response = self.serial_port.read(self.serial_port.in_waiting)
                if response:
                    response_hex = response.hex().upper()
                    if self.test_count % 100 == 0:  # 减少日志频率
                        self.log(f"收到响应: {response_hex}")
                
        except Exception as e:
            self.log(f"串口通信错误: {str(e)}")
    
    def update_realtime_precision(self):
        """更新实时精度显示"""
        if not self.timing_data:
            return
        
        # 计算统计数据
        avg_interval = mean(self.timing_data)
        std_deviation = stdev(self.timing_data) if len(self.timing_data) > 1 else 0
        expected_interval = self.interval_spin.value()
        
        # 计算精度
        avg_error = abs(avg_interval - expected_interval)
        accuracy = (1 - avg_error / expected_interval) * 100
        
        # 更新显示
        self.avg_interval_label.setText(f"平均间隔: {avg_interval:.3f} ms")
        self.std_dev_label.setText(f"标准偏差: {std_deviation:.3f} ms")
        self.accuracy_label.setText(f"精度: {accuracy:.2f} %")
        self.cmd_count_label.setText(f"命令发送: {self.cmd_count}")
        self.progress_label.setText(f"进度: {self.test_count}/{self.max_test_count}")
    
    def update_precision_display(self, message):
        """更新精度显示"""
        self.log_text.append(message)
    
    def show_final_results(self):
        """显示最终测试结果"""
        if not self.timing_data:
            return
        
        # 计算最终统计
        avg_interval = mean(self.timing_data)
        std_deviation = stdev(self.timing_data)
        min_interval = min(self.timing_data)
        max_interval = max(self.timing_data)
        expected_interval = self.interval_spin.value()
        
        # 计算误差
        errors = [abs(interval - expected_interval) for interval in self.timing_data]
        avg_error = mean(errors)
        max_error = max(errors)
        accuracy = (1 - avg_error / expected_interval) * 100
        
        # 生成报告
        report = f"\n{'='*50}\n"
        report += f"QTimer 精度测试最终结果\n"
        report += f"{'='*50}\n"
        report += f"期望间隔: {expected_interval} ms\n"
        report += f"测试次数: {len(self.timing_data)}\n"
        report += f"串口命令发送: {self.cmd_count} 次\n\n"
        report += f"时间间隔统计:\n"
        report += f"  平均间隔: {avg_interval:.3f} ms\n"
        report += f"  标准偏差: {std_deviation:.3f} ms\n"
        report += f"  最小间隔: {min_interval:.3f} ms\n"
        report += f"  最大间隔: {max_interval:.3f} ms\n\n"
        report += f"精度分析:\n"
        report += f"  平均误差: {avg_error:.3f} ms\n"
        report += f"  最大误差: {max_error:.3f} ms\n"
        report += f"  精度百分比: {accuracy:.2f}%\n"
        
        self.precision_update.emit(report)
    
    def log(self, message):
        """添加日志"""
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.append(f"[{timestamp}] {message}")
        # 自动滚动到底部
        self.log_text.verticalScrollBar().setValue(
            self.log_text.verticalScrollBar().maximum()
        )
    
    def closeEvent(self, event):
        """关闭事件处理"""
        self.precision_timer.stop()
        self.disconnect_serial()
        event.accept()


def main():
    app = QApplication(sys.argv)
    
    test_window = SerialTimerTest()
    test_window.show()
    
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()