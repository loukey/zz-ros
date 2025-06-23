#!/usr/bin/env python3
"""
QTimer精度测试工具
测试不同时间间隔下QTimer的精度和稳定性
"""

import sys
import time
from statistics import mean, stdev
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QTextEdit, QSpinBox, QComboBox
from PyQt5.QtCore import QTimer, Qt, pyqtSignal
from PyQt5.QtGui import QFont


class TimerPrecisionTest(QWidget):
    test_completed = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self.init_ui()
        self.timer = QTimer()
        self.timer.timeout.connect(self.on_timer_timeout)
        
        # 测试数据
        self.test_data = []
        self.start_time = 0
        self.last_time = 0
        self.expected_interval = 0
        self.test_count = 0
        self.max_tests = 0
        
    def init_ui(self):
        self.setWindowTitle("QTimer 精度测试工具")
        self.setGeometry(100, 100, 800, 600)
        
        layout = QVBoxLayout()
        
        # 标题
        title = QLabel("QTimer 精度测试")
        title.setFont(QFont("Arial", 16, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        # 控制面板
        control_layout = QHBoxLayout()
        
        # 时间间隔设置
        control_layout.addWidget(QLabel("时间间隔(ms):"))
        self.interval_spinbox = QSpinBox()
        self.interval_spinbox.setRange(1, 10000)
        self.interval_spinbox.setValue(100)
        control_layout.addWidget(self.interval_spinbox)
        
        # 测试次数设置
        control_layout.addWidget(QLabel("测试次数:"))
        self.count_spinbox = QSpinBox()
        self.count_spinbox.setRange(10, 10000)
        self.count_spinbox.setValue(100)
        control_layout.addWidget(self.count_spinbox)
        
        # 定时器类型选择
        control_layout.addWidget(QLabel("定时器类型:"))
        self.timer_type_combo = QComboBox()
        self.timer_type_combo.addItems([
            "Qt.PreciseTimer", 
            "Qt.CoarseTimer", 
            "Qt.VeryCoarseTimer"
        ])
        control_layout.addWidget(self.timer_type_combo)
        
        # 开始测试按钮
        self.start_button = QPushButton("开始测试")
        self.start_button.clicked.connect(self.start_test)
        control_layout.addWidget(self.start_button)
        
        # 停止测试按钮
        self.stop_button = QPushButton("停止测试")
        self.stop_button.clicked.connect(self.stop_test)
        self.stop_button.setEnabled(False)
        control_layout.addWidget(self.stop_button)
        
        # 预设测试按钮
        self.preset_button = QPushButton("运行预设测试")
        self.preset_button.clicked.connect(self.run_preset_tests)
        control_layout.addWidget(self.preset_button)
        
        layout.addLayout(control_layout)
        
        # 状态显示
        self.status_label = QLabel("就绪")
        self.status_label.setFont(QFont("Arial", 12))
        layout.addWidget(self.status_label)
        
        # 结果显示区域
        self.result_text = QTextEdit()
        self.result_text.setFont(QFont("Consolas", 10))
        layout.addWidget(self.result_text)
        
        self.setLayout(layout)
        
        # 连接信号
        self.test_completed.connect(self.display_results)
    
    def get_timer_type(self):
        """获取选中的定时器类型"""
        type_map = {
            "Qt.PreciseTimer": 0,  # Qt.PreciseTimer
            "Qt.CoarseTimer": 1,   # Qt.CoarseTimer  
            "Qt.VeryCoarseTimer": 2  # Qt.VeryCoarseTimer
        }
        return type_map[self.timer_type_combo.currentText()]
    
    def start_test(self):
        """开始单个测试"""
        self.expected_interval = self.interval_spinbox.value()
        self.max_tests = self.count_spinbox.value()
        
        self.test_data = []
        self.test_count = 0
        
        # 设置定时器类型
        self.timer.setTimerType(self.get_timer_type())
        
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.preset_button.setEnabled(False)
        
        self.status_label.setText(f"测试进行中... 间隔: {self.expected_interval}ms, 类型: {self.timer_type_combo.currentText()}")
        
        # 记录开始时间并启动定时器
        self.start_time = time.perf_counter()
        self.last_time = self.start_time
        self.timer.start(self.expected_interval)
    
    def stop_test(self):
        """停止测试"""
        self.timer.stop()
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.preset_button.setEnabled(True)
        self.status_label.setText("测试已停止")
        
        if self.test_data:
            self.analyze_and_display_results()
    
    def on_timer_timeout(self):
        """定时器超时回调"""
        current_time = time.perf_counter()
        
        if self.test_count > 0:  # 跳过第一次，因为没有间隔参考
            actual_interval = (current_time - self.last_time) * 1000  # 转换为毫秒
            self.test_data.append(actual_interval)
        
        self.last_time = current_time
        self.test_count += 1
        
        # 更新状态
        self.status_label.setText(f"测试进行中... {self.test_count}/{self.max_tests}")
        
        if self.test_count >= self.max_tests:
            self.stop_test()
    
    def analyze_and_display_results(self):
        """分析并显示测试结果"""
        if not self.test_data:
            return
        
        # 计算统计数据
        intervals = self.test_data
        avg_interval = mean(intervals)
        std_deviation = stdev(intervals) if len(intervals) > 1 else 0
        min_interval = min(intervals)
        max_interval = max(intervals)
        
        # 计算误差
        errors = [abs(interval - self.expected_interval) for interval in intervals]
        avg_error = mean(errors)
        max_error = max(errors)
        
        # 计算精度百分比
        accuracy = (1 - avg_error / self.expected_interval) * 100
        
        # 生成结果报告
        result = self.generate_result_report(
            self.expected_interval, avg_interval, std_deviation,
            min_interval, max_interval, avg_error, max_error, accuracy
        )
        
        self.test_completed.emit(result)
    
    def generate_result_report(self, expected, avg, std, min_val, max_val, avg_error, max_error, accuracy):
        """生成结果报告"""
        timer_type = self.timer_type_combo.currentText()
        
        report = f"\n{'='*60}\n"
        report += f"QTimer 精度测试结果\n"
        report += f"{'='*60}\n"
        report += f"测试配置:\n"
        report += f"  - 期望间隔: {expected:.2f} ms\n"
        report += f"  - 定时器类型: {timer_type}\n"
        report += f"  - 测试次数: {len(self.test_data)}\n\n"
        
        report += f"时间间隔统计:\n"
        report += f"  - 平均间隔: {avg:.3f} ms\n"
        report += f"  - 标准偏差: {std:.3f} ms\n"
        report += f"  - 最小间隔: {min_val:.3f} ms\n"
        report += f"  - 最大间隔: {max_val:.3f} ms\n\n"
        
        report += f"精度分析:\n"
        report += f"  - 平均误差: {avg_error:.3f} ms\n"
        report += f"  - 最大误差: {max_error:.3f} ms\n"
        report += f"  - 精度百分比: {accuracy:.2f}%\n\n"
        
        # 误差分布统计
        error_ranges = [
            (0, 1), (1, 5), (5, 10), (10, 50), (50, float('inf'))
        ]
        
        report += f"误差分布:\n"
        for min_err, max_err in error_ranges:
            count = sum(1 for err in [abs(i - expected) for i in self.test_data] 
                       if min_err <= err < max_err)
            percentage = (count / len(self.test_data)) * 100
            range_str = f"[{min_err}-{max_err if max_err != float('inf') else '∞'})"
            report += f"  - {range_str:>8} ms: {count:>4} 次 ({percentage:>5.1f}%)\n"
        
        return report
    
    def display_results(self, result):
        """显示测试结果"""
        self.result_text.append(result)
        self.status_label.setText("测试完成")
    
    def run_preset_tests(self):
        """运行预设的多组测试"""
        self.result_text.clear()
        self.result_text.append("开始运行预设测试套件...\n")
        
        # 预设测试配置：(间隔ms, 测试次数, 定时器类型)
        test_configs = [
            (1, 100, "Qt.PreciseTimer"),
            (10, 100, "Qt.PreciseTimer"),
            (50, 100, "Qt.PreciseTimer"),
            (100, 100, "Qt.PreciseTimer"),
            (500, 50, "Qt.PreciseTimer"),
            (1000, 30, "Qt.PreciseTimer"),
            (100, 100, "Qt.CoarseTimer"),
            (100, 100, "Qt.VeryCoarseTimer"),
        ]
        
        self.preset_tests = iter(test_configs)
        self.run_next_preset_test()
    
    def run_next_preset_test(self):
        """运行下一个预设测试"""
        try:
            interval, count, timer_type = next(self.preset_tests)
            
            # 设置参数
            self.interval_spinbox.setValue(interval)
            self.count_spinbox.setValue(count)
            
            # 设置定时器类型
            index = self.timer_type_combo.findText(timer_type)
            if index >= 0:
                self.timer_type_combo.setCurrentIndex(index)
            
            # 连接完成信号到下一个测试
            self.test_completed.disconnect()
            self.test_completed.connect(self.on_preset_test_completed)
            
            # 开始测试
            self.start_test()
            
        except StopIteration:
            # 所有预设测试完成
            self.result_text.append("\n所有预设测试完成！")
            self.status_label.setText("预设测试套件完成")
            # 重新连接到正常显示
            self.test_completed.disconnect()
            self.test_completed.connect(self.display_results)
    
    def on_preset_test_completed(self, result):
        """预设测试完成回调"""
        self.display_results(result)
        # 延迟一点时间再运行下一个测试，避免界面阻塞
        QTimer.singleShot(100, self.run_next_preset_test)


def main():
    app = QApplication(sys.argv)
    
    # 创建测试窗口
    test_window = TimerPrecisionTest()
    test_window.show()
    
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
