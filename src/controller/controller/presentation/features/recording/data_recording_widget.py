"""
数据录制功能界面 - Presentation层
提供录制控制按钮和状态显示
"""
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, 
    QLabel, QGroupBox
)
from PyQt5.QtCore import Qt, pyqtSlot
from controller.application.services.data_recording_application_service import DataRecordingApplicationService

class DataRecordingWidget(QWidget):
    def __init__(self, recording_service: DataRecordingApplicationService):
        super().__init__()
        self.recording_service = recording_service
        self.init_ui()
        self.connect_signals()
        
    def init_ui(self):
        layout = QVBoxLayout(self)
        
        # 控制面板
        control_group = QGroupBox("数据录制控制")
        control_layout = QVBoxLayout(control_group)
        
        # 状态标签
        self.status_label = QLabel("状态: 未录制")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("font-size: 14px; font-weight: bold; color: gray;")
        control_layout.addWidget(self.status_label)
        
        # 详细信息标签
        self.info_label = QLabel("等待指令...")
        self.info_label.setAlignment(Qt.AlignCenter)
        self.info_label.setWordWrap(True)
        control_layout.addWidget(self.info_label)
        
        # 按钮容器
        btn_layout = QHBoxLayout()
        
        # 录制按钮
        self.record_btn = QPushButton("开始录制")
        self.record_btn.setMinimumHeight(50)
        self.record_btn.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50; 
                color: white; 
                font-size: 16px; 
                border-radius: 5px;
            }
            QPushButton:hover { background-color: #45a049; }
            QPushButton:pressed { background-color: #3e8e41; }
        """)
        btn_layout.addWidget(self.record_btn)
        
        control_layout.addLayout(btn_layout)
        layout.addWidget(control_group)
        
        # 说明
        note_label = QLabel(
            "说明：\n"
            "1. 请确保 'src/record/record_node.py' 已经在后台运行\n"
            "2. 点击开始后，系统会自动在 ./record_data/record_n 目录下保存数据\n"
            "3. 录制期间会自动广播 ROS 关节状态数据"
        )
        note_label.setStyleSheet("color: #666; font-style: italic;")
        layout.addWidget(note_label)
        
        layout.addStretch()
        
    def connect_signals(self):
        # UI -> Service
        self.record_btn.clicked.connect(self._on_record_btn_clicked)
        
        # Service -> UI
        self.recording_service.recording_status_changed.connect(self._on_status_changed)
        
    @pyqtSlot()
    def _on_record_btn_clicked(self):
        if self.recording_service.is_recording():
            # 停止录制
            self.recording_service.stop_recording()
            self.record_btn.setEnabled(False) # 防抖
            self.record_btn.setText("正在停止...")
        else:
            # 开始录制
            self.recording_service.start_recording()
            self.record_btn.setEnabled(False) # 防抖
            self.record_btn.setText("正在启动...")
            
    @pyqtSlot(bool, str)
    def _on_status_changed(self, is_recording, message):
        self.record_btn.setEnabled(True)
        self.info_label.setText(message)
        
        if is_recording:
            self.status_label.setText("状态: 🔴 正在录制")
            self.status_label.setStyleSheet("font-size: 14px; font-weight: bold; color: red;")
            self.record_btn.setText("停止录制")
            self.record_btn.setStyleSheet("""
                QPushButton {
                    background-color: #f44336; 
                    color: white; 
                    font-size: 16px; 
                    border-radius: 5px;
                }
                QPushButton:hover { background-color: #da190b; }
            """)
        else:
            self.status_label.setText("状态: 未录制")
            self.status_label.setStyleSheet("font-size: 14px; font-weight: bold; color: gray;")
            self.record_btn.setText("开始录制")
            self.record_btn.setStyleSheet("""
                QPushButton {
                    background-color: #4CAF50; 
                    color: white; 
                    font-size: 16px; 
                    border-radius: 5px;
                }
                QPushButton:hover { background-color: #45a049; }
            """)
