"""
数据录制组件 - Presentation层
提供录制控制按钮和状态显示
"""
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, 
    QLabel, QGroupBox
)
from PyQt5.QtCore import Qt, pyqtSlot
from ..base_component import BaseComponent, default_font

class DataRecordingWidget(BaseComponent):
    """数据录制控制组件"""
    
    def __init__(self, parent=None, view_model=None):
        super().__init__(parent, view_model)
        
    def setup_ui(self):
        """初始化UI布局 (BaseComponent 自动调用)"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)
        
        # 控制面板
        control_group = QGroupBox("数据录制控制")
        control_group.setFont(default_font)
        control_layout = QVBoxLayout(control_group)
        control_layout.setSpacing(15)
        
        # 状态显示区域
        status_layout = QVBoxLayout()
        
        # 状态标签
        self.status_label = QLabel("状态: 未录制")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setFont(default_font)
        status_layout.addWidget(self.status_label)
        
        # 详细信息标签
        self.info_label = QLabel("等待指令...")
        self.info_label.setAlignment(Qt.AlignCenter)
        self.info_label.setWordWrap(True)
        self.info_label.setFont(default_font)
        status_layout.addWidget(self.info_label)
        
        control_layout.addLayout(status_layout)
        
        # 按钮容器
        btn_layout = QHBoxLayout()
        
        # 录制按钮
        self.record_btn = QPushButton("开始录制")
        self.record_btn.setFont(default_font)
        self.record_btn.setMinimumHeight(40)
        btn_layout.addWidget(self.record_btn)
        
        control_layout.addLayout(btn_layout)
        layout.addWidget(control_group)
        
        # 说明区域
        note_group = QGroupBox("使用说明")
        note_group.setFont(default_font)
        note_layout = QVBoxLayout(note_group)
        
        note_label = QLabel(
            "1. 请确保后台已运行录制节点：\n"
            "   ros2 run record record\n\n"
            "2. 录制数据将自动保存至：\n"
            "   ./record_data/record_n/\n\n"
            "3. 录制内容包括：\n"
            "   - video.mp4 (视频流)\n"
            "   - data.csv (机械臂状态)\n\n"
            "4. 录制期间将自动开启 ROS 状态广播"
        )
        note_label.setFont(default_font)
        note_label.setWordWrap(True)
        note_layout.addWidget(note_label)
        
        layout.addWidget(note_group)
        
        layout.addStretch()
        
    def connect_signals(self):
        """连接信号与槽"""
        # UI -> ViewModel
        self.record_btn.clicked.connect(self._on_record_btn_clicked)
        
        # ViewModel -> UI
        self.view_model.status_changed.connect(self._on_status_changed)
        
    @pyqtSlot()
    def _on_record_btn_clicked(self):
        # 禁用按钮防止重复点击，等待ViewModel回调恢复
        self.record_btn.setEnabled(False) 
        self.view_model.toggle_recording()
            
    @pyqtSlot(bool, str)
    def _on_status_changed(self, is_recording, message):
        self.record_btn.setEnabled(True)
        self.info_label.setText(message)
        
        if is_recording:
            self.status_label.setText("状态: 🔴 正在录制")
            self.record_btn.setText("停止录制")
            self.record_btn.setStyleSheet("background-color: #ffcccc;") 
        else:
            self.status_label.setText("状态: 未录制")
            self.record_btn.setText("开始录制")
            self.record_btn.setStyleSheet("")
