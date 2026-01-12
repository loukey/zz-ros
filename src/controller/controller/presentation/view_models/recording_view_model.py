"""
数据录制视图模型 - Presentation Layer
负责处理录制界面的逻辑，连接 View 和 Application Service
"""
from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot
from controller.application import DataRecordingApplicationService

class RecordingViewModel(QObject):
    """数据录制视图模型"""
    
    # 信号：通知View状态变化
    # is_recording: bool, message: str
    status_changed = pyqtSignal(bool, str)
    
    def __init__(self, recording_service: DataRecordingApplicationService):
        super().__init__()
        self._service = recording_service
        self._is_recording = False
        self._message = "准备就绪"
        
        # 连接 Service 信号
        self._service.recording_status_changed.connect(self._on_service_status_changed)
        
    @property
    def is_recording(self) -> bool:
        return self._service.is_recording()
        
    @pyqtSlot()
    def toggle_recording(self):
        """切换录制状态（供View调用）"""
        if self.is_recording:
            self._service.stop_recording()
            # 立即反馈给UI，防止重复点击
            # 实际状态会通过Service回调更新
            self.status_changed.emit(True, "正在停止...")
        else:
            self._service.start_recording()
            self.status_changed.emit(False, "正在启动...")
    
    @pyqtSlot(bool, str)
    def _on_service_status_changed(self, is_recording, message):
        """处理 Service 状态变化"""
        self._is_recording = is_recording
        self._message = message
        self.status_changed.emit(is_recording, message)
