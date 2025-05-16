"""
串口通信控制器
"""
from PyQt5.QtCore import QThread, pyqtSignal, pyqtSlot
from .base_controller import BaseController

class SerialController(BaseController):
    ports_updated = pyqtSignal(list)
    connection_changed = pyqtSignal(bool)

    def __init__(self, serial_model, motion_model):
        super().__init__()
        self.serial_model = serial_model
        self.motion_model = motion_model
        self.read_thread = QThread()
        self.serial_model.moveToThread(self.read_thread)
        self.read_thread.started.connect(self.serial_model.read_data)
    
    def refresh_ports(self):
        ports = self.serial_model.get_available_ports()
        self.ports_updated.emit(ports)

    def connect(self, 
                port, 
                config={
                    'baud_rate': 115200,
                    'data_bits': 8,
                    'parity': 'N',
                    'stop_bits': 1,
                    'flow_control': None
                }):
        config_str = f"波特率:{config['baud_rate']}, 数据位:{config['data_bits']}, 校验位:{config['parity']}, 停止位:{config['stop_bits']}, 流控制:{config['flow_control']}"
        self.display(config_str, "参数")
        success = self.serial_model.connect(
            port=port,
            baud_rate=config['baud_rate'],
            data_bits=config['data_bits'],
            parity=config['parity'],
            stop_bits=config['stop_bits'],
            flow_control=config['flow_control']
        )
        
        if success:
            # 启动读取线程
            self.read_thread.start()
            self.display(f"已连接到串口 {port}", "串口")
            self.connection_changed.emit(True)
        else:
            self.display("连接串口失败", "错误")
            self.connection_changed.emit(False)
    
    def disconnect(self):
        """断开串口连接"""
        if self.read_thread.isRunning():
            self.serial_model.stop()
            self.read_thread.quit()
            self.read_thread.wait()
        
        success = self.serial_model.disconnect()
        if success:
            self.display("已断开串口连接", "串口")
            self.connection_changed.emit(False)
        else:
            self.display("断开串口失败", "错误")
    
