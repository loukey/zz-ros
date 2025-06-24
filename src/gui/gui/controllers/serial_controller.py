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
        self.send_thread = QThread()
    
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
            # 将read_handler和send_handler移动到各自的线程
            self.serial_model.read_handler.moveToThread(self.read_thread)
            self.serial_model.send_handler.moveToThread(self.send_thread)
            
            # 连接线程启动信号到对应的方法
            self.read_thread.started.connect(self.serial_model.read_handler.read_data)
            self.send_thread.started.connect(self.serial_model.send_handler.send_data)
            
            # 启动线程
            self.read_thread.start()
            self.send_thread.start()
            
            self.display(f"已连接到串口 {port}", "串口")
            self.connection_changed.emit(True)
        else:
            self.display("连接串口失败", "错误")
            self.connection_changed.emit(False)
    
    def disconnect(self):
        """断开串口连接"""
        # 停止读取和发送
        if self.serial_model.read_handler:
            self.serial_model.read_handler.stop()
        if self.serial_model.send_handler:
            self.serial_model.send_handler.stop()
            
        # 等待线程结束
        if self.read_thread.isRunning():
            self.read_thread.quit()
            self.read_thread.wait()
        if self.send_thread.isRunning():
            self.send_thread.quit()
            self.send_thread.wait()
        
        # 断开信号连接，避免重复连接
        try:
            self.read_thread.started.disconnect()
            self.send_thread.started.disconnect()
        except:
            pass
        
        # 断开串口连接
        success = self.serial_model.disconnect()
        if success:
            self.display("已断开串口连接", "串口")
            self.connection_changed.emit(False)
        else:
            self.display("断开串口失败", "错误")
    
