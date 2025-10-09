"""
串口领域服务 - Domain层
提供串口连接、读写操作的核心业务逻辑
可被其他Domain服务复用
"""
from typing import List, Dict, Any, Optional
from PyQt5.QtCore import QThread, QObject, pyqtSignal
from infrastructure import PortScanner, SerialAdapter, SerialReader, SerialWriter


class SerialReaderThread(QThread):
    """串口读取线程"""
    
    def __init__(self, serial_reader):
        super().__init__()
        self.serial_reader = serial_reader
    
    def run(self):
        """运行读取循环"""
        if self.serial_reader:
            self.serial_reader.read_data()


class SerialWriterThread(QThread):
    """串口写入线程"""
    
    def __init__(self, serial_writer):
        super().__init__()
        self.serial_writer = serial_writer
    
    def run(self):
        """运行写入循环"""
        if self.serial_writer:
            self.serial_writer.send_data()


class SerialDomainService(QObject):
    """串口领域服务 - 管理串口连接的核心业务逻辑"""
    
    # 领域事件信号
    data_received = pyqtSignal(str)
    connection_status_changed = pyqtSignal(bool)
    
    def __init__(self):
        """初始化串口领域服务"""
        super().__init__()
        
        # 串口相关对象
        self.serial_adapter = SerialAdapter()
        self.serial_reader: Optional[SerialReader] = None
        self.serial_writer: Optional[SerialWriter] = None
        
        # 线程管理
        self.reader_thread: Optional[SerialReaderThread] = None
        self.writer_thread: Optional[SerialWriterThread] = None
    
    def scan_available_ports(self) -> List[str]:
        """
        扫描可用端口列表
        
        返回:
            List[str]: 可用端口名称列表
        """
        try:
            return PortScanner.scan_ports()
        except Exception as e:
            raise Exception(f"扫描端口失败: {str(e)}")
    
    def get_port_info(self, port_name: str) -> Optional[Dict[str, str]]:
        """
        获取指定端口信息
        
        参数:
            port_name: 端口名称
            
        返回:
            Optional[Dict[str, str]]: 端口信息字典，失败时返回None
        """
        try:
            return PortScanner.get_port_info(port_name)
        except Exception as e:
            raise Exception(f"获取端口信息失败: {str(e)}")
    
    def get_current_port(self) -> Optional[str]:
        """
        获取当前连接的端口名称
        
        返回:
            Optional[str]: 端口名称，未连接时返回None
        """
        return self.serial_adapter.get_port_name()
    
    def is_connected(self) -> bool:
        """
        检查串口是否已连接
        
        返回:
            bool: 是否已连接
        """
        return self.serial_adapter.is_connected()
    
    def connect_port(self, port: str, config: Dict[str, Any]) -> bool:
        """
        连接串口
        
        参数:
            port: 串口名称
            config: 串口配置参数
            
        返回:
            bool: 连接是否成功
        """
        try:
            # 如果已连接，先断开
            if self.is_connected():
                self.disconnect_port()
            
            # 尝试连接串口
            if not self.serial_adapter.connect(port, config):
                raise Exception(f"连接端口 {port} 失败")
            
            # 获取串口对象
            serial_port = self.serial_adapter.get_serial_port()
            if not serial_port:
                raise Exception("获取串口对象失败")
            
            # 创建读写器
            self.serial_reader = SerialReader(serial_port)
            self.serial_writer = SerialWriter(serial_port)
            
            # 连接Infrastructure层信号到Domain层
            self._connect_infrastructure_signals()
            
            # 启动线程
            self._start_threads()
            
            # 发送连接成功信号
            self.connection_status_changed.emit(True)
            return True
            
        except Exception as e:
            self.disconnect_port()
            raise Exception(f"连接失败: {str(e)}")
    
    def disconnect_port(self) -> bool:
        """
        断开串口连接
        
        返回:
            bool: 断开是否成功
        """
        try:
            # 停止线程
            self._stop_threads()
            
            # 断开信号连接
            self._disconnect_infrastructure_signals()
            
            # 停止读写器
            if self.serial_reader:
                self.serial_reader.stop()
                self.serial_reader = None
            
            if self.serial_writer:
                self.serial_writer.stop()
                self.serial_writer = None
            
            # 断开适配器连接
            success = self.serial_adapter.disconnect()
            
            # 发送断开连接信号
            self.connection_status_changed.emit(False)
            return success
            
        except Exception as e:
            raise Exception(f"断开连接失败: {str(e)}")
    
    def send_data(self, data: str) -> bool:
        """
        发送数据
        
        参数:
            data: 要发送的数据
            
        返回:
            bool: 是否成功添加到发送队列
        """
        if self.serial_writer and self.is_connected():
            self.serial_writer.add_to_queue(data)
            return True
        else:
            raise Exception("串口未连接，无法发送数据")
    
    def _connect_infrastructure_signals(self) -> None:
        """连接Infrastructure层信号到Domain层"""
        if self.serial_reader:
            self.serial_reader.data_received.connect(self._on_data_received)
    
    def _disconnect_infrastructure_signals(self) -> None:
        """断开Infrastructure层信号连接"""
        if self.serial_reader:
            try:
                self.serial_reader.data_received.disconnect(self._on_data_received)
            except TypeError:
                # 如果信号未连接，disconnect会抛出TypeError，忽略即可
                pass
    
    def _on_data_received(self, data: str) -> None:
        """处理Infrastructure层的数据接收，转发到Domain层信号"""
        self.data_received.emit(data)
    
    def _start_threads(self) -> None:
        """启动读写线程"""
        # 启动读取线程
        if self.serial_reader:
            self.reader_thread = SerialReaderThread(self.serial_reader)
            self.reader_thread.start()
        
        # 启动写入线程
        if self.serial_writer:
            self.writer_thread = SerialWriterThread(self.serial_writer)
            self.writer_thread.start()
    
    def _stop_threads(self) -> None:
        """停止读写线程"""
        # 停止读取线程
        if self.reader_thread and self.reader_thread.isRunning():
            if self.serial_reader:
                self.serial_reader.stop()
            self.reader_thread.quit()
            self.reader_thread.wait(3000)  # 等待3秒
            self.reader_thread = None
        
        # 停止写入线程
        if self.writer_thread and self.writer_thread.isRunning():
            if self.serial_writer:
                self.serial_writer.stop()
            self.writer_thread.quit()
            self.writer_thread.wait(3000)  # 等待3秒
            self.writer_thread = None

    def cleanup(self) -> None:
        """清理资源"""
        try:
            self.disconnect_port()
        except Exception:
            pass  # 忽略清理时的异常
