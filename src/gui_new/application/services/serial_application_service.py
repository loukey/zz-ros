"""
串口应用服务 - Application层
协调串口连接的业务逻辑，包括线程管理和信号管理
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


class SerialApplicationService(QObject):
    """串口应用服务 - 管理所有串口相关的业务逻辑"""
    

    data_received = pyqtSignal(str)
    connection_status_changed = pyqtSignal(bool)
    port_list_updated = pyqtSignal(list)
    message_display = pyqtSignal(str, str)
    
    def __init__(self):
        """初始化串口服务"""
        super().__init__()
        
        self.serial_adapter = SerialAdapter()
        self.serial_reader: Optional[SerialReader] = None
        self.serial_writer: Optional[SerialWriter] = None
        
        # 线程管理
        self.reader_thread: Optional[SerialReaderThread] = None
        self.writer_thread: Optional[SerialWriterThread] = None
    
    def _display_message(self, message: str, message_type: str) -> None:
        """统一的消息显示方法 - 发送信号到Presentation层"""
        # 发送消息显示信号，由DisplayViewModel统一处理
        self.message_display.emit(message, message_type)
    
    def refresh_ports(self) -> List[str]:
        """
        刷新可用端口列表
        
        返回:
            List[str]: 可用端口名称列表
        """
        try:
            ports = PortScanner.scan_ports()
            self.port_list_updated.emit(ports)
            self._display_message(f"发现 {len(ports)} 个可用端口", "系统")
            return ports
        except Exception as e:
            self._display_message(f"刷新端口失败: {str(e)}", "错误")
            return []
    
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
            self._display_message(f"获取端口信息失败: {str(e)}", "错误")
            return None
    
    def get_available_ports(self) -> List[str]:
        """
        获取当前可用端口列表（不刷新）
        
        返回:
            List[str]: 可用端口名称列表
        """
        try:
            return PortScanner.scan_ports()
        except Exception as e:
            self._display_message(f"获取端口列表失败: {str(e)}", "错误")
            return []
    
    def get_current_port(self) -> Optional[str]:
        """
        获取当前连接的端口名称
        
        返回:
            Optional[str]: 端口名称，未连接时返回None
        """
        return self.serial_adapter.get_port_name()
    
    def connect_serial(self, port: str, config: Dict[str, Any]) -> bool:
        """
        连接串口
        
        参数:
            port: 串口名称
            config: 串口配置参数
            
        返回:
            bool: 连接是否成功
        """
        self._display_message(f"连接串口: {port}, 配置: {config}", "系统")
        try:
            # 如果已连接，先断开
            if self.is_connected():
                self.disconnect_serial()
            
            # 尝试连接串口
            if not self.serial_adapter.connect(port, config):
                self._display_message(f"连接端口 {port} 失败", "错误")
                return False
            
            # 获取串口对象
            serial_port = self.serial_adapter.get_serial_port()
            if not serial_port:
                self._display_message("获取串口对象失败", "错误")
                return False
            
            # 创建读写器
            self.serial_reader = SerialReader(serial_port)
            self.serial_writer = SerialWriter(serial_port)
            
            # 连接Infrastructure层信号到Application层
            self._connect_infrastructure_signals()
            
            # 启动线程
            self._start_threads()
            
            # 发送连接成功信号
            self.connection_status_changed.emit(True)
            self._display_message(f"成功连接到端口 {port}", "系统")
            return True
            
        except Exception as e:
            self._display_message(f"连接失败: {str(e)}", "错误")
            self.disconnect_serial()
            return False
    
    def disconnect_serial(self) -> bool:
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
            if success:
                self._display_message("已断开串口连接", "系统")
            return success
            
        except Exception as e:
            self._display_message(f"断开连接失败: {str(e)}", "错误")
            return False
    
    def is_connected(self) -> bool:
        """
        检查串口是否已连接
        
        返回:
            bool: 是否已连接
        """
        return self.serial_adapter.is_connected()
    
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
            self._display_message("发送数据失败：串口未连接", "错误")
            return False
    
    def _connect_infrastructure_signals(self) -> None:
        """连接Infrastructure层信号到Application层"""
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
        """处理Infrastructure层的数据接收，转发到Application层信号"""
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