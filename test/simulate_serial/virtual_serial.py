"""
虚拟串口模块 - 支持十六进制数据收发
- 创建虚拟串口对
- 支持十六进制数据的收发
- 支持文本格式数据的收发
"""
import os
import pty
import threading
import time
import logging
import select

class VirtualSerial:
    """虚拟串口类 - 支持十六进制和文本格式数据处理"""
    
    def __init__(self):
        """初始化虚拟串口"""
        self.master_fd = None
        self.slave_fd = None
        self.slave_name = None
        self.running = False
        self.read_thread = None
        self.callback = None
        self.hex_mode = False  # 是否使用十六进制模式
        
        # 设置日志记录器
        self.logger = self._setup_logger()
    
    def _setup_logger(self):
        """设置日志记录器"""
        logger = logging.getLogger("VirtualSerial")
        handler = logging.StreamHandler()
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        handler.setFormatter(formatter)
        logger.addHandler(handler)
        logger.setLevel(logging.DEBUG)
        return logger
    
    def open(self):
        """打开虚拟串口
        
        返回:
            (bool, str): 成功状态和设备名称或错误信息
        """
        try:
            # 创建pty对
            self.master_fd, self.slave_fd = pty.openpty()
            self.slave_name = os.ttyname(self.slave_fd)
            self.running = True
            
            # 启动读取线程
            self.read_thread = threading.Thread(target=self._read_loop)
            self.read_thread.daemon = True
            self.read_thread.start()
            
            self.logger.info(f"虚拟串口创建成功，设备名: {self.slave_name}")
            return True, self.slave_name
        except Exception as e:
            self.logger.error(f"创建虚拟串口失败: {str(e)}")
            return False, str(e)
    
    def close(self):
        """关闭虚拟串口"""
        self.running = False
        
        # 等待读取线程结束
        if self.read_thread and self.read_thread.is_alive():
            self.read_thread.join(1.0)
        
        # 关闭文件描述符
        if self.master_fd:
            os.close(self.master_fd)
            self.master_fd = None
            
        if self.slave_fd:
            os.close(self.slave_fd)
            self.slave_fd = None
            
        self.slave_name = None
        self.logger.info("虚拟串口已关闭")
    
    def set_hex_mode(self, enabled):
        """设置是否使用十六进制模式
        
        参数:
            enabled: 是否启用十六进制模式
        """
        self.hex_mode = enabled
        self.logger.info(f"十六进制模式: {'启用' if enabled else '禁用'}")
    
    def _read_loop(self):
        """读取数据循环 - 支持十六进制和文本数据"""
        buffer_size = 1024
        # 缓存未完成行的数据
        incomplete_line = bytearray()
        
        while self.running and self.master_fd:
            try:
                # 从主设备读取数据
                r, w, e = select.select([self.master_fd], [], [], 0.1)
                if self.master_fd in r:
                    data = os.read(self.master_fd, buffer_size)
                    if data:
                        if self.hex_mode:
                            # 十六进制模式：直接处理原始数据
                            if self.callback:
                                self.callback(data)
                        else:
                            # 文本模式：按行处理
                            incomplete_line.extend(data)
                            
                            # 检查是否有完整行 (\r\n)
                            while b'\r\n' in incomplete_line:
                                # 分割一个完整行和剩余部分
                                line, incomplete_line = incomplete_line.split(b'\r\n', 1)
                                
                                # 处理完整行
                                self._process_complete_line(line + b'\r\n')  # 添加回行尾
                
            except Exception as e:
                self.logger.error(f"读取数据错误: {str(e)}")
                break
                
            time.sleep(0.01)
        
        self.logger.info("读取线程结束")
    
    def _process_complete_line(self, line_data):
        """处理一个完整的文本行
        
        参数:
            line_data: 包含\r\n的完整行数据(二进制)
        """
        # 强制解码为UTF-8文本，出错时替换为问号
        text_line = line_data.decode('utf-8', errors='replace')
        
        # 记录接收到的文本行
        self.logger.debug(f"接收到文本行: '{text_line.strip()}'")
        
        # 如果设置了回调函数，调用它
        if self.callback:
            self.callback(line_data)
    
    def write(self, data):
        """向虚拟串口写入数据
        
        参数:
            data: 要写入的数据 (字符串、字节或十六进制字符串)
            
        返回:
            bool: 是否成功
        """
        if not self.running or self.master_fd is None:
            self.logger.error("虚拟串口未打开")
            return False
        
        try:
            if self.hex_mode:
                # 十六进制模式
                if isinstance(data, str):
                    # 如果是十六进制字符串，转换为字节
                    try:
                        # 移除所有空格
                        hex_str = data.replace(" ", "")
                        # 确保字符串长度为偶数
                        if len(hex_str) % 2 != 0:
                            hex_str = "0" + hex_str
                        # 转换为字节
                        data = bytes.fromhex(hex_str)
                    except ValueError as e:
                        self.logger.error(f"无效的十六进制字符串: {str(e)}")
                        return False
                
                # 记录十六进制数据
                self.logger.debug(f"发送十六进制数据: {data.hex().upper()}")
            else:
                # 文本模式
                if isinstance(data, str):
                    self.logger.debug(f"发送文本: '{data}'")
                    data = data.encode('utf-8')
                else:
                    self.logger.debug(f"发送二进制数据: {len(data)}字节")
            
            # 写入数据到主设备
            os.write(self.master_fd, data)
            return True
            
        except Exception as e:
            self.logger.error(f"写入数据错误: {str(e)}")
            return False
    
    def set_callback(self, callback):
        """设置数据接收回调函数
        
        参数:
            callback: 回调函数，接收一个参数(接收到的数据)
        """
        self.callback = callback 