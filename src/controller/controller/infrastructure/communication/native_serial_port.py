"""
跨平台原生串口 - Infrastructure层
绕过 pyserial 的阻塞读写，直接用 OS 级非阻塞 I/O。

Windows: Win32 FILE_FLAG_OVERLAPPED + WaitForSingleObject
Linux:   termios + select + os.read/os.write

解决 pyserial 在 STM32 reset 时句柄失效的问题。
"""
import sys
import time
from typing import Optional, Dict, Any


if sys.platform == "win32":
    import ctypes
    from ctypes import wintypes

    kernel32 = ctypes.windll.kernel32

    _GENERIC_RW = 0xC0000000
    _OPEN_EXISTING = 3
    _FILE_FLAG_OVERLAPPED = 0x40000000
    _ERROR_IO_PENDING = 997
    _WAIT_OBJECT_0 = 0
    _SETDTR = 5
    _CLRDTR = 6

    class _OVERLAPPED(ctypes.Structure):
        _fields_ = [
            ("Internal", ctypes.POINTER(ctypes.c_ulong)),
            ("InternalHigh", ctypes.POINTER(ctypes.c_ulong)),
            ("Offset", wintypes.DWORD),
            ("OffsetHigh", wintypes.DWORD),
            ("hEvent", wintypes.HANDLE),
        ]

    class _DCB(ctypes.Structure):
        _fields_ = [
            ("DCBlength", wintypes.DWORD), ("BaudRate", wintypes.DWORD),
            ("flags", wintypes.DWORD), ("wReserved", wintypes.WORD),
            ("XonLim", wintypes.WORD), ("XoffLim", wintypes.WORD),
            ("ByteSize", wintypes.BYTE), ("Parity", wintypes.BYTE),
            ("StopBits", wintypes.BYTE), ("XonChar", ctypes.c_char),
            ("XoffChar", ctypes.c_char), ("ErrorChar", ctypes.c_char),
            ("EofChar", ctypes.c_char), ("EvtChar", ctypes.c_char),
            ("wReserved1", wintypes.WORD),
        ]

    class _COMMTIMEOUTS(ctypes.Structure):
        _fields_ = [
            ("ReadIntervalTimeout", wintypes.DWORD),
            ("ReadTotalTimeoutMultiplier", wintypes.DWORD),
            ("ReadTotalTimeoutConstant", wintypes.DWORD),
            ("WriteTotalTimeoutMultiplier", wintypes.DWORD),
            ("WriteTotalTimeoutConstant", wintypes.DWORD),
        ]

    class NativeSerialPort:
        """Windows 原生串口：FILE_FLAG_OVERLAPPED + WaitForSingleObject"""

        def __init__(self):
            self._handle = None
            self.is_open = False
            self.port: Optional[str] = None

        def open(self, port: str, config: Dict[str, Any]) -> bool:
            self.close()
            try:
                name = f"\\\\.\\{port}" if not port.startswith("\\\\.\\") else port
                self._handle = kernel32.CreateFileW(
                    name, _GENERIC_RW, 0, None, _OPEN_EXISTING,
                    _FILE_FLAG_OVERLAPPED, None)
                if self._handle == -1:
                    self._handle = None
                    return False

                dcb = _DCB()
                dcb.DCBlength = ctypes.sizeof(_DCB)
                kernel32.GetCommState(self._handle, ctypes.byref(dcb))
                dcb.BaudRate = config.get('baudrate', 115200)
                dcb.ByteSize = config.get('bytesize', 8)
                dcb.Parity = {'N': 0, 'E': 2, 'O': 1}.get(
                    config.get('parity', 'N'), 0)
                dcb.StopBits = {1: 0, 1.5: 1, 2: 2}.get(
                    config.get('stopbits', 1), 0)
                # fBinary=1, fDtrControl=DTR_CONTROL_ENABLE
                dcb.flags = 0x01 | (0x01 << 4)
                kernel32.SetCommState(self._handle, ctypes.byref(dcb))

                # ReadIntervalTimeout=MAXDWORD: 非阻塞，立即返回可用数据
                timeouts = _COMMTIMEOUTS(0xFFFFFFFF, 0, 0, 0, 1000)
                kernel32.SetCommTimeouts(
                    self._handle, ctypes.byref(timeouts))

                kernel32.EscapeCommFunction(self._handle, _SETDTR)
                kernel32.PurgeComm(self._handle, 0xF)

                self.port = port
                self.is_open = True
                return True
            except Exception:
                self.close()
                return False

        def read(self, size: int = 1024, timeout_ms: int = 100) -> bytes:
            if not self.is_open or not self._handle:
                return b""
            # 清除 comm 错误标志（STM32 reset 后必须调用，否则 ReadFile 永久失败）
            errors = wintypes.DWORD()
            kernel32.ClearCommError(self._handle, ctypes.byref(errors), None)
            ov = _OVERLAPPED()
            ov.hEvent = kernel32.CreateEventW(None, True, False, None)
            buf = ctypes.create_string_buffer(size)
            n = wintypes.DWORD()
            try:
                r = kernel32.ReadFile(
                    self._handle, buf, size,
                    ctypes.byref(n), ctypes.byref(ov))
                if not r and kernel32.GetLastError() == _ERROR_IO_PENDING:
                    if kernel32.WaitForSingleObject(
                            ov.hEvent, timeout_ms) == _WAIT_OBJECT_0:
                        kernel32.GetOverlappedResult(
                            self._handle, ctypes.byref(ov),
                            ctypes.byref(n), False)
                    else:
                        kernel32.CancelIo(self._handle)
                        return b""
                return buf.raw[:n.value]
            finally:
                kernel32.CloseHandle(ov.hEvent)

        def write(self, data: bytes) -> int:
            if not self.is_open or not self._handle:
                return 0
            ov = _OVERLAPPED()
            ov.hEvent = kernel32.CreateEventW(None, True, False, None)
            n = wintypes.DWORD()
            try:
                r = kernel32.WriteFile(
                    self._handle, data, len(data),
                    ctypes.byref(n), ctypes.byref(ov))
                if not r and kernel32.GetLastError() == _ERROR_IO_PENDING:
                    kernel32.WaitForSingleObject(ov.hEvent, 3000)
                    kernel32.GetOverlappedResult(
                        self._handle, ctypes.byref(ov),
                        ctypes.byref(n), False)
                return n.value
            finally:
                kernel32.CloseHandle(ov.hEvent)

        def close(self):
            if self._handle and self._handle != -1:
                kernel32.CloseHandle(self._handle)
            self._handle = None
            self.is_open = False
            self.port = None

else:
    # ============================================================
    # Linux / macOS
    # ============================================================
    import os
    import select
    import termios
    import fcntl
    import struct

    _BAUD_MAP = {
        9600: termios.B9600, 19200: termios.B19200,
        38400: termios.B38400, 57600: termios.B57600,
        115200: termios.B115200, 230400: termios.B230400,
        460800: termios.B460800, 921600: termios.B921600,
    }

    class NativeSerialPort:
        """POSIX 原生串口：termios + select + os.read/os.write"""

        def __init__(self):
            self._fd = -1
            self.is_open = False
            self.port: Optional[str] = None
            self._config: Optional[Dict[str, Any]] = None

        def open(self, port: str, config: Dict[str, Any]) -> bool:
            self.close()
            try:
                for attempt in range(10):
                    try:
                        self._fd = os.open(
                            port,
                            os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
                        break
                    except OSError:
                        if attempt == 9:
                            return False
                        time.sleep(0.5)

                self._config = config.copy()
                self._configure()
                self.port = port
                self.is_open = True
                return True
            except Exception:
                self.close()
                return False

        def _configure(self):
            baud = _BAUD_MAP.get(
                self._config.get('baudrate', 115200), termios.B115200)
            attrs = termios.tcgetattr(self._fd)
            attrs[0] = 0  # iflag: 无输入处理
            attrs[1] = 0  # oflag: 无输出处理
            attrs[2] = (termios.CS8 | termios.CREAD
                        | termios.CLOCAL | baud)  # cflag: 8N1
            attrs[3] = 0  # lflag: raw mode
            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 1  # 100ms
            attrs[4] = baud
            attrs[5] = baud
            termios.tcsetattr(self._fd, termios.TCSAFLUSH, attrs)
            # DTR
            try:
                fcntl.ioctl(
                    self._fd, 0x5416,
                    struct.pack('I', 0x002))  # TIOCMBIS, TIOCM_DTR
            except OSError:
                pass

        def reset(self) -> bool:
            """轻量恢复：flush + 重配 termios（不关 fd）"""
            if self._fd < 0:
                return False
            try:
                termios.tcflush(self._fd, termios.TCIOFLUSH)
                self._configure()
                return True
            except OSError:
                return False

        def reopen(self) -> bool:
            """重量恢复：close + open"""
            if not self.port or not self._config:
                return False
            port, config = self.port, self._config
            self.close()
            return self.open(port, config)

        def read(self, size: int = 1024, timeout_ms: int = 100) -> bytes:
            if not self.is_open or self._fd < 0:
                return b""
            try:
                ready, _, _ = select.select(
                    [self._fd], [], [], timeout_ms / 1000.0)
                if ready:
                    return os.read(self._fd, size)
                return b""
            except OSError:
                return b""

        def write(self, data: bytes) -> int:
            if not self.is_open or self._fd < 0:
                return 0
            try:
                return os.write(self._fd, data)
            except OSError:
                return 0

        def close(self):
            if self._fd >= 0:
                try:
                    os.close(self._fd)
                except OSError:
                    pass
                self._fd = -1
            self.is_open = False
