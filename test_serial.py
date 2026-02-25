"""跨平台串口：绕过 pyserial，直接用 OS 级非阻塞 I/O"""
import sys
import time
import threading


# ============================================================
# 跨平台串口抽象
# ============================================================

if sys.platform == "win32":
    import ctypes
    from ctypes import wintypes

    kernel32 = ctypes.windll.kernel32
    GENERIC_RW = 0xC0000000
    OPEN_EXISTING = 3
    FILE_FLAG_OVERLAPPED = 0x40000000
    ERROR_IO_PENDING = 997
    WAIT_OBJECT_0 = 0

    class OVERLAPPED(ctypes.Structure):
        _fields_ = [
            ("Internal", ctypes.POINTER(ctypes.c_ulong)),
            ("InternalHigh", ctypes.POINTER(ctypes.c_ulong)),
            ("Offset", wintypes.DWORD), ("OffsetHigh", wintypes.DWORD),
            ("hEvent", wintypes.HANDLE),
        ]

    class DCB(ctypes.Structure):
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

    class COMMTIMEOUTS(ctypes.Structure):
        _fields_ = [
            ("ReadIntervalTimeout", wintypes.DWORD),
            ("ReadTotalTimeoutMultiplier", wintypes.DWORD),
            ("ReadTotalTimeoutConstant", wintypes.DWORD),
            ("WriteTotalTimeoutMultiplier", wintypes.DWORD),
            ("WriteTotalTimeoutConstant", wintypes.DWORD),
        ]

    class NativeSerialPort:
        """Windows: FILE_FLAG_OVERLAPPED + WaitForSingleObject"""

        def __init__(self, port: str, baudrate: int = 115200):
            self.port = port
            self.is_open = False
            self._handle = kernel32.CreateFileW(
                f"\\\\.\\{port}", GENERIC_RW, 0, None, OPEN_EXISTING,
                FILE_FLAG_OVERLAPPED, None)
            if self._handle == -1:
                raise OSError(f"无法打开 {port}: err={kernel32.GetLastError()}")

            dcb = DCB()
            dcb.DCBlength = ctypes.sizeof(DCB)
            kernel32.GetCommState(self._handle, ctypes.byref(dcb))
            dcb.BaudRate = baudrate
            dcb.ByteSize = 8
            dcb.Parity = 0
            dcb.StopBits = 0
            dcb.flags = 0x01 | (0x01 << 4)  # fBinary + DTR_CONTROL_ENABLE
            kernel32.SetCommState(self._handle, ctypes.byref(dcb))

            timeouts = COMMTIMEOUTS(0xFFFFFFFF, 0, 0, 0, 1000)
            kernel32.SetCommTimeouts(self._handle, ctypes.byref(timeouts))
            kernel32.EscapeCommFunction(self._handle, 5)  # SETDTR
            kernel32.PurgeComm(self._handle, 0xF)
            self.is_open = True

        def read(self, size: int = 1024, timeout_ms: int = 100) -> bytes:
            if not self.is_open:
                return b""
            ov = OVERLAPPED()
            ov.hEvent = kernel32.CreateEventW(None, True, False, None)
            buf = ctypes.create_string_buffer(size)
            n = wintypes.DWORD()
            r = kernel32.ReadFile(self._handle, buf, size, ctypes.byref(n), ctypes.byref(ov))
            if not r and kernel32.GetLastError() == ERROR_IO_PENDING:
                if kernel32.WaitForSingleObject(ov.hEvent, timeout_ms) == WAIT_OBJECT_0:
                    kernel32.GetOverlappedResult(
                        self._handle, ctypes.byref(ov), ctypes.byref(n), False)
                else:
                    kernel32.CancelIo(self._handle)
                    kernel32.CloseHandle(ov.hEvent)
                    return b""
            kernel32.CloseHandle(ov.hEvent)
            return buf.raw[:n.value]

        def write(self, data: bytes) -> int:
            if not self.is_open:
                return 0
            ov = OVERLAPPED()
            ov.hEvent = kernel32.CreateEventW(None, True, False, None)
            n = wintypes.DWORD()
            r = kernel32.WriteFile(
                self._handle, data, len(data), ctypes.byref(n), ctypes.byref(ov))
            if not r and kernel32.GetLastError() == ERROR_IO_PENDING:
                kernel32.WaitForSingleObject(ov.hEvent, 3000)
                kernel32.GetOverlappedResult(
                    self._handle, ctypes.byref(ov), ctypes.byref(n), False)
            kernel32.CloseHandle(ov.hEvent)
            return n.value

        def close(self):
            if self.is_open and self._handle and self._handle != -1:
                kernel32.CloseHandle(self._handle)
                self._handle = None
            self.is_open = False

else:
    # Linux / macOS: 直接 termios + select + os.read（完全绕过 pyserial）
    import os
    import select
    import termios
    import fcntl
    import struct

    class NativeSerialPort:
        """POSIX: 最小化 termios 配置 + select 非阻塞读"""

        BAUD_MAP = {
            9600: termios.B9600, 19200: termios.B19200,
            38400: termios.B38400, 57600: termios.B57600,
            115200: termios.B115200, 230400: termios.B230400,
            460800: termios.B460800, 921600: termios.B921600,
        }

        def __init__(self, port: str, baudrate: int = 115200):
            self.port = port
            self.baudrate = baudrate
            self.is_open = False
            self._fd = -1
            self._open()

        def _configure(self):
            """配置 termios: raw 8N1"""
            baud = self.BAUD_MAP.get(self.baudrate, termios.B115200)
            attrs = [0, 0, 0, 0, baud, baud, [0] * 32]
            # cflag: CREAD + CLOCAL + CS8
            attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL | baud
            # cc: VMIN=0, VTIME=1 (100ms 超时)
            attrs[6] = list(attrs[6])
            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 1
            termios.tcsetattr(self._fd, termios.TCSAFLUSH, attrs)
            # DTR
            try:
                fcntl.ioctl(self._fd, 0x5416, struct.pack('I', 0x002))
            except OSError:
                pass

        def _open(self):
            """打开串口"""
            if self._fd >= 0:
                try:
                    os.close(self._fd)
                except OSError:
                    pass
                self._fd = -1

            for attempt in range(10):
                try:
                    self._fd = os.open(
                        self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
                    break
                except OSError:
                    if attempt == 9:
                        raise
                    time.sleep(0.5)

            self._configure()
            self.is_open = True

        def reset(self):
            """不关闭 fd，只 flush + 重配 termios（轻量恢复）"""
            if self._fd < 0:
                return False
            try:
                termios.tcflush(self._fd, termios.TCIOFLUSH)
                self._configure()
                return True
            except OSError:
                return False

        def reopen(self):
            """完整关闭重开（重量恢复）"""
            try:
                self._open()
                return True
            except OSError:
                self.is_open = False
                return False

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


# ============================================================
# 测试
# ============================================================

PORT = "COM5" if sys.platform == "win32" else "/dev/ttyUSB0"
BAUD = 115200


def main():
    print(f"平台: {sys.platform}, 端口: {PORT}")
    print("请先按 STM32 reset，等 3 秒按回车")
    input()

    ser = NativeSerialPort(PORT, BAUD)
    stop = threading.Event()
    last_rx_time = [time.time()]
    last_tx_time = [0.0]

    def read_loop():
        silence_detected = False
        while not stop.is_set():
            data = ser.read(1024, timeout_ms=100)
            if data:
                last_rx_time[0] = time.time()
                silence_detected = False
                try:
                    text = data.decode("utf-8", errors="replace")
                    print(f"[RX] {text}", end="", flush=True)
                except Exception:
                    print(f"[RX hex] {data.hex().upper()}", flush=True)
            else:
                # Linux: 如果发送了数据但 2 秒内没有回应，重开 fd
                if (sys.platform != "win32"
                    and last_tx_time[0] > last_rx_time[0]
                    and time.time() - last_tx_time[0] > 2.0
                    and not silence_detected):
                    silence_detected = True
                    print("\n[!] 检测到通信中断，尝试恢复...")

                    # Linux: 先尝试轻量 reset（不关 fd）
                    recovered = False
                    if sys.platform != "win32":
                        ser.reset()
                        # 等 STM32 启动消息（最多 5 秒）
                        t0 = time.time()
                        while time.time() - t0 < 5:
                            d = ser.read(1024, timeout_ms=200)
                            if d and (b"Booted" in d or b"UART" in d or b"init" in d):
                                print(f"[OK] 轻量恢复成功")
                                recovered = True
                                break

                    # 如果轻量恢复失败，完整 close + reopen
                    if not recovered:
                        for attempt in range(3):
                            time.sleep(1)
                            if ser.reopen():
                                t0 = time.time()
                                while time.time() - t0 < 5:
                                    d = ser.read(1024, timeout_ms=200)
                                    if d and (b"Booted" in d or b"UART" in d or b"init" in d):
                                        print(f"[OK] 重开恢复成功")
                                        recovered = True
                                        break
                                if recovered:
                                    break
                            print(f"[.] 重试 {attempt+1}/3...")

                    if recovered:
                        ser.read(1024, timeout_ms=100)  # 清残余
                        last_rx_time[0] = time.time()
                    else:
                        print("[!] 恢复失败，继续重试...")
                    silence_detected = False

    reader = threading.Thread(target=read_loop, daemon=True)
    reader.start()

    time.sleep(1)
    print("\n输入文字回车发送 | 'q' 退出 | 随时可按 STM32 reset 测试")
    print("-" * 50)

    while True:
        try:
            cmd = input()
        except (EOFError, KeyboardInterrupt):
            break
        if cmd.strip().lower() == "q":
            break
        try:
            ser.write((cmd + "\r\n").encode("utf-8"))
            last_tx_time[0] = time.time()
        except Exception as e:
            print(f"[!] 发送失败: {e}")

    stop.set()
    ser.close()
    print("已退出")


if __name__ == "__main__":
    main()
