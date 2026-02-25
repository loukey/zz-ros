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
    # Linux / macOS: pyserial 打开配置 + select/os.read 非阻塞读写
    import os
    import select
    import serial

    class NativeSerialPort:
        """POSIX: pyserial 打开配置 + select+os.read 非阻塞读（绕过 pyserial 的阻塞读）"""

        def __init__(self, port: str, baudrate: int = 115200):
            self.port = port
            self.baudrate = baudrate
            self.is_open = False
            self._ser = None
            self._fd = -1
            self._open()

        def _open(self):
            """用 pyserial 打开并配置，获取 fd 用于直接读写"""
            if self._ser:
                try:
                    self._ser.close()
                except Exception:
                    pass

            for attempt in range(10):
                try:
                    self._ser = serial.Serial(
                        self.port, self.baudrate,
                        timeout=0, write_timeout=1)
                    break
                except (serial.SerialException, OSError):
                    if attempt == 9:
                        raise
                    time.sleep(0.5)

            self._ser.dtr = True
            self._ser.rts = True
            self._ser.reset_input_buffer()
            self._fd = self._ser.fileno()
            self.is_open = True

        def reopen(self):
            """关闭后重开，重置驱动状态"""
            try:
                self._open()
                return True
            except (serial.SerialException, OSError):
                self.is_open = False
                return False

        def read(self, size: int = 1024, timeout_ms: int = 100) -> bytes:
            if not self.is_open or self._fd < 0:
                return b""
            try:
                ready, _, _ = select.select([self._fd], [], [], timeout_ms / 1000.0)
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
            if self._ser:
                try:
                    self._ser.close()
                except Exception:
                    pass
                self._ser = None
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
                    print("\n[!] 检测到通信中断，重新打开串口...")
                    # 先等 STM32 完成重启
                    time.sleep(2)
                    if ser.reopen():
                        # 等待启动消息，确认 STM32 就绪
                        print("[*] 等待 STM32 启动...")
                        boot_start = time.time()
                        ready = False
                        while time.time() - boot_start < 8:
                            d = ser.read(1024, timeout_ms=200)
                            if d:
                                try:
                                    txt = d.decode("utf-8", errors="replace")
                                    print(f"[BOOT] {txt}", end="", flush=True)
                                except Exception:
                                    pass
                                if b"Booted" in d or b"UART" in d:
                                    ready = True
                                    break
                        if ready:
                            ser.read(1024, timeout_ms=100)  # 清残余
                            print("\n[OK] STM32 已就绪，通信恢复")
                            last_rx_time[0] = time.time()
                        else:
                            print("\n[!] 未检测到启动消息，继续重试...")
                            silence_detected = False
                    else:
                        print("[!] 重开失败，2秒后重试...")
                        time.sleep(2)
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
