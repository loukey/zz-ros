"""
虚拟串口模拟器
使用虚拟串口对（socat创建）来模拟机械臂通信
"""
import os
import sys
import time
import serial
import threading
from message_builder import MessageBuilder
from robot_simulator import RobotSimulator


class VirtualSerialPort:
    """虚拟串口模拟器"""
    
    def __init__(self, port: str, baudrate: int = 115200):
        """
        初始化虚拟串口
        
        Args:
            port: 串口设备路径（例如：/tmp/ttyV1）
            baudrate: 波特率
        """
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.running = False
        
        # 机械臂模拟器和消息构建器
        self.robot = RobotSimulator()
        self.builder = MessageBuilder()
        
        # 统计信息
        self.received_count = 0
        self.sent_count = 0
    
    def connect(self) -> bool:
        """连接虚拟串口"""
        try:
            # 检查串口是否存在
            if not os.path.exists(self.port):
                print(f"❌ 串口设备不存在: {self.port}")
                print(f"💡 请先使用 socat 创建虚拟串口对：")
                print(f"   socat -d -d pty,raw,echo=0,link=/tmp/ttyV0 pty,raw,echo=0,link=/tmp/ttyV1")
                return False
            
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            
            print(f"✅ 虚拟串口已连接: {self.port}")
            print(f"   波特率: {self.baudrate}")
            return True
            
        except Exception as e:
            print(f"❌ 连接失败: {e}")
            return False
    
    def start(self):
        """启动虚拟串口监听"""
        if not self.connect():
            return
        
        self.running = True
        
        # 启动接收线程
        receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
        receive_thread.start()
        
        print(f"\n🤖 虚拟机械臂已启动")
        print(f"📡 正在监听命令...\n")
        
        try:
            # 主线程等待
            while self.running:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n\n⚠️  收到停止信号")
            self.stop()
    
    def _receive_loop(self):
        """接收命令循环"""
        buffer = b""
        
        while self.running:
            try:
                # 读取数据
                if self.serial_conn and self.serial_conn.in_waiting > 0:
                    data = self.serial_conn.read(self.serial_conn.in_waiting)
                    buffer += data
                    
                    # 查找完整消息（以0D0A结尾）
                    while b'\x0D\x0A' in buffer:
                        # 找到消息结束位置
                        end_idx = buffer.find(b'\x0D\x0A') + 2
                        message = buffer[:end_idx]
                        buffer = buffer[end_idx:]
                        
                        # 处理消息
                        self._handle_command(message)
                
                time.sleep(0.01)  # 避免CPU占用过高
                
            except Exception as e:
                if self.running:
                    print(f"⚠️  接收数据出错: {e}")
                time.sleep(0.1)
    
    def _handle_command(self, command: bytes):
        """处理接收到的命令"""
        try:
            # 转换为16进制字符串
            command_hex = command.hex().upper()
            
            self.received_count += 1
            print(f"\n📥 [{self.received_count}] 接收命令: {command_hex[:50]}{'...' if len(command_hex) > 50 else ''}")
            
            # 解析命令
            if command_hex.startswith("AA55"):
                control = int(command_hex[4:6], 16)
                mode = int(command_hex[6:8], 16)
                
                control_names = {
                    0x00: "夹爪控制",
                    0x01: "初始化",
                    0x02: "停止",
                    0x06: "运动控制"
                }
                
                print(f"   命令类型: {control_names.get(control, f'未知(0x{control:02X})')}")
                print(f"   运行模式: 0x{mode:02X}")
            
            # 机械臂处理命令
            state = self.robot.process_command(command_hex)
            
            # 构建响应消息
            response_hex = self.builder.build_message_in(**state)
            
            # 发送响应
            self._send_response(response_hex)
            
        except Exception as e:
            print(f"⚠️  处理命令出错: {e}")
            import traceback
            traceback.print_exc()
    
    def _send_response(self, response_hex: str):
        """发送响应消息"""
        try:
            response_bytes = bytes.fromhex(response_hex)
            
            if self.serial_conn:
                self.serial_conn.write(response_bytes)
                self.serial_conn.flush()
                
                self.sent_count += 1
                print(f"📤 [{self.sent_count}] 发送响应: {response_hex[:50]}{'...' if len(response_hex) > 50 else ''}")
                
                # 解析并显示位置信息
                self._display_position_info(response_hex)
                
        except Exception as e:
            print(f"⚠️  发送响应出错: {e}")
    
    def _display_position_info(self, response_hex: str):
        """显示位置信息（弧度）"""
        try:
            # positions 位置：AA55(4) + init_status(2) + control(2) + mode(2) = 10
            start_idx = 10
            end_idx = start_idx + 48  # 6个关节 * 4字节 * 2字符
            
            if len(response_hex) < end_idx:
                return
            
            positions_hex = response_hex[start_idx:end_idx]
            
            # 解析位置值
            positions = []
            for i in range(6):
                joint_hex = positions_hex[i*8:(i+1)*8]
                position = int.from_bytes(bytes.fromhex(joint_hex), byteorder='big', signed=True)
                positions.append(position)
            
            # 转换为弧度（使用与RobotUtils相同的逻辑）
            from math import pi
            JOINT_OFFSETS = [78623, 369707, 83986, 391414, 508006, 456372]
            POS_TO_RADIAN_SCALE_FACTOR = (2 * pi) / (2**19)
            init_radians = [0, -pi/2, 0, pi/2, 0, 0]
            
            radians = []
            for i, pos in enumerate(positions):
                rad = init_radians[i] + (pos - JOINT_OFFSETS[i]) * POS_TO_RADIAN_SCALE_FACTOR
                radians.append(rad)
            
            angles_str = ", ".join([f"{rad:.4f}" for rad in radians])
            print(f"   当前位置(弧度): [{angles_str}]")
            
        except Exception as e:
            pass
    
    def stop(self):
        """停止虚拟串口"""
        self.running = False
        
        if self.serial_conn:
            try:
                self.serial_conn.close()
                print(f"✅ 虚拟串口已关闭")
            except Exception as e:
                print(f"⚠️  关闭串口出错: {e}")
        
        print(f"\n📊 统计信息:")
        print(f"   接收命令数: {self.received_count}")
        print(f"   发送响应数: {self.sent_count}")


def main():
    """主函数"""
    # 默认使用 /tmp/ttyV1 作为虚拟串口
    port = sys.argv[1] if len(sys.argv) > 1 else "/tmp/ttyV1"
    
    print("=" * 60)
    print("🤖 虚拟机械臂串口模拟器")
    print("=" * 60)
    print(f"\n💡 使用方法：")
    print(f"1. 创建虚拟串口对（在另一个终端）：")
    print(f"   socat -d -d pty,raw,echo=0,link=/tmp/ttyV0 pty,raw,echo=0,link=/tmp/ttyV1")
    print(f"\n2. 在controller中配置串口为: /tmp/ttyV0")
    print(f"3. 启动此模拟器监听: /tmp/ttyV1")
    print(f"\n按 Ctrl+C 停止\n")
    print("=" * 60 + "\n")
    
    virtual_port = VirtualSerialPort(port)
    virtual_port.start()


if __name__ == "__main__":
    main()

