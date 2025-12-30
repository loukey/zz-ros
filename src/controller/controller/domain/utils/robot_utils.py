from math import pi


class RobotUtils:
    """机器人工具类。
    
    提供机器人相关的数值转换、校验和计算等工具方法。
    """
    
    def __init__(self):
        """初始化机器人参数。"""
        self.JOINT_OFFSETS = [79119, 369835, 83627, 392414, 507293, 456372]
        self.RADIAN_TO_POS_SCALE_FACTOR = (2**19) / (2 * pi)  # 弧度转位置值的系数
        self.POS_TO_RADIAN_SCALE_FACTOR = (2 * pi) / (2**19)  # 位置值转弧度的系数
        self.init_radians = [0, -pi/2, 0, pi/2, 0, 0]
    
    def position2radian(self, position: list) -> list:
        """编码器位置值转换为弧度。
        
        Args:
            position (list): 编码器位置值列表。
            
        Returns:
            list: 关节角度列表（弧度）。
        """
        radians = self.init_radians.copy()
        for i, pos in enumerate(position):
            if isinstance(pos, str):
                pos = int(pos)
            radians[i] += (pos - self.JOINT_OFFSETS[i]) * self.POS_TO_RADIAN_SCALE_FACTOR
        return radians
    
    def radian2position(self, radian: list) -> list:
        """弧度转换为编码器位置值。
        
        Args:
            radian (list): 关节角度列表（弧度）。
            
        Returns:
            list: 编码器位置值列表（32位整数）。
        """
        positions = []
        for i, rad in enumerate(radian):
            position = int((rad - self.init_radians[i]) * self.RADIAN_TO_POS_SCALE_FACTOR + self.JOINT_OFFSETS[i]) & 0xFFFFFFFF
            positions.append(position)
        return positions

    def speed2position(self, speed: list) -> list:
        """速度值转换为位置增量值。
        
        Args:
            speed (list): 速度列表。
            
        Returns:
            list: 对应的时间步长内的位置增量值。
        """
        return [int(position * self.RADIAN_TO_POS_SCALE_FACTOR) & 0xFFFFFF for position in speed]

    @staticmethod
    def torque_transfer(torque: list) -> list:
        """力矩值转换与归一化。
        
        Args:
            torque (list): 力矩值列表，长度不超过6。
            
        Returns:
            list: 转换后的力矩值列表（16位整数）。
            
        Raises:
            ValueError: 如果力矩列表长度超过6。
        """
        if len(torque) > 6:
            raise ValueError("力矩值列表长度不能超过6")
        transfer_torque = []
        for t in torque[:3]:
            t = min(max(t, -50), 50)
            transfer_torque.append(int(t / 87 * 1000))
        for t in torque[3:]:
            t = min(max(t, -10), 10)
            transfer_torque.append(int(t * 100))
        return [int(t) & 0xFFFF for t in transfer_torque]

    @staticmethod
    def effector2hex(effector_data: float) -> list:
        """末端执行器数据转换为十六进制字节列表。
        
        将浮点数拆分为整数部分和小数部分，分别转换为 2 字节的大端序十六进制。
        
        Args:
            effector_data (float): 末端执行器数据。
            
        Returns:
            list: 包含两个十六进制字符串的列表 [整数部分hex, 小数部分hex]。
        """
        effector_data = str(float(effector_data))
        arr = effector_data.split('.')
        return [int(arr[0]).to_bytes(2, 'big').hex(), int(arr[1]).to_bytes(2, 'big').hex()]

    @staticmethod
    def calculate_crc16(data) -> int:
        """计算 CRC16 校验和 (CCITT)。
        
        Args:
            data (Union[bytes, str]): 字节数组或十六进制字符串。
            
        Returns:
            int: 16位校验和。如果输入格式错误返回 False。
        """
        if isinstance(data, str):
            try:
                data = bytes.fromhex(data.strip())
            except Exception as e:
                return False
                
        # 计算CRC16-CCITT
        crc = 0xFFFF
        for byte in data:
            crc ^= (int(byte) << 8)
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc = crc << 1
                crc &= 0xFFFF
                
        return crc
    