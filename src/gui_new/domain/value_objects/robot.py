from math import pi


class Robot:
    def __init__(self):
        self.JOINT_OFFSETS = [78623, 369707, 83986, 391414, 508006, 456372]
        self.RADIAN_TO_POS_SCALE_FACTOR = (2**19) / (2 * pi)  # 弧度转位置值的系数
        self.POS_TO_RADIAN_SCALE_FACTOR = (2 * pi) / (2**19)  # 位置值转弧度的系数
        self.init_radians = [0, -pi/2, 0, pi/2, 0, 0]

    @staticmethod
    def calculate_crc16(data):
        """
        计算CRC16校验和
        
        参数:
            data: 字节数组或字符串
            
        返回:
            crc16: 16位校验和
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
    
    def position2radian(self, position):
        radians = self.init_radians.copy()
        for i, pos in enumerate(position):
            if isinstance(pos, str):
                pos = int(pos)
            radians[i] += (pos - self.JOINT_OFFSETS[i]) * self.POS_TO_RADIAN_SCALE_FACTOR
        return radians
    
    def radian2position(self, radian):
        positions = []
        for i, rad in enumerate(radian):
            position = int((rad - self.init_radians[i]) * self.RADIAN_TO_POS_SCALE_FACTOR + self.JOINT_OFFSETS[i]) & 0xFFFFFFFF
            positions.append(position)
        return positions

    def speed2position(self, speed):
        return [int(position * self.RADIAN_TO_POS_SCALE_FACTOR) & 0xFFFFFF for position in speed]

    @staticmethod
    def torque_transfer(torque):
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
    def effector2hex(effector_data):
        effector_data = str(float(effector_data))
        arr = effector_data.split('.')
        return [int(arr[0]).to_bytes(2, 'big').hex(), int(arr[1]).to_bytes(2, 'big').hex()]