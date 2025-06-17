class GlobalVars:
    # 示教模式标志位
    dynamic_teach_flag = False
    
    # 6维固定长度的数组（最大长度20）
    _fixed_array = []  # 每个元素是6维向量 [x1, x2, x3, x4, x5, x6]
    MAX_ARRAY_LENGTH = 20
    
    # 状态数组，长度为6，初始值都为0
    _state_array = [0, 0, 0, 0, 0, 0]
    
    # 摩擦力数组
    _friction_array = [4, 0, 3, 0.5, 0.5, 0.5]
    
    # 工作数组（用于存储计算结果）
    _work_array = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    @classmethod
    def set_dynamic_teach_flag(cls, flag):
        cls.dynamic_teach_flag = flag
    
    @classmethod
    def add_to_array(cls, six_dim_value):
        """
        向6维固定长度数组添加值
        参数: six_dim_value - 6维列表或元组 [x1, x2, x3, x4, x5, x6]
        """
        if not isinstance(six_dim_value, (list, tuple)) or len(six_dim_value) != 6:
            raise ValueError("输入值必须是6维数组")
        
        # 转换为列表确保类型一致
        value = list(six_dim_value)
        
        if len(cls._fixed_array) >= cls.MAX_ARRAY_LENGTH:
            # 移除第一个元素（最老的元素）
            cls._fixed_array.pop(0)
        
        # 添加新元素到末尾
        cls._fixed_array.append(value)
        
        # 如果数组满了，执行摩擦力控制逻辑
        if len(cls._fixed_array) == cls.MAX_ARRAY_LENGTH:
            cls._process_friction_control()
    
    @classmethod
    def _process_friction_control(cls):
        """处理摩擦力控制逻辑"""
        if len(cls._fixed_array) < cls.MAX_ARRAY_LENGTH:
            return
        
        # 获取最后一个值和前19个值
        latest_value = cls._fixed_array[-1]
        previous_19_values = cls._fixed_array[:-1]
        
        # 计算前19个值的平均数（每个维度分别计算）
        averages = []
        for dim in range(6):
            dim_sum = sum(value[dim] for value in previous_19_values)
            averages.append(dim_sum / 19)
        
        # 对每个维度进行处理
        for dim in range(6):
            difference = latest_value[dim] - averages[dim]
            
            if cls._state_array[dim] == 0:
                # 状态为0时的逻辑
                if difference > 20:
                    cls._work_array[dim] = cls._friction_array[dim]
                    cls._state_array[dim] = 1
                elif difference < -20:
                    cls._work_array[dim] = -cls._friction_array[dim]
                    cls._state_array[dim] = 1
            
            elif cls._state_array[dim] == 1:
                # 状态为1时的逻辑
                if abs(difference) < 5:
                    cls._work_array[dim] = 0
                    cls._state_array[dim] = 0
    
    @classmethod
    def get_array(cls):
        """获取当前6维数组"""
        return [value.copy() for value in cls._fixed_array]  # 返回深度副本
    
    @classmethod
    def get_array_length(cls):
        """获取当前数组长度"""
        return len(cls._fixed_array)
    
    @classmethod
    def clear_array(cls):
        """清空数组并重置状态"""
        cls._fixed_array.clear()
        cls._state_array = [0, 0, 0, 0, 0, 0]
        cls._work_array = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
    @classmethod
    def get_latest_value(cls):
        """获取最新添加的6维值"""
        return cls._fixed_array[-1].copy() if cls._fixed_array else None
    
    @classmethod
    def get_oldest_value(cls):
        """获取最老的6维值"""
        return cls._fixed_array[0].copy() if cls._fixed_array else None
    
    @classmethod
    def get_average_value(cls):
        """获取6维数组的平均值（每个维度分别计算）"""
        if not cls._fixed_array:
            return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        averages = []
        for dim in range(6):
            dim_sum = sum(value[dim] for value in cls._fixed_array)
            averages.append(dim_sum / len(cls._fixed_array))
        
        return averages
    
    @classmethod
    def get_state_array(cls):
        """获取状态数组"""
        return cls._state_array.copy()
    
    @classmethod
    def get_work_array(cls):
        """获取工作数组（摩擦力结果）"""
        return cls._work_array.copy()
    
    @classmethod
    def get_friction_array(cls):
        """获取摩擦力数组"""
        return cls._friction_array.copy()
    
    @classmethod
    def set_friction_array(cls, friction_values):
        """设置摩擦力数组"""
        if not isinstance(friction_values, (list, tuple)) or len(friction_values) != 6:
            raise ValueError("摩擦力数组必须是6维数组")
        cls._friction_array = list(friction_values)
    
    @classmethod
    def get_dimension_difference(cls, dim_index):
        """获取指定维度的最新值与前19个值平均数的差值"""
        if len(cls._fixed_array) < cls.MAX_ARRAY_LENGTH or dim_index < 0 or dim_index >= 6:
            return 0.0
        
        latest_value = cls._fixed_array[-1][dim_index]
        previous_19_values = cls._fixed_array[:-1]
        
        dim_sum = sum(value[dim_index] for value in previous_19_values)
        average = dim_sum / 19
        
        return latest_value - average
        
GlobalVars.__new__ = lambda cls:None
