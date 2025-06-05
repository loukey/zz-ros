class GlobalVars:
    # 示教模式标志位
    dynamic_teach_flag = False

    @classmethod
    def set_dynamic_teach_flag(cls, flag):
        cls.dynamic_teach_flag = flag
        

GlobalVars.__new__ = lambda cls:None
