"""
参数配置管理
"""
import json
import os
from typing import Dict, Any


class ParamConfig:
    """参数配置类"""
    
    # 全局变量
    velocity_x = 0.0  # X方向速度
    velocity_y = 0.0  # Y方向速度
    
    # 配置文件路径
    CONFIG_FILE = os.path.join(os.path.dirname(__file__), "param.json")
    
    @classmethod
    def load_config(cls):
        """从JSON文件加载参数配置"""
        try:
            if os.path.exists(cls.CONFIG_FILE):
                with open(cls.CONFIG_FILE, 'r', encoding='utf-8') as f:
                    config_data = json.load(f)
                    
                # 更新全局变量
                cls.velocity_x = config_data.get('velocity_x', 0.0)
                cls.velocity_y = config_data.get('velocity_y', 0.0)
                
            else:
                print("参数配置文件不存在，使用默认值")
                cls._create_default_config()
                
        except Exception as e:
            print(f"加载参数配置失败: {e}")
            cls._create_default_config()
    
    @classmethod
    def save_config(cls):
        """保存参数配置到JSON文件"""
        try:
            # 确保目录存在
            config_dir = os.path.dirname(cls.CONFIG_FILE)
            if not os.path.exists(config_dir):
                os.makedirs(config_dir)
            
            config_data = {
                'velocity_x': cls.velocity_x,
                'velocity_y': cls.velocity_y
            }
            
            with open(cls.CONFIG_FILE, 'w', encoding='utf-8') as f:
                json.dump(config_data, f, indent=4, ensure_ascii=False)
            
            print(f"参数配置已保存: velocity_x={cls.velocity_x}, velocity_y={cls.velocity_y}")
            return True
            
        except Exception as e:
            print(f"保存参数配置失败: {e}")
            return False
    
    @classmethod
    def _create_default_config(cls):
        """创建默认配置文件"""
        cls.velocity_x = 0.0
        cls.velocity_y = 0.0
        cls.save_config()
    
    @classmethod
    def update_velocity(cls, vx: float, vy: float):
        """更新速度值"""
        cls.velocity_x = vx
        cls.velocity_y = vy
    
    @classmethod
    def get_velocity(cls) -> tuple:
        """获取当前速度值"""
        return cls.velocity_x, cls.velocity_y
    
    @classmethod
    def get_config_dict(cls) -> Dict[str, Any]:
        """获取配置字典"""
        return {
            'velocity_x': cls.velocity_x,
            'velocity_y': cls.velocity_y
        }


# 在模块导入时自动加载配置
ParamConfig.load_config()
