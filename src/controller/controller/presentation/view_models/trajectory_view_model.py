"""
轨迹规划视图模型 - Presentation层
"""
from .base_view_model import BaseViewModel


class TrajectoryViewModel(BaseViewModel):
    """轨迹规划视图模型。
    
    负责轨迹规划相关的 UI 交互逻辑。
    """
    
    def __init__(self, parent=None):
        """初始化轨迹规划视图模型。
        
        Args:
            parent (QObject, optional): 父对象. Defaults to None.
        """
        super().__init__(parent)

