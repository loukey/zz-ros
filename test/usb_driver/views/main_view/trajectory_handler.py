"""
轨迹处理模块 - 简化版
"""


class TrajectoryHandler:
    """轨迹处理类，现在只负责处理计算错误"""
    
    def __init__(self, main_window):
        self.main_window = main_window
    
    def handle_calculation_error(self, error_message):
        """处理轨迹计算错误
        
        Args:
            error_message: 错误信息
        """
        self.main_window.data_display.append_message(f"轨迹计算错误: {error_message}", "错误")
        
        # 更新UI状态，例如禁用某些按钮或显示错误指示
        if hasattr(self.main_window, 'status_bar'):
            self.main_window.status_bar.showMessage("轨迹计算失败") 
