"""
简化版机器人控制界面主程序
"""
import sys
import os
from PyQt5.QtWidgets import QApplication, QMessageBox

# 添加当前目录到Python路径，确保可以导入本地模块
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, current_dir)


def main():
    """主函数"""
    # 创建QApplication实例
    app = QApplication(sys.argv)
    
    # 设置应用信息
    app.setApplicationName("Robot Control System")
    app.setApplicationVersion("2.0.0 - Simplified")
    app.setOrganizationName("RoboticsLab")
    
    # 导入并创建主窗口
    try:
        from presentation.gui.main_window import MainWindow
        main_window = MainWindow()
        main_window.show()
        print("主窗口创建完成")
    except Exception as e:
        print(f"主窗口创建失败: {str(e)}")
        QMessageBox.critical(None, "启动失败", f"主窗口创建失败:\n{str(e)}")
        sys.exit(1)
    
    # 运行应用程序事件循环
    sys.exit(app.exec_())


if __name__ == "__main__":
    main() 