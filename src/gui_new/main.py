"""
新架构机器人控制界面主程序
"""
import sys
import os
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTranslator, QLocale

# 添加当前目录到Python路径，确保可以导入本地模块
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, current_dir)

# 初始化依赖注入容器 - 必须在导入使用ServiceLocator的模块之前
from shared.config.container import configure_services


def main():
    """主函数"""
    # 创建QApplication实例
    app = QApplication(sys.argv)
    
    # 设置应用信息
    app.setApplicationName("Robot Control System")
    app.setApplicationVersion("2.0.0")
    app.setOrganizationName("RoboticsLab")
    
    # 配置依赖注入容器
    try:
        container = configure_services()
        print("依赖注入容器初始化完成")
    except Exception as e:
        print(f"依赖注入容器初始化失败: {str(e)}")
        sys.exit(1)
    
    # 在依赖注入容器初始化完成后导入主窗口
    from presentation.gui.main_window import MainWindow
    
    # 创建并显示主窗口
    try:
        main_window = MainWindow()
        main_window.show()
        print("主窗口创建完成")
    except Exception as e:
        print(f"主窗口创建失败: {str(e)}")
        sys.exit(1)
    
    # 运行应用程序事件循环
    sys.exit(app.exec_())


if __name__ == "__main__":
    main() 