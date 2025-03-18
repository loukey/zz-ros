"""
应用入口模块
"""
import sys
import os
from PyQt5.QtWidgets import QApplication

# 添加当前目录到Python路径中
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# 直接导入
from views.main_window import MainWindow


def main():
    """主函数"""
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main() 