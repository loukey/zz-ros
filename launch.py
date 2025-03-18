"""
机器人控制界面启动脚本

使用方法:
python launch.py
"""
import sys
import os
from PyQt5.QtWidgets import QApplication

# 将当前项目根目录添加到Python路径
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, current_dir)

# 导入主窗口类
try:
    from test.ui.views.main_window import MainWindow
except ImportError as e:
    print(f"错误: {e}")
    print("请确保已安装所需的依赖库，如PyQt5、matplotlib等")
    sys.exit(1)

def main():
    """启动主程序"""
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main() 