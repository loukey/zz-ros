#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
主程序入口
"""

import sys
from PyQt5.QtWidgets import QApplication
from gui.windows.main_window import MainWindow

def main():
    # 创建应用程序
    app = QApplication(sys.argv)
    app.setApplicationName("机器人控制界面")
    
    # 创建主窗口
    window = MainWindow()
    window.show()
    
    # 运行应用程序
    sys.exit(app.exec_())

if __name__ == "__main__":
    main() 