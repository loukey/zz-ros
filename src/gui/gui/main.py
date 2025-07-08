"""
应用入口模块
"""
import sys
from PyQt5.QtWidgets import QApplication
from gui.views import MainWindow


def main():
    app = QApplication([])
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
    
if __name__ == "__main__":
    main()
    
