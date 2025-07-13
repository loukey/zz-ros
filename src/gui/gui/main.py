"""
应用入口模块
"""
import sys
from PyQt5.QtWidgets import QApplication
from gui.views import MainWindow
from gui.config.params import ParamConfig
import os
os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = '/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms'


def main():
    # 启动时加载参数配置
    ParamConfig.load_config()
    
    app = QApplication([])
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
    
if __name__ == "__main__":
    main()
    
