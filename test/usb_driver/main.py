"""
USB串口通信界面主程序
使能 ENABLE
取消使能 DISABlE
释放刹车 RELEASE
锁止刹车 LOCK
立刻停止 STOP
运动状态 MOTION
运动状态实时显示  安全限位设置  显示状态码  获取当前位置

帧头: 0xAA 0x55  
地址: 0x01  
命令: 0x01
数据长度: 0x18（24字节）  
数据域:  
  轴1位置: 0x0000FFFF 
  轴2位置: 0x0001FFFE  
  轴3位置: 0x0002FFFD  
  轴4位置: 0x0003FFFC  
  轴5位置: 0x0004FFFB  
  轴6位置: 0x0005FFFA  
其他信息
校验: CRC16(0x12 0x34)  
帧尾: 0x0D 0x0A

0x01 使能 ENABLE 
0x02 取消使能 DISABlE
0x03 释放刹车 RELEASE
0x04 锁止刹车 LOCK
0x05 立刻停止 STOP
0x06 运动状态 MOTION
"""

import sys
import os
from PyQt5.QtWidgets import QApplication
from gui.qt_main_window import MainWindow


def main():
    # 设置环境变量
    os.environ['QT_QPA_PLATFORM'] = 'xcb'  # 使用 X11 平台
    
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main() 
