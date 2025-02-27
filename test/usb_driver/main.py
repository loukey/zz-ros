"""
USB串口通信界面主程序
使能 ENABLE
取消使能 DISABlE
释放刹车 RELEASE
锁止刹车 LOCK
立刻停止 STOP
运动状态 MOTION
运动状态实时显示 安全限位设置  显示状态码  获取当前位置

"""

import tkinter as tk
import sys
import os
from gui.app import USBSerialApp


def main():
    """应用程序入口点"""
    root = tk.Tk()
    app = USBSerialApp(root)
    
    try:
        app.run()
    except Exception as e:
        print(f"程序运行出错: {str(e)}")
    finally:
        # 确保程序完全退出
        try:
            root.destroy()
        except:
            pass
        
        # 强制退出进程
        os._exit(0)


if __name__ == "__main__":
    main()
