import os
import subprocess
import sys

def build_exe():
    print("开始构建USB串口测试工具可执行文件...")
    
    # 检查是否安装了PyInstaller
    try:
        import PyInstaller
        print("PyInstaller已安装。")
    except ImportError:
        print("正在安装PyInstaller...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", "pyinstaller>=5.6.2"])
    
    # 检查是否安装了pyserial
    try:
        import serial
        print("PySerial已安装。")
    except ImportError:
        print("正在安装PySerial...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", "pyserial>=3.5"])
    
    # 创建构建目录
    if not os.path.exists("build"):
        os.makedirs("build")
    
    # 创建dist目录
    if not os.path.exists("dist"):
        os.makedirs("dist")
    
    # 定义PyInstaller命令
    pyinstaller_cmd = [
        "pyinstaller",
        "--name=USB串口测试工具",  # 可执行文件名称
        "--onefile",              # 创建单个可执行文件
        "--windowed",             # Windows特定：不打开控制台窗口
        "--clean",                # 清理PyInstaller缓存并删除临时文件
        "--hidden-import=serial", # 确保包含pyserial
        "--hidden-import=serial.tools.list_ports", # 包含list_ports模块
        "main.py"                 # 要打包的主脚本
    ]
    
    # 运行PyInstaller
    print("正在运行PyInstaller，命令:", " ".join(pyinstaller_cmd))
    subprocess.check_call(pyinstaller_cmd)
    
    print("\n构建成功完成！")
    print("可执行文件位于'dist'文件夹中。")
    
    # 检查可执行文件是否已创建
    exe_path = os.path.join("dist", "USB串口测试工具.exe")
    if os.path.exists(exe_path):
        print(f"可执行文件已创建: {os.path.abspath(exe_path)}")
        print(f"文件大小: {os.path.getsize(exe_path) / (1024*1024):.2f} MB")
    else:
        print("警告：在预期位置未找到可执行文件。")

if __name__ == "__main__":
    build_exe() 