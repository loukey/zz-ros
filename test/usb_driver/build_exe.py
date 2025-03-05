import os
import subprocess
import sys
import shutil

def build_exe():
    print("Starting build process for USB Serial UI application...")
    
    # Check if PyInstaller is installed
    try:
        import PyInstaller
        print("PyInstaller is already installed.")
    except ImportError:
        print("Installing PyInstaller...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", "pyinstaller>=5.6.2"])
    
    # Check if pyserial is installed
    try:
        import serial
        print("PySerial is already installed.")
    except ImportError:
        print("Installing PySerial...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", "pyserial>=3.5"])
    
    # Check if matplotlib is installed
    try:
        import matplotlib
        print("Matplotlib is already installed.")
    except ImportError:
        print("Installing Matplotlib...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", "matplotlib>=3.5.0"])
    
    # Create a directory for the build files if it doesn't exist
    if not os.path.exists("build"):
        os.makedirs("build")
    
    # Create a directory for the dist files if it doesn't exist
    if not os.path.exists("dist"):
        os.makedirs("dist")
    
    # Create a README.md file if it doesn't exist
    if not os.path.exists("README.md"):
        with open("README.md", "w", encoding="utf-8") as f:
            f.write("# 镇中科技USB串口测试工具v0.0.9\n\n")
            f.write("这是一个用于测试USB串口设备的工具，支持选择串口、配置参数、发送和接收数据。\n\n")
            f.write("## 使用方法\n\n")
            f.write("1. 从下拉列表中选择串口设备\n")
            f.write("2. 配置串口参数（波特率、数据位、校验位等）\n")
            f.write("3. 点击\"连接串口\"按钮\n")
            f.write("4. 使用角度控制区域发送数据\n")
            f.write("5. 在数据显示区域查看发送和接收的数据\n")
            f.write("6. 点击\"发送角度\"按钮后可在曲线显示标签页查看速度、位置和加速度曲线\n\n")
            f.write("## 注意事项\n\n")
            f.write("- 确保您的计算机已正确安装USB串口设备驱动\n")
            f.write("- 如果找不到设备，请点击\"刷新\"按钮更新设备列表\n")
    
    # Check if spec file exists
    if os.path.exists("usb_serial_ui.spec"):
        print("Using existing spec file for PyInstaller...")
        pyinstaller_cmd = [
            "pyinstaller",
            "--clean",  # Clean PyInstaller cache and remove temporary files
            "usb_serial_ui.spec"
        ]
    else:
        print("No spec file found. Using command line arguments for PyInstaller...")
        pyinstaller_cmd = [
            "pyinstaller",
            "--name=镇中科技USB串口测试工具v0.0.9",  # Name of the executable
            "--onefile",              # Create a single executable file
            "--windowed",             # Windows specific: do not open a console window
            "--icon=NONE",            # No icon (you can specify an .ico file here if you have one)
            "--clean",                # Clean PyInstaller cache and remove temporary files
            "--add-data=README.md;.", # Include README file
            "--hidden-import=serial", # Ensure pyserial is included
            "--hidden-import=serial.tools.list_ports", # Include list_ports module
            "--hidden-import=matplotlib", # Include matplotlib
            "--hidden-import=numpy", # Include numpy
            "--hidden-import=matplotlib.backends.backend_tkagg", # Include matplotlib backend
            "main.py"                 # The main script to package
        ]
    
    # Run PyInstaller
    print("Running PyInstaller with command:", " ".join(pyinstaller_cmd))
    subprocess.check_call(pyinstaller_cmd)
    
    print("\nBuild completed successfully!")
    print("The executable can be found in the 'dist' folder.")
    
    # Check if the executable was created
    exe_path = os.path.join("dist", "镇中科技USB串口测试工具v0.0.9.exe")
    if os.path.exists(exe_path):
        print(f"Executable created: {os.path.abspath(exe_path)}")
        print(f"Size: {os.path.getsize(exe_path) / (1024*1024):.2f} MB")
    else:
        print("Warning: Executable file not found in the expected location.")

if __name__ == "__main__":
    build_exe() 