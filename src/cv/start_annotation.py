#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OBB标注工具启动脚本
自动检查依赖、设置环境并启动标注工具
"""

import sys
import os
import subprocess
import importlib.util
from pathlib import Path

def check_python_version():
    """检查Python版本"""
    if sys.version_info < (3, 6):
        print("❌ 错误：需要Python 3.6或更高版本")
        print(f"   当前版本：{sys.version}")
        return False
    else:
        print(f"✅ Python版本检查通过：{sys.version.split()[0]}")
        return True

def check_dependencies():
    """检查必需的依赖包"""
    required_packages = {
        'tkinter': 'tkinter (Python内置)',
        'cv2': 'opencv-python',
        'PIL': 'Pillow',
        'numpy': 'numpy'
    }
    
    missing_packages = []
    
    print("\n🔍 检查依赖包...")
    
    for package, install_name in required_packages.items():
        try:
            if package == 'tkinter':
                import tkinter
            elif package == 'cv2':
                import cv2
            elif package == 'PIL':
                from PIL import Image
            elif package == 'numpy':
                import numpy
            
            print(f"✅ {install_name}")
            
        except ImportError:
            print(f"❌ {install_name}")
            missing_packages.append(install_name)
    
    return missing_packages

def install_missing_packages(missing_packages):
    """安装缺失的包"""
    if not missing_packages:
        return True
    
    print(f"\n📦 发现缺失的包：{', '.join(missing_packages)}")
    
    try:
        user_input = input("是否自动安装缺失的包？(y/n): ").lower().strip()
        if user_input in ['y', 'yes', '是']:
            print("\n⏳ 正在安装缺失的包...")
            
            for package in missing_packages:
                if package == 'tkinter (Python内置)':
                    print("❌ tkinter是Python内置模块，请检查Python安装")
                    continue
                
                print(f"📦 安装 {package}...")
                result = subprocess.run([sys.executable, '-m', 'pip', 'install', package], 
                                      capture_output=True, text=True)
                
                if result.returncode == 0:
                    print(f"✅ {package} 安装成功")
                else:
                    print(f"❌ {package} 安装失败: {result.stderr}")
                    return False
            
            print("\n🎉 所有包安装完成！")
            return True
        else:
            print("\n⚠️  请手动安装缺失的包后再运行")
            print("安装命令：")
            for package in missing_packages:
                if package != 'tkinter (Python内置)':
                    print(f"   pip install {package}")
            return False
    
    except KeyboardInterrupt:
        print("\n\n⚠️  安装被用户取消")
        return False

def check_annotation_tool():
    """检查标注工具文件是否存在"""
    tool_path = Path(__file__).parent / "obb_annotation_tool.py"
    
    if tool_path.exists():
        print(f"✅ 找到标注工具：{tool_path}")
        return tool_path
    else:
        print(f"❌ 未找到标注工具文件：{tool_path}")
        return None

def create_desktop_shortcut():
    """创建桌面快捷方式（Windows）"""
    try:
        if os.name == 'nt':  # Windows
            import winshell
            from win32com.client import Dispatch
            
            desktop = winshell.desktop()
            shortcut_path = os.path.join(desktop, "OBB标注工具.lnk")
            
            shell = Dispatch('WScript.Shell')
            shortcut = shell.CreateShortCut(shortcut_path)
            shortcut.Targetpath = sys.executable
            shortcut.Arguments = f'"{Path(__file__)}"'
            shortcut.WorkingDirectory = str(Path(__file__).parent)
            shortcut.IconLocation = sys.executable
            shortcut.save()
            
            print(f"✅ 已创建桌面快捷方式：{shortcut_path}")
    except Exception as e:
        print(f"⚠️  创建桌面快捷方式失败：{e}")

def print_banner():
    """打印欢迎横幅"""
    banner = """
╔═══════════════════════════════════════════════════════════════╗
║                                                               ║
║                    🎯 OBB标注工具 v1.0                       ║
║                                                               ║
║    专业的旋转边界框(OBB)标注工具，支持YOLO格式训练数据生成    ║
║                                                               ║
╚═══════════════════════════════════════════════════════════════╝
"""
    print(banner)

def print_usage_tips():
    """打印使用提示"""
    tips = """
📝 快速开始：
   1. 点击"加载图像目录"选择包含图像的文件夹
   2. 在图像上按顺序点击4个点创建旋转边界框
   3. 使用键盘方向键快速切换图像
   4. 切换图像时自动保存标注

⌨️  常用快捷键：
   • 左/右方向键：切换图像
   • Ctrl+S：保存标注
   • Delete：删除选中标注
   • Escape：取消当前绘制
   • 鼠标右键：取消当前绘制

📁 输出格式：
   • 标注文件：images/labels/*.txt (YOLO OBB格式)
   • 类别文件：images/labels/classes.txt
   
🔧 如需帮助，请查看 OBB_ANNOTATION_GUIDE.md
"""
    print(tips)

def main():
    """主函数"""
    try:
        print_banner()
        
        # 检查Python版本
        if not check_python_version():
            input("\n按Enter键退出...")
            return
        
        # 检查依赖
        missing_packages = check_dependencies()
        if missing_packages:
            if not install_missing_packages(missing_packages):
                input("\n按Enter键退出...")
                return
            
            # 重新检查依赖
            missing_packages = check_dependencies()
            if missing_packages:
                print("\n❌ 仍有缺失的依赖包，请手动安装")
                input("按Enter键退出...")
                return
        
        # 检查标注工具文件
        tool_path = check_annotation_tool()
        if not tool_path:
            input("\n按Enter键退出...")
            return
        
        print_usage_tips()
        
        # 询问是否创建桌面快捷方式
        try:
            if os.name == 'nt':  # Windows
                create_shortcut = input("是否创建桌面快捷方式？(y/n): ").lower().strip()
                if create_shortcut in ['y', 'yes', '是']:
                    create_desktop_shortcut()
        except KeyboardInterrupt:
            pass
        
        print("\n🚀 正在启动OBB标注工具...")
        print("=" * 60)
        
        # 启动标注工具
        import obb_annotation_tool
        obb_annotation_tool.main()
        
    except KeyboardInterrupt:
        print("\n\n⚠️  程序被用户中断")
    except Exception as e:
        print(f"\n❌ 启动失败：{e}")
        print("\n🔧 故障排除：")
        print("1. 确保Python版本 >= 3.6")
        print("2. 检查所有依赖包是否正确安装")
        print("3. 确保obb_annotation_tool.py文件存在")
        print("4. 查看OBB_ANNOTATION_GUIDE.md获取详细帮助")
        input("\n按Enter键退出...")

if __name__ == "__main__":
    main() 