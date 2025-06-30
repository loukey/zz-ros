@echo off
chcp 65001 > nul
title OBB标注工具启动器

echo.
echo ╔═══════════════════════════════════════════════════════════════╗
echo ║                                                               ║
echo ║                    🎯 OBB标注工具 v1.0                       ║
echo ║                                                               ║
echo ║    专业的旋转边界框(OBB)标注工具，支持YOLO格式训练数据生成    ║
echo ║                                                               ║
echo ╚═══════════════════════════════════════════════════════════════╝
echo.

cd /d "%~dp0"

echo 🔍 检查Python环境...
python --version >nul 2>&1
if errorlevel 1 (
    echo ❌ 错误：未找到Python，请先安装Python 3.6+
    echo.
    echo 📥 Python下载地址：https://www.python.org/downloads/
    echo.
    pause
    exit /b 1
)

echo ✅ Python环境检查通过
echo.

echo 🚀 正在启动OBB标注工具...
echo.
python start_annotation.py

echo.
echo 程序已结束，按任意键退出...
pause >nul 