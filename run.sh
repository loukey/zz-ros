#!/bin/bash

# 设置环境变量解决Qt插件冲突
export QT_QPA_PLATFORM_PLUGIN_PATH=""
export QT_PLUGIN_PATH=""
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PYTHONPATH"

# 禁用OpenCV的Qt插件，使用系统Qt
export OPENCV_IO_ENABLE_OPENEXR=1
export QT_QPA_PLATFORM=xcb

# 确保使用系统Qt库而不是OpenCV捆绑的Qt
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH"

# 禁用Qt调试输出
export QT_LOGGING_RULES="*.debug=false;qt.qpa.plugin.debug=false"

# 使用uv运行GUI应用
cd src/gui/gui && uv run python main.py
