"""
简化版机器人控制界面主程序
使用依赖注入管理服务
"""
import sys
import os
import platform
from PyQt5.QtWidgets import QApplication, QMessageBox
from controller.shared.config.service_registry import configure_services, get_main_view_model, get_listener_service
from controller.presentation.gui.main_window import MainWindow
from controller.presentation.theme import get_stylesheet
if platform.system() == 'Linux':
    os.environ.setdefault('QT_QPA_PLATFORM_PLUGIN_PATH',
                          '/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms')


def main():
    """主函数"""
    # 创建QApplication实例
    app = QApplication(sys.argv)
    
    # 设置应用信息
    app.setApplicationName("镇中科技机械臂控制系统")
    app.setApplicationVersion("0.6.0 - Simplified")
    app.setOrganizationName("镇中科技")

    # 应用全局主题样式
    app.setStyleSheet(get_stylesheet())
    
    try:
        # 初始化依赖注入容器
        print("[*] 初始化依赖注入容器...")
        container = configure_services()
        print("[+] 依赖注入容器初始化完成")
        
        # 通过DI获取主视图模型
        main_view_model = get_main_view_model()
        print("[+] 主视图模型创建完成")
        get_listener_service()
        # 导入并创建主窗口
        main_window = MainWindow(view_model=main_view_model)
        main_window.show()
        print("[+] 主窗口创建完成")
        
        # 设置应用程序退出时的清理
        def cleanup():
            try:
                main_view_model.cleanup()
                container.clear()
            except Exception as e:
                pass
        
        app.aboutToQuit.connect(cleanup)
        
    except Exception as e:
        print(f"[-] 启动失败: {str(e)}")
        import traceback
        traceback.print_exc()
        QMessageBox.critical(None, "启动失败", f"应用程序启动失败:\n{str(e)}")
        sys.exit(1)
    
    # 运行应用程序事件循环
    sys.exit(app.exec_())


if __name__ == "__main__":
    main() 