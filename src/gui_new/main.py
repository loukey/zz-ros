"""
简化版机器人控制界面主程序
使用依赖注入管理服务
"""
import sys
import os
from PyQt5.QtWidgets import QApplication, QMessageBox
from shared.config.service_registry import configure_services, get_main_view_model, get_listener_service
from presentation.gui.main_window import MainWindow


# 添加当前目录到Python路径，确保可以导入本地模块
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, current_dir)


def main():
    """主函数"""
    # 创建QApplication实例
    app = QApplication(sys.argv)
    
    # 设置应用信息
    app.setApplicationName("镇中科技机械臂控制系统")
    app.setApplicationVersion("0.6.0 - Simplified")
    app.setOrganizationName("镇中科技")
    
    try:
        # 初始化依赖注入容器
        print("🔧 初始化依赖注入容器...")
        container = configure_services()
        print("✅ 依赖注入容器初始化完成")
        
        # 通过DI获取主视图模型
        main_view_model = get_main_view_model()
        print("✅ 主视图模型创建完成")
        get_listener_service()
        # 导入并创建主窗口
        main_window = MainWindow(view_model=main_view_model)
        main_window.show()
        print("✅ 主窗口创建完成")
        
        # 设置应用程序退出时的清理
        def cleanup():
            print("🧹 清理应用资源...")
            try:
                main_view_model.cleanup()
                container.clear()
                print("✅ 资源清理完成")
            except Exception as e:
                print(f"⚠️ 清理时出现错误: {str(e)}")
        
        app.aboutToQuit.connect(cleanup)
        
    except Exception as e:
        print(f"❌ 启动失败: {str(e)}")
        import traceback
        traceback.print_exc()
        QMessageBox.critical(None, "启动失败", f"应用程序启动失败:\n{str(e)}")
        sys.exit(1)
    
    # 运行应用程序事件循环
    sys.exit(app.exec_())


if __name__ == "__main__":
    main() 