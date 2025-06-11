"""
应用入口模块
"""
import sys
from PyQt5.QtWidgets import QApplication
from gui.views import MainWindow
from rclpy.node import Node
import rclpy


class GuiRos2Node(Node):
    """GUI ROS2节点"""
    def __init__(self):
        super().__init__('gui_control_node')
        self.get_logger().info('GUI控制节点已启动')
        app = QApplication([])
        window = MainWindow()
        window.show()
        sys.exit(app.exec_())


def main():
    rclpy.init()
    node = GuiRos2Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
    
