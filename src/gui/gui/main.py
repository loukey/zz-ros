"""
应用入口模块
"""
import rclpy
from rclpy.node import Node
import sys
from PyQt5.QtWidgets import QApplication
from gui.views.main_window import MainWindow


class GUINode(Node):
    def __init__(self):
        super().__init__("gui_node")
        self.app = QApplication([])
        self.window = MainWindow()
        self.window.show()
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        self.app.processEvents()


def main():
    rclpy.init()
    gui_node = GUINode()
    try:
        rclpy.spin(gui_node)
    except KeyboardInterrupt:
        pass
    finally:
        gui_node.destroy_node()
        rclpy.shutdown()
    