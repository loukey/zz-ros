import tkinter as tk
import threading
import rclpy
from .pose_publisher import PosePublisher


class PoseControlUI(tk.Tk):
    def __init__(self, publisher):
        super().__init__()
        self.publisher = publisher
        self.title("发送位姿")
        self.geometry("300x300")
        self.create_widgets()

    def create_widgets(self):
        # 定义标签名称，对应位置和欧拉角
        labels = ["X", "Y", "Z", "px", "py", "pz"]
        self.entries = {}
        for i, label in enumerate(labels):
            lbl = tk.Label(self, text=label, font=("Arial", 12))
            lbl.grid(row=i, column=0, padx=5, pady=5, sticky="e")
            entry = tk.Entry(self, width=14, font=("Arial", 12))
            entry.grid(row=i, column=1, padx=5, pady=5)
            # 初始化默认值为 0
            entry.insert(0, "0")
            self.entries[label] = entry

        btn = tk.Button(self, text="发送位姿", command=self.send_pose, font=("Arial", 12, "bold"), width=12)
        btn.grid(row=len(labels), column=0, columnspan=2, pady=15)

    def send_pose(self):
        try:
            # 依次从各个输入框获取数据，并转换为 float
            pose_values = [float(self.entries[label].get()) for label in ["X", "Y", "Z", "px", "py", "pz"]]
        except ValueError:
            print("输入格式错误，请输入数字！")
            return
        # 调用 ROS2 节点的 publish 方法进行发布
        self.publisher.publish(pose_values)

def main():
    # 初始化 ROS2
    rclpy.init()
    pose_publisher = PosePublisher()
    # 创建 GUI，并将 ROS2 发布节点传入
    gui = PoseControlUI(pose_publisher)

    # 使用线程运行 rclpy.spin，确保 ROS2 回调能够执行，不阻塞 GUI
    ros_thread = threading.Thread(target=rclpy.spin, args=(pose_publisher,), daemon=True)
    ros_thread.start()

    gui.mainloop()

    # GUI 关闭后清理 ROS2 资源
    pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()