import tkinter as tk
import threading
import rclpy
from .pose_publisher import PosePublisher
from core.kinematic import Kinematic6DOF


class PoseControlUI(tk.Tk):
    def __init__(self, publisher):
        super().__init__()
        self.publisher = publisher
        self.title("发送位姿")
        self.geometry("600x300")  # 窗口宽度增大以适应更多控件
        self.create_widgets()

    def create_widgets(self):
        # 左侧：位姿相关输入框（6个）
        pose_labels = ["X", "Y", "Z", "px", "py", "pz"]
        self.entries = {}
        for i, label in enumerate(pose_labels):
            lbl = tk.Label(self, text=label, font=("Arial", 12))
            lbl.grid(row=i, column=0, padx=5, pady=5, sticky="e")
            entry = tk.Entry(self, width=14, font=("Arial", 12))
            entry.grid(row=i, column=1, padx=5, pady=5)
            entry.insert(0, "0")  # 默认值为 0
            self.entries[label] = entry

        # 右侧：角度相关输入框（6个）
        angle_labels = ["角1", "角2", "角3", "角4", "角5", "角6"]
        self.angle_entries = {}
        for i, label in enumerate(angle_labels):
            lbl = tk.Label(self, text=label, font=("Arial", 12))
            lbl.grid(row=i, column=2, padx=5, pady=5, sticky="e")
            entry = tk.Entry(self, width=14, font=("Arial", 12))
            entry.grid(row=i, column=3, padx=5, pady=5)
            entry.insert(0, "0")  # 默认值为 0
            self.angle_entries[label] = entry

        # 左侧按钮：发送位姿
        btn_pose = tk.Button(self, text="发送位姿", command=self.send_pose, font=("Arial", 12, "bold"), width=12)
        btn_pose.grid(row=len(pose_labels), column=0, columnspan=2, pady=15)

        # 右侧按钮：发送角度
        btn_angles = tk.Button(self, text="发送角度", command=self.send_angles, font=("Arial", 12, "bold"), width=12)
        btn_angles.grid(row=len(angle_labels), column=2, columnspan=2, pady=15)

    def send_pose(self):
        try:
            # 从左侧输入框获取位姿数据，转换为 float 类型
            pose_values = [float(self.entries[label].get()) for label in ["X", "Y", "Z", "px", "py", "pz"]]
            k = Kinematic6DOF()
            angle_values = k.inverse_kinematic(*pose_values)
        except ValueError:
            print("cant reach this pose!")
            return
        # 调用 ROS2 节点的 publish 方法进行位姿发布
        self.publisher.publish(angle_values)

    def send_angles(self):
        try:
            # 从右侧输入框获取角度数据，转换为 float 类型
            angle_values = [float(self.angle_entries[label].get()) for label in ["角1", "角2", "角3", "角4", "角5", "角6"]]
        except ValueError:
            print("输入格式错误，请输入数字！")
            return
        # 调用 ROS2 节点的 publish_angles 方法进行角度发布
        # 注意：这里假设 PosePublisher 类中有 publish_angles 方法，
        # 如果没有，需要在 PosePublisher 中添加该方法或者使用 publish 方法区分数据类型。
        self.publisher.publish(angle_values)

def main():
    # 初始化 ROS2
    rclpy.init()
    pose_publisher = PosePublisher()
    # 创建 GUI，并将 ROS2 发布节点传入
    gui = PoseControlUI(pose_publisher)

    # 使用线程运行 rclpy.spin，确保 ROS2 回调能够执行而不阻塞 GUI
    ros_thread = threading.Thread(target=rclpy.spin, args=(pose_publisher,), daemon=True)
    ros_thread.start()

    gui.mainloop()

    # GUI 关闭后清理 ROS2 资源
    pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
