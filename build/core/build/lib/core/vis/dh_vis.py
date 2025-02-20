import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from core.kinematic import Kinematic6DOF
from math import pi

class DHVisualizer:
    def __init__(self):
        self.robot = Kinematic6DOF()
        self.fig = plt.figure(figsize=(12, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
    def plot_frame(self, T, scale=0.1):
        """绘制坐标系"""
        origin = T[:3, 3]
        
        # 绘制三个轴（x红，y绿，z蓝）
        x_axis = origin + scale * T[:3, 0]
        y_axis = origin + scale * T[:3, 1]
        z_axis = origin + scale * T[:3, 2]
        
        self.ax.plot([origin[0], x_axis[0]], [origin[1], x_axis[1]], 
                    [origin[2], x_axis[2]], 'r', linewidth=2)
        self.ax.plot([origin[0], y_axis[0]], [origin[1], y_axis[1]], 
                    [origin[2], y_axis[2]], 'y', linewidth=2)
        self.ax.plot([origin[0], z_axis[0]], [origin[1], z_axis[1]], 
                    [origin[2], z_axis[2]], 'g', linewidth=2)
    
    def plot_link(self, T1, T2):
        """绘制连杆"""
        p1 = T1[:3, 3]
        p2 = T2[:3, 3]
        self.ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], 
                    'k', linewidth=2, alpha=0.5)
    
    def visualize(self, theta_list=None):
        """可视化机器人"""
        if theta_list is not None:
            self.robot.update(theta_list)
        
        # 清除当前图形
        self.ax.cla()
        
        # 设置坐标轴标签
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        
        # 设置视角
        self.ax.view_init(elev=20, azim=45)
        
        # 设置坐标轴范围
        limit = 0.5
        self.ax.set_xlim([-limit, limit])
        self.ax.set_ylim([-limit, limit])
        self.ax.set_zlim([-limit, limit])
        
        # 绘制基座坐标系
        T0 = np.eye(4)
        self.plot_frame(T0)
        
        # 逐个绘制每个关节的坐标系和连杆
        T = T0
        transforms = [T]
        
        for i in range(6):
            T = T @ self.robot.rm_list[i]
            transforms.append(T)
            self.plot_frame(T)
            if i > 0:
                self.plot_link(transforms[-2], transforms[-1])
        
        # 添加标题，显示当前关节角度
        angles = [f"θ{i+1}={np.degrees(angle):.1f}°" 
                 for i, angle in enumerate(self.robot.theta_list)]
        self.ax.set_title("\n".join(angles))
        
        plt.tight_layout()
        plt.show()

def test_visualization():
    """测试不同姿态的可视化"""
    vis = DHVisualizer()
    
    # 测试不同姿态
    test_cases = [
        ([0.0, -pi/2, 0.0, pi/2, 0.0, 0.0], "零位姿态"),
        ([pi/4, 0, 0, 0, 0, 0], "关节1旋转45度"),
        ([0, pi/4, 0, 0, 0, 0], "关节2旋转45度"),
        ([0, pi/4, pi/4, 0, 0, 0], "关节2、3旋转45度"),
        ([pi/4, pi/4, pi/4, pi/4, pi/4, pi/4], "所有关节旋转45度")
    ]
    
    for angles, title in test_cases:
        print(f"\n测试{title}")
        vis.visualize(angles)
        input("按回车继续...")

