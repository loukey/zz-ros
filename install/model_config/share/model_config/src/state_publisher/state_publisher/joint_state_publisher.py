#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
from math import sin, cos
from time import time

# 如果需要的话，可以导入并使用你写的运动学模块，比如：
# from your_kinematics_module import Kinematic6DOF

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        # 定时器，间隔0.1秒发布一次
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.joint_state = JointState()
        # 注意这里关节名称要与 URDF 中的 joint 名称一致
        self.joint_state.name = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
        self.start_time = time()
        # 如果要使用你的运动学代码，可以在这里初始化
        # self.kinematics = Kinematic6DOF()

    def timer_callback(self):
        now = self.get_clock().now().to_msg()
        self.joint_state.header.stamp = now
        t = time() - self.start_time
        
        # 示例：用简单的正弦、余弦函数产生周期性运动
        self.joint_state.position = [
            0.5 * sin(t),
            -0.5 * cos(t),
            0.5 * sin(t + math.pi/4),
            0.5 * cos(t + math.pi/4),
            0.3 * sin(t + math.pi/2),
            0.3 * cos(t + math.pi/2)
        ]
        
        # 如果使用你的运动学模块，可以调用 update() 或者计算逆运动学，然后获得各个关节角
        # 例如：
        # self.kinematics.update(calculated_theta_list)
        # self.joint_state.position = calculated_theta_list

        self.publisher_.publish(self.joint_state)

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
