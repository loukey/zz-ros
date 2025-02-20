#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
from math import sin, cos, pi
from time import time
import sys
from core.kinematic import Kinematic6DOF


class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        # 定时器，间隔0.1秒发布一次
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.joint_state = JointState()
        # 注意这里关节名称要与 URDF 中的 joint 名称一致
        self.joint_state.name = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
         # 初始角度（可以根据你的实际情况修改）
        self.initial_angles = [0, 0, 0, 0, 0, 0]
        # 目标角度
        self.target_angles  = [0, math.pi/3, math.pi/4, math.pi/5, math.pi/6, math.pi/7]
        # 过渡时间（单位秒）
        self.transition_duration = 5.0
        
        # 记录开始转换的时间
        self.start_time = time() + 4
        # 标记是否已经完成转换
        self.transition_done = False

        self.kinematics = Kinematic6DOF()

    def timer_callback(self):
        now = self.get_clock().now().to_msg()
        self.joint_state.header.stamp = now
        self.IK(0, 0, 0, -0.1, -0.2, 0.3)

        t_elapsed = time() - self.start_time
        if t_elapsed < self.transition_duration:
            fraction = t_elapsed / self.transition_duration
            self.joint_state.position = [
                init + fraction * (target - init)
                for init, target in zip(self.initial_angles, self.target_angles)
            ]
        else:
            return 0
        
        # 如果使用你的运动学模块，可以调用 update() 或者计算逆运动学，然后获得各个关节角
        # 例如：
        # self.kinematics.update(calculated_theta_list)
        # self.joint_state.position = calculated_theta_list

        self.publisher_.publish(self.joint_state)

    def IK(self, A, B, C, px, py, pz):
        temp_angles = self.kinematics.inverse_kinematic(A, B, C, px, py, pz)
        self.target_angles = temp_angles 

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
