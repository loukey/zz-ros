#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
from math import sin, cos, pi



class RobotPoseSimulator(Node):
    """
    机器人位姿模拟器
    发布模拟的机器人末端执行器位姿用于手眼标定
    """
    
    def __init__(self):
        super().__init__('robot_pose_simulator')
        
        # 声明参数
        self.declare_parameter('robot_pose_topic', '/robot_pose')
        self.declare_parameter('publish_rate', 10.0)
        
        # 获取参数
        self.robot_pose_topic = self.get_parameter('robot_pose_topic').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        
        # 创建发布者
        self.pose_publisher = self.create_publisher(
            PoseStamped, 
            self.robot_pose_topic, 
            10
        )
        
        # 当前位姿索引
        self.current_pose_index = 0
        self.pose_switch_counter = 0
        self.pose_switch_interval = int(self.publish_rate)  # 每3秒切换一次位姿
        
        # 预定义手眼标定位姿序列
        self.predefined_poses = self.generate_calibration_poses()
        
        # 定时器发布
        self.timer = self.create_timer(1.0/self.publish_rate, self.publish_pose)
        
        self.get_logger().info('机器人位姿模拟器启动')
        self.get_logger().info(f'发布话题: {self.robot_pose_topic}')
        self.get_logger().info(f'发布频率: {self.publish_rate} Hz')
        self.get_logger().info(f'总共 {len(self.predefined_poses)} 个标定位姿，每1秒切换一次')
        self.get_logger().info('开始自动发布位姿...')
    
    def generate_calibration_poses(self):
        """生成用于手眼标定的位姿序列"""
        poses = []
        
        # 基础位姿 - 工作台上方
        base_position = [0.5, 0.0, 0.3]  # 距离基座0.5m，高度0.3m
        
        # 位姿1: 正上方看下
        poses.append({
            'position': [0.5, 0.0, 0.4],
            'orientation': [0.0, 0.707, 0.0, 0.707]  # 向下看
        })
        
        # 位姿2: 左侧倾斜
        poses.append({
            'position': [0.45, 0.1, 0.35], 
            'orientation': [0.259, 0.707, -0.259, 0.612]
        })
        
        # 位姿3: 右侧倾斜
        poses.append({
            'position': [0.45, -0.1, 0.35],
            'orientation': [-0.259, 0.707, 0.259, 0.612]
        })
        
        # 位姿4: 前方倾斜
        poses.append({
            'position': [0.4, 0.0, 0.35],
            'orientation': [0.0, 0.612, 0.0, 0.791]
        })
        
        # 位姿5: 后方倾斜  
        poses.append({
            'position': [0.6, 0.0, 0.35],
            'orientation': [0.0, 0.791, 0.0, 0.612]
        })
        
        # 位姿6: 左前倾斜
        poses.append({
            'position': [0.4, 0.1, 0.32],
            'orientation': [0.183, 0.683, -0.183, 0.683]
        })
        
        # 位姿7: 右前倾斜
        poses.append({
            'position': [0.4, -0.1, 0.32],
            'orientation': [-0.183, 0.683, 0.183, 0.683]
        })
        
        # 位姿8: 左后倾斜
        poses.append({
            'position': [0.6, 0.1, 0.32],
            'orientation': [0.183, 0.683, -0.183, 0.683]
        })
        
        # 位姿9: 右后倾斜
        poses.append({
            'position': [0.6, -0.1, 0.32],
            'orientation': [-0.183, 0.683, 0.183, 0.683]
        })
        
        # 位姿10: 大角度倾斜
        poses.append({
            'position': [0.45, 0.0, 0.25],
            'orientation': [0.0, 0.5, 0.0, 0.866]
        })
        
        return poses
    
    def publish_pose(self):
        """发布当前位姿"""
        if not self.predefined_poses:
            return
        
        # 获取当前位姿
        current_pose = self.predefined_poses[self.current_pose_index]
        
        # 创建PoseStamped消息
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'base_link'
        
        # 设置位置
        pose_msg.pose.position.x = current_pose['position'][0]
        pose_msg.pose.position.y = current_pose['position'][1] 
        pose_msg.pose.position.z = current_pose['position'][2]
        
        # 设置姿态(四元数)
        pose_msg.pose.orientation.x = current_pose['orientation'][0]
        pose_msg.pose.orientation.y = current_pose['orientation'][1]
        pose_msg.pose.orientation.z = current_pose['orientation'][2]
        pose_msg.pose.orientation.w = current_pose['orientation'][3]
        
        # 发布
        self.pose_publisher.publish(pose_msg)
        
        # 自动切换位姿逻辑
        self.pose_switch_counter += 1
        if self.pose_switch_counter >= self.pose_switch_interval:
            self.next_pose()
            self.pose_switch_counter = 0
    
    def next_pose(self):
        """切换到下一个位姿"""
        self.current_pose_index = (self.current_pose_index + 1) % len(self.predefined_poses)
        current_pose = self.predefined_poses[self.current_pose_index]
        self.get_logger().info(f'自动切换到位姿 {self.current_pose_index + 1}/{len(self.predefined_poses)}: '
                              f'位置[{current_pose["position"][0]:.3f}, '
                              f'{current_pose["position"][1]:.3f}, '
                              f'{current_pose["position"][2]:.3f}]')
    



def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        robot_pose_simulator = RobotPoseSimulator()
        rclpy.spin(robot_pose_simulator)
    except KeyboardInterrupt:
        print("程序被中断")
    except Exception as e:
        print(f"程序运行错误: {str(e)}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main() 