"""
机器人模型
处理实时位姿发布
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from PyQt5.QtCore import QObject, pyqtSignal, QThread
from gui.kinematic import Kinematic6DOF, quaternion_from_euler


class RobotPosePublisher(Node):
    """机器人位姿发布节点"""
    
    def __init__(self):
        super().__init__('robot_pose_publisher')
        
        # 声明参数
        self.declare_parameter('robot_pose_topic', '/robot_pose')
        
        # 获取参数
        self.robot_pose_topic = self.get_parameter('robot_pose_topic').get_parameter_value().string_value
        
        # 创建发布者
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            self.robot_pose_topic,
            10
        )
        
        # 初始化运动学求解器
        self.kinematic_solver = Kinematic6DOF()
        
        self.get_logger().info(f'机器人位姿发布节点启动，发布话题: {self.robot_pose_topic}')
    
    def publish_pose_from_joint_angles(self, joint_angles):
        """
        根据关节角度计算并发布末端位姿
        
        参数:
            joint_angles: 6个关节角度的列表 [theta1, theta2, theta3, theta4, theta5, theta6]
        """
        try:
            # 使用Kinematic6DOF类计算末端位姿
            A, B, C, position = self.kinematic_solver.get_end_position(joint_angles)
            
            # 将欧拉角转换为四元数
            quaternion = quaternion_from_euler(A, B, C)  # [x, y, z, w]
            
            # 创建PoseStamped消息
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'base_link'
            
            # 设置位置
            pose_msg.pose.position.x = float(position[0])
            pose_msg.pose.position.y = float(position[1])
            pose_msg.pose.position.z = float(position[2])
            
            # 设置姿态(四元数)
            pose_msg.pose.orientation.x = float(quaternion[0])
            pose_msg.pose.orientation.y = float(quaternion[1])
            pose_msg.pose.orientation.z = float(quaternion[2])
            pose_msg.pose.orientation.w = float(quaternion[3])
            
            # 发布
            self.pose_publisher.publish(pose_msg)
            
            self.get_logger().debug(f'发布位姿: 位置[{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]')
            
        except Exception as e:
            self.get_logger().error(f'发布位姿失败: {str(e)}')
    



class RobotModel(QObject):
    """机器人模型类 - 管理位姿发布"""
    
    # 定义信号
    error_occurred = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self.pose_publisher_node = None
        self.ros_thread = None
        self.is_publishing = False
        
    def start_pose_publishing(self):
        """启动位姿发布节点"""
        try:
            if self.is_publishing:
                return True
                
            # 创建ROS2线程
            self.ros_thread = RosThread()
            self.ros_thread.start()
            
            # 获取位姿发布节点
            self.pose_publisher_node = self.ros_thread.get_pose_publisher()
            
            self.is_publishing = True
            return True
            
        except Exception as e:
            self.error_occurred.emit(f"启动位姿发布失败: {str(e)}")
            return False
    
    def stop_pose_publishing(self):
        """停止位姿发布节点"""
        try:
            if not self.is_publishing:
                return True
                
            self.is_publishing = False
            
            if self.ros_thread:
                self.ros_thread.stop()
                self.ros_thread.wait()
                self.ros_thread = None
                
            self.pose_publisher_node = None
            return True
            
        except Exception as e:
            self.error_occurred.emit(f"停止位姿发布失败: {str(e)}")
            return False
    
    def publish_pose(self, joint_angles):
        """发布位姿"""
        try:
            if self.is_publishing and self.pose_publisher_node:
                self.pose_publisher_node.publish_pose_from_joint_angles(joint_angles)
        except Exception as e:
            self.error_occurred.emit(f"发布位姿失败: {str(e)}")


class RosThread(QThread):
    """ROS2线程"""
    
    def __init__(self):
        super().__init__()
        self.pose_publisher = None
        self.should_stop = False
        
    def run(self):
        """运行ROS2节点"""
        try:
            # 初始化ROS2
            if not rclpy.ok():
                rclpy.init()
            
            # 创建位姿发布节点
            self.pose_publisher = RobotPosePublisher()
            
            # 运行节点
            while not self.should_stop and rclpy.ok():
                rclpy.spin_once(self.pose_publisher, timeout_sec=0.1)
                
        except Exception as e:
            print(f"ROS线程运行错误: {str(e)}")
        finally:
            if self.pose_publisher:
                self.pose_publisher.destroy_node()
    
    def get_pose_publisher(self):
        """获取位姿发布节点"""
        return self.pose_publisher
    
    def stop(self):
        """停止线程"""
        self.should_stop = True
