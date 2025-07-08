"""
机器人模型
处理实时位姿发布
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from PyQt5.QtCore import QObject, pyqtSignal, QThread, QTimer
from gui.kinematic import *
from gui.config import GlobalVars
import time


class RobotPosePublisher(Node):
    """机器人位姿发布节点"""
    
    def __init__(self):
        super().__init__('robot_pose_publisher')
        
        # 声明参数
        self.declare_parameter('robot_pose_topic', '/robot_pose')
        self.declare_parameter('publish_rate', 10.0)  # 发布频率
        
        # 获取参数
        self.robot_pose_topic = self.get_parameter('robot_pose_topic').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        
        # 创建发布者
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            self.robot_pose_topic,
            10
        )
        
        # 初始化运动学求解器
        self.kinematic_solver = Kinematic6DOF()
        
        # 创建定时器，定时发布位姿
        self.timer = self.create_timer(1.0/self.publish_rate, self.publish_current_pose)
        
        self.get_logger().info(f'机器人位姿发布节点启动，发布话题: {self.robot_pose_topic}')
        self.get_logger().info(f'发布频率: {self.publish_rate} Hz')
    
    def publish_current_pose(self):
        """定时发布当前位姿（从全局变量读取）"""
        try:
            # 从全局变量获取当前关节角度
            joint_angles = GlobalVars.get_current_joint_angles()
            
            # 检查数据是否有效（避免发布零位姿）
            if all(angle == 0.0 for angle in joint_angles):
                # 如果所有角度都为0，可能数据还未初始化，跳过本次发布
                return
            
            # 使用Kinematic6DOF类计算末端位姿
            quaternion, position = self.kinematic_solver.get_end_position(joint_angles)
            
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
            
            # 等待线程启动并创建节点
            max_wait_time = 5.0  # 最多等待5秒
            wait_interval = 0.1  # 每100ms检查一次
            elapsed_time = 0.0
            
            while elapsed_time < max_wait_time:
                self.pose_publisher_node = self.ros_thread.get_pose_publisher()
                if self.pose_publisher_node is not None:
                    break
                time.sleep(wait_interval)
                elapsed_time += wait_interval
            
            if self.pose_publisher_node is None:
                return False
            
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
            try:
                rclpy.init()
            except RuntimeError as e:
                # ROS2 already initialized
                pass
            
            # 创建位姿发布节点
            self.pose_publisher = RobotPosePublisher()
            
            while not self.should_stop:
                try:
                    rclpy.spin_once(self.pose_publisher, timeout_sec=0.1)
                except Exception as e:
                    break
                
        except Exception as e:
            pass
        finally:
            if self.pose_publisher:
                self.pose_publisher.destroy_node()
    
    def get_pose_publisher(self):
        """获取位姿发布节点"""
        return self.pose_publisher
    
    def stop(self):
        """停止线程"""
        self.should_stop = True
