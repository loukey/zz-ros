# """
# ROS控制器模块，用于与ROS系统进行交互
# """
# import rclpy
# from rclpy.node import Node
# from interface.msg import Pose
# from threading import Thread, Lock
# import time
# import numpy as np


# class PosePublisher(Node):
#     """位姿发布器"""
#     def __init__(self):
#         super().__init__("pose_publisher")
#         self.publisher_ = self.create_publisher(Pose, "position_now", 10)
#         self.position_now = [0.0] * 6
    
#     def publish_pose(self, position_now):
#         """发布位姿"""
#         msg = Pose()
#         msg.positions = position_now  # 使用positions字段
#         self.publisher_.publish(msg)
#         self.get_logger().info("position_now: {}".format(position_now))


class ROSController:
    """ROS控制器类，用于与ROS系统进行交互"""
    
    def __init__(self):
        """初始化ROS控制器"""
        pass
        # self.node = None
        # self.publisher = None
        # self.ros_thread = None
        # self.is_running = False
        # self.initialized = False
        
#         # 轨迹发布相关属性
#         self.trajectory_lock = Lock()
#         self.trajectory_thread = None
#         self.is_publishing_trajectory = False
#         self.trajectory_points = []
#         self.trajectory_times = []
#         self.trajectory_index = 0
    
#     def initialize(self):
#         """初始化ROS节点和发布器"""
#         if self.initialized:
#             return True
            
#         try:
#             # 初始化ROS
#             if not rclpy.ok():
#                 rclpy.init()
            
#             # 创建节点
#             self.node = PosePublisher()
            
#             # 创建并启动ROS线程
#             self.is_running = True
#             self.ros_thread = Thread(target=self._ros_spin)
#             self.ros_thread.daemon = True  # 设置为守护线程，主线程结束时自动结束
#             self.ros_thread.start()
            
#             self.initialized = True
#             return True
            
#         except Exception as e:
#             print(f"ROS控制器初始化失败: {str(e)}")
#             self.shutdown()
#             return False
    
#     def _ros_spin(self):
#         """ROS消息处理循环"""
#         while self.is_running and rclpy.ok():
#             try:
#                 rclpy.spin_once(self.node, timeout_sec=0.01)
#             except Exception as e:
#                 print(f"ROS消息处理异常: {str(e)}")
#                 break
    
#     def publish_angles(self, angles):
#         """发布单个角度数据点到ROS系统
        
#         Args:
#             angles: 包含6个关节角度的列表或元组
#         """
#         if not self.initialized or not rclpy.ok():
#             return False
            
#         try:
#             # 发布消息
#             self.node.publish_pose(list(angles))
#             return True
            
#         except Exception as e:
#             print(f"发布关节角度失败: {str(e)}")
#             return False
    
#     def publish_trajectory(self, angles, start_angles=None, duration=5.0, frequency=0.01, curve_type="Trapezoid"):
#         """发布轨迹到ROS系统，按照指定频率同步发送
        
#         Args:
#             angles: 目标角度列表
#             start_angles: 起始角度列表，如果为None则使用全零
#             duration: 轨迹总时长(秒)
#             frequency: 发送频率(秒)
#             curve_type: 轨迹类型，"Trapezoid"或"S-Curve"
#         """
#         if not self.initialized or not rclpy.ok():
#             return False
        
#         # 停止正在发布的轨迹
#         self.stop_trajectory()
        
#         try:
#             # 检查和准备参数
#             if start_angles is None:
#                 start_angles = [0.0] * len(angles)
            
#             # 生成轨迹点
#             points, times = self._generate_trajectory(start_angles, angles, duration, frequency, curve_type)
            
#             # 设置轨迹数据
#             with self.trajectory_lock:
#                 self.trajectory_points = points
#                 self.trajectory_times = times
#                 self.trajectory_index = 0
            
#             # 创建并启动轨迹发布线程
#             self.is_publishing_trajectory = True
#             self.trajectory_thread = Thread(target=self._publish_trajectory_thread)
#             self.trajectory_thread.daemon = True
#             self.trajectory_thread.start()
            
#             return True
            
#         except Exception as e:
#             print(f"准备轨迹发布失败: {str(e)}")
#             return False
    
#     def _generate_trajectory(self, start_angles, end_angles, duration, frequency, curve_type):
#         """生成轨迹点和时间列表
        
#         Args:
#             start_angles: 起始角度列表
#             end_angles: 目标角度列表
#             duration: 轨迹总时长(秒)
#             frequency: 发送频率(秒)
#             curve_type: 轨迹类型，"Trapezoid"或"S-Curve"
        
#         Returns:
#             (points, times): 轨迹点列表和对应的时间点列表
#         """
#         # 轨迹点数量
#         num_points = int(duration / frequency) + 1
        
#         # 创建时间列表
#         times = np.linspace(0, duration, num_points)
        
#         # 针对每个关节生成轨迹
#         all_joint_positions = []
        
#         for i in range(len(start_angles)):
#             start = start_angles[i]
#             end = end_angles[i]
            
#             if curve_type == "Trapezoid":
#                 # 梯形速度规划
#                 # 加速度时间为1/4的总时长
#                 accel_time = duration / 4.0
                
#                 positions = []
#                 for t in times:
#                     if t < accel_time:  # 加速阶段
#                         pos = start + (end - start) * (t / duration) * (t / accel_time) / 2.0
#                     elif t < duration - accel_time:  # 匀速阶段
#                         pos = start + (end - start) * (t / duration - (accel_time / duration) / 4.0)
#                     else:  # 减速阶段
#                         decel_t = duration - t
#                         pos = end - (end - start) * (decel_t / duration) * (decel_t / accel_time) / 2.0
#                     positions.append(pos)
#             else:  # S-Curve
#                 # S形速度规划 (简化实现)
#                 positions = []
#                 for t in times:
#                     # 使用正弦函数模拟S曲线
#                     normalized_t = t / duration
#                     scale = (1 - np.cos(np.pi * normalized_t)) / 2.0
#                     pos = start + (end - start) * scale
#                     positions.append(pos)
            
#             all_joint_positions.append(positions)
        
#         # 将每个关节的轨迹转置为时间点的列表
#         points = []
#         for i in range(num_points):
#             point = [all_joint_positions[j][i] for j in range(len(start_angles))]
#             points.append(point)
        
#         return points, times
    
#     def _publish_trajectory_thread(self):
#         """轨迹发布线程函数"""
#         try:
#             start_time = time.time()
#             normal_completion = False
            
#             while self.is_publishing_trajectory and rclpy.ok():
#                 # 获取当前要发布的点
#                 with self.trajectory_lock:
#                     if self.trajectory_index >= len(self.trajectory_points):
#                         self.is_publishing_trajectory = False
#                         normal_completion = True  # 标记为正常完成
#                         break
                    
#                     current_point = self.trajectory_points[self.trajectory_index]
#                     expected_time = self.trajectory_times[self.trajectory_index]
#                     self.trajectory_index += 1
                
#                 # 发布当前点
#                 self.publish_angles(current_point)
                
#                 # 计算下一个发布时间
#                 elapsed = time.time() - start_time
#                 time_to_next = expected_time - elapsed
                
#                 # 等待到下一个发布时间
#                 if time_to_next > 0 and self.is_publishing_trajectory:
#                     time.sleep(time_to_next)
#                 else:
#                     # 如果已经落后于计划时间，跳过中间点
#                     with self.trajectory_lock:
#                         while (self.trajectory_index < len(self.trajectory_times) and 
#                                elapsed > self.trajectory_times[self.trajectory_index]):
#                             self.trajectory_index += 1
            
#             # 只有在正常完成轨迹时才发布最后一个点，被暂停时不发送最后一点
#             if normal_completion and self.trajectory_points and rclpy.ok():
#                 self.publish_angles(self.trajectory_points[-1])
#                 print("轨迹正常完成，发送最后一点")
#             else:
#                 print("轨迹被中断，不发送最后一点")
            
#         except Exception as e:
#             print(f"轨迹发布线程异常: {str(e)}")
#         finally:
#             self.is_publishing_trajectory = False
    
#     def stop_trajectory(self):
#         """停止正在发布的轨迹"""
#         # 先检查是否正在发布轨迹
#         if not self.is_publishing_trajectory:
#             return
            
#         print("正在停止ROS轨迹发布...")
        
#         # 设置停止标志
#         self.is_publishing_trajectory = False
        
#         # 等待轨迹线程结束，但设置较短的超时时间，避免长时间等待
#         if self.trajectory_thread and self.trajectory_thread.is_alive():
#             self.trajectory_thread.join(timeout=0.1)  # 等待最多100毫秒
            
#             # 如果线程仍在运行，则我们不再等待（避免阻塞UI）
#             if self.trajectory_thread.is_alive():
#                 print("ROS轨迹线程未能立即停止，但已设置停止标志")
#             else:
#                 print("ROS轨迹线程已停止")
            
#             # 无论线程是否已经结束，我们都不再引用它
#             self.trajectory_thread = None
            
#         # 清除轨迹数据
#         with self.trajectory_lock:
#             self.trajectory_points = []
#             self.trajectory_times = []
#             self.trajectory_index = 0
    
    def shutdown(self):
        """关闭ROS控制器"""
        # 由于我们的ROS功能实际上是禁用的，我们只需要添加一个空方法以避免错误
        pass
