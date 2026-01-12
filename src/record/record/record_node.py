#!/usr/bin/env python3
"""
独立数据录制节点 (Independent Data Recorder Node)
功能：
1. 接收 /recording/control 服务指令 (start/stop)
2. 自动管理存储目录 (./record_data/record_n)
3. 同步订阅图像和机械臂状态，并写入磁盘
"""

import os
import sys
import time
import csv
import cv2
import threading
import numpy as np
from queue import Queue

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_srvs.srv import SetBool
from cv_bridge import CvBridge

class DataRecorderNode(Node):
    def __init__(self):
        super().__init__('data_recorder_node')
        
        # 参数配置：保存数据在当前运行目录下的 record_data 文件夹
        self.base_dir = './record_data'
        
        # 状态变量
        self.is_recording = False
        self.video_writer = None
        self.csv_file = None
        self.csv_writer = None
        self.frame_count = 0
        self.latest_joint_state = None 
        self.joint_state_lock = threading.Lock()
        
        # 工具
        self.bridge = CvBridge()
        
        # 1. 服务 Server
        self.srv = self.create_service(SetBool, '/recording/control', self.handle_control)
        
        # 2. 订阅者 (Subscribers)
        self.image_sub = self.create_subscription(
            Image, 
            '/camera/color/image_raw', 
            self.image_callback, 
            10
        )
        
        self.joint_sub = self.create_subscription(
            JointState, 
            '/robot/joint_states', 
            self.joint_callback, 
            10
        )
        
        self.get_logger().info(f'数据录制节点已启动，保存根目录: {os.path.abspath(self.base_dir)}')
        
        if not os.path.exists(self.base_dir):
            os.makedirs(self.base_dir)

    def handle_control(self, request, response):
        """处理 Start/Stop 请求"""
        if request.data:
            # === START ===
            if self.is_recording:
                response.success = False
                response.message = "Already recording"
                return response
            
            try:
                save_dir = self._prepare_directory()
                self._init_writers(save_dir)
                self.is_recording = True
                
                response.success = True
                response.message = f"Recording started at {save_dir}"
                self.get_logger().info(response.message)
                
            except Exception as e:
                response.success = False
                response.message = f"Failed to start: {str(e)}"
                self.get_logger().error(response.message)
                self._cleanup()
                
        else:
            # === STOP ===
            if not self.is_recording:
                response.success = True
                response.message = "Not recording"
                return response
            
            self.is_recording = False
            self._cleanup()
            
            response.success = True
            response.message = f"Recording stopped. Saved {self.frame_count} frames."
            self.get_logger().info(response.message)
            
        return response

    def joint_callback(self, msg):
        """缓存最新的关节状态"""
        with self.joint_state_lock:
            self.latest_joint_state = msg

    def image_callback(self, msg):
        """图像回调：触发写入"""
        if not self.is_recording:
            return
            
        try:
            # 1. 转换图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 2. 获取当前时刻的关节状态
            current_joints = []
            with self.joint_state_lock:
                if self.latest_joint_state:
                    current_joints = list(self.latest_joint_state.position)
                else:
                    current_joints = [0.0] * 6
            
            # 3. 写入数据
            timestamp = time.time()
            
            # Video
            if self.video_writer:
                self.video_writer.write(cv_image)
            
            # CSV
            if self.csv_writer:
                row = [self.frame_count, timestamp] + current_joints
                self.csv_writer.writerow(row)
                
            self.frame_count += 1
            
            if self.frame_count % 30 == 0:
                print(f'\r已录制 {self.frame_count} 帧...', end='', flush=True)
                
        except Exception as e:
            self.get_logger().error(f"Error processing frame: {e}")

    def _prepare_directory(self):
        """扫描并创建 record_n 目录"""
        n = 0
        while True:
            dir_name = os.path.join(self.base_dir, f'record_{n}')
            if not os.path.exists(dir_name):
                os.makedirs(dir_name)
                return dir_name
            n += 1

    def _init_writers(self, save_dir):
        """初始化视频和CSV写入器"""
        video_path = os.path.join(save_dir, 'video.mp4')
        # mp4v 编码兼容性好
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        # 假设分辨率 640x480, FPS 30 (应根据实际相机的输出调整)
        self.video_writer = cv2.VideoWriter(video_path, fourcc, 30.0, (640, 480))
        
        csv_path = os.path.join(save_dir, 'data.csv')
        self.csv_file = open(csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['frame_index', 'timestamp', 'j1', 'j2', 'j3', 'j4', 'j5', 'j6'])
        
        self.frame_count = 0

    def _cleanup(self):
        """释放资源"""
        if self.video_writer:
            self.video_writer.release()
            self.video_writer = None
            
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None
        print("\nCleaned up.")

def main(args=None):
    rclpy.init(args=args)
    node = DataRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
