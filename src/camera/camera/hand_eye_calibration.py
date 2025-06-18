#!/usr/bin/env python3
"""
Eye-in-Hand 手眼标定 ROS2节点
用于标定安装在机器人末端执行器上的相机与机器人坐标系之间的变换关系
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import yaml
import threading
from datetime import datetime
from typing import List, Tuple, Optional
import json

# 导入相机工具
from .camera_utils import CameraCalibrationUtils, create_your_camera_utils


class HandEyeCalibration(Node):
    """
    Eye-in-Hand 手眼标定节点
    
    标定流程：
    1. 将标定板固定在工作空间中
    2. 机器人移动到不同位姿观察标定板
    3. 记录每个位姿的机器人末端坐标和相机观察到的标定板位姿
    4. 求解从末端执行器到相机的固定变换
    """
    
    def __init__(self):
        super().__init__('hand_eye_calibration')
        
        # 声明参数
        self.declare_parameter('camera_topic', '/camera/color/image_raw')
        self.declare_parameter('robot_pose_topic', '/robot_pose')
        self.declare_parameter('chessboard_size', '9,6')
        self.declare_parameter('square_size', 0.025)
        self.declare_parameter('save_dir', './hand_eye_calibration_data')
        self.declare_parameter('min_poses', 10)  # 最少标定位姿数
        self.declare_parameter('calibration_file', './calibration_data/camera_calibration.yaml')
        
        # 获取参数
        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.robot_pose_topic = self.get_parameter('robot_pose_topic').get_parameter_value().string_value
        chessboard_str = self.get_parameter('chessboard_size').get_parameter_value().string_value
        self.chessboard_size = tuple(map(int, chessboard_str.split(',')))
        self.square_size = self.get_parameter('square_size').get_parameter_value().double_value
        self.save_dir = self.get_parameter('save_dir').get_parameter_value().string_value
        self.min_poses = self.get_parameter('min_poses').get_parameter_value().integer_value
        self.calibration_file = self.get_parameter('calibration_file').get_parameter_value().string_value
        
        # 初始化变量
        self.bridge = CvBridge()
        self.collection_mode = False
        self.current_robot_pose = None
        self.current_image = None
        
        # 标定数据存储
        self.robot_poses = []  # 机器人末端执行器位姿列表
        self.camera_poses = []  # 相机观察到的标定板位姿列表
        self.calibration_images = []  # 标定图像
        
        # 创建保存目录
        os.makedirs(self.save_dir, exist_ok=True)
        
        # 加载相机标定参数
        if os.path.exists(self.calibration_file):
            self.camera_utils = CameraCalibrationUtils(self.calibration_file)
            self.get_logger().info(f'加载相机标定参数: {self.calibration_file}')
        else:
            self.camera_utils = create_your_camera_utils()
            self.get_logger().info('使用内置相机标定参数')
        
        # 创建棋盘格3D点
        self.prepare_object_points()
        
        # 订阅话题
        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10
        )
        
        self.robot_pose_sub = self.create_subscription(
            PoseStamped,
            self.robot_pose_topic,
            self.robot_pose_callback,
            10
        )
        
        # 手眼标定结果
        self.hand_eye_transform = None
        self.calibration_error = None
        
        self.get_logger().info('Eye-in-Hand 手眼标定节点启动')
        self.get_logger().info(f'相机话题: {self.camera_topic}')
        self.get_logger().info(f'机器人位姿话题: {self.robot_pose_topic}')
        self.get_logger().info(f'棋盘格尺寸: {self.chessboard_size}')
        self.get_logger().info(f'最少位姿数: {self.min_poses}')
        
        # 启动交互界面
        self.start_interactive_interface()
    
    def prepare_object_points(self):
        """准备棋盘格3D点坐标"""
        objp = np.zeros((self.chessboard_size[0] * self.chessboard_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.chessboard_size[0], 0:self.chessboard_size[1]].T.reshape(-1, 2)
        objp *= self.square_size
        self.objp = objp
    
    def image_callback(self, msg):
        """图像回调函数"""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            if self.collection_mode:
                self.process_calibration_image(self.current_image)
        except Exception as e:
            self.get_logger().error(f'图像处理错误: {str(e)}')
    
    def robot_pose_callback(self, msg):
        """机器人位姿回调函数"""
        self.current_robot_pose = msg
    
    def process_calibration_image(self, image):
        """处理标定图像"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # 检测棋盘格角点
        ret, corners = cv2.findChessboardCorners(gray, self.chessboard_size, None)
        
        if ret:
            # 亚像素精度角点检测
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            
            # 绘制角点
            cv2.drawChessboardCorners(image, self.chessboard_size, corners2, ret)
            
            # 显示图像
            cv2.imshow('Hand-Eye Calibration', image)
            key = cv2.waitKey(1) & 0xFF
            
            # 按空格键保存标定数据
            if key == ord(' '):
                self.save_calibration_pose(corners2, image)
        else:
            # 显示原图像
            cv2.imshow('Hand-Eye Calibration', image)
            cv2.waitKey(1)
    
    def save_calibration_pose(self, corners, image):
        """保存标定位姿数据"""
        if self.current_robot_pose is None:
            self.get_logger().warn('未接收到机器人位姿数据')
            return
        
        # 计算标定板在相机坐标系中的位姿
        camera_pose = self.estimate_board_pose(corners)
        if camera_pose is None:
            self.get_logger().warn('无法估计标定板位姿')
            return
        
        # 保存机器人位姿 (末端执行器到基座的变换)
        robot_pose = self.pose_stamped_to_matrix(self.current_robot_pose)
        
        # 保存数据
        self.robot_poses.append(robot_pose)
        self.camera_poses.append(camera_pose)
        
        # 保存图像
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        image_path = os.path.join(self.save_dir, f'pose_{len(self.robot_poses)}_{timestamp}.jpg')
        cv2.imwrite(image_path, image)
        self.calibration_images.append(image_path)
        
        self.get_logger().info(f'已保存第 {len(self.robot_poses)} 个标定位姿')
        self.get_logger().info(f'机器人位姿: [{robot_pose[0,3]:.3f}, {robot_pose[1,3]:.3f}, {robot_pose[2,3]:.3f}]')
        self.get_logger().info(f'标定板位姿: [{camera_pose[0,3]:.3f}, {camera_pose[1,3]:.3f}, {camera_pose[2,3]:.3f}]')
        
        if len(self.robot_poses) >= self.min_poses:
            self.get_logger().info(f'已收集足够位姿数据 ({len(self.robot_poses)}个)')
            self.get_logger().info('输入 \'calibrate\' 开始手眼标定')
    
    def estimate_board_pose(self, corners):
        """估计标定板在相机坐标系中的位姿"""
        try:
            # 使用PnP求解标定板位姿
            success, rvec, tvec = cv2.solvePnP(
                self.objp,
                corners,
                self.camera_utils.camera_matrix,
                self.camera_utils.dist_coeffs
            )
            
            if success:
                # 转换为变换矩阵
                R, _ = cv2.Rodrigues(rvec)
                T = np.eye(4)
                T[:3, :3] = R
                T[:3, 3] = tvec.flatten()
                return T
            else:
                return None
        except Exception as e:
            self.get_logger().error(f'位姿估计错误: {str(e)}')
            return None
    
    def pose_stamped_to_matrix(self, pose_stamped):
        """将PoseStamped转换为4x4变换矩阵"""
        pose = pose_stamped.pose
        
        # 提取位置
        t = np.array([pose.position.x, pose.position.y, pose.position.z])
        
        # 提取四元数并转换为旋转矩阵
        q = np.array([pose.orientation.x, pose.orientation.y, 
                     pose.orientation.z, pose.orientation.w])
        
        # 四元数到旋转矩阵
        R = self.quaternion_to_rotation_matrix(q)
        
        # 构建4x4变换矩阵
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t
        
        return T
    
    def quaternion_to_rotation_matrix(self, q):
        """四元数转旋转矩阵"""
        x, y, z, w = q
        
        # 归一化四元数
        norm = np.sqrt(x*x + y*y + z*z + w*w)
        x, y, z, w = x/norm, y/norm, z/norm, w/norm
        
        # 计算旋转矩阵
        R = np.array([
            [1-2*y*y-2*z*z, 2*x*y-2*z*w, 2*x*z+2*y*w],
            [2*x*y+2*z*w, 1-2*x*x-2*z*z, 2*y*z-2*x*w],
            [2*x*z-2*y*w, 2*y*z+2*x*w, 1-2*x*x-2*y*y]
        ])
        
        return R
    
    def perform_hand_eye_calibration(self):
        """执行手眼标定"""
        if len(self.robot_poses) < self.min_poses:
            self.get_logger().error(f'位姿数据不足，需要至少 {self.min_poses} 个位姿')
            return False
        
        self.get_logger().info('开始Eye-in-Hand手眼标定...')
        
        try:
            # 手眼标定数学原理说明：
            # Eye-in-Hand配置下，我们要求解方程: A * X = X * B
            # 其中：
            # A = T_gripper1_to_gripper2 (末端执行器在不同位姿间的相对变换)
            # B = T_board1_to_board2_in_cam (标定板在相机中的相对变换)
            # X = T_gripper_to_cam (要求解的手眼变换)
            
            # OpenCV的calibrateHandEye函数需要绝对位姿，内部会计算相对变换
            # 输入要求：
            # R_gripper2base, t_gripper2base: 末端执行器到基座的变换列表
            # R_target2cam, t_target2cam: 标定板到相机的变换列表
            
            R_gripper2base = []  # 机器人末端到基座的旋转
            t_gripper2base = []  # 机器人末端到基座的平移
            R_target2cam = []    # 标定板到相机的旋转
            t_target2cam = []    # 标定板到相机的平移
            
            for robot_pose, camera_pose in zip(self.robot_poses, self.camera_poses):
                # 机器人位姿转换：从 T_base_to_gripper 转为 T_gripper_to_base
                T_base_to_gripper = robot_pose
                T_gripper_to_base = np.linalg.inv(T_base_to_gripper)
                R_gripper2base.append(T_gripper_to_base[:3, :3])
                t_gripper2base.append(T_gripper_to_base[:3, 3])
                
                # 相机位姿转换：从 T_cam_to_board 转为 T_board_to_cam
                T_cam_to_board = camera_pose
                T_board_to_cam = np.linalg.inv(T_cam_to_board)
                R_target2cam.append(T_board_to_cam[:3, :3])
                t_target2cam.append(T_board_to_cam[:3, 3])
            
            # 使用OpenCV进行手眼标定
            R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
                R_gripper2base, t_gripper2base,
                R_target2cam, t_target2cam,
                method=cv2.CALIB_HAND_EYE_TSAI
            )
            
            # 构建手眼变换矩阵 (末端执行器到相机)
            self.hand_eye_transform = np.eye(4)
            self.hand_eye_transform[:3, :3] = R_cam2gripper
            self.hand_eye_transform[:3, 3] = t_cam2gripper.flatten()
            
            # 计算标定误差
            self.calibration_error = self.compute_calibration_error()
            
            self.get_logger().info('手眼标定完成！')
            self.get_logger().info(f'标定误差: {self.calibration_error:.6f} 米')
            
            # 保存标定结果
            self.save_hand_eye_results()
            return True
            
        except Exception as e:
            self.get_logger().error(f'手眼标定过程中发生错误: {str(e)}')
            return False
    
    def compute_calibration_error(self):
        """计算标定误差"""
        if self.hand_eye_transform is None:
            return float('inf')
        
        total_error = 0.0
        
        for i in range(len(self.robot_poses)):
            for j in range(i + 1, len(self.robot_poses)):
                # 计算机器人位姿变化
                robot_rel = np.linalg.inv(self.robot_poses[i]) @ self.robot_poses[j]
                
                # 计算相机观察到的变化
                camera_rel = self.camera_poses[j] @ np.linalg.inv(self.camera_poses[i])
                
                # 通过手眼变换计算预测的相机变化
                predicted_camera_rel = np.linalg.inv(self.hand_eye_transform) @ robot_rel @ self.hand_eye_transform
                
                # 计算误差
                error_matrix = camera_rel @ np.linalg.inv(predicted_camera_rel)
                
                # 平移误差
                translation_error = np.linalg.norm(error_matrix[:3, 3])
                total_error += translation_error
        
        num_pairs = len(self.robot_poses) * (len(self.robot_poses) - 1) // 2
        return total_error / num_pairs if num_pairs > 0 else float('inf')
    
    def save_hand_eye_results(self):
        """保存手眼标定结果"""
        if self.hand_eye_transform is None:
            return
        
        # 提取旋转和平移
        R = self.hand_eye_transform[:3, :3]
        t = self.hand_eye_transform[:3, 3]
        
        # 转换为四元数
        q = self.rotation_matrix_to_quaternion(R)
        
        # 转换为欧拉角
        euler = self.rotation_matrix_to_euler(R)
        
        # 准备保存数据
        calibration_data = {
            'hand_eye_transform_matrix': self.hand_eye_transform.tolist(),
            'translation': {
                'x': float(t[0]),
                'y': float(t[1]),
                'z': float(t[2])
            },
            'rotation_quaternion': {
                'x': float(q[0]),
                'y': float(q[1]),
                'z': float(q[2]),
                'w': float(q[3])
            },
            'rotation_euler_degrees': {
                'roll': float(np.degrees(euler[0])),
                'pitch': float(np.degrees(euler[1])),
                'yaw': float(np.degrees(euler[2]))
            },
            'calibration_error': float(self.calibration_error),
            'num_poses': len(self.robot_poses),
            'calibration_date': datetime.now().isoformat(),
            'calibration_type': 'eye_in_hand'
        }
        
        # 保存为YAML文件
        yaml_path = os.path.join(self.save_dir, 'hand_eye_calibration.yaml')
        with open(yaml_path, 'w') as f:
            yaml.dump(calibration_data, f, default_flow_style=False)
        
        # 保存为numpy文件
        np_path = os.path.join(self.save_dir, 'hand_eye_calibration.npz')
        np.savez(np_path, 
                hand_eye_transform=self.hand_eye_transform,
                calibration_error=self.calibration_error)
        
        # 保存位姿数据
        poses_data = {
            'robot_poses': [pose.tolist() for pose in self.robot_poses],
            'camera_poses': [pose.tolist() for pose in self.camera_poses],
            'calibration_images': self.calibration_images
        }
        
        poses_path = os.path.join(self.save_dir, 'calibration_poses.json')
        with open(poses_path, 'w') as f:
            json.dump(poses_data, f, indent=2)
        
        self.get_logger().info(f'手眼标定结果已保存至: {yaml_path}')
        self.get_logger().info(f'标定参数已保存至: {np_path}')
        self.get_logger().info(f'位姿数据已保存至: {poses_path}')
        
        # 打印标定结果
        self.print_calibration_results()
    
    def rotation_matrix_to_quaternion(self, R):
        """旋转矩阵转四元数"""
        trace = np.trace(R)
        if trace > 0:
            s = np.sqrt(trace + 1.0) * 2
            w = 0.25 * s
            x = (R[2, 1] - R[1, 2]) / s
            y = (R[0, 2] - R[2, 0]) / s
            z = (R[1, 0] - R[0, 1]) / s
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
                w = (R[2, 1] - R[1, 2]) / s
                x = 0.25 * s
                y = (R[0, 1] + R[1, 0]) / s
                z = (R[0, 2] + R[2, 0]) / s
            elif R[1, 1] > R[2, 2]:
                s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
                w = (R[0, 2] - R[2, 0]) / s
                x = (R[0, 1] + R[1, 0]) / s
                y = 0.25 * s
                z = (R[1, 2] + R[2, 1]) / s
            else:
                s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
                w = (R[1, 0] - R[0, 1]) / s
                x = (R[0, 2] + R[2, 0]) / s
                y = (R[1, 2] + R[2, 1]) / s
                z = 0.25 * s
        
        return np.array([x, y, z, w])
    
    def rotation_matrix_to_euler(self, R):
        """旋转矩阵转欧拉角 (ZYX顺序)"""
        sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        
        singular = sy < 1e-6
        
        if not singular:
            x = np.arctan2(R[2, 1], R[2, 2])
            y = np.arctan2(-R[2, 0], sy)
            z = np.arctan2(R[1, 0], R[0, 0])
        else:
            x = np.arctan2(-R[1, 2], R[1, 1])
            y = np.arctan2(-R[2, 0], sy)
            z = 0
        
        return np.array([x, y, z])
    
    def print_calibration_results(self):
        """打印标定结果"""
        if self.hand_eye_transform is None:
            return
        
        R = self.hand_eye_transform[:3, :3]
        t = self.hand_eye_transform[:3, 3]
        q = self.rotation_matrix_to_quaternion(R)
        euler = self.rotation_matrix_to_euler(R)
        
        print("\n" + "="*60)
        print("Eye-in-Hand 手眼标定结果")
        print("="*60)
        print(f"使用位姿数量: {len(self.robot_poses)}")
        print(f"标定误差: {self.calibration_error:.6f} 米")
        
        print(f"\n末端执行器到相机的变换:")
        print(f"平移 (x, y, z): ({t[0]:.6f}, {t[1]:.6f}, {t[2]:.6f}) 米")
        print(f"四元数 (x, y, z, w): ({q[0]:.6f}, {q[1]:.6f}, {q[2]:.6f}, {q[3]:.6f})")
        print(f"欧拉角 (roll, pitch, yaw): ({np.degrees(euler[0]):.2f}°, {np.degrees(euler[1]):.2f}°, {np.degrees(euler[2]):.2f}°)")
        
        print(f"\n变换矩阵:")
        print(self.hand_eye_transform)
        print("="*60)
    
    def start_interactive_interface(self):
        """启动交互式界面"""
        def interface_thread():
            print("\n" + "="*60)
            print("Eye-in-Hand 手眼标定交互界面")
            print("="*60)
            print("操作流程:")
            print("1. 将标定板固定在机器人工作空间中")
            print("2. 移动机器人到不同位姿观察标定板")
            print("3. 在每个位姿下按空格键保存数据")
            print("4. 收集足够数据后执行标定")
            print("="*60)
            print("命令说明:")
            print("  start     - 开始收集标定数据")
            print("  stop      - 停止收集数据")
            print("  calibrate - 执行手眼标定")
            print("  status    - 显示当前状态")
            print("  clear     - 清除已收集的数据")
            print("  load      - 加载标定结果")
            print("  quit      - 退出程序")
            print("="*60)
            print("注意: 确保机器人位姿话题正在发布!")
            print("="*60)
            
            while rclpy.ok():
                try:
                    command = input("\n请输入命令: ").strip().lower()
                    
                    if command == 'start':
                        self.collection_mode = True
                        print("开始收集手眼标定数据")
                        print("移动机器人到不同位姿，在每个位姿下按空格键保存")
                        
                    elif command == 'stop':
                        self.collection_mode = False
                        cv2.destroyAllWindows()
                        print("停止收集标定数据")
                        
                    elif command == 'calibrate':
                        self.collection_mode = False
                        cv2.destroyAllWindows()
                        self.perform_hand_eye_calibration()
                        
                    elif command == 'status':
                        print(f"当前状态:")
                        print(f"  收集模式: {'开启' if self.collection_mode else '关闭'}")
                        print(f"  已收集位姿: {len(self.robot_poses)}")
                        print(f"  最少需要: {self.min_poses}")
                        print(f"  机器人位姿可用: {'是' if self.current_robot_pose else '否'}")
                        print(f"  相机图像可用: {'是' if self.current_image is not None else '否'}")
                        
                    elif command == 'clear':
                        self.robot_poses.clear()
                        self.camera_poses.clear()
                        self.calibration_images.clear()
                        print("已清除所有收集的标定数据")
                        
                    elif command == 'load':
                        file_path = input("请输入手眼标定文件路径: ").strip()
                        self.load_hand_eye_results(file_path)
                        
                    elif command == 'quit':
                        print("退出程序...")
                        cv2.destroyAllWindows()
                        rclpy.shutdown()
                        break
                        
                    else:
                        print("未知命令，请重新输入")
                        
                except KeyboardInterrupt:
                    print("\n检测到中断信号，退出程序...")
                    cv2.destroyAllWindows()
                    rclpy.shutdown()
                    break
                except Exception as e:
                    print(f"命令执行错误: {str(e)}")
        
        # 在单独线程中运行交互界面
        thread = threading.Thread(target=interface_thread, daemon=True)
        thread.start()
    
    def load_hand_eye_results(self, file_path):
        """加载手眼标定结果"""
        try:
            if file_path.endswith('.yaml'):
                with open(file_path, 'r') as f:
                    data = yaml.safe_load(f)
                self.hand_eye_transform = np.array(data['hand_eye_transform_matrix'])
                self.calibration_error = data.get('calibration_error', 0.0)
            elif file_path.endswith('.npz'):
                data = np.load(file_path)
                self.hand_eye_transform = data['hand_eye_transform']
                self.calibration_error = float(data['calibration_error'])
            else:
                raise ValueError("不支持的文件格式")
            
            self.get_logger().info(f'手眼标定结果加载成功: {file_path}')
            self.print_calibration_results()
            return True
        except Exception as e:
            self.get_logger().error(f'加载手眼标定结果失败: {str(e)}')
            return False


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        hand_eye_calibration = HandEyeCalibration()
        rclpy.spin(hand_eye_calibration)
    except KeyboardInterrupt:
        print("程序被中断")
    except Exception as e:
        print(f"程序运行错误: {str(e)}")
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 