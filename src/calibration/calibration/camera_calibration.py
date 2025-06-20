#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import yaml
import argparse
import threading
from datetime import datetime
from typing import List, Tuple, Optional


class CameraCalibration(Node):
    """
    ROS2相机标定节点
    功能：
    1. 订阅相机话题
    2. 收集标定图像
    3. 执行相机标定
    4. 保存标定参数
    """
    
    def __init__(self):
        super().__init__('camera_calibration')
        
        # 声明参数
        self.declare_parameter('camera_topic', '/camera/color/image_raw')
        self.declare_parameter('chessboard_size', '9,6')  # 棋盘格内角点数目
        self.declare_parameter('square_size', 0.025)  # 棋盘格方格尺寸(米)
        self.declare_parameter('save_dir', './calibration_data')
        self.declare_parameter('min_samples', 20)  # 最少标定图像数
        
        # 获取参数
        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        chessboard_str = self.get_parameter('chessboard_size').get_parameter_value().string_value
        self.chessboard_size = tuple(map(int, chessboard_str.split(',')))
        self.square_size = self.get_parameter('square_size').get_parameter_value().double_value
        self.save_dir = self.get_parameter('save_dir').get_parameter_value().string_value
        self.min_samples = self.get_parameter('min_samples').get_parameter_value().integer_value
        
        # 初始化变量
        self.bridge = CvBridge()
        self.calibration_data = []  # 存储标定数据
        self.object_points = []  # 3D点
        self.image_points = []   # 2D点
        self.image_size = None
        self.is_calibrating = False
        self.collection_mode = False
        
        # 创建保存目录
        os.makedirs(self.save_dir, exist_ok=True)
        
        # 订阅相机话题
        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10
        )
        
        # 创建棋盘格世界坐标系中的点
        self.prepare_object_points()
        
        # 初始化标定参数
        self.camera_matrix = None
        self.dist_coeffs = None
        self.calibration_error = None
        
        self.get_logger().info(f'相机标定节点启动')
        self.get_logger().info(f'订阅话题: {self.camera_topic}')
        self.get_logger().info(f'棋盘格尺寸: {self.chessboard_size}')
        self.get_logger().info(f'方格尺寸: {self.square_size}m')
        self.get_logger().info(f'保存路径: {self.save_dir}')
        
        # 启动交互式界面
        self.start_interactive_interface()
    
    def prepare_object_points(self):
        """准备棋盘格的3D点坐标"""
        # 创建棋盘格角点的3D坐标
        objp = np.zeros((self.chessboard_size[0] * self.chessboard_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.chessboard_size[0], 0:self.chessboard_size[1]].T.reshape(-1, 2)
        objp *= self.square_size
        self.objp = objp
    
    def image_callback(self, msg):
        """图像回调函数"""
        try:
            # 将ROS图像转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            if self.image_size is None:
                self.image_size = (cv_image.shape[1], cv_image.shape[0])
            
            # 如果在收集模式，处理图像
            if self.collection_mode:
                self.process_calibration_image(cv_image)
            
        except Exception as e:
            self.get_logger().error(f'图像处理错误: {str(e)}')
    
    def process_calibration_image(self, image):
        """处理标定图像"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # 寻找棋盘格角点
        ret, corners = cv2.findChessboardCorners(gray, self.chessboard_size, None)
        
        if ret:
            # 亚像素精度角点检测
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            
            # 绘制角点
            cv2.drawChessboardCorners(image, self.chessboard_size, corners2, ret)
            
            # 显示找到角点的图像
            cv2.imshow('Calibration', image)
            key = cv2.waitKey(1) & 0xFF
            
            # 按空格键保存标定图像
            if key == ord(' '):
                self.save_calibration_image(corners2, image)
            
        else:
            # 显示原图像
            cv2.imshow('Calibration', image)
            cv2.waitKey(1)
    
    def save_calibration_image(self, corners, image):
        """保存标定图像和角点数据"""
        # 保存角点数据
        self.object_points.append(self.objp)
        self.image_points.append(corners)
        
        # 保存图像
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        image_path = os.path.join(self.save_dir, f'calib_image_{timestamp}.jpg')
        cv2.imwrite(image_path, image)
        
        self.get_logger().info(f'已保存标定图像 {len(self.image_points)}/{self.min_samples}: {image_path}')
        
        # 检查是否收集到足够的图像
        if len(self.image_points) >= self.min_samples:
            self.get_logger().info(f'已收集足够的标定图像 ({len(self.image_points)}张)')
            self.get_logger().info('输入 \'calibrate\' 开始标定')
    
    def perform_calibration(self):
        """执行相机标定"""
        if len(self.image_points) < self.min_samples:
            self.get_logger().error(f'标定图像不足，需要至少 {self.min_samples} 张图像')
            return False
        
        self.get_logger().info('开始相机标定...')
        
        try:
            # 执行相机标定
            ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
                self.object_points, 
                self.image_points, 
                self.image_size, 
                None, 
                None
            )
            
            if ret:
                self.camera_matrix = camera_matrix
                self.dist_coeffs = dist_coeffs
                self.calibration_error = ret
                
                # 计算重投影误差
                total_error = 0
                for i in range(len(self.object_points)):
                    imgpoints2, _ = cv2.projectPoints(
                        self.object_points[i], 
                        rvecs[i], 
                        tvecs[i], 
                        camera_matrix, 
                        dist_coeffs
                    )
                    error = cv2.norm(self.image_points[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
                    total_error += error
                
                mean_error = total_error / len(self.object_points)
                
                self.get_logger().info('相机标定完成！')
                self.get_logger().info(f'标定误差: {ret:.6f}')
                self.get_logger().info(f'平均重投影误差: {mean_error:.6f} 像素')
                
                # 保存标定结果
                self.save_calibration_results(camera_matrix, dist_coeffs, ret, mean_error)
                return True
            else:
                self.get_logger().error('相机标定失败')
                return False
                
        except Exception as e:
            self.get_logger().error(f'标定过程中发生错误: {str(e)}')
            return False
    
    def save_calibration_results(self, camera_matrix, dist_coeffs, rms_error, mean_error):
        """保存标定结果"""
        # 准备保存的数据
        calibration_data = {
            'camera_matrix': camera_matrix.tolist(),
            'distortion_coefficients': dist_coeffs.tolist(),
            'image_size': list(self.image_size),
            'rms_error': float(rms_error),
            'mean_reprojection_error': float(mean_error),
            'num_images': len(self.image_points),
            'chessboard_size': list(self.chessboard_size),
            'square_size': self.square_size,
            'calibration_date': datetime.now().isoformat()
        }
        
        # 保存为YAML文件
        yaml_path = os.path.join(self.save_dir, 'camera_calibration.yaml')
        with open(yaml_path, 'w') as f:
            yaml.dump(calibration_data, f, default_flow_style=False)
        
        # 保存为numpy文件
        np_path = os.path.join(self.save_dir, 'camera_calibration.npz')
        np.savez(np_path, 
                camera_matrix=camera_matrix,
                dist_coeffs=dist_coeffs,
                image_size=np.array(self.image_size))
        
        self.get_logger().info(f'标定结果已保存至: {yaml_path}')
        self.get_logger().info(f'标定参数已保存至: {np_path}')
        
        # 打印标定结果
        self.print_calibration_results(camera_matrix, dist_coeffs, rms_error, mean_error)
    
    def print_calibration_results(self, camera_matrix, dist_coeffs, rms_error, mean_error):
        """打印标定结果"""
        print("\n" + "="*50)
        print("相机标定结果")
        print("="*50)
        print(f"图像尺寸: {self.image_size[0]} x {self.image_size[1]}")
        print(f"标定图像数量: {len(self.image_points)}")
        print(f"RMS误差: {rms_error:.6f}")
        print(f"平均重投影误差: {mean_error:.6f} 像素")
        print("\n相机内参矩阵:")
        print(camera_matrix)
        print(f"\n焦距 (fx, fy): ({camera_matrix[0,0]:.2f}, {camera_matrix[1,1]:.2f})")
        print(f"主点 (cx, cy): ({camera_matrix[0,2]:.2f}, {camera_matrix[1,2]:.2f})")
        print("\n畸变系数:")
        print(f"k1: {dist_coeffs[0,0]:.6f}")
        print(f"k2: {dist_coeffs[0,1]:.6f}")
        print(f"p1: {dist_coeffs[0,2]:.6f}")
        print(f"p2: {dist_coeffs[0,3]:.6f}")
        print(f"k3: {dist_coeffs[0,4]:.6f}")
        print("="*50)
    
    def load_calibration_results(self, file_path):
        """加载标定结果"""
        try:
            if file_path.endswith('.yaml'):
                with open(file_path, 'r') as f:
                    data = yaml.safe_load(f)
                self.camera_matrix = np.array(data['camera_matrix'])
                self.dist_coeffs = np.array(data['distortion_coefficients'])
            elif file_path.endswith('.npz'):
                data = np.load(file_path)
                self.camera_matrix = data['camera_matrix']
                self.dist_coeffs = data['dist_coeffs']
            else:
                raise ValueError("不支持的文件格式")
            
            self.get_logger().info(f'标定结果加载成功: {file_path}')
            return True
        except Exception as e:
            self.get_logger().error(f'加载标定结果失败: {str(e)}')
            return False
    
    def start_interactive_interface(self):
        """启动交互式界面"""
        def interface_thread():
            print("\n" + "="*60)
            print("相机标定交互界面")
            print("="*60)
            print("命令说明:")
            print("  start    - 开始收集标定图像")
            print("  stop     - 停止收集标定图像")
            print("  calibrate - 执行相机标定")
            print("  status   - 显示当前状态")
            print("  clear    - 清除已收集的图像")
            print("  load     - 加载标定结果")
            print("  quit     - 退出程序")
            print("="*60)
            print("在收集模式下，按空格键保存当前图像用于标定")
            print("="*60)
            
            while rclpy.ok():
                try:
                    command = input("\n请输入命令: ").strip().lower()
                    
                    if command == 'start':
                        self.collection_mode = True
                        print("开始收集标定图像模式")
                        print("将棋盘格放在相机前，按空格键保存图像")
                        
                    elif command == 'stop':
                        self.collection_mode = False
                        cv2.destroyAllWindows()
                        print("停止收集标定图像")
                        
                    elif command == 'calibrate':
                        self.collection_mode = False
                        cv2.destroyAllWindows()
                        self.perform_calibration()
                        
                    elif command == 'status':
                        print(f"当前状态:")
                        print(f"  收集模式: {'开启' if self.collection_mode else '关闭'}")
                        print(f"  已收集图像: {len(self.image_points)}")
                        print(f"  最少需要: {self.min_samples}")
                        print(f"  相机话题: {self.camera_topic}")
                        print(f"  棋盘格尺寸: {self.chessboard_size}")
                        
                    elif command == 'clear':
                        self.object_points.clear()
                        self.image_points.clear()
                        print("已清除所有收集的标定图像")
                        
                    elif command == 'load':
                        file_path = input("请输入标定文件路径: ").strip()
                        self.load_calibration_results(file_path)
                        
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


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        camera_calibration = CameraCalibration()
        
        # 自旋
        rclpy.spin(camera_calibration)
        
    except KeyboardInterrupt:
        print("程序被中断")
    except Exception as e:
        print(f"程序运行错误: {str(e)}")
    finally:
        # 清理
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
