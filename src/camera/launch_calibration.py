#!/usr/bin/env python3
"""
相机标定启动脚本
使用方法：
    python3 launch_calibration.py --camera_topic /camera/image_raw
"""

import argparse
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description='启动相机标定节点')
    parser.add_argument('--camera_topic', 
                        default='/camera/image_raw',
                        help='相机话题名称 (默认: /camera/image_raw)')
    parser.add_argument('--chessboard_size', 
                        default='9,6',
                        help='棋盘格尺寸 (默认: 9,6)')
    parser.add_argument('--square_size', 
                        type=float, 
                        default=0.025,
                        help='棋盘格方格尺寸(米) (默认: 0.025)')
    parser.add_argument('--save_dir', 
                        default='./calibration_data',
                        help='标定数据保存目录 (默认: ./calibration_data)')
    parser.add_argument('--min_samples', 
                        type=int, 
                        default=20,
                        help='最少标定图像数 (默认: 20)')
    
    args = parser.parse_args()
    
    # 构建ROS2命令
    cmd = [
        'ros2', 'run', 'camera', 'camera_calibration',
        '--ros-args',
        '-p', f'camera_topic:={args.camera_topic}',
        '-p', f'chessboard_size:={args.chessboard_size}',
        '-p', f'square_size:={args.square_size}',
        '-p', f'save_dir:={args.save_dir}',
        '-p', f'min_samples:={args.min_samples}'
    ]
    
    print("启动相机标定节点...")
    print(f"相机话题: {args.camera_topic}")
    print(f"棋盘格尺寸: {args.chessboard_size}")
    print(f"方格尺寸: {args.square_size}m")
    print(f"保存目录: {args.save_dir}")
    print(f"最少图像数: {args.min_samples}")
    print()
    
    try:
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError as e:
        print(f"启动失败: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("程序被中断")
        sys.exit(0)


if __name__ == '__main__':
    main() 