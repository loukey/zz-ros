from rclpy.node import Node
from geometry_msgs.msg import Pose
from core.kinematic import quaternion_from_euler


class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self.publisher_ = self.create_publisher(Pose, 'target_pose', 10)

    def publish(self, pose_values):
        pose_msg = Pose()
        # 将欧拉角转换为四元数
        roll = pose_values[0]
        pitch = pose_values[1]
        yaw = pose_values[2]
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
        pose_msg.orientation.x = qx
        pose_msg.orientation.y = qy
        pose_msg.orientation.z = qz
        pose_msg.orientation.w = qw

        pose_msg.position.x = pose_values[3]
        pose_msg.position.y = pose_values[4]
        pose_msg.position.z = pose_values[5]

        self.publisher_.publish(pose_msg)
        self.get_logger().info(f"Published Pose: position=({pose_msg.position.x}, {pose_msg.position.y}, {pose_msg.position.z}) "
                                 f"orientation=({qx}, {qy}, {qz}, {qw})")
        