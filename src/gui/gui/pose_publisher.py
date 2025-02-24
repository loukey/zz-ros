from rclpy.node import Node
from core.kinematic import quaternion_from_euler
from interface.msg import Pose


class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self.publisher_ = self.create_publisher(Pose, 'target_pose', 10)

    def publish(self, pose_values):
        pose_msg = Pose()
        pose_msg.angles = pose_values
        self.publisher_.publish(pose_msg)

        