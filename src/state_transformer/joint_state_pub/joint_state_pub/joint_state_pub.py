import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from interface.msg import Pose
import time
from core.kinematic import Kinematic6DOF


class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        # # 定时器，间隔0.1秒发布一次
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.joint_state = JointState()
        self.joint_state.name = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']

        self.initial_angles = [0.0] * 6
        self.target_angles  = [0.0] * 6
        self.update_angles = self.target_angles
        self.interval = 0.1
        self.transition_duration = 5.0
        self.start_time = time.time()

        self.pose_subscriber = self.create_subscription(Pose, 'target_pose', self.pose_callback, 10)
        self.kinematics = Kinematic6DOF()

    def pose_callback(self, msg):
        self.update_angles = msg.angles
        self.get_logger().info("update_angles: {}".format(self.update_angles))

            
    def timer_callback(self):
        if self.update_angles != self.target_angles:
            self.initial_angles = self.joint_state.position
            self.target_angles = self.update_angles
            self.start_time = time.time()
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        t_elapsed = time.time() - self.start_time
        if t_elapsed <= self.transition_duration + 1e-6:
            fraction = t_elapsed / self.transition_duration
            self.joint_state.position = [
                init + fraction * (target - init)
                for init, target in zip(self.initial_angles, self.target_angles)
            ]
            self.get_logger().info("joint_state.position: {}".format(self.joint_state.position))
        else:
            return 0
        self.publisher_.publish(self.joint_state)

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
