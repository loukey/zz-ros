import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from interface.msg import Pose


class JointStatePublisher(Node):
    def __init__(self, initial_position=[0.0] * 6):
        super().__init__('joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.joint_state = JointState()
        self.joint_state.name = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']

        self.position_now = initial_position
        self.pose_subscriber = self.create_subscription(Pose, 'position_now', self.pose_callback, 10)

    def pose_callback(self, msg):
        self.position_now = msg.positions
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_state.position = self.position_now
        self.publisher_.publish(self.joint_state)
        self.get_logger().info("position_now: {}".format(self.position_now))

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
