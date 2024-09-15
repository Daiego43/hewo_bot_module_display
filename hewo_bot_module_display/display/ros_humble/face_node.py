import rclpy
from rclpy.node import Node



class FaceNode(Node):
    def __init__(self):
        super().__init__('face_node')
        self.get_logger().info('Face Node has been started')
        self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('Face Node is running')


def main(args=None):
    rclpy.init(args=args)
    face_node = FaceNode()
    rclpy.spin(face_node)
    face_node.destroy_node()
    rclpy.shutdown()