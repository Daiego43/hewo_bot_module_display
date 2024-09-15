import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class FaceNode(Node):
    def __init__(self):
        super().__init__('face_node')
        self.face_state = {
            'face': 'hewo',
            'emotion': 'happy',
            'vector': [0, 0, 0, 0, 0, 0],
        }
        self.face_state_publisher = self.create_publisher(String, 'face_state', 10)
        # self.publish_face_state()
        self.create_timer(1, self.publish_face_state)

    def publish_face_state(self):
        json_str = json.dumps(self.face_state)
        msg = String()
        msg.data = json_str
        self.face_state_publisher.publish(msg)
        self.get_logger().info(f'Published face state: {json_str}')


def main(args=None):
    rclpy.init(args=args)
    face_node = FaceNode()
    rclpy.spin(face_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
