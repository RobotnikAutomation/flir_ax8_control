
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import src.flir.flir
import json

class FlirPublisher(Node):
    def __init__(self):
        super().__init__('flir_engine_node')
        self.flir = src.flir.flir.Flir()
        self.publisher_ = self.create_publisher(String, 'flir_engine', 10)
        self.timer = self.create_timer(0.1, self.publish_boxes)

    def publish_boxes(self):
        status = self.flir.getBoxes()
        msg = String()
        msg.data = json.dumps(status)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FlirPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
