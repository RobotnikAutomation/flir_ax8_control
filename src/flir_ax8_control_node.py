import rclpy
from rclpy.node import Node
from flir_ax8_control import FlirAx8Control

def main(args=None):
    rclpy.init(args=args)
    node = FlirAx8Control()
    node.get_logger().info(f'{node.get_name()}: starting')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
