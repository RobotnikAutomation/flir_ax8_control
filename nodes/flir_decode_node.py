pubs = {}
def callback(data):

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import json

class FlirDecodeNode(Node):
    def __init__(self):
        super().__init__('flir_decode')
        self.pubs = {}
        self.subscription = self.create_subscription(
            String,
            'udp/flir_engine',
            self.callback,
            10)

    def callback(self, data):
        boxes = json.loads(data.data)
        for b in boxes:
            bn = b['boxNumber']
            if bn not in self.pubs:
                self.pubs[bn] = {}
                self.pubs[bn]['minT'] = self.create_publisher(Float32, f'flir_engine/{bn}/minT', 10)
                self.pubs[bn]['avgT'] = self.create_publisher(Float32, f'flir_engine/{bn}/avgT', 10)
                self.pubs[bn]['maxT'] = self.create_publisher(Float32, f'flir_engine/{bn}/maxT', 10)
            if b['active'] == '"true"':
                self.pubs[bn]['minT'].publish(Float32(data=float(b['minT'].strip('"')[:-1])))
                self.pubs[bn]['avgT'].publish(Float32(data=float(b['avgT'].strip('"')[:-1])))
                self.pubs[bn]['maxT'].publish(Float32(data=float(b['maxT'].strip('"')[:-1])))

def main(args=None):
    rclpy.init(args=args)
    node = FlirDecodeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()