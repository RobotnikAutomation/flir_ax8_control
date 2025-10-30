tempData = {'t':[],'min':[],'avg':[],'max':[]}
def callback(data):

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import matplotlib.pyplot as plt
import datetime
import json

class FlirGraphNode(Node):
    def __init__(self):
        super().__init__('flir_graph')
        self.tempData = {'t':[],'min':[],'avg':[],'max':[]}
        plt.ion()
        self.subscription = self.create_subscription(
            String,
            'udp/flir_engine',
            self.callback,
            10)

    def callback(self, data):
        boxes = json.loads(data.data)
        b = boxes[0]
        print(b['minT'], b['avgT'], b['maxT'])
        self.tempData['t'].append(datetime.datetime.utcnow())
        self.tempData['min'].append(float(b['minT'].strip('"')[:-1]))
        self.tempData['avg'].append(float(b['avgT'].strip('"')[:-1]))
        self.tempData['max'].append(float(b['maxT'].strip('"')[:-1]))
        plt.plot(self.tempData['t'], self.tempData['min'])
        plt.plot(self.tempData['t'], self.tempData['avg'])
        plt.plot(self.tempData['t'], self.tempData['max'])
        plt.pause(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = FlirGraphNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

