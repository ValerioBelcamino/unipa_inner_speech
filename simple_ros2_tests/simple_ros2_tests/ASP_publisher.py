import rclpy
import sys
import json
import time
from rclpy.node import Node

from std_msgs.msg import String


class ASP_Publisher(Node):

    def __init__(self, string_list):
        super().__init__('ASP_publisher')
        self.publisher_ = self.create_publisher(String, '/ASP_output', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.string_list = string_list
        

    def timer_callback(self):
        msg = String()
        msg.data = json.dumps(self.string_list[self.i%5])
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
        time.sleep(5)

def main(args=None):
    rclpy.init(args=args)
    print(f"Python version: {sys.version}")
    path = '/home/belca/Desktop/Palermo/sample_ASP_output.json'

    ASP_examples = []

    with open(path, 'r') as f:
        ASP_examples = json.load(f)

    # print(len(ASP_examples))
    # print(len(ASP_examples["examples"]))

    minimal_publisher = ASP_Publisher(ASP_examples["examples"])

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()