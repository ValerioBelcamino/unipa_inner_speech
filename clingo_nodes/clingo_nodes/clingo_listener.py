import rclpy
from rclpy.node import Node
import json
from std_msgs.msg import Bool, String
import subprocess
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from clingo_call import clingo_call

class Clingo_Listener(Node):

    def __init__(self):
        super().__init__('Clingo_listener')
        self.subscription = self.create_subscription(
            Bool,
            '/Clingo_start',
            self.listener_callback,
            10
        )
        self.subscription  # Prevent unused variable warning
        self.publisher_ = self.create_publisher(String, '/EX_clingo', 10)

        print("Started Listening!!!")

    def listener_callback(self, msg):
        # Parse the JSON data from the message
        print("Message received. Starting solver!")
        answers = clingo_call()
        msg = String()
        msg.data = answers
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    clingo_listener = Clingo_Listener()

    rclpy.spin(clingo_listener)

    # Shutdown
    clingo_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
