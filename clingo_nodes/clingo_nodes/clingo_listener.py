import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from clingo_call import clingo_call

class Clingo_Listener(Node):

    def __init__(self):
        super().__init__('clingo_listener')
        self.clingo_start_topic = '/clingo_start'
        self.clingo_explanation_topic = '/ex_clingo'

        self.subscription = self.create_subscription(
            Bool,
            self.clingo_start_topic,
            self.listener_callback,
            10
        )
        self.subscription  # Prevent unused variable warning
        self.publisher_ = self.create_publisher(String, self.clingo_explanation_topic, 10)

        print("\033[34mClingo Listener Node started!!!\033[0m")
        print("\033[34mInitialized publishers to {self.clingo_explanation_topic}!!!\033[0m")
        print("\033[34mStarted Listening to {self.clingo_start_topic}!!!\033[0m")


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
