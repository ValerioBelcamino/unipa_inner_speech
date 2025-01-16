import rclpy
from rclpy.node import Node
import subprocess
import sys

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('ROS 2 Node is running!')


def call_python312_script():
    subprocess.run(["/home/belca/miniconda3/bin/python", "/home/belca/Desktop/ros2_foxy_ws/src/simple_ros2_tests/simple_ros2_tests/py312_routine.py"])


def main(args=None):
    print(f"Python version: {sys.version}")
    call_python312_script()
    rclpy.init(args=args)
    node = MinimalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

