import rclpy
import sys
import json
import time
from multiprocessing.shared_memory import SharedMemory
from rclpy.node import Node
import subprocess
from std_msgs.msg import Bool, String
import os 


# Assuming your workspace structure is standard
ws_dir = os.getenv("ROS2_WORKSPACE", "/home/belca/Desktop/ros2_foxy_ws")  # Replace with your workspace path if needed
source_dir = os.path.join(ws_dir, 'src', 'neo4j_nodes', 'neo4j_nodes')

class Clingo_Publisher(Node):

    def __init__(self):
        super().__init__('Clingo_publisher')
        self.publisher_clingo_start = self.create_publisher(Bool, '/Clingo_start', 10)
        self.publisher_inner_speech_queries = self.create_publisher(String, '/IS_queries', 10)

        # Create a shared memory block
        shm_size = 2048  # Size in bytes
        shm = SharedMemory(create=True, size=shm_size)
        shared_memory_name = shm.name

        try:
            # Start the subprocess and pass the shared memory name
            res = subprocess.run(
                ["/home/belca/miniconda3/bin/python", os.path.join(source_dir, "llm_neo4j_query.py"), shared_memory_name],
                check=True
            )

            # Read from shared memory
            data = bytes(shm.buf[:]).decode("utf-8").strip("\x00")  # Remove null bytes
            print("Data from subprocess:")
            print(data)
        except Exception as e:
            print(f"Error: {e}")
        finally:
            # Cleanup shared memory
            shm.close()
            shm.unlink()        
     
        print("Subprocess finito")
        self.send_msg_clingo()
        self.send_msg_inner_speech(data)

    def send_msg_clingo(self):
        msg = Bool()
        msg.data = True
        self.publisher_clingo_start.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    def send_msg_inner_speech(self, res):
        msg = String()
        msg.data = res
        self.publisher_inner_speech_queries.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)




    minimal_publisher = Clingo_Publisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()