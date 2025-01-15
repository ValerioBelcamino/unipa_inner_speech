import rclpy
from rclpy.node import Node
import json
from std_msgs.msg import Bool, String
from multiprocessing.shared_memory import SharedMemory
import subprocess
import sys
import os


sys.path.append(os.path.dirname(os.path.abspath(__file__)))

ws_dir = os.getenv("ROS2_WORKSPACE", "/home/belca/Desktop/ros2_foxy_ws")  # Replace with your workspace path if needed
source_dir = os.path.join(ws_dir, 'src', 'neo4j_nodes', 'neo4j_nodes')


class User_Insertion_Listener(Node):


    def __init__(self):
        super().__init__('User_Insertion_Listener')

        self.listener = self.create_subscription(
            String,
            '/user_insertion',
            self.insertion_listener_callback,
            10
        )
        print('Started Listening to /user_insertion !!!')


    def insertion_listener_callback(self, msg):
        # Parse the JSON data from the message
        print("Message received.")

        # Create a shared memory block
        shm_size = 2048  # Size in bytes
        shm = SharedMemory(create=True, size=shm_size)
        shared_memory_name = shm.name

        try:
            shm.buf[:len(msg.data)] = msg.data.encode("utf-8")
            # Start the subprocess and pass the shared memory name
            res = subprocess.run(
                ["/home/belca/miniconda3/bin/python", os.path.join(source_dir, "llm_neo4j_query_insertion.py"), shared_memory_name],
                check=True
            )

            # # Read from shared memory
            # data = bytes(shm.buf[:]).decode("utf-8").strip("\x00")  # Remove null bytes
            # print("Data from subprocess:")
            # print(data)
        except Exception as e:
            print(f"Error: {e}")
        finally:
            # Cleanup shared memory
            shm.close()
            shm.unlink()        



def main(args=None):
    rclpy.init(args=args)
    ui_listener = User_Insertion_Listener()

    rclpy.spin(ui_listener)

    # Shutdown
    ui_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
