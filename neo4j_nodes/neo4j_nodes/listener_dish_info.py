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


class Dish_Info_Listener(Node):


    def __init__(self):
        super().__init__('Dish_Info_Listener')

        self.listener = self.create_subscription(
            String,
            '/dish_info',
            self.insertion_listener_callback,
            10
        )
        print('Started Listening to /dish_info !!!')

        self.publisher_explainability_queries = self.create_publisher(String, '/EX_queries', 10)


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
                ["/home/belca/miniconda3/bin/python", os.path.join(source_dir, "llm_neo4j_query_dish_info.py"), shared_memory_name],
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
        self.send_msg_explainability(data)


    def send_msg_explainability(self, res):
        msg = String()
        msg.data = res
        self.publisher_explainability_queries.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)    



def main(args=None):
    rclpy.init(args=args)
    di_listener = Dish_Info_Listener()

    rclpy.spin(di_listener)

    # Shutdown
    di_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
