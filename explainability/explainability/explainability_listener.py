import rclpy
from rclpy.node import Node
import json
from std_msgs.msg import String
import subprocess
import os


# Assuming your workspace structure is standard
ws_dir = os.getenv("ROS2_WORKSPACE", "/home/belca/Desktop/ros2_foxy_ws")  # Replace with your workspace path if needed
source_dir = os.path.join(ws_dir, 'src', 'explainability', 'explainability')


class Explainability_Listener(Node):

    def __init__(self):
        super().__init__('Explainability_Listener')
        self.subscription_queries = self.create_subscription(
            String,
            '/EX_queries',
            self.queries_callback,
            10
        )
        self.subscription_queries  # Prevent unused variable warning

        self.subscription_clingo = self.create_subscription(
            String,
            '/EX_clingo',
            self.clingo_callback,
            10
        )
        self.subscription_clingo  # Prevent unused variable warning
        

    def queries_callback(self, msg):
        # Parse the JSON data from the message
        try:
            # Parse the JSON data
            data = json.loads(msg.data)
            # self.get_logger().info('Received data: "%s"\n' % data)
            self.get_logger().info('Received data!\n')

            # Call the external Python file and pass the JSON string
            subprocess.run(['/home/belca/miniconda3/bin/python', os.path.join(source_dir, 'llm_query_explaination.py'), json.dumps(data)], check=True)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to decode JSON: {e}")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Subprocess failed: {e}")

    def clingo_callback(self, msg):
        self.get_logger().info('Received data!\n')

        # Call the external Python file and pass the JSON string
        subprocess.run(['/home/belca/miniconda3/bin/python', os.path.join(source_dir, 'llm_clingo_explaination.py'), msg.data], check=True)



def main(args=None):
    rclpy.init(args=args)
    explainability_listener = Explainability_Listener()

    rclpy.spin(explainability_listener)

    # Shutdown
    explainability_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
