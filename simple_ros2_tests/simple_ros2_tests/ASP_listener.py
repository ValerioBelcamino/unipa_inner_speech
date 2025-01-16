import rclpy
from rclpy.node import Node
import json
from std_msgs.msg import String
import subprocess


class ASP_Listener(Node):

    def __init__(self):
        super().__init__('ASP_listener')
        self.subscription = self.create_subscription(
            String,
            '/ASP_output',
            self.listener_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

    def listener_callback(self, msg):
        # Parse the JSON data from the message
        try:
            # Parse the JSON data
            data = json.loads(msg.data)
            # self.get_logger().info('Received data: "%s"\n' % data)
            self.get_logger().info('Received data!\n')

            # Call the external Python file and pass the JSON string
            subprocess.run(['/home/belca/miniconda3/bin/python', '/home/belca/Desktop/Palermo/llm_ASP_fewshot.py', json.dumps(data)], check=True)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to decode JSON: {e}")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Subprocess failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    asp_listener = ASP_Listener()

    rclpy.spin(asp_listener)

    # Shutdown
    asp_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
