import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from common_msgs.msg import Intent
from llm import get_LLM_response


class Intent_Recognition(Node):
    def __init__(self):
        self.node_name = 'intent_recognition'
        super().__init__(f'{self.node_name}_node')
        self.in_topic = '/user_input'
        self.out_topic = '/user_intent'

        self.subscription = self.create_subscription(
            String,
            self.in_topic,
            self.listener_callback,
            10)


        self.publisher = self.create_publisher(Intent, self.out_topic, 10)

        print(f"\033[34mIntent Recognition Node started!!!\033[0m")
        print(f"\033[34mInitialized publishers to {self.out_topic}!!!\033[0m")
        print(f"\033[34mStarted Listening to {self.in_topic}!!!\033[0m")

    def listener_callback(self, msg):
        self.get_logger().info('Received: "%s"\n' % msg.data)
        user_input = msg.data.strip()
        tool_name, tool_result = get_LLM_response(user_input)

        intent_msg = Intent()
        intent_msg.user_input = user_input
        intent_msg.action_name = tool_name
        intent_msg.parameters = json.dumps(tool_result)

        self.publisher.publish(intent_msg)
        self.get_logger().info('\033[32mPublished: "%s"\033[0m' % intent_msg)


def main(args=None):
    rclpy.init(args=args)
    intent_recognition = Intent_Recognition()
    rclpy.spin(intent_recognition)
    intent_recognition.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
