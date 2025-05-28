from scope_detection.scope_detection_llm import ScopeDetection_LLM
from common_msgs.msg import InnerSpeech
from std_msgs.msg import String
from rclpy.node import Node
import rclpy



class Scope_Detection(Node):
    def __init__(self):
        self.node_name = 'scope_detection'
        super().__init__(f'{self.node_name}_node')
        self.in_topic = '/out_of_scope'
        self.out_topic = '/change_scenario'

        self.subscription = self.create_subscription(
            InnerSpeech,
            self.in_topic,
            self.listener_callback,
            10)


        self.change_scenario_publisher = self.create_publisher(String, self.out_topic, 10)

        print(f"\033[34mIntent Recognition Node started!!!\033[0m")
        # print(f"\033[34mInitialized publishers to {self.out_topic}!!!\033[0m")
        print(f"\033[34mStarted Listening to {self.in_topic}!!!\033[0m")

        self.SD_LLM = ScopeDetection_LLM(node_name=self.node_name)

    def listener_callback(self, msg):
        self.get_logger().info('Received: "%s"\n' % msg)
        user_input = msg.user_input
        inner_speech = msg.inner_speech

        print(f"\033[34m{user_input}, {inner_speech}\033[0m")

        tool_name = self.SD_LLM.get_LLM_response(user_input, inner_speech)

        change_scenario_msg = String(data=tool_name['scenario'])

        self.change_scenario_publisher.publish(change_scenario_msg)
        self.get_logger().info('\033[32mPublished: "%s"\033[0m' % change_scenario_msg)



def main(args=None):
    rclpy.init(args=args)
    scope_detection = Scope_Detection()
    rclpy.spin(scope_detection)
    scope_detection.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
