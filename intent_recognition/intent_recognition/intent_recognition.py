from intent_recognition.intent_recognition_llm import IntentRecognition_LLM
from memory_service.memory_client import MemoryClient
from common_msgs.msg import Intent
from std_msgs.msg import String
from rclpy.node import Node
import rclpy
import json



class Intent_Recognition(Node):
    def __init__(self):
        self.node_name = 'intent_recognition'
        super().__init__(f'{self.node_name}_node')
        self.in_topic = '/user_input'
        self.out_topic = '/user_intent'
        self.change_scenario_topic = '/change_scenario'

        self.subscription = self.create_subscription(
            String,
            self.in_topic,
            self.listener_callback,
            10)
        
        self.listener_change_scenario = self.create_subscription(
            String,
            self.change_scenario_topic,
            self.change_scenario_callback,
            10)
        

        self.publisher = self.create_publisher(Intent, self.out_topic, 10)

        print(f"\033[34mIntent Recognition Node started!!!\033[0m")
        print(f"\033[34mInitialized publishers to {self.out_topic}!!!\033[0m")
        print(f"\033[34mStarted Listening to {self.in_topic}!!!\033[0m")

        self.memory_client = MemoryClient()
        self.get_response = self.memory_client.send_get_request()
        print('Current Memory:', self.get_response.memory_list)

        self.IR_LLM = IntentRecognition_LLM(node_name = self.node_name)


    def change_scenario_callback(self, msg):
        self.get_logger().info('Activating: "%s" scenario\n' % msg.data)
        self.IR_LLM.change_scenario(msg.data)


    def listener_callback(self, msg):
        self.get_logger().info('Received: "%s"\n' % msg.data)

        # Update Memory
        self.get_response = self.memory_client.send_get_request()
        memories = self.get_response.memory_list + self.get_response.last_messages
        print('Current Memory:', memories)
        
        user_input = msg.data.strip()
        tool_name, tool_result = self.IR_LLM.get_LLM_response(user_input, memories)
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
