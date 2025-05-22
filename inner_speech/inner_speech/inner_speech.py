from inner_speech.inner_speech_llm import InnerSpeech_LLM
from common_msgs.msg import Intent, InnerSpeech
from std_msgs.msg import String
from rclpy.node import Node
import rclpy
import json


class Inner_Speech(Node):
    def __init__(self):
        self.node_name = 'inner_speech'
        super().__init__(f'{self.node_name}_node')
        self.in_topic = '/user_intent'
        self.user_input_topic = '/user_input_activation'
        self.query_generation_topic = '/query_generation'
        self.inner_speech_explanation_topic = '/ex_inner_speech'

        self.subscription = self.create_subscription(
            Intent,
            self.in_topic,
            self.listener_callback,
            10)
        
        self.publisher_user_input = self.create_publisher(String, self.user_input_topic, 10)
        self.publisher_action_dispatch = self.create_publisher(Intent, self.query_generation_topic, 10)
        self.publisher_inner_speech = self.create_publisher(InnerSpeech, self.inner_speech_explanation_topic, 10)

        print(f"\033[34mInner Speech Node started!!!\033[0m")
        print(f"\033[34mInitialized publishers to {self.user_input_topic}!!!\033[0m")
        print(f"\033[34mInitialized publishers to {self.query_generation_topic}!!!\033[0m")
        print(f"\033[34mStarted Listening to {self.in_topic}!!!\033[0m")

        self.IS_LLM = InnerSpeech_LLM(node_name = self.node_name)
        

    def listener_callback(self, intent_msg):
        self.get_logger().info('Received: "%s"\n' % intent_msg)

        user_input = intent_msg.user_input
        action_name = intent_msg.action_name
        parameters = json.loads(intent_msg.parameters)

        print(f"\033[34m" + "Parameters: " + str(parameters) + "\033[0m")

        required_parameters = self.IS_LLM.action_name_to_required_parameters[action_name]
        missing_parameters = [param for param in required_parameters if param not in parameters]
        missing_parameters.extend([
                                param for param in list(set(required_parameters) - set(missing_parameters)) 
                                if parameters[param] in [0, None, '']
                                ])
        

        result = self.IS_LLM.get_LLM_response(user_input, action_name, parameters, missing_parameters)
        result_string = json.dumps(result)

        print("\033[32m"+result_string+"\033[0m")

        completed = True

        if missing_parameters or action_name == 'OutOfScope' or not result['can_proceed']:
            completed = False 
            print(f"\033[34m" + "Missing parameters: " + str(missing_parameters) + "\033[0m")


            IS_msg = InnerSpeech(
                user_input=user_input, 
                action_name=action_name, 
                action_description=self.IS_LLM.action_name_to_description[action_name], 
                parameters=json.dumps(parameters), 
                inner_speech=result['inner_speech'],
                missing_parameters=missing_parameters)

        if not completed:
            print(f"\033[34m" + "Incomplete or out of scope answer, let's ask for more details" + "\033[0m")

            self.publisher_inner_speech.publish(IS_msg)
            self.get_logger().info('Published {} on topic {}'.format(IS_msg, self.inner_speech_explanation_topic))

        else:
            print(f"\033[34m" + "Complete answer, we can procede!" + "\033[0m")
            
            self.publisher_action_dispatch.publish(intent_msg)
            self.get_logger().info(f'Publishing {intent_msg} on topic {self.query_generation_topic}')


def main(args=None):
    rclpy.init(args=args)
    inner_speech = Inner_Speech()
    rclpy.spin(inner_speech)
    inner_speech.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
