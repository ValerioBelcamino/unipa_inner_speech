from explainability.explainability_llm import QueryExplanation_LLM, InnerSpeechExplanation_LLM
from memory_service.memory_client import MemoryClient
from common_msgs.msg import QueryOutput, InnerSpeech
from std_msgs.msg import String
from rclpy.node import Node
import rclpy
import json



class Explainability(Node):
    def __init__(self):
        self.node_name = 'explainability'
        super().__init__(f'{self.node_name}_node')
        self.inner_speech_explanation_topic = '/ex_inner_speech'
        self.user_input_topic = '/user_input_activation'
        self.change_scenario_topic = '/change_scenario'

        self.query_explanation_topic = '/ex_queries'
        self.clingo_explanation_topic = '/ex_clingo'

        self.robot_dialogue_topic = '/speak'

        self.listener_query_explanation = self.create_subscription(
            QueryOutput,
            self.query_explanation_topic,
            self.query_explanation_callback,
            10)
        
        # self.listener_clingo_explanation = self.create_subscription(
        #     String,
        #     self.clingo_explanation_topic,
        #     self.clingo_explanation_callback,
        #     10)
        
        self.listener_inner_speech = self.create_subscription(
            InnerSpeech,
            self.inner_speech_explanation_topic,
            self.inner_speech_explanation_callback,
            10)
        
        self.listener_change_scenario = self.create_subscription(
            String,
            self.change_scenario_topic,
            self.change_scenario_callback,
            10)
        

        
        self.robot_dialogue_publisher = self.create_publisher(String, self.robot_dialogue_topic, 10)
        self.publisher_user_input = self.create_publisher(String, self.user_input_topic, 10)

        print(f"\033[34mExplainability Node started!!!\033[0m")
        print(f"\033[34mInitialized publishers to {self.robot_dialogue_topic}!!!\033[0m")
        print(f"\033[34mStarted Listening to {self.query_explanation_topic}!!!\033[0m")
        print(f"\033[34mStarted Listening to {self.clingo_explanation_topic}!!!\033[0m")
        print(f"\033[34mStarted Listening to {self.inner_speech_explanation_topic}!!!\033[0m")

        self.memory_client = MemoryClient()
        self.get_response = self.memory_client.send_get_request()
        print('Current Memory:', self.get_response.memory_list)

        self.QueryEXP_LLM = QueryExplanation_LLM(node_name=self.node_name)
        self.ISEXP_LLM = InnerSpeechExplanation_LLM(node_name=self.node_name)

    def change_scenario_callback(self, msg):
        self.get_logger().info('Activating: "%s" scenario\n' % msg.data)
        self.QueryEXP_LLM.change_scenario(msg.data)
        

    def query_explanation_callback(self, msg):
        action_name = msg.action_name
        self.get_logger().info('Received: "%s" query_explanation_callback\n' % action_name)

        print(f"\033[34m{msg.user_input=}, {msg.queries=}, {msg.results=}\033[0m")
        
        explanation = self.QueryEXP_LLM.get_LLM_response(msg.user_input, msg.action_name, msg.queries, msg.results)

        response_dict = {'question':msg.user_input, 'response':explanation}
        response_string = json.dumps(response_dict)
        self.publisher_user_input.publish(String(data=response_string))
        self.get_logger().info('Published: "%s"' % response_string)

        # Output the explanation
        print(f"\033[1;34mExplanation:\033[0m")
        print(f"\033[1;32m{explanation}\033[0m")
        
        # Update memory
        update_response = self.memory_client.send_update_request(
            msg.user_input,
            explanation
        )
        print('Updated list:', update_response.memory_list)


    # def clingo_explanation_callback(self, msg):
    #     self.get_logger().info('Received: "%s" __ clingo_explanation_callback\n' % msg.data)
    #     msg_dict = json.loads(msg.data)

    #     few_shot_prompt = prepare_few_shot_prompt(
    #                                                 instructions=self.clingo_instructions,
    #                                                 suffix=self.clingo_suffix, 
    #                                                 examples=self.examples['clingo'],
    #                                                 example_variables=["results", "explanation"],
    #                                                 example_template=self.clingo_example_template,
    #                                                 input_variables=["results"],
    #                                                 )

    #     formatted_prompt = few_shot_prompt.format(results = msg_dict['results'])
        
    #     explanation = self.llm_response.invoke(formatted_prompt)
    #     response_dict = {'question':msg.user_input, 'response':explanation}
    #     response_string = json.dumps(response_dict)
    #     self.publisher_user_input.publish(String(data=response_string))
    #     self.get_logger().info('Published: "%s"' % response_string)

    #     # Output the explanation
    #     print(f"\033[1;34mExplanation:\033[0m")
    #     print(f"\033[1;32m{explanation}\033[0m")

    def inner_speech_explanation_callback(self, msg):
        self.get_logger().info('Received: "%s" __ inner_speech_explanation_callback\n' % msg)
        user_input = msg.user_input
        parameters = msg.parameters
        action_name = msg.action_name
        action_description = msg.action_description
        missing_parameters = msg.missing_parameters
        inner_speech = msg.inner_speech

        response = self.ISEXP_LLM.get_LLM_response(
                                                    user_input, 
                                                    action_name, 
                                                    action_description, 
                                                    parameters, 
                                                    missing_parameters,
                                                    inner_speech
                                                    )

        response_dict = {'question':user_input, 'response':response}
        response_string = json.dumps(response_dict)
        self.publisher_user_input.publish(String(data=response_string))
        self.get_logger().info('Published: "%s"' % response_string)

        # Output the explanation
        print(f"\033[1;34mExplanation:\033[0m")
        print(f"\033[1;32m{response}\033[0m")

def main(args=None):
    rclpy.init(args=args)
    explainability = Explainability()
    rclpy.spin(explainability)
    explainability.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
