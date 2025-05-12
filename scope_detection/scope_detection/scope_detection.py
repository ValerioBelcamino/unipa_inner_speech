import os, re
import json
from langchain.chat_models import init_chat_model
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from dotenv import load_dotenv
import ast
from groq import BadRequestError
from common_msgs.msg import Intent
from shared_utils.customization_helpers import create_scenario_tools
from typing import Any, get_origin, get_args, Union


# Load environment variables from .env file
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
BASE_DIR = os.path.abspath(os.path.join(CURRENT_DIR, "../../../../../../unipa_inner_speech"))
dotenv_path = os.path.join(BASE_DIR, ".env")
load_dotenv(dotenv_path)
dotconfig_path = os.path.join(BASE_DIR, ".config")
load_dotenv(dotconfig_path)



class Scope_Detection(Node):
    def __init__(self):
        self.node_name = 'scope_detection'
        super().__init__(f'{self.node_name}_node')
        self.in_topic = '/user_input'
        # self.out_topic = '/user_intent'

        self.subscription = self.create_subscription(
            String,
            self.in_topic,
            self.listener_callback,
            10)


        # self.publisher = self.create_publisher(Intent, self.out_topic, 10)

        print(f"\033[34mIntent Recognition Node started!!!\033[0m")
        # print(f"\033[34mInitialized publishers to {self.out_topic}!!!\033[0m")
        print(f"\033[34mStarted Listening to {self.in_topic}!!!\033[0m")

        self.scenario_tools_dict = create_scenario_tools()
        print(f"\033[1;38;5;207mWe have the following scenarios:\033[0m")
        print(f"\033[1;38;5;207m", [f'{name}: {pcls.__doc__}' for name, pcls in self.scenario_tools_dict.items()], "\033[0m")

        # Get LLM configuration
        self.llm_config = ast.literal_eval(os.getenv("LLM_CONFIG"))[self.node_name]

        self.llm = init_chat_model(
                                    model=self.llm_config['model_name'], 
                                    model_provider=self.llm_config['model_provider'], 
                                    temperature=self.llm_config['temperature'], 
                                    api_key=os.getenv("GROQ_API_KEY")
                                )
        
        self.llm_with_tools = self.llm.bind_tools(self.scenario_tools_dict.values())
        print(f"\033[1;38;5;207mLoaded {len(self.scenario_tools_dict)} scenario tool(s).\033[0m")
        print()
        
    
    def listener_callback(self, msg):
        self.get_logger().info('Received: "%s"\n' % msg.data)
        user_input = msg.data.strip()
        try:
            # start_time = time.time()
            llm_response = self.llm_with_tools.invoke(
                [
                    ("system", 'you have to understand the scope of the question provided by the user. You have a list of supported scenarios with their description and you have to select the most appropriate. if the user prompt is not related to the tools you should not answer'),
                    ("human", user_input),
                ]
            )
            print(llm_response)
            # print(f'\033[91m{time.time() - start_time}\033[0m')
            tool_calls = llm_response.tool_calls
        except BadRequestError as e:
            print(f"\033[31mError: {e}\033[0m")
            llm_response = re.findall(r"<tool-use>(.*)</tool-use>", str(e))[0]
            llm_response = ast.literal_eval(llm_response)
            tool_calls = llm_response['tool_calls']
        print(f"\033[32m{llm_response}\033[0m")

        tool_calls = [tool_call for tool_call in tool_calls if tool_call['name'] in self.scenario_tools_dict.keys()]

        if tool_calls == []: # no tool called -> out of scope
            tool_result = {}
            tool_name = 'OutOfScope'
        else:
            tool_name = tool_calls[0]['name']

        intent_msg = Intent()
        intent_msg.action_name = tool_name

        # self.publisher.publish(intent_msg)
        self.get_logger().info('\033[32mPublished: "%s"\033[0m' % intent_msg)

    def destroy_node(self):
        # Clean up database connection when the node is destroyed
        if hasattr(self, 'db'):
            self.db.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    scope_detection = Scope_Detection()
    rclpy.spin(scope_detection)
    scope_detection.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
