import os, re
import json
from db_adapters import DBFactory  # Import the DBFactory
from langchain.chat_models import init_chat_model
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from dotenv import load_dotenv
import ast
from groq import BadRequestError
import time 
from intent_post_processing.loader import load_plugins
from common_msgs.msg import Intent
from shared_utils.customization_helpers import load_all_intent_models



# Load environment variables from .env file
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
BASE_DIR = os.path.abspath(os.path.join(CURRENT_DIR, "../../../../../../unipa_inner_speech"))
dotenv_path = os.path.join(BASE_DIR, ".env")
load_dotenv(dotenv_path)
dotconfig_path = os.path.join(BASE_DIR, ".config")
load_dotenv(dotconfig_path)



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

        self.ws_dir = os.getenv("ROS2_WORKSPACE")
        self.source_dir = os.path.join(self.ws_dir, 'intent_recognition', 'intent_recognition')

        # Initialize the database adapter
        self.db_type = os.getenv("DB_TYPE")  # Default to neo4j if not specified
        self.db = DBFactory.create_adapter(self.db_type)
        self.schema = self.db.get_schema()

        # Get LLM configuration
        self.llm_config = ast.literal_eval(os.getenv("LLM_CONFIG"))[self.node_name]

        self.llm = init_chat_model(
                                    model=self.llm_config['model_name'], 
                                    model_provider=self.llm_config['model_provider'], 
                                    temperature=self.llm_config['temperature'], 
                                    api_key=os.getenv("GROQ_API_KEY")
                                )
        
        print()
        self.dynamic_intent_tools_dict = load_all_intent_models()
        self.dynamic_intent_toolnames = [dit.__name__ for dit in self.dynamic_intent_tools_dict.values()]

        self.llm_with_tools = self.llm.bind_tools(self.dynamic_intent_tools_dict.values())
        print(f"\033[1;38;5;207mLoaded {len(self.dynamic_intent_toolnames)} intent_tool(s).\033[0m")
        print()

        # Load plugins dynamically from the config file
        self.plugins = load_plugins()
        print(f"\033[1;38;5;208mLoaded {len(self.plugins)} processing plugin(s).\033[0m")


    def execute_plugin_pipeline(self, db_adapter, action_name, intent_parameters):
        """
        Function to execute the loaded plugins with the appropriate parameters.
        """
        # print(f"Executing plugin pipeline with action ID {action_id} and parameters {intent_parameters}")

        # Prepare the context for each plugin: db_adapter, action_id, and specific parameters
        context = {
            "db_adapter": db_adapter,  # Pass the adapter instead of the driver
            "action_name": action_name,
            "intent_parameters": intent_parameters  # Adding the dynamic parameters extracted after LLM computation
        }

        # Iterate over each loaded plugin (each wrapped function)
        for plugin_function in self.plugins:
            try:
                # Call the plugin with the context
                context['intent_parameters'] = plugin_function(context)

            except Exception as e:
                print(f"Error executing plugin: {e}")


    def check_undeclared_parameters(self, tool_class, tool_result):
        print(tool_class.model_fields.keys())
        parameter_list = [field for field in tool_class.model_fields.keys()]
        for parameter in parameter_list:
            if parameter not in tool_result:
                tool_result[parameter] = ''
        return tool_result

    
    def listener_callback(self, msg):
        self.get_logger().info('Received: "%s"\n' % msg.data)
        user_input = msg.data.strip()
        try:
            # start_time = time.time()
            llm_response = self.llm_with_tools.invoke(user_input)
            # print(f'\033[91m{time.time() - start_time}\033[0m')
            tool_calls = llm_response.tool_calls
        except BadRequestError as e:
            print(f"\033[31mError: {e}\033[0m")
            llm_response = re.findall(r"<tool-use>(.*)</tool-use>", str(e))[0]
            llm_response = ast.literal_eval(llm_response)
            tool_calls = llm_response['tool_calls']
        print(f"\033[32m{llm_response}\033[0m")

        tool_calls = [tool_call for tool_call in tool_calls if tool_call['name'] in self.dynamic_intent_toolnames]

        if tool_calls == []: # no tool called -> out of scope
            tool_result = {'action_name': ''}
        else:
            tool_name = tool_calls[0]['name']
            tool_result = tool_calls[0]['args']

            # defaults missing parameters to '' or None
            tool_result = self.check_undeclared_parameters(self.dynamic_intent_tools_dict[tool_name], tool_result)

            # execute post processing plugin pipeline 
            self.execute_plugin_pipeline(self.db, tool_name, tool_result)

        intent_msg = Intent()
        intent_msg.user_input = user_input
        intent_msg.action_name = tool_name
        intent_msg.parameters = json.dumps(tool_result)

        self.publisher.publish(intent_msg)
        self.get_logger().info('\033[32mPublished: "%s"\033[0m' % intent_msg)

    def destroy_node(self):
        # Clean up database connection when the node is destroyed
        if hasattr(self, 'db'):
            self.db.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    intent_recognition = Intent_Recognition()
    rclpy.spin(intent_recognition)
    intent_recognition.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
