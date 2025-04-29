import os, re
import json
from langchain_neo4j import Neo4jGraph
from neo4j import GraphDatabase 
from langchain.chat_models import init_chat_model
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from dotenv import load_dotenv
import ast
from pydantic import BaseModel, Field
from groq import BadRequestError
import time 
from typing import List
from intent_post_processing.loader import load_plugins
from common_msgs.msg import Intent

# Define Pydantic classes with tools
class AddToDatabase(BaseModel):
    """
    A new user asks you to add them to the database.

    Extract necessary information from the user message.
    Do not generate any new information, use only what user provided for you.
    If you don't have some piece of information, leave the corresponding field blank.
    """

    nome_utente: str = Field(description="The name of the user in lowercase")
    calorie: int = Field(description="How many calories user should eat per day", default=0)
    proteine: int = Field(description="How many grams of protein user should eat per day", default=0)
    carboidrati: int = Field(description="How many carbohydrates user should eat per day", default=0)
    grassi: int = Field(description="How many fats user should eat per day", default=0)
    intolleranze: List[str] = Field(description="User's intollerances", default='')

class DishInfo(BaseModel):
    """
    User asks you to give him information about a specific dish.

    For example, about its nurtients, allergens or if this dish is suitable for the user.
    """

    nome_utente: str = Field(description="The name of the user in lowercase", default='')
    nome_piatto: str = Field(description="The name of the dish in lowercase")
    controllo_ingredienti: List[str] = Field(description="Ingredients to check for in the dish", default_factory=list)

class SubstituteDish(BaseModel):
    """
    User asks you to propose an alternative dish based on their allergies and dietary plan.

    Extract necessary information from the user message. 
    Do not generate any new information, use only what user provided for you.
    IMPORTANT: Always return ALL fields in the response, even with empty values.
    If you don't have some piece of information, leave the corresponding field blank.
    """

    nome_utente: str = Field(description="The name of the user in lowercase")
    ingredienti_rimossi: List[str] = Field(description="Ingredients that the user wants to exclude", default_factory=list)
    ingredienti_preferiti: List[str] = Field(description="Ingredients that the user wants to include", default_factory=list)
    solo_questi_ingredienti: List[str] = Field(description="User wants the dish to consist only of these ingredients. If you fill it, leave ingredienti_preferiti empty", default_factory=list)
    giorno: str = Field(description="Day of the week in italian when the user wants the dish", 
                        examples=['lunedi', 'martedi', 'mercoledi', 'giovedi', 'venerdi', 'sabato', 'domenica'],
                        default='')
    pasto: str = Field(description="Type of meal for which the user wants the dish. Map 'stasera'/'sera' to 'cena', 'mattina' to 'colazione', etc.",
                       examples=['colazione', 'pranzo', 'cena'],
                       default='')



# Load environment variables from .env file
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
BASE_DIR = os.path.abspath(os.path.join(CURRENT_DIR, "../../../../../../src"))
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

        self.tool_name_2_id = {'AddToDatabase': '1', 'DishInfo': '2', 'SubstituteDish': '3'}

        self.publisher = self.create_publisher(Intent, self.out_topic, 10)

        print(f"\033[34mIntent Recognition Node started!!!\033[0m")
        print(f"\033[34mInitialized publishers to {self.out_topic}!!!\033[0m")
        print(f"\033[34mStarted Listening to {self.in_topic}!!!\033[0m")

        self.uri = os.getenv("NEO4J_URI")
        self.username = os.getenv("NEO4J_USERNAME")
        self.password = os.getenv("NEO4J_PASSWORD")

        self.llm_config = ast.literal_eval(os.getenv("LLM_CONFIG"))[self.node_name]

        self.ws_dir = os.getenv("ROS2_WORKSPACE")

        self.source_dir = os.path.join(self.ws_dir, 'intent_recognition', 'intent_recognition')

        self.graph = Neo4jGraph(self.uri, self.username, self.password)
        self.schema = self.graph.schema
        self.driver = GraphDatabase.driver(self.uri, auth=(self.username, self.password))

        self.llm = init_chat_model(
                                    model=self.llm_config['model_name'], 
                                    model_provider=self.llm_config['model_provider'], 
                                    temperature=self.llm_config['temperature'], 
                                    api_key=os.getenv("GROQ_API_KEY")
                                )
        
        self.llm_with_tools = self.llm.bind_tools([AddToDatabase, DishInfo, SubstituteDish])

        # Load plugins dynamically from the config file
        self.plugins = load_plugins()
        print(f"\033[38;5;208mLoaded {len(self.plugins)} processing plugin(s).\033[0m")


    def execute_plugin_pipeline(self, db_driver, action_name, intent_parameters):
        """
        Function to execute the loaded plugins with the appropriate parameters.
        """
        # print(f"Executing plugin pipeline with action ID {action_id} and parameters {intent_parameters}")

        # Prepare the context for each plugin: db_driver, action_id, and specific parameters
        context = {
            "db_driver": db_driver,
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


    def check_undeclared_parameters(self, tool_name, tool_result):
        parameter_list = [field for field in eval(tool_name).__fields__.keys()]
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

        tool_calls = [tool_call for tool_call in tool_calls if tool_call['name'] in self.tool_name_2_id.keys()]
        if tool_calls == []: # no tool called -> out of scope
            tool_result = {}
            tool_name = 'OutOfScope'
        else:
            tool_name = tool_calls[0]['name']
            tool_result = tool_calls[0]['args']
            # tool_id = self.tool_name_2_id[tool_name]

            # defaults missing parameters to '' or None
            tool_result = self.check_undeclared_parameters(tool_name, tool_result)

            # execute post processing plugin pipeline 
            self.execute_plugin_pipeline(self.driver, tool_name, tool_result)

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
