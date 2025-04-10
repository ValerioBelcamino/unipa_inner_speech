import os, re
import json
from langchain_neo4j import Neo4jGraph
from langchain.chat_models import init_chat_model
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from dotenv import load_dotenv
from datetime import datetime, timedelta
import ast
from pydantic import BaseModel, Field
from groq import BadRequestError
from unidecode import unidecode

# Define Pydantic classes with tools
class AddToDatabase(BaseModel):
    """A new user asks you to add them to the database. 
    Extract necessary information from the user message. 
    Do not generate any new information, use only what user provided for you.
    If you don't have some piece of information, leave the corresponding field blank."""

    nome_utente: str = Field(description="The name of the user")
    calorie: int = Field(description="How many calories user should eat per day")
    proteine: int = Field(description="How many grams of protein user should eat per day")
    carboidrati: int = Field(description="How many carbohydrates user should eat per day")
    grassi: int = Field(description="How many fats user should eat per day")
    intolleranze: str = Field(description="User's intollerances")

class DishInfo(BaseModel):
    """User asks you to give him information about a specific dish."""

    nome_utente: str = Field(description="The name of the user")
    nome_piatto: str = Field(description="The name of the dish in lowercase")

class SubstituteDish(BaseModel):
    """User asks you to propose an alternative dish based on their allergies and dietary plan.
    Extract necessary information from the user message. 
    Do not generate any new information, use only what user provided for you.
    If you don't have some piece of information, leave the corresponding field blank."""

    nome_utente: str = Field(description="The name of the user")
    ingredienti_rimossi: list[str] = Field(description="Ingredients that the user wants to exclude")
    ingredienti_preferiti: list[str] = Field(description="Ingredients that the user wants to include")
    solo_questi_ingredienti: list[str] = Field(description="User wants the dish to consist only of these ingredients. If you fill it, leave ingredienti_preferiti empty")
    giorno: str = Field(description="Day of the week in italian when the user wants the dish", 
                        examples=['lunedi', 'martedi', 'mercoledi', 'giovedi', 'venerdi', 'sabato', 'domenica'])
    pasto: str = Field(description="Type of meal for which the user wants the dish",
                       examples=['colazione', 'pranzo', 'cena'])



# Load environment variables from .env file
BASE_DIR = "/home/belca/Desktop/ros2_humble_ws/src"
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
        
        self.days_of_the_week = {
            0: "lunedi",
            1: "martedi",
            2: "mercoledi",
            3: "giovedi",
            4: "venerdi",
            5: "sabato",
            6: "domenica",
        }

        self.tool_name_2_id = {'AddToDatabase': '1', 'DishInfo': '2', 'SubstituteDish': '3'}

        self.publisher = self.create_publisher(String, self.out_topic, 10)

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

        self.llm = init_chat_model(
                                    model=self.llm_config['model_name'], 
                                    model_provider=self.llm_config['model_provider'], 
                                    temperature=self.llm_config['temperature'], 
                                    api_key=os.getenv("GROQ_API_KEY")
                                )
        
        self.llm_with_tools = self.llm.bind_tools([AddToDatabase, DishInfo, SubstituteDish])


    def get_day_of_the_week(self, llm_output: str) -> str:
        
        if llm_output['giorno'] in ['oggi', '']:
            llm_output['giorno'] = self.days_of_the_week[datetime.today().weekday()]

        elif llm_output['giorno'] == 'domani':
            domani = datetime.today() + timedelta(days=1)
            llm_output['giorno'] = self.days_of_the_week[domani.weekday()]

        elif llm_output['giorno'] == 'ieri':
            ieri = datetime.today() + timedelta(days=-1)
            llm_output['giorno'] = self.days_of_the_week[ieri.weekday()]

        return llm_output
    

    def get_next_meal(self, llm_output):
        print(f'{llm_output}')

        current_time = datetime.now().time() #11:34:30.263342

        colazione = datetime.strptime("11:00", "%H:%M").time()
        pranzo = datetime.strptime("14:00", "%H:%M").time()
        cena = datetime.strptime("22:00", "%H:%M").time()

        if current_time < colazione:
            llm_output['pasto'] = 'colazione'
        elif current_time < pranzo:
            llm_output['pasto'] = 'pranzo'
        elif current_time < cena:
            llm_output['pasto'] = 'cena'

        return llm_output
    

    def tool_to_lower(self, tool_output):
        '''Updates the parameters extracted by the tools to match our KG conventions'''
        for k,v in tool_output.items():
            # Remove accents from italian names of the week
            if k == 'giorno':
                v = unidecode(v)

            if k in ['ingredienti_rimossi', 'ingredienti_preferiti', 'solo_questi_ingredienti']:
                for i in range(len(v)):
                    v[i] = v[i].lower()
                    v[i] = v[i].replace(' ', '_')
            else:
                # Cast to lowercase
                v = v.lower()
                # Replace spaces with underscores
                v = v.replace(' ', '_')
            tool_output[k] = v
    
    def listener_callback(self, msg):
        self.get_logger().info('Received: "%s"\n' % msg.data)
        user_input = msg.data.strip()
        try:
            llm_response = self.llm_with_tools.invoke(user_input)
        except BadRequestError as e:
            print(f"\033[31mError: {e}\033[0m")
            llm_response = re.findall(r"<tool-use>(.*)</tool-use>", str(e))[0]
            llm_response = ast.literal_eval(llm_response)
            print(f"\033[31m{llm_response}\033[0m")
            
        if llm_response.tool_calls == []: # no tool called -> out of scope
            tool_result = {'action_id': '0'}
        else:
            tool_name = llm_response.tool_calls[0]['name']
            tool_id = self.tool_name_2_id[tool_name]
            tool_result = llm_response.tool_calls[0]['args']
            self.tool_to_lower(tool_result)
            tool_result['action_id'] = tool_id

            # change relative days to days of the week
            if tool_result['action_id'] == '3':
                tool_result = self.get_day_of_the_week(tool_result)
                
                # fill 'pasto' field for action 3 if not already filled
                if tool_result['pasto'] == '':
                    tool_result = self.get_next_meal(tool_result)

        result = {}
        result['question'] = user_input
        result['answer'] = str(tool_result)
        result_string = json.dumps(result)

        print("\033[32m"+result['answer']+"\033[0m")

        self.publisher.publish(String(data=result_string))
        self.get_logger().info('Published: "%s"' % result)

def main(args=None):
    rclpy.init(args=args)
    intent_recognition = Intent_Recognition()
    rclpy.spin(intent_recognition)
    intent_recognition.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
