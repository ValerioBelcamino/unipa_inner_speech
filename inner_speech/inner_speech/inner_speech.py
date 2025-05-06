import os
import json
from langchain.chat_models import init_chat_model
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from common_msgs.msg import Intent, InnerSpeech
import ast
from dotenv import load_dotenv
from db_adapters import DBFactory
from shared_utils.customization_helpers import load_all_intent_models
import time 


# Load environment variables from .env file
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
BASE_DIR = os.path.abspath(os.path.join(CURRENT_DIR, "../../../../../../unipa_inner_speech"))
dotenv_path = os.path.join(BASE_DIR, ".env")
load_dotenv(dotenv_path)
dotconfig_path = os.path.join(BASE_DIR, ".config")
load_dotenv(dotconfig_path)

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

        self.db_type = os.getenv("DB_TYPE")
        self.db = DBFactory.create_adapter(self.db_type)
        self.schema = self.db.get_schema()

        self.llm_config = ast.literal_eval(os.getenv("LLM_CONFIG"))[self.node_name]

        self.ws_dir = os.getenv("ROS2_WORKSPACE")  # Replace with your workspace path if needed
        self.source_dir = os.path.join(self.ws_dir, 'inner_speech', 'inner_speech')

        self.llm = init_chat_model(
                                    model=self.llm_config['model_name'], 
                                    model_provider=self.llm_config['model_provider'], 
                                    temperature=self.llm_config['temperature'], 
                                    api_key=os.getenv("GROQ_API_KEY")
                                )
        
        self.action_name_to_required_parameters = {'AddToDatabase': ['nome_utente', 'calorie', 'proteine', 'carboidrati', 'grassi'],
                                            'DishInfo': ['nome_piatto'],
                                            'SubstituteDish': ['nome_utente', 'giorno', 'pasto', 'ha_piano_settimanale'],
                                            'OutOfScope': []}  # Out of scope doesn't require any parameters

        self.dynamic_intent_tools_dict = load_all_intent_models()
        self.action_name_to_description = {k:v.__doc__ for k,v in self.dynamic_intent_tools_dict.items()}
        self.action_name_to_description['OutOfScope'] = 'L\'azione non è rilevante per il sistema, quindi il sistema non è in grado di fornire una risposta all\'utente.'

    def listener_callback(self, intent_msg):
        self.get_logger().info('Received: "%s"\n' % intent_msg)

        user_input = intent_msg.user_input
        action_name = intent_msg.action_name
        parameters = ast.literal_eval(intent_msg.parameters)

        completed = True
        print(f"\033[34m" + "Parameters: " + str(parameters) + "\033[0m")

        required_parameters = self.action_name_to_required_parameters[action_name]
        missing_parameters = [param for param in required_parameters if param not in parameters]
        missing_parameters.extend([
                                param for param in list(set(required_parameters) - set(missing_parameters)) 
                                if parameters[param] in [0, None, '']
                                ])
        if missing_parameters or action_name == 'OutOfScope':
            completed = False 
            print(f"\033[34m" + "Missing parameters: " + str(missing_parameters) + "\033[0m")


            IS_msg = InnerSpeech(
                user_input=user_input, 
                action_name=action_name, 
                action_description=self.action_name_to_description[action_name], 
                parameters=json.dumps(parameters), 
                missing_parameters=missing_parameters)


        prompt = f"""
            Riassumi questo in un paragrafo in linguaggio naturale come se fosse il tuo discorso interiore.
            Non tralasciare alcun dettaglio.
            
            L'utente desidera eseguire l'azione {action_name}: {self.action_name_to_description[action_name]}.
            La loro domanda è: {user_input}.
            Il riconoscimento dell'intento ha estratto i seguenti parametri: {parameters}.
            L'azione può essere completata: {completed}.
            Parametri mancanti: {missing_parameters}
            
            Il tuo discorso interiore in italiano:"""

        # start_time = time.time()
        llm_response = self.llm.invoke(prompt)
        # print(f'\033[91m{time.time() - start_time}\033[0m')

        result = {}
        result['question'] = user_input
        result['inner_speech'] = llm_response.content
        result_string = json.dumps(result)

        print("\033[32m"+result_string+"\033[0m")

        if not completed:
            print(f"\033[34m" + "Incomplete answer, let's ask for more details" + "\033[0m")

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
