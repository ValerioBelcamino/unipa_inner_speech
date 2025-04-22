import os
import json
from langchain.chat_models import init_chat_model
from langchain_neo4j import Neo4jGraph
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import ast
from dotenv import load_dotenv
import time 


# Load environment variables from .env file
BASE_DIR = "/home/belca/Desktop/ros2_humble_ws/src"
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

        self.subscription = self.create_subscription(
            String,
            self.in_topic,
            self.listener_callback,
            10)
        
        self.publisher_user_input = self.create_publisher(String, self.user_input_topic, 10)
        self.publisher_action_dispatch = self.create_publisher(String, self.query_generation_topic, 10)

        print(f"\033[34mInner Speech Node started!!!\033[0m")
        print(f"\033[34mInitialized publishers to {self.user_input_topic}!!!\033[0m")
        print(f"\033[34mInitialized publishers to {self.query_generation_topic}!!!\033[0m")
        print(f"\033[34mStarted Listening to {self.in_topic}!!!\033[0m")

        self.uri = os.getenv("NEO4J_URI")
        self.username = os.getenv("NEO4J_USERNAME")
        self.password = os.getenv("NEO4J_PASSWORD")

        self.llm_config = ast.literal_eval(os.getenv("LLM_CONFIG"))[self.node_name]

        self.ws_dir = os.getenv("ROS2_WORKSPACE")  # Replace with your workspace path if needed
        self.source_dir = os.path.join(self.ws_dir, 'inner_speech', 'inner_speech')

        self.graph = Neo4jGraph(self.uri, self.username, self.password)
        self.schema = self.graph.schema

        self.llm = init_chat_model(
                                    model=self.llm_config['model_name'], 
                                    model_provider=self.llm_config['model_provider'], 
                                    temperature=self.llm_config['temperature'], 
                                    api_key=os.getenv("GROQ_API_KEY")
                                )
        
        self.action_id_to_required_parameters = {'1': ['nome_utente', 'calorie', 'proteine', 'carboidrati', 'grassi'],
                                            '2': ['nome_piatto'],
                                            '3': ['nome_utente', 'giorno', 'pasto', 'ha_piano_settimanale'],
                                            '0': []}  # Out of scope doesn't require any parameters

        self.action_id_to_description = {'1': 'Aggiungere un nuovo utente alla base di conoscenza.',
                                    '2': 'Dare informazioni a un utente riguardo uno specifico piatto.',
                                    '3': "Proporre un pasto sostitutivo all'utente basandomi sulle sue esigenze alimentari e sul suo piano alimentare.",
                                    '0': 'Azione non pertinente.'} 


    def listener_callback(self, msg):
        self.get_logger().info('Received: "%s"\n' % msg.data)
        msg_json = json.loads(msg.data)
        user_input = msg_json['question']
        json_data = ast.literal_eval(msg_json['answer'])

        action_id = str(json_data['action_id'])
        json_data.pop('action_id', None)

        completed = True
        parameters = json.dumps(json_data)
        print(f"\033[34m" + "Parameters: " + str(parameters) + "\033[0m")

        if action_id == '0':
            print(f"\033[34m" + "Action ID is 0, no action needed!" + "\033[0m")
            completed = False
            missing_parameters = []
            available_actions = self.action_id_to_description.values()

            answer_prompt = f"""
                L'utente sta ponendo una domanda che non è rilevante per il sistema, 
                quindi il sistema non è in grado di fornire una risposta all'utente.
                Spiega all'utente che il sistema non è in grado di fornire una risposta.
                
                La domanda dell'utente è: {user_input}.
                Le azioni disponibili sono: {available_actions}.

                Fornire una spiegazione in italiano:"""
            
        else:
            required_parameters = self.action_id_to_required_parameters[action_id]
            missing_parameters = [param for param in required_parameters if param not in json_data]
            missing_parameters.extend([
                                    param for param in list(set(required_parameters) - set(missing_parameters)) 
                                    if json_data[param] in [0, None, '']
                                    ])
            if missing_parameters:
                completed = False 
                print(f"\033[34m" + "Missing parameters: " + str(missing_parameters) + "\033[0m")
                print(f"\033[34m" + "Incomplete answer, let's ask for more details" + "\033[0m")
                
                answer_prompt = f"""
                Chiedi all'utente maggiori dettagli per completare l'azione {action_id}: {self.action_id_to_description[action_id]}.

                La sua domanda è: {user_input}.
                Il riconoscimento dell'intento ha estratto i seguenti parametri: {parameters}.
                L'azione non può essere completata perché mancano i seguenti parametri: {missing_parameters}.
                Chiedi all'utente di fornire i parametri mancanti con una domanda formulata in linguaggio naturale.

                Formula la tua risposta in italiano:"""

        prompt = f"""
            Riassumi questo in un paragrafo in linguaggio naturale come se fosse il tuo discorso interiore.
            Non tralasciare alcun dettaglio.
            
            L'utente desidera eseguire l'azione {action_id}: {self.action_id_to_description[action_id]}.
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
            result['answer'] = self.llm.invoke(answer_prompt).content
            response_dict = {'question':user_input, 'response':result['answer']}
            response_string = json.dumps(response_dict)
            self.publisher_user_input.publish(String(data=response_string))
            self.get_logger().info('Published: "%s"' % response_string)

        else:
            print(f"\033[34m" + "Complete answer, we can procede!" + "\033[0m")

            msg = String()
            msg_json = {'question': user_input, 'action_id': action_id, 'parameters': parameters}
            msg.data = json.dumps(msg_json)

            
            self.publisher_action_dispatch.publish(msg)
            self.get_logger().info(f'Publishing {msg.data} on topic {self.query_generation_topic}')


def main(args=None):
    rclpy.init(args=args)
    inner_speech = Inner_Speech()
    rclpy.spin(inner_speech)
    inner_speech.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
