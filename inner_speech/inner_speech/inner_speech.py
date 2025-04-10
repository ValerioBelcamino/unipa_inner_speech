import os
import json
from langchain_groq import ChatGroq
from langchain_core.prompts import FewShotPromptTemplate, PromptTemplate
from langchain_core.output_parsers.string import StrOutputParser
from langchain_neo4j import Neo4jGraph
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import ast
from dotenv import load_dotenv
from pydantic import BaseModel, Field

# Load environment variables from .env file
BASE_DIR = "/home/kimary/unipa/src/unipa_inner_speech"
dotenv_path = os.path.join(BASE_DIR, ".env")
load_dotenv(dotenv_path)


class Inner_Speech(Node):
    def __init__(self):
        super().__init__('inner_speech_node')
        self.in_topic = '/user_intent'
        self.out_topic = '/user_input_activation'

        self.subscription = self.create_subscription(
            String,
            self.in_topic,
            self.listener_callback,
            10)
        
        self.publisher_user_input = self.create_publisher(String, '/user_input_activation', 10)

        print(f"\033[34mInner Speech Node started!!!\033[0m")
        print(f"\033[34mInitialized publishers to {self.out_topic}!!!\033[0m")
        print(f"\033[34mStarted Listening to {self.in_topic}!!!\033[0m")

        self.action_dict = {0: '/out_of_scope', 1: '/user_insertion', 2: '/dish_info', 3: '/meal_prep'}

        self.uri = os.getenv("NEO4J_URI")
        self.username = os.getenv("NEO4J_USERNAME")
        self.password = os.getenv("NEO4J_PASSWORD")

        self.ws_dir = os.getenv("ROS2_WORKSPACE")  # Replace with your workspace path if needed
        self.source_dir = os.path.join(self.ws_dir, 'inner_speech', 'inner_speech')

        self.graph = Neo4jGraph(self.uri, self.username, self.password)
        self.schema = self.graph.schema

        self.llm = ChatGroq(model="llama3-70b-8192", temperature=0, api_key=os.getenv("GROQ_API_KEY"))

        self.action_id_to_required_parameters = {'1': ['nome_utente', 'calorie', 'proteine', 'carboidrati', 'grassi'],
                                            '2': ['nome_piatto'],
                                            '3': ['nome_utente', 'giorno', 'pasto'],
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
        parameters = json.dumps(json_data)
        print(f"\033[34m" + "Parameters: " + str(parameters) + "\033[0m")

        required_parameters = self.action_id_to_required_parameters[action_id]
        missing_parameters = [param for param in required_parameters if json_data[param] in [0, None, '']]
        if missing_parameters:
            print(f"\033[34m" + "Missing parameters: " + str(missing_parameters) + "\033[0m")
            completed = False
        else:
            completed = True

        prompt = f"""
            Riassumi questo in un paragrafo in linguaggio naturale come se fosse il tuo discorso interiore.
            Non tralasciare alcun dettaglio.
            
            L'utente desidera eseguire l'azione {action_id}: {self.action_id_to_description[action_id]}.
            La loro domanda è: {user_input}.
            Il riconoscimento dell'intento ha estratto i seguenti parametri: {parameters}.
            L'azione può essere completata: {completed}.
            Parametri mancanti: {missing_parameters}
            
            Il tuo discorso interiore in italiano:"""

        llm_response = self.llm.invoke(prompt)

        result = {}
        result['question'] = user_input
        result['inner_speech'] = llm_response.content
        result_string = json.dumps(result)

        print("\033[32m"+result_string+"\033[0m")

        if not completed:
            print(f"\033[34m" + "Incomplete answer, let's ask for more details" + "\033[0m")
            answer_prompt = f"""
                Chiedi all'utente maggiori dettagli per completare l'azione {action_id}: {self.action_id_to_description[action_id]}.

                La sua domanda è: {user_input}.
                Il riconoscimento dell'intento ha estratto i seguenti parametri: {parameters}.
                L'azione non può essere completata perché mancano i seguenti parametri: {missing_parameters}.
                Chiedi all'utente di fornire i parametri mancanti con una domanda formulata in linguaggio naturale.

                Formula la tua risposta in italiano:"""
            result['answer'] = self.llm.invoke(answer_prompt).content
            response_dict = {'question':user_input, 'response':result['answer']}
            response_string = json.dumps(response_dict)
            self.publisher_user_input.publish(String(data=response_string))
            self.get_logger().info('Published: "%s"' % result_string)

        else:
            print(f"\033[34m" + "Complete answer, we can procede!" + "\033[0m")

            msg = String()
            msg_json = {'question': user_input, 'parameters': parameters}
            msg.data = json.dumps(msg_json)

            publisher_action_dispatch = self.create_publisher(String, self.action_dict[int(action_id)], 10)
            publisher_action_dispatch.publish(msg)
            self.get_logger().info(f'Publishing {msg.data} on topic {self.action_dict[int(action_id)]}')


def main(args=None):
    rclpy.init(args=args)
    inner_speech = Inner_Speech()
    rclpy.spin(inner_speech)
    inner_speech.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
