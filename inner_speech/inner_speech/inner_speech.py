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

# Load environment variables from .env file
BASE_DIR = "/home/belca/Desktop/ros2_humble_ws/src"
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

        print("\033[34mInner Speech Node started!!!\033[0m")
        print("\033[34mInitialized publishers to {self.out_topic}!!!\033[0m")
        print("\033[34mStarted Listening to {self.in_topic}!!!\033[0m")

        self.action_dict = {0: '/out_of_scope', 1: '/user_insertion', 2: '/dish_info', 3: '/meal_prep'}

        self.uri = os.getenv("NEO4J_URI")
        self.username = os.getenv("NEO4J_USERNAME")
        self.password = os.getenv("NEO4J_PASSWORD")

        self.ws_dir = os.getenv("ROS2_WORKSPACE")  # Replace with your workspace path if needed
        self.source_dir = os.path.join(self.ws_dir, 'inner_speech', 'inner_speech')

        self.graph = Neo4jGraph(self.uri, self.username, self.password)
        self.schema = self.graph.schema

        self.examples = []
        with open(os.path.join(self.source_dir, 'few_shot_examples/FewShot_intent_IS.json'), 'r') as f:
            self.examples = json.load(f)["examples"]


        self.llm = ChatGroq(model="llama3-70b-8192", temperature=0, api_key=os.getenv("GROQ_API_KEY"))

        self.example_prompt = PromptTemplate.from_template(
            """Question: {question}\nAction_ID: {action_id}\nParameters: {parameters}\nAnswer: {answer}\n"""
        )

        self.prompt = FewShotPromptTemplate(
            examples=self.examples,
            example_prompt=self.example_prompt,
            prefix="""Sono un Robot di nome Pepper, esperto in Neo4j. Devo aiutare un utente a soddisfare le sue esigenze alimentari ed ho a disposizione una base di conoscenza con il seguente schema: {schema}.
        Data una richiesta dell'utente, devo capire se è pertinente all'argomento e determinare quale funzione desidera utilizzare.
        Le azioni che sono in grado di effettuare sono le seguenti:
        0. Azione non pertinente.
        1. Aggiungere un nuovo utente alla base di conoscenza.
            Parametri: nome_utente (obbligatorio), calorie (obbligatorio), proteine (obbligatorio), carboidrati (obbligatorio), grassi (obbligatorio), intolleranze (facoltativo).
        2. Dare informazioni a un utente riguardo uno specifico piatto.
            Parametri: nome_utente (facoltativo), nome_piatto (obbligatorio).
        3. Proporre un pasto sostitutivo all'utente basandomi sulle sue esigenze alimentari e sul suo piano alimentare.
            Parametri: nome_utente (obbligatorio), allergeni (facoltativo), giorno (facoltativo), pasto (facoltativo).
        I parametri indicati come "obbligatorio" devono sempre essere forniti per eseguire correttamente l'azione richiesta.""",
            suffix='''Ritorna solamente la risposta in formato json senza alcun altro tipo di testo. La risposta deve essere schematica e deve rispettare il seguente formato:
    Question: {question}\nAction_ID: {action_id}\nParameters: {parameters}\n''',
            input_variables=["question", "action_id", "parameters", "schema"],
        )

        self.llm_response = (
            self.llm.bind()
            | StrOutputParser()
            | self.extract_answer
        )


    def extract_answer(self, llm_output: str) -> str:
        if "Answer:" in llm_output:
            return llm_output.split("Answer:", 1)[1].strip()
        return llm_output.strip()


    def listener_callback(self, msg):
        # self.get_logger().info('Received: "%s"\n' % msg.data)
        msg_json = json.loads(msg.data)
        user_input = msg_json['question']
        json_data = ast.literal_eval(msg_json['answer'])

        action_id = str(json_data['action_id'])
        json_data.pop('action_id', None)
        parameters = json.dumps(json_data)
  
        # self.get_logger().info('Extracted -> \nQuestion: "%s", \nAction_ID: "%s", \nParameters: "%s"' % (user_input, action_id, parameters))
        print("\033[34m" + f'{user_input},\n {action_id},\n {parameters}' + "\033[0m")

        input_data = {"question": user_input, "action_id": action_id, "parameters": parameters}
        formatted_prompt = self.prompt.format(question=input_data["question"], action_id=input_data["action_id"], parameters=input_data["parameters"], schema=self.schema)
        llm_response = self.llm_response.invoke(formatted_prompt)

        with open(os.path.join(self.ws_dir, 'inner_speech', 'inners.json'), 'a+') as f:
            f.write(json.dumps({"question": user_input, "action_id": action_id, "parameters": parameters, "answer": llm_response}) + ',\n')

        result = {}
        result['question'] = user_input
        result['answer'] = llm_response
        result_string = json.dumps(result)

        print("\033[32m"+result['answer']+"\033[0m")

        result['answer'] = ast.literal_eval(result['answer'])
        complete = result['answer']['completed'].lower() == 'true'

        if not complete:
            print("\033[34m" + "Incomplete answer, let's ask for more details" + "\033[0m")
            response_dict = {'question':user_input, 'response':result['answer']['response']}
            response_string = json.dumps(response_dict)
            self.publisher_user_input.publish(String(data=response_string))
            self.get_logger().info('Published: "%s"' % result_string)

        else:
            print("\033[34m" + "Complete answer, we can procede!" + "\033[0m")

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
