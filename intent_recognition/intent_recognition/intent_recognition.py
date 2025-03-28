import os
import json
from langchain_neo4j import Neo4jGraph
from langchain_groq import ChatGroq
from langchain_openai import ChatOpenAI
from langchain_core.prompts import FewShotPromptTemplate, PromptTemplate
from langchain_core.output_parsers.string import StrOutputParser
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

os.environ["GROQ_API_KEY"]



class Inner_Speech(Node):
    def __init__(self):
        super().__init__('intent_recognition_node')
        self.in_topic = '/user_input'
        self.out_topic = '/user_intent'

        self.subscription = self.create_subscription(
            String,
            self.in_topic,
            self.listener_callback,
            10)

        self.publisher = self.create_publisher(String, self.out_topic, 10)

        print("\033[34mIntent Recognition Node started!!!\033[0m")
        print("\033[34mInitialized publishers to {self.out_topic}!!!\033[0m")
        print("\033[34mStarted Listening to {self.in_topic}!!!\033[0m")

        self.uri = "bolt://localhost:7689"  # Replace with your URI if not localhost
        self.username = "neo4j"             # Replace with your username
        self.password = "12341234"          # Replace with your password

        self.ws_dir = os.getenv("ROS2_WORKSPACE", "/home/belca/Desktop/ros2_humble_ws")  # Replace with your workspace path if needed
        self.source_dir = os.path.join(self.ws_dir, 'src', 'intent_recognition', 'intent_recognition')

        self.examples = []
        with open(os.path.join(self.source_dir, 'few_shot_examples/FewShot_intent.json'), 'r') as f:
            self.examples = json.load(f)["examples"]

        self.graph = Neo4jGraph(self.uri, self.username, self.password)
        self.schema = self.graph.schema

        self.llm = ChatGroq(model="llama3-70b-8192", temperature=0)

        self.example_prompt = PromptTemplate.from_template(
            """question: {question}\nanswer: {answer}\n"""
        )

        self.prompt = FewShotPromptTemplate(
            examples=self.examples,
            example_prompt=self.example_prompt,
            prefix='''Sono un Robot di nome Pepper, esperto in Neo4j. Devo aiutare un utente a soddisfare le sue esigenze alimentari ed ho a disposizione una base di conoscenza con il seguente schema: {schema}.
    Data una richiesta dell'utente, devo capire se è pertinente all'argomento e devo capire che funzione desidera utilizzare. 
    Le azioni che sono in grado di effettuare sono le seguenti:
    0) Azione non pertinente.
    1) Aggiungere un nuovo utente alla base di conoscenza. Parametri [nome_utente!, calorie!, proteine!, carboidrati!, grassi!, intolleranze].
    2) Dare informazioni a un utente riguardo uno specifico piatto. Parametri [nome_utente, nome_piatto!].
    3) Proporre un pasto all'utente basandomi sulle sue esigenze alimentari. Un pasto è inteso come una combinazione di piatti. Parametri [nome_utente!, allergeni].
    I parametri seguiti da un punto esclamativo sono obbligatori.
    
    ''',
            suffix='''Ritorna solamente la risposta in formato json senza alcun altro tipo di testo. La risposta deve essere schematica e deve rispettare il seguente formato:
    question: {question},\n''',
            input_variables=["question", "schema"],
        )

        self.llm_response = (
            self.llm.bind()
            | StrOutputParser()
            | self.extract_answer
        )

    def extract_answer(self, llm_output: str) -> str:
        if "Answer:" in llm_output:
            return llm_output.split("Answer:", 1)[1].strip()
        if "answer:" in llm_output:
            return llm_output.split("answer:", 1)[1].strip()
        return llm_output.strip()

    def listener_callback(self, msg):
        self.get_logger().info('Received: "%s"\n' % msg.data)
        user_input = msg.data
        input_data = {"question": user_input}
        formatted_prompt = self.prompt.format(question=input_data["question"], schema=self.schema)
        llm_response = self.llm_response.invoke(formatted_prompt)

        result = {}
        # print(llm_response)
        # print(result.keys())
        result['question'] = user_input
        result['answer'] = llm_response
        result_string = json.dumps(result)

        print("\033[32m"+result['answer']+"\033[0m")

        self.publisher.publish(String(data=result_string))
        self.get_logger().info('Published: "%s"' % result_string)

def main(args=None):
    rclpy.init(args=args)
    inner_speech = Inner_Speech()
    rclpy.spin(inner_speech)
    inner_speech.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
