import os
import json
from langchain.chat_models import init_chat_model
from langchain_core.prompts import FewShotPromptTemplate, PromptTemplate
from langchain_core.output_parsers.string import StrOutputParser
from langchain_neo4j import Neo4jGraph
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from dotenv import load_dotenv
import ast

# Load environment variables from .env file
BASE_DIR = "/home/kimary/unipa/src/unipa_inner_speech"
dotenv_path = os.path.join(BASE_DIR, ".env")
load_dotenv(dotenv_path)
dotconfig_path = os.path.join(BASE_DIR, ".config")
load_dotenv(dotconfig_path)

def escape_curly_braces(text):
    """
    Escape curly braces in the text by doubling them.
    """
    return text.replace("{", "{{").replace("}", "}}")

class Explainability(Node):
    def __init__(self):
        self.node_name = 'explainability'
        super().__init__(f'{self.node_name}_node')
        self.query_explanation_topic = '/ex_queries'
        self.clingo_explanation_topic = '/ex_clingo'
        self.robot_dialogue_topic = '/speak'

        self.listener_query_explanation = self.create_subscription(
            String,
            self.query_explanation_topic,
            self.query_explanation_callback,
            10)
        
        self.listener_clingo_explanation = self.create_subscription(
            String,
            self.clingo_explanation_topic,
            self.clingo_explanation_callback,
            10)
        
        self.robot_dialogue_publisher = self.create_publisher(String, self.robot_dialogue_topic, 10)

        print(f"\033[34mExplainability Node started!!!\033[0m")
        print(f"\033[34mInitialized publishers to {self.robot_dialogue_topic}!!!\033[0m")
        print(f"\033[34mStarted Listening to {self.query_explanation_topic}!!!\033[0m")
        print(f"\033[34mStarted Listening to {self.clingo_explanation_topic}!!!\033[0m")

        self.uri = os.getenv("NEO4J_URI")
        self.username = os.getenv("NEO4J_USERNAME")
        self.password = os.getenv("NEO4J_PASSWORD")

        self.llm_config = ast.literal_eval(os.getenv("LLM_CONFIG"))[self.node_name]

        self.ws_dir = os.getenv("ROS2_WORKSPACE")
        self.source_dir = os.path.join(self.ws_dir, 'explainability', 'explainability')

        self.graph = Neo4jGraph(self.uri, self.username, self.password)
        self.schema = self.graph.schema

        self.example_filenames = ['FewShot_queries_explanation_insertion.json', 'FewShot_queries_explanation_dish_info.json', 'FewShot_queries_explanation_meal_prep.json']
        self.examples = {}
        for i, file in enumerate(self.example_filenames):
            with open(os.path.join(self.source_dir, 'fewshot_examples', file), 'r') as f:
                self.examples[i]=json.load(f)["examples"]
                for example in self.examples[i]:
                    for k,v in example.items():
                        example[k] = escape_curly_braces(v)
        # with open(os.path.join(self.source_dir, 'fewshot_examples/FewShot_queries_explanation.json'), 'r') as f:
        #     self.examples['queries'] = json.load(f)["examples"]
        #     for example in self.examples['queries']:
        #         for k,v in example.items():
        #             example[k] = escape_curly_braces(v)
        with open(os.path.join(self.source_dir, 'fewshot_examples/FewShot_clingo_explanation.json'), 'r') as f:
            self.examples['clingo'] = json.load(f)["examples"]

        self.llm = init_chat_model(
                                    model=self.llm_config['model_name'], 
                                    model_provider=self.llm_config['model_provider'], 
                                    temperature=self.llm_config['temperature'], 
                                    api_key=os.getenv("GROQ_API_KEY")
                                )
        
        self.llm_response = (
            self.llm.bind()
            | StrOutputParser()
            # | self.extract_answer
        )


    def extract_answer(self, llm_output: str) -> str:
        if "Answer:" in llm_output:
            return llm_output.split("Answer:", 1)[1].strip()
        return llm_output.strip()


    def query_explanation_callback(self, msg):
        msg_dict = json.loads(msg.data)
        action_id = msg_dict['action_id']
        self.get_logger().info('Received: "%s" __ query_explanation_callback\n' % action_id)

        example_prompt = PromptTemplate.from_template("\nUser Input: {user_input}\nQueries: {queries}\nQuery Results: {query_results}\nExplanation: {explanation}")

        prompt = FewShotPromptTemplate(
            examples=self.examples[action_id],
            example_prompt=example_prompt,
            prefix="Tu sei un Robot di nome Pepper e devi supportare i tuoi utenti nel seguire un corretto piano alimentare. Data una richiesta e la sua traduzione in cypher query con i relativi risultati, devi spiegare all'utente il processo decisionale ed il risulato.",
            suffix="Rispondini in linguaggio naturale in lingua Italiana in modo sintetico.\nUser Input: {user_input}\nQueries: {queries}\nQuery Results: {query_results}\nExplanation: ",
            input_variables=["user_input", "queries", "query_results"],
        )

        print(prompt)
        print(f"\033[34m{msg_dict['user_input']=}, {msg_dict['queries']=}, {msg_dict['query_results']=}\033[0m")


        formatted_prompt = prompt.format(
                                        user_input = msg_dict['user_input'], 
                                        queries = msg_dict['queries'], 
                                        query_results = msg_dict['query_results']
                                        )
        
        explanation = self.llm_response.invoke(formatted_prompt)

        # Output the explanation
        print(f"\033[1;34mExplanation:\033[0m")
        print(f"\033[1;32m{explanation}\033[0m")


    def clingo_explanation_callback(self, msg):
        self.get_logger().info('Received: "%s" __ clingo_explanation_callback\n' % msg.data)
        msg_dict = json.loads(msg.data)

        example_prompt = PromptTemplate.from_template("Results: {results}\nExplanation: {explanation}")

        prompt = FewShotPromptTemplate(
            examples=self.examples['clingo'],
            example_prompt=example_prompt,
            prefix="Tu sei un Robot di nome Pepper e devi supportare un utente nel seguire un corretto piano alimentare basato sui suoi bisogni e preferenze. A questo punto del processo abbiamo escluso già i piatti non adatti allo stile alimentare dell'utente e, in questo step, abbiamo generato diverse combinazioni di piatti in grado di soddisfare i vincoli di calorie e macronutrienti rimanenti. Data una una lista di combinazioni di piatti, il tuo compito è spiegare all'utente come sono stati scelti. Il numero di piatti in ogni risposta può essere 1, N, o 0 dipendentemente dai requisiti.",
            suffix="Rispondini in linguaggio naturale in lingua Italiana.\nUser Input: {results}\nExplanation: ",
            input_variables=["results"],
        )

        formatted_prompt = prompt.format(results = msg_dict['results'])
        
        explanation = self.llm_response.invoke(formatted_prompt)

        # Output the explanation
        print(f"\033[1;34mExplanation:\033[0m")
        print(f"\033[1;32m{explanation}\033[0m")


def main(args=None):
    rclpy.init(args=args)
    explainability = Explainability()
    rclpy.spin(explainability)
    explainability.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
