import os
import json
from .export_query_results import generate_pl_file, generate_csv_file
from langchain.prompts import FewShotPromptTemplate, PromptTemplate
from langchain.chat_models import init_chat_model
from langchain_core.prompts import FewShotPromptTemplate, PromptTemplate
from langchain_neo4j import Neo4jGraph
from neo4j import GraphDatabase 
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from dotenv import load_dotenv
from pydantic import BaseModel, Field
import ast 
from typing import Optional, List

# Load environment variables from .env file
BASE_DIR = "/home/belca/Desktop/ros2_humble_ws/src"
dotenv_path = os.path.join(BASE_DIR, ".env")
dotconfig_path = os.path.join(BASE_DIR, ".config")
load_dotenv(dotconfig_path)
load_dotenv(dotenv_path)

class UserInsertionTool(BaseModel):
    """Inserts user details (calories, macros, allergies) into the knowledge graph. Additionally, you must create the relations to allergens if provided."""
    query: str = Field(description="Cypher query to insert a user.")

class DishInfoTool(BaseModel):
    """Returns a query to fetch dish info, and optionally evaluate user compatibility."""
    query: str = Field(description="Cypher query to fetch dish information and allergy compatibility.")

class MealPreparationTool(BaseModel):
    """Generates 2 queries: 
    first one to check user's allergies,
    second one return the dishes compatible with the user's allergies."""
    query: List[str] = Field(description="List of Cypher queries for meal planning and preparation.")


action_id_2_action_class = {1: UserInsertionTool, 2: DishInfoTool, 3: MealPreparationTool}

def escape_curly_braces(text):
    """
    Escape curly braces in the text by doubling them.
    """
    return text.replace("{", "{{").replace("}", "}}")


class Query_Generation(Node):
    def __init__(self):
        self.node_name = 'query_generation'
        super().__init__(f'{self.node_name}_node')
        self.query_generation_topic = '/query_generation'

        self.out_clingo_topic = '/clingo_start'
        self.out_query_explanation = '/ex_queries'

        self.query_generation_listener = self.create_subscription(
            String,
            self.query_generation_topic,
            self.query_generation_callback,
            10
        )

        self.publisher_clingo_start = self.create_publisher(Bool, self.out_clingo_topic, 10)
        self.publisher_explainability_queries = self.create_publisher(String, self.out_query_explanation, 10)
        
        self.get_logger().info('Inner Speech Node has been started')

        print(f"\033[34mQuery Generation Node started!!!\033[0m")
        print(f"\033[34mInitialized publishers to {self.out_clingo_topic}!!!\033[0m")
        print(f"\033[34mInitialized publishers to {self.out_query_explanation}!!!\033[0m")
        print(f"\033[34mStarted Listening to {self.query_generation_topic}!!!\033[0m")

        self.uri = os.getenv("NEO4J_URI")
        self.username = os.getenv("NEO4J_USERNAME")
        self.password = os.getenv("NEO4J_PASSWORD")

        self.llm_config = ast.literal_eval(os.getenv("LLM_CONFIG"))[self.node_name]

        self.ws_dir = os.getenv("ROS2_WORKSPACE")
        self.source_dir = os.path.join(self.ws_dir, 'query_generation', 'query_generation')

        self.graph = Neo4jGraph(self.uri, self.username, self.password)
        self.schema = escape_curly_braces(self.graph.schema)

        # self.instruction = f"""You are an expert Neo4j Cypher translator who understands questions in Italian 
        # and converts them to Cypher strictly following the instructions below:

        # 1. Generate a Cypher query compatible ONLY with Neo4j Version 5.
        # 2. Do not use EXISTS, SIZE keywords in the query. Use an alias when using the WITH keyword.
        # 3. Do not use the same variable names for different nodes and relationships.
        # 4. Use only the nodes and relationships mentioned in the schema.
        # 5. Always enclose the Cypher output inside three backticks.
        # 6. Always use the AS keyword to assign aliases to the returned nodes and relationships.
        # 7. Always use aliases to refer to nodes throughout the query.
        # 8. Do not use the word 'Answer' in the query (it is not a Cypher keyword).
        # 9. You may generate multiple queries if required.
        # 10. Every query must start with the MATCH keyword.

        # Schema:
        # {self.schema}"""

        self.instruction = f"""You are an expert Neo4j Cypher translator who understands questions in Italian 
        and converts them to Cypher strictly following the instructions below:

        1. Generate a Cypher query compatible ONLY with Neo4j Version 5.
        2. Do not use the same variable names for different nodes and relationships.
        3. Use only the nodes and relationships mentioned in the schema.
        4. Always enclose the Cypher output inside three backticks.
        5. Always use the AS keyword to assign aliases to the returned nodes and relationships.
        6. Always use aliases to refer to nodes throughout the query.
        7. Do not use the word 'Answer' in the query (it is not a Cypher keyword).
        8. You may generate multiple queries if required.

        Schema:
        {self.schema}"""

        self.example_filenames = ['FewShot_query_insertion.json', 'FewShot_query_dish_info.json', 'FewShot_query_meal_prep.json']

        self.examples = {}
        for i, file in enumerate(self.example_filenames, start=1):
            with open(os.path.join(self.source_dir, 'fewshot_examples', file), 'r') as f:
                self.examples[i]=json.load(f)
                for j, example in enumerate(self.examples[i]):
                    for k,v in example.items():
                        example[k] = escape_curly_braces(v)
                    self.examples[i][j] = self.queries_to_query_list(example)

        self.example_template = """User asks: {question}\nParameters: {parameters}\nCypher queries: {queries}"""
        # self.example_template_3 = """User asks: {question}\nParameters: {parameters}\nQuery1: {query1}\nQuery2: {query2}"""
        self.suffix = """User asks: {question}\nParameters: {parameters}\nCypher query: """

        self.llm = init_chat_model(
            model=self.llm_config['model_name'], 
            model_provider=self.llm_config['model_provider'], 
            temperature=self.llm_config['temperature'], 
            api_key=os.getenv("GROQ_API_KEY")
        )

    def queries_to_query_list(self, example):
        query_list = []
        for k,v in example.items():
            if 'query' in k.lower():
                query_list.append(v)
        updated_dict = {k: v for k,v in example.items() if 'query' not in k.lower()}
        updated_dict['queries'] = query_list
        return updated_dict


        
    def prepare_few_shot_prompt(self, action_id):
        few_shot_prompt = FewShotPromptTemplate(
            input_variables=["question", "parameters"],
            examples=self.examples[action_id],
            example_prompt=PromptTemplate(
                input_variables=["question", "parameters", "queries"],
                template=self.example_template
            ),
            prefix=self.instruction,
            suffix=self.suffix
            )
        return few_shot_prompt
    

    def query_generation_callback(self, msg):
        self.get_logger().info('Received: "%s" __ query_generation_callback\n')
        msg_dict = json.loads(msg.data)
        
        action_id = int(msg_dict['action_id'])
        user_message = msg_dict['question']
        parameters = msg_dict['parameters']

        print(f"\033[34m{user_message=}\n {parameters=}\n{action_id=}\033[0m")

        few_shot_prompt = self.prepare_few_shot_prompt(action_id)
        # print(f"\033[34m{few_shot_prompt=}\033[0m")
        llm_with_query = self.llm.with_structured_output(action_id_2_action_class[action_id])
        llm_cypher_chain = few_shot_prompt | llm_with_query

        cypher = llm_cypher_chain.invoke({"question": user_message, "parameters": parameters})

        queries = getattr(cypher, 'query')

        if type(queries) == str:
            queries = [queries]

        cypher, query_results = self.query_execution(queries)
        query_results = self.prepare_results_string(query_results)

        self.send_query_output(cypher, query_results, user_message, action_id)


    def prepare_results_string(self, result_list):
        string_repr = ''
        for i,qr in enumerate(result_list):
            if i == len(result_list) - 1:
                string_repr = string_repr + str(qr)
            else:
                string_repr = string_repr + str(qr) + '\n'
        return string_repr
    
    
    def query_execution(self, cypher_list):
        driver = GraphDatabase.driver(self.uri, auth=(self.username, self.password))

        multi_query_results = []
        for cypher in cypher_list:
            query_results = []
            try:
                with driver.session() as session:
                    # Execute the query
                    result = session.run(cypher)
                    print("Query:")
                    print(f"\033[32m{cypher}\033[0m\n")
                    print(f"Query results:")

                    for record in result:
                        recdict = {}
                        for key, value in record.items():
                            if isinstance(value, list):
                                # It's a collected list of nodes
                                sublist = []
                                for item in value:
                                    if hasattr(item, "items"):
                                        subdict = {k: v for k, v in item.items()}
                                        sublist.append(subdict)
                                    else:
                                        sublist.append(item)  # fallback if not a node
                                recdict[key] = sublist
                            elif hasattr(value, "items"):
                                # It's a single node
                                subdict = {k: v for k, v in value.items()}
                                recdict[key] = subdict
                            else:
                                recdict[key] = value  # fallback for primitives
                        query_results.append(recdict)
                        print("\033[32m" + str(recdict) + "\033[0m\n")

            except Exception as e:
                print("Error:", e.message)
                query_results = e.message

            multi_query_results.append(query_results)
                
        driver.close()

        # print("\033[32m" + str(multi_query_results) + "\033[0m\n")

        return cypher, multi_query_results
    

    def send_query_output(self, cypher, results, user_prompt, action_id):
        result_dict = {}
        result_dict['user_input'] = user_prompt
        result_dict['action_id'] = action_id
        result_dict['queries'] = cypher
        result_dict['query_results'] = str(results)

        result_string = json.dumps(result_dict)
        self.get_logger().info('Published: "%s"' % result_string)
        self.publisher_explainability_queries.publish(String(data=result_string))


def main(args=None):
    rclpy.init(args=args)
    query_generation = Query_Generation()
    rclpy.spin(query_generation)
    query_generation.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
