import os
import json
# from .export_query_results import generate_pl_file, generate_csv_file
from langchain.chat_models import init_chat_model
from langchain_neo4j import Neo4jGraph
from shared_utils.fewshot_helpers import escape_curly_braces, prepare_few_shot_prompt
from neo4j import GraphDatabase 
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from common_msgs.msg import Intent, QueryOutput
from dotenv import load_dotenv
import ast 
from shared_utils.customization_helpers import load_all_query_models, load_all_query_examples


# Load environment variables from .env file
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
BASE_DIR = os.path.abspath(os.path.join(CURRENT_DIR, "../../../../../../unipa_inner_speech"))
dotenv_path = os.path.join(BASE_DIR, ".env")
load_dotenv(dotenv_path)
dotconfig_path = os.path.join(BASE_DIR, ".config")
load_dotenv(dotconfig_path)




class Query_Generation(Node):
    def __init__(self):
        self.node_name = 'query_generation'
        super().__init__(f'{self.node_name}_node')
        self.query_generation_topic = '/query_generation'
        self.out_clingo_topic = '/clingo_start'
        self.out_query_explanation = '/ex_queries'

        self.query_generation_listener = self.create_subscription(
            Intent,
            self.query_generation_topic,
            self.query_generation_callback,
            10
        )

        self.publisher_clingo_start = self.create_publisher(
                                                                Bool, 
                                                                self.out_clingo_topic, 
                                                                10
                                                            )
        self.publisher_explainability_queries = self.create_publisher(
                                                                        QueryOutput, 
                                                                        self.out_query_explanation, 
                                                                        10
                                                                    )
        
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

        self.instructions = f"""You are an expert Neo4j Cypher translator who understands questions in Italian 
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

        print()
        self.scenario = os.getenv("SCENARIO")
        print(f"\033[34mUsing {self.scenario}!\033[0m")
        self.dynamic_intent_tools_dict = load_all_query_models(self.scenario)
        print(f"\033[1;38;5;207mLoaded {len(self.dynamic_intent_tools_dict.values())} intent_tool(s).\033[0m")
        print()

        self.scenario = os.getenv("SCENARIO")
        print(f"\033[34mUsing {self.scenario}!\033[0m")
        self.examples = load_all_query_examples(self.scenario)
        print(f"\033[1;38;5;207mLoaded {len(self.examples.keys())} example file(s).\033[0m")

        self.example_template = """User asks: {question}\nParameters: {parameters}\nCypher queries: {queries}"""

        self.suffix = """User asks: {question}\nParameters: {parameters}\nCypher query: """

        self.llm = init_chat_model(
            model=self.llm_config['model_name'], 
            model_provider=self.llm_config['model_provider'], 
            temperature=self.llm_config['temperature'], 
            api_key=os.getenv("GROQ_API_KEY")
        )

        
    def query_generation_callback(self, intent_msg):
        self.get_logger().info('Received: "%s" __ query_generation_callback\n')

        user_input = intent_msg.user_input
        action_name = intent_msg.action_name
        parameters = json.loads(intent_msg.parameters)

        print(f"\033[34m{user_input=}\n {parameters=}\n{action_name=}\033[0m")

        few_shot_prompt = prepare_few_shot_prompt(
                                                    instructions=self.instructions,
                                                    suffix=self.suffix, 
                                                    examples=self.examples[action_name],
                                                    example_variables=["question", "parameters", "queries"],
                                                    example_template=self.example_template,
                                                    input_variables=["question", "parameters"],
                                                    )

        llm_with_query = self.llm.with_structured_output(self.dynamic_intent_tools_dict[action_name])
        llm_cypher_chain = few_shot_prompt | llm_with_query

        cypher = llm_cypher_chain.invoke({"question": user_input, "parameters": parameters})

        queries = getattr(cypher, 'query')

        if type(queries) == str:
            queries = [queries]

        query_results = self.query_execution(queries)
        query_results = self.prepare_results_string(query_results)

        self.send_query_output(queries, query_results, user_input, action_name)


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

        return multi_query_results
    

    def send_query_output(self, cypher, results, user_input, action_name):

        query_output_msg = QueryOutput()
        query_output_msg.action_name = action_name
        query_output_msg.queries = cypher
        query_output_msg.user_input = user_input
        query_output_msg.results = str(results)

        self.get_logger().info('Published: "%s"' % query_output_msg)
        self.publisher_explainability_queries.publish(query_output_msg)


def main(args=None):
    rclpy.init(args=args)
    query_generation = Query_Generation()
    rclpy.spin(query_generation)
    query_generation.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
