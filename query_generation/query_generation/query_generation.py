import os
import json
from .export_query_results import generate_pl_file, generate_csv_file
from langchain_groq import ChatGroq
from langchain.prompts import FewShotPromptTemplate, PromptTemplate
from langchain.chat_models import init_chat_model
from langchain_core.prompts import FewShotPromptTemplate, PromptTemplate
from langchain_core.output_parsers.string import StrOutputParser
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
BASE_DIR = "/home/kimary/unipa/src/unipa_inner_speech"
dotenv_path = os.path.join(BASE_DIR, ".env")
dotconfig_path = os.path.join(BASE_DIR, ".config")
load_dotenv(dotconfig_path)
load_dotenv(dotenv_path)

class UserInsertionTool(BaseModel):
    """Inserts user details (calories, macros, allergies) into the knowledge graph."""
    query: str = Field(description="Cypher query to insert a user.")

    def get_queries(self):
        return [self.query]


class DishInfoTool(BaseModel):
    """Returns a query to fetch dish info, and optionally evaluate user compatibility."""
    query: str = Field(description="Cypher query to fetch dish information and allergy compatibility.")

    def get_queries(self):
        return [self.query]


class MealPreparationTool(BaseModel):
    """Generates two queries: one to suggest meals and another to prepare or log the meal."""
    queries: List[str] = Field(description="List of Cypher queries for meal planning and preparation.")

    def get_queries(self):
        return self.queries


class QueryGeneratorTool(BaseModel):
    """Wrapper for modular Cypher query generation based on input intent."""
    user_insertion: Optional[UserInsertionTool] = Field(None, description="Insert a user into the graph.")
    dish_info: Optional[DishInfoTool] = Field(None, description="Get dish info and user compatibility.")
    meal_preparation: Optional[MealPreparationTool] = Field(None, description="Generate meal preparation queries.")


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
        self.in_dish_info_topic = '/dish_info'
        self.in_meal_prep_topic = '/meal_prep'

        self.out_clingo_topic = '/clingo_start'
        self.out_query_explanation = '/ex_queries'

        self.user_insertion_listener = self.create_subscription(
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

        # print(self.schema)

        self.instruction = f"""You are an expert Neo4j Cypher translator who understands questions in Italian 
        and converts them to Cypher strictly following the instructions below:

        1. Generate a Cypher query compatible ONLY with Neo4j Version 5.
        2. Do not use EXISTS, SIZE keywords in the query. Use an alias when using the WITH keyword.
        3. Do not use the same variable names for different nodes and relationships.
        4. Use only the nodes and relationships mentioned in the schema.
        5. Always enclose the Cypher output inside three backticks.
        6. Always perform case-insensitive and fuzzy searches for any property-based filtering. 
        E.g., to search for a company name, use `toLower(c.name) contains 'neo4j'`.
        7. Always use the AS keyword to assign aliases to the returned nodes and relationships.
        8. Always use aliases to refer to nodes throughout the query.
        9. Do not use the word 'Answer' in the query (it is not a Cypher keyword).
        10. You may generate multiple queries if required.
        11. Every query must start with the MATCH keyword.

        Schema:
        {self.schema}"""


        self.example_filenames = ['insertion.json', 'dish_info.json', 'meal_prep.json']
        self.tool_fields = list(QueryGeneratorTool.model_fields.keys())

        self.examples = {}
        for i, file in enumerate(self.example_filenames):
            with open(os.path.join(self.source_dir, 'fewshot_examples', file), 'r') as f:
                self.examples[i]=json.load(f)
                for example in self.examples[i]:
                    for k,v in example.items():
                        example[k] = escape_curly_braces(v)

        self.example_template = """User asks: {question}\nParameters: {parameters}\nCypher query: {query}"""
        self.suffix = """User asks: {question}\nParameters: {parameters}\nCypher query: """

        self.llm = init_chat_model(
                                    model=self.llm_config['model_name'], 
                                    model_provider=self.llm_config['model_provider'], 
                                    temperature=self.llm_config['temperature'], 
                                    api_key=os.getenv("GROQ_API_KEY")
                                )
        self.llm_with_query = self.llm.with_structured_output(QueryGeneratorTool)
        
        self.llm_response = (
            self.llm.bind()
            | StrOutputParser()
        )
        
    def prepare_few_shot_prompt(self, action_id):
        few_shot_prompt = FewShotPromptTemplate(
            input_variables=["question", "parameters"],
            examples=self.examples[action_id],
            example_prompt=PromptTemplate(
                input_variables=["question", "parameters", "query"],
                template=self.example_template
            ),
            prefix=self.instruction,
            suffix=self.suffix
            )
        return few_shot_prompt
    

    def query_generation_callback(self, msg):
        self.get_logger().info('Received: "%s" __ query_generation_callback\n')
        msg_dict = json.loads(msg.data)
        
        action_id = int(msg_dict['action_id']) - 1
        user_message = msg_dict['question']
        parameters = msg_dict['parameters']

        print(f"\033[34m{user_message=}, {parameters=}\033[0m")

        few_shot_prompt = self.prepare_few_shot_prompt(action_id)
        llm_cypher_chain = few_shot_prompt | self.llm_with_query

        cypher = llm_cypher_chain.invoke({"question": user_message, "parameters": parameters})

        field_name = self.tool_fields[action_id]
        selected_tool = getattr(cypher, field_name)
        queries = selected_tool.get_queries()

        print(f"\033[1;32m{queries}\033[0m")

        cypher, query_results = self.query_execution(queries)
        query_results = self.prepare_results_string(query_results)
        # print(f"\033[1;32m{query_results}\033[0m")
        self.send_query_output(cypher, query_results, user_message)


    def prepare_results_string(self, result_list):
        string_repr = ''
        for i,qr in enumerate(result_list):
            if i == len(result_list) - 1:
                string_repr = string_repr + str(qr)
            else:
                string_repr = string_repr + str(qr) + '\n'
        return string_repr

    
    def meal_prep_listener_callback(self, msg):
        self.get_logger().info('Received: "%s" __ meal_prep_listener_callback\n' % msg.data)
        msg_dict = json.loads(msg.data)

        example_prompt = PromptTemplate.from_template("Question: {question}\nParameters: {parameters}\nQuery1: {query1}\nQuery2: {query2}\nQuery3: {query3}")

        few_shot_prompt = self.prepare_few_shot_prompt(3, example_prompt)
        self.get_logger().info('Prepared Few Shot Prompt for Action ID: 3')

        cypher = self.llm_query_generation(few_shot_prompt, msg_dict['question'], msg_dict['parameters'])
        queries, user_results, recipes, current_meal = self.query_execution_meal_prep(cypher)

        user_string = f'''user_request:{msg_dict["question"]}.
name:{user_results[0]['name']},
calories:{user_results[0]['daily_calories']},
proteins:{user_results[0]['daily_proteins']},
carbs:{user_results[0]['daily_carbs']},
fats:{user_results[0]['daily_fats']},
allergies: {', '.join(user_results[0]['allergies'])}'''

        result = {}
        result['user_input'] = user_string
        result['queries'] = ',\n'.join(queries)

        result_string = json.dumps(result)
        self.get_logger().info('Published: "%s"' % result_string)
        self.publisher_explainability_queries.publish(String(data=result_string))

        generate_pl_file(user_results, recipes)
        generate_csv_file(user_results, recipes)

        self.publisher_clingo_start.publish(Bool(data=True))


    def llm_query_generation(self, few_shot_prompt, question, parameters):
        # print(few_shot_prompt)
        print(f"\033[34m" + f'{question=},\n {parameters=}' + "\033[0m")

        input_data = {"question": question, "parameters": parameters}
        formatted_prompt = few_shot_prompt.format(question=input_data["question"], parameters=input_data["parameters"], schema=self.schema)
        # llm_response = self.llm_response.invoke(formatted_prompt)

        cypher = self.llm_response.invoke(formatted_prompt)
        print("\033[1;32mCypher query\033[0m")
        print()
        print("\033[32m"+cypher+"\033[0m")
        print()
        return cypher
    

    def query_execution_meal_prep(self, cypher):
        if 'uery' in cypher:
            queries = [':'.join(s.split(':')[1:]).strip() for s in cypher.split('\n')]
        else:
            queries = [s.strip() for s in cypher.split('\n')]
        queries = [q for q in queries if q.startswith('MATCH')]
        print(f'Found {len(queries)} queries')

        driver = GraphDatabase.driver(self.uri, auth=(self.username, self.password))

        user_results = []
        current_meal = []
        recipes = []

        try:
            for i, q in enumerate(queries):
                print(f"\033[34m" + f'Executing Query {i+1}: {q}\n' + "\033[0m")
                with driver.session() as session:
                    # Execute the query
                    result = session.run(q)
                    print(f"\033[34mQuery results:\033[0m")
                    # print(result)
                    for record in result:
                        if i == 0:
                            user_results.append(record)
                        if i == 1:
                            current_meal.append(record)
                        if i == 2:
                            recipes.append(record)
                    if i == 0:
                        print("\033[32m" + str(user_results) + "\033[0m")
                    if i == 1:
                        print("\033[32m" + str(current_meal) + "\033[0m")
                    if i == 2:
                        print("\033[32m" + str(recipes) + "\033[0m")
                print('\n\n')
        except Exception as e:
            print("Error:", e.message)
        finally:
            driver.close()

        return  queries, user_results, recipes, current_meal


    
    def query_execution(self, cypher_list):
        driver = GraphDatabase.driver(self.uri, auth=(self.username, self.password))

        multi_query_results = []
        for cypher in cypher_list:
            query_results = []
            try:
                with driver.session() as session:
                    # Execute the query
                    result = session.run(cypher)
                    print("Query results:")


                    for record in result:
                        # print(f"\033[1;32m{record}\033[0m")
                        # query_results.append(record)
                        recdict = {}
                        for key, value in record.items():
                            # print(f'{key}: {value}')
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
                    # print("\033[32m" + str(query_results) + "\033[0m\n")

            except Exception as e:
                print("Error:", e.message)
                query_results = e.message

            multi_query_results.append(query_results)
                
        driver.close()

        print("\033[32m" + str(multi_query_results) + "\033[0m\n")

        return cypher, multi_query_results
    

    def send_query_output(self, cypher, results, user_prompt):
        result_dict = {}
        result_dict['user_input'] = user_prompt
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
