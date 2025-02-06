import os
import json
from langchain_groq import ChatGroq
from langchain_openai import ChatOpenAI
from langchain_core.prompts import FewShotPromptTemplate, PromptTemplate
from langchain_core.output_parsers.string import StrOutputParser
from langchain_neo4j import Neo4jGraph
from neo4j import GraphDatabase 
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import ast


class Query_Generation(Node):
    def __init__(self):
        super().__init__('inner_speech_node')
        self.in_user_insertion_topic = '/user_insertion'
        self.in_dish_info_topic = '/dish_info'
        self.in_meal_prep_topic = '/meal_prep'

        self.out_clingo_topic = '/clingo_start'
        self.out_query_explanation = '/ex_queries'

        self.user_insertion_listener = self.create_subscription(
            String,
            self.in_user_insertion_topic,
            self.user_insertion_listener_callback,
            10
        )
        self.dish_info_listener = self.create_subscription(
            String,
            self.in_dish_info_topic,
            self.dish_info_listener_callback,
            10
        )
        self.meal_prep_listener = self.create_subscription(
            String,
            self.in_meal_prep_topic,
            self.meal_prep_listener_callback,
            10
        )


        self.publisher_clingo_start = self.create_publisher(Bool, self.out_clingo_topic, 10)
        self.publisher_explainability_queries = self.create_publisher(String, self.out_query_explanation, 10)

        self.get_logger().info('Inner Speech Node has been started')
        self.get_logger().info('Publishing: "%s"' % self.out_clingo_topic)
        self.get_logger().info('Publishing: "%s"' % self.out_query_explanation)
        print(f'Started Listening to {self.in_user_insertion_topic}!!!')
        print(f'Started Listening to {self.in_dish_info_topic}!!!')
        print(f'Started Listening to {self.in_meal_prep_topic}!!!')

        self.uri = "bolt://localhost:7689"  # Replace with your URI if not localhost
        self.username = "neo4j"             # Replace with your username
        self.password = "12341234"          # Replace with your password

        self.ws_dir = os.getenv("ROS2_WORKSPACE", "/home/belca/Desktop/ros2_humble_ws")  # Replace with your workspace path if needed
        self.source_dir = os.path.join(self.ws_dir, 'src', 'query_generation', 'query_generation')

        self.graph = Neo4jGraph(self.uri, self.username, self.password)
        self.schema = self.graph.schema


        self.example_filenames = ['FewShot_query_insertion.json', 'FewShot_query_dish_info.json', 'FewShot_query_meal_prep.json']
        self.examples = {}
        for i, file in enumerate(self.example_filenames):
            with open(os.path.join(self.source_dir, 'fewshot_examples', file), 'r') as f:
                self.examples[i]=json.load(f)["examples"]

        self.llm = ChatGroq(model="llama3-70b-8192",temperature=0)#llama3-70b-8192


        self.llm_response = (
            self.llm.bind()
            | StrOutputParser()
            # | self.extract_answer
        )

    def extract_answer(self, llm_output: str) -> str:
        if "Answer:" in llm_output:
            return llm_output.split("Answer:", 1)[1].strip()
        return llm_output.strip()
    
    def prepare_few_shot_prompt(self, action_id, _example_prompt):
        print(self.example_filenames[action_id-1])
        if action_id == 0 or action_id == 1:
            _suffix='''Ritornami esclusivamente una singola Cypher query e non aggiungere altro testo.\nQuestion: {question},\nParameters: {parameters},\n'''
        else:
            _suffix='''Ritornami esclusivamente una singola Cypher query e non aggiungere altro testo.\nQuestion: {question},\nParameters: {parameters},\n'''
        return FewShotPromptTemplate(
            examples=self.examples[action_id-1],
            example_prompt=_example_prompt,
            prefix='''Tu sei un Robot di nome Pepper, esperto in Neo4j. Dato una domanda in input crea due query Cypher sintatticamente corrette da eseguire. Hai a disposizione lo schema con le informazioni del database neo4j: {schema}. Inoltre, sotto trovi un numero di esempi di domande con la relativa traduzione in codice Cypher.''',
            suffix=_suffix,
            input_variables=["question", "schema", "parameters"],
        )
    
    def user_insertion_listener_callback(self, msg):
        self.get_logger().info('Received: "%s" __ user_insertion_listener_callback' % msg.data)
        msg_dict = json.loads(msg.data)

        example_prompt = PromptTemplate.from_template("Question: {question}\nParameters: {parameters}\n{query}")

        few_shot_prompt = self.prepare_few_shot_prompt(1, example_prompt)
        self.get_logger().info('Prepared Few Shot Prompt for Action ID: 1')

        self.llm_query_generation(few_shot_prompt, msg_dict['question'], msg_dict['parameters'])


    def dish_info_listener_callback(self, msg):
        self.get_logger().info('Received: "%s" __ dish_info_listener_callback' % msg.data)
        msg_dict = json.loads(msg.data)

        example_prompt = PromptTemplate.from_template("Question: {question}\nParameters: {parameters}\n{query}")

        few_shot_prompt = self.prepare_few_shot_prompt(2, example_prompt)
        self.get_logger().info('Prepared Few Shot Prompt for Action ID: 2')

        self.llm_query_generation(few_shot_prompt, msg_dict['question'], msg_dict['parameters'])



    def meal_prep_listener_callback(self, msg):
        self.get_logger().info('Received: "%s" __ meal_prep_listener_callback' % msg.data)
        msg_dict = json.loads(msg.data)

        example_prompt = PromptTemplate.from_template("Question: {question}\nParameters: {parameters}\nQuery1: {query1}\nQuery2: {query2}")

        few_shot_prompt = self.prepare_few_shot_prompt(3, example_prompt)
        self.get_logger().info('Prepared Few Shot Prompt for Action ID: 3')

        cypher = self.llm_query_generation(few_shot_prompt, msg_dict['question'], msg_dict['parameters'])
        self.query_execution_meal_prep(cypher)


    def llm_query_generation(self, few_shot_prompt, question, parameters):
        print(few_shot_prompt)
        print("\033[34m" + f'{question=},\n {parameters=}' + "\033[0m")

        input_data = {"question": question, "parameters": parameters}
        formatted_prompt = few_shot_prompt.format(question=input_data["question"], parameters=input_data["parameters"], schema=self.schema)
        llm_response = self.llm_response.invoke(formatted_prompt)

        cypher = self.llm_response.invoke(formatted_prompt)
        print("\033[1;32mCypher query\033[0m")
        print()
        print("\033[32m"+cypher+"\033[0m")
        print()
        return cypher
    

    def query_execution_meal_prep(self, cypher):
        queries = [':'.join(s.split(':')[1:]).strip() for s in cypher.split('\n')]
        queries = [q for q in queries if len(q) > 0]
        print(f'Found {len(queries)} queries')

        driver = GraphDatabase.driver(self.uri, auth=(self.username, self.password))

        user_results = []
        recipes = []

        try:
            for i, q in enumerate(queries):
                print("\033[34m" + f'Executing Query {i+1}: {q}\n' + "\033[0m")
                with driver.session() as session:
                    # Execute the query
                    result = session.run(q)
                    print("\033[34mQuery results:\033[0m")
                    # print(result)
                    for record in result:
                        if i == 0:
                            user_results.append(record)
                        if i == 1:
                            recipes.append(record)
                    if i == 0:
                        print("\033[32m" + str(user_results) + "\033[0m")
                    if i == 1:
                        print("\033[32m" + str(recipes) + "\033[0m")
                print('\n\n')
        except Exception as e:
            print("Error:", e.message)
        finally:
            driver.close()



def main(args=None):
    rclpy.init(args=args)
    query_generation = Query_Generation()
    rclpy.spin(query_generation)
    query_generation.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
