import os
import json
from .export_query_results import generate_pl_file, generate_csv_file
from langchain_groq import ChatGroq
from langchain_core.prompts import FewShotPromptTemplate, PromptTemplate
from langchain_core.output_parsers.string import StrOutputParser
from langchain_neo4j import Neo4jGraph
from neo4j import GraphDatabase 
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from dotenv import load_dotenv

# Load environment variables from .env file
BASE_DIR = "/home/belca/Desktop/ros2_humble_ws/src"
dotenv_path = os.path.join(BASE_DIR, ".env")
load_dotenv(dotenv_path)


class Query_Generation(Node):
    def __init__(self):
        super().__init__('query_generation_node')
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

        print(f"\033[34mQuery Generation Node started!!!\033[0m")
        print(f"\033[34mInitialized publishers to {self.out_clingo_topic}!!!\033[0m")
        print(f"\033[34mInitialized publishers to {self.out_query_explanation}!!!\033[0m")
        print(f"\033[34mStarted Listening to {self.in_user_insertion_topic}!!!\033[0m")
        print(f"\033[34mStarted Listening to {self.in_dish_info_topic}!!!\033[0m")
        print(f"\033[34mStarted Listening to {self.in_meal_prep_topic}!!!\033[0m")

        self.uri = os.getenv("NEO4J_URI")
        self.username = os.getenv("NEO4J_USERNAME")
        self.password = os.getenv("NEO4J_PASSWORD")

        self.ws_dir = os.getenv("ROS2_WORKSPACE")
        self.source_dir = os.path.join(self.ws_dir, 'query_generation', 'query_generation')

        # print(f'{self.username}, {self.password}')

        self.graph = Neo4jGraph(self.uri, self.username, self.password)
        self.schema = self.graph.schema

        # print(self.schema)

        self.example_filenames = ['FewShot_query_insertion.json', 'FewShot_query_dish_info.json', 'FewShot_query_meal_prep.json']
        self.examples = {}
        for i, file in enumerate(self.example_filenames):
            with open(os.path.join(self.source_dir, 'fewshot_examples', file), 'r') as f:
                self.examples[i]=json.load(f)["examples"]

        self.llm = ChatGroq(model="llama3-70b-8192", temperature=0, api_key=os.getenv("GROQ_API_KEY")) #llama3-70b-8192

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
        # print(self.example_filenames[action_id-1])
        if action_id == 1 or action_id == 2:
            _prefix='''Tu sei un Robot di nome Pepper, esperto in Neo4j. Dato una domanda in input crea una query Cypher sintatticamente corrette da eseguire. Hai a disposizione lo schema con le informazioni del database neo4j: {schema}. Inoltre, sotto trovi un numero di esempi di domande con la relativa traduzione in codice Cypher. Rispondi solo con le query Cypher, non aggiungere nient'altro.'''

            _suffix='''Ritornami esclusivamente una singola Cypher query e non aggiungere altro testo.\nQuestion: {question},\nParameters: {parameters},\n'''

        else:
            _prefix='''Tu sei un Robot di nome Pepper, esperto in Neo4j. Dato una domanda in input crea tre query Cypher sintatticamente corrette da eseguire. Hai a disposizione lo schema con le informazioni del database neo4j: {schema}. Inoltre, sotto trovi un numero di esempi di domande con la relativa traduzione in codice Cypher. Rispondi solo con le query Cypher, non aggiungere nient'altro.'''

            _suffix='''Ritornami esclusivamente le tre query Cypher query e non aggiungere altro testo.\nQuestion: {question},\nParameters: {parameters},\n'''

        return FewShotPromptTemplate(
            examples=self.examples[action_id-1],
            example_prompt=_example_prompt,
            prefix=_prefix,
            suffix=_suffix,
            input_variables=["question", "schema", "parameters"],
        )
    
    def user_insertion_listener_callback(self, msg):
        self.get_logger().info('Received: "%s" __ user_insertion_listener_callback\n' % msg.data)
        msg_dict = json.loads(msg.data)

        example_prompt = PromptTemplate.from_template("Question: {question}\nParameters: {parameters}\n{query}")

        few_shot_prompt = self.prepare_few_shot_prompt(1, example_prompt)
        self.get_logger().info('Prepared Few Shot Prompt for Action ID: 1')

        cypher = self.llm_query_generation(few_shot_prompt, msg_dict['question'], msg_dict['parameters'])

        cypher, user_results = self.query_execution_user_insertion(cypher)

        user_node = user_results['u']        # Extract the user node
        fabbisogno_node = user_results['f']    # Extract the fabbisogno node


        # Dynamically extract all allergy nodes (keys starting with 'a')
        allergy_names = []
        for key in user_results.keys():
            # Check if key represents an allergen node.
            if key.startswith('a'):
                allergen_node = user_results[key]
                # It's good to check if the allergen node is not None
                if allergen_node is not None:
                    allergy_names.append(allergen_node.get('name'))

        # Create a dictionary with the desired keys. Note: Adjust the keys if your actual properties differ.
        user_results_dict = {
            'name': user_node.get('name'),
            'daily_calories': fabbisogno_node.get('calories'),
            'daily_proteins': fabbisogno_node.get('proteins'),
            'daily_carbs': fabbisogno_node.get('carbs'),
            'daily_fats': fabbisogno_node.get('fats'),
            # If you have allergy nodes, you'll need to extract them separately.
            # For example, if you also returned allergy nodes in a list under key 'allergies':
            'allergies': allergy_names  # Placeholder if no allergies were returned.
        }

        # Assuming msg_dict is already defined and contains a "question" key:
        user_string = f'''user_request: {msg_dict["question"]}.
        name: {user_results_dict['name']},
        calories: {user_results_dict['daily_calories']},
        proteins: {user_results_dict['daily_proteins']},
        carbs: {user_results_dict['daily_carbs']},
        fats: {user_results_dict['daily_fats']},
        allergies: {', '.join(user_results_dict['allergies'])}'''

        result = {}
        result['user_input'] = user_string
        result['queries'] = cypher

        result_string = json.dumps(result)
        self.get_logger().info('Published: "%s"' % result_string)
        self.publisher_explainability_queries.publish(String(data=result_string))


    def dish_info_listener_callback(self, msg):
        self.get_logger().info('Received: "%s" __ dish_info_listener_callback\n' % msg.data)
        msg_dict = json.loads(msg.data)

        example_prompt = PromptTemplate.from_template("Question: {question}\nParameters: {parameters}\n{query}")
        print(json.loads(msg_dict['parameters']))
        print(type(json.loads(msg_dict['parameters'])))

        few_shot_prompt = self.prepare_few_shot_prompt(2, example_prompt)
        self.get_logger().info('Prepared Few Shot Prompt for Action ID: 2')

        cypher = self.llm_query_generation(few_shot_prompt, msg_dict['question'], msg_dict['parameters'])

        cypher, query_results = self.query_execution_dish_info(cypher)

        # Initialize an empty dictionary to hold extracted data
        result = {}

        # Extract recipe information
        recipe_node = query_results[0]['r']  # The Recipe node
        recipe_allergens = query_results[0]['recipe_allergens']  # List of Allergen nodes
        user_allergies = query_results[0]['user_allergies']  # List of Allergen nodes

        # Extract recipe details
        recipe_name = recipe_node['name']
        recipe_calories = recipe_node['calories']
        recipe_proteins = recipe_node['proteins']
        recipe_carbs = recipe_node['carbs']
        recipe_fats = recipe_node['fats']
        recipe_type = recipe_node['type']

        # Extract allergens (if any)
        recipe_allergens_names = [allergen for allergen in recipe_allergens] if recipe_allergens else []
        user_allergies_names = [allergen for allergen in user_allergies] if user_allergies else []

        # Format user string
        user_string = f'''user_request: {msg_dict['question']}.
recipe_name: {recipe_name},
type: {recipe_type},
calories: {recipe_calories},
proteins: {recipe_proteins},
carbs: {recipe_carbs},
fats: {recipe_fats},
recipe_allergens: [{', '.join(recipe_allergens_names) if recipe_allergens_names else ''}],
user_allergies: [{', '.join(user_allergies_names) if user_allergies_names else ''}]'''

        # Populate the result dictionary
        result['user_input'] = user_string

        # For demonstration, include the raw Cypher query used (optional)
        result['queries'] = cypher

        result_string = json.dumps(result)
        self.get_logger().info('Published: "%s"' % result_string)
        self.publisher_explainability_queries.publish(String(data=result_string))



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


    
    def query_execution_user_insertion(self, cypher):
        driver = GraphDatabase.driver(self.uri, auth=(self.username, self.password))
        try:
            with driver.session() as session:
                # Execute the query
                result = session.run(cypher)
                records = list(result)  # Convert result into a list

                print(f"\033[34mQuery results:\033[0m")
                for record in records:
                    print(f'\033[32m{record}\033[0m')  # Print each record

                # Check if insertion was successful
                if records:
                    print("\033[32mUser was inserted successfully!\033[0m")
                else:
                    print("\033[31mNo user was inserted.\033[0m")

        except Exception as e:
            print("Error:", e.message)
        finally:
            driver.close()

        return cypher, record
    

    def query_execution_dish_info(self, cypher):
        driver = GraphDatabase.driver(self.uri, auth=(self.username, self.password))
        query_results = []

        try:
            with driver.session() as session:
                # Execute the query
                result = session.run(cypher)
                print("Query results:")
                for record in result:
                    query_results.append(record)
                print("\033[32m" + str(query_results) + "\033[0m")

        except Exception as e:
            print("Error:", e.message)
        finally:
            driver.close()

        return cypher, query_results



def main(args=None):
    rclpy.init(args=args)
    query_generation = Query_Generation()
    rclpy.spin(query_generation)
    query_generation.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
