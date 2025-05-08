import os
import json
from langchain.chat_models import init_chat_model
from langchain_core.output_parsers.string import StrOutputParser
from db_adapters import DBFactory
from shared_utils.fewshot_helpers import queries_to_query_list, escape_curly_braces, prepare_few_shot_prompt
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from common_msgs.msg import QueryOutput, InnerSpeech
from dotenv import load_dotenv
import ast
from shared_utils.customization_helpers import load_all_explainability_examples


# Load environment variables from .env file
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
BASE_DIR = os.path.abspath(os.path.join(CURRENT_DIR, "../../../../../../unipa_inner_speech"))
dotenv_path = os.path.join(BASE_DIR, ".env")
load_dotenv(dotenv_path)
dotconfig_path = os.path.join(BASE_DIR, ".config")
load_dotenv(dotconfig_path)


class Explainability(Node):
    def __init__(self):
        self.node_name = 'explainability'
        super().__init__(f'{self.node_name}_node')
        self.inner_speech_explanation_topic = '/ex_inner_speech'
        self.user_input_topic = '/user_input_activation'

        self.query_explanation_topic = '/ex_queries'
        self.clingo_explanation_topic = '/ex_clingo'

        self.robot_dialogue_topic = '/speak'

        self.listener_query_explanation = self.create_subscription(
            QueryOutput,
            self.query_explanation_topic,
            self.query_explanation_callback,
            10)
        
        self.listener_clingo_explanation = self.create_subscription(
            String,
            self.clingo_explanation_topic,
            self.clingo_explanation_callback,
            10)
        
        self.listener_inner_speech = self.create_subscription(
            InnerSpeech,
            self.inner_speech_explanation_topic,
            self.inner_speech_explanation_callback,
            10)

        
        self.robot_dialogue_publisher = self.create_publisher(String, self.robot_dialogue_topic, 10)
        self.publisher_user_input = self.create_publisher(String, self.user_input_topic, 10)

        print(f"\033[34mExplainability Node started!!!\033[0m")
        print(f"\033[34mInitialized publishers to {self.robot_dialogue_topic}!!!\033[0m")
        print(f"\033[34mStarted Listening to {self.query_explanation_topic}!!!\033[0m")
        print(f"\033[34mStarted Listening to {self.clingo_explanation_topic}!!!\033[0m")
        print(f"\033[34mStarted Listening to {self.inner_speech_explanation_topic}!!!\033[0m")

        self.llm_config = ast.literal_eval(os.getenv("LLM_CONFIG"))[self.node_name]

        self.ws_dir = os.getenv("ROS2_WORKSPACE")
        self.source_dir = os.path.join(self.ws_dir, 'explainability', 'explainability')

        self.db_type = os.getenv("DB_TYPE")
        self.db = DBFactory.create_adapter(self.db_type)
        self.schema = self.db.get_schema()

        self.scenario = os.getenv("SCENARIO")


        # QUERY EXPLAINABILITY LLM VARIABLES
        if self.scenario == "MOVIES":
            scenario_description = "Un assistente che aiuta gli utenti a ottenere informazioni aggiornate sui film attualmente in programmazione a Genova."
        elif self.scenario == "ADVISOR":
            scenario_description = "Tu sei un Robot di nome Pepper e devi supportare i tuoi utenti nel seguire un corretto piano alimentare."

        self.query_instructions = f"{scenario_description} Data una richiesta e la sua traduzione in query con i relativi risultati, devi spiegare all'utente il processo decisionale ed il risulato."
        self.query_suffix = "Rispondini in linguaggio naturale in lingua Italiana in modo sintetico.\nUser Input: {user_input}\nQueries: {queries}\nQuery Results: {results}\nExplanation: "
        self.query_example_template = """User Input: {user_input}\nQueries: {queries}\nQuery Results: {results}\nExplanation: {explanation}"""

        # CLINGO EXPLAINABILITY LLM VARIABLES
        self.clingo_instructions = "Tu sei un Robot di nome Pepper e devi supportare un utente nel seguire un corretto piano alimentare basato sui suoi bisogni e preferenze. A questo punto del processo abbiamo escluso già i piatti non adatti allo stile alimentare dell'utente e, in questo step, abbiamo generato diverse combinazioni di piatti in grado di soddisfare i vincoli di calorie e macronutrienti rimanenti. Data una una lista di combinazioni di piatti, il tuo compito è spiegare all'utente come sono stati scelti. Il numero di piatti in ogni risposta può essere 1, N, o 0 dipendentemente dai requisiti."
        self.clingo_suffix = "Rispondini in linguaggio naturale in lingua Italiana in modo sintetico."
        self.clingo_example_template = """User Input: {results}\nExplanation: {explanation}"""

        print()
        self.scenario = os.getenv("SCENARIO")
        print(f"\033[34mUsing {self.scenario}!\033[0m")
        self.examples = load_all_explainability_examples(self.scenario)
        print(f"\033[1;38;5;207mLoaded {len(self.examples.keys())} example file(s).\033[0m")
        print()

        with open(os.path.join(self.source_dir, 'fewshot_examples/FewShot_clingo_explanation.json'), 'r') as f:
            self.examples['clingo'] = json.load(f)

        self.llm = init_chat_model(
                                    model=self.llm_config['model_name'], 
                                    model_provider=self.llm_config['model_provider'], 
                                    temperature=self.llm_config['temperature'], 
                                    api_key=os.getenv("GROQ_API_KEY")
                                )
        
        self.llm_response = (
            self.llm.bind()
            | StrOutputParser()
        )


    def query_explanation_callback(self, msg):
        action_name = msg.action_name
        self.get_logger().info('Received: "%s" query_explanation_callback\n' % action_name)

        # print(self.examples[action_name])

        few_shot_prompt = prepare_few_shot_prompt(
                                                    instructions=self.query_instructions,
                                                    suffix=self.query_suffix, 
                                                    examples=self.examples[action_name],
                                                    example_variables=["user_input", "queries", "results", "explanation"],
                                                    example_template=self.query_example_template,
                                                    input_variables=["user_input", "queries", "results"],
                                                    )

        # print(few_shot_prompt)
        print(f"\033[34m{msg.user_input=}, {msg.queries=}, {msg.results=}\033[0m")


        formatted_prompt = few_shot_prompt.format(
                                        user_input = msg.user_input, 
                                        queries = msg.queries, 
                                        results = msg.results
                                        )
        
        explanation = self.llm_response.invoke(formatted_prompt)
        response_dict = {'question':msg.user_input, 'response':explanation}
        response_string = json.dumps(response_dict)
        self.publisher_user_input.publish(String(data=response_string))
        self.get_logger().info('Published: "%s"' % response_string)

        # Output the explanation
        print(f"\033[1;34mExplanation:\033[0m")
        print(f"\033[1;32m{explanation}\033[0m")


    def clingo_explanation_callback(self, msg):
        self.get_logger().info('Received: "%s" __ clingo_explanation_callback\n' % msg.data)
        msg_dict = json.loads(msg.data)

        few_shot_prompt = prepare_few_shot_prompt(
                                                    instructions=self.clingo_instructions,
                                                    suffix=self.clingo_suffix, 
                                                    examples=self.examples['clingo'],
                                                    example_variables=["results", "explanation"],
                                                    example_template=self.clingo_example_template,
                                                    input_variables=["results"],
                                                    )

        formatted_prompt = few_shot_prompt.format(results = msg_dict['results'])
        
        explanation = self.llm_response.invoke(formatted_prompt)
        response_dict = {'question':msg.user_input, 'response':explanation}
        response_string = json.dumps(response_dict)
        self.publisher_user_input.publish(String(data=response_string))
        self.get_logger().info('Published: "%s"' % response_string)

        # Output the explanation
        print(f"\033[1;34mExplanation:\033[0m")
        print(f"\033[1;32m{explanation}\033[0m")

    def inner_speech_explanation_callback(self, msg):
        self.get_logger().info('Received: "%s" __ inner_speech_explanation_callback\n' % msg)
        user_input = msg.user_input
        parameters = msg.parameters
        action_name = msg.action_name
        action_description = msg.action_description
        missing_parameters = msg.missing_parameters

        answer_prompt = f"""
                Tu sei un Robot di nome Pepper e devi supportare i tuoi utenti.
                Chiedi all'utente maggiori dettagli per completare l'azione {action_name}: {action_description}.

                La sua domanda è: {user_input}.
                Il riconoscimento dell'intento ha estratto i seguenti parametri: {parameters}.
                L'azione non può essere completata perché mancano i seguenti parametri: {missing_parameters}.
                Chiedi all'utente di fornire i parametri mancanti con una domanda formulata in linguaggio naturale se è relevante alla sistema.
                Oppure, se non è rilevante, chiedi all'utente di riformulare la domanda in modo che il sistema possa fornire una risposta.

                Formula la tua risposta in italiano:"""
        
        answer = self.llm.invoke(answer_prompt).content
        response_dict = {'question':user_input, 'response':answer}
        response_string = json.dumps(response_dict)
        self.publisher_user_input.publish(String(data=response_string))
        self.get_logger().info('Published: "%s"' % response_string)

        # Output the explanation
        print(f"\033[1;34mExplanation:\033[0m")
        print(f"\033[1;32m{answer}\033[0m")

def main(args=None):
    rclpy.init(args=args)
    explainability = Explainability()
    rclpy.spin(explainability)
    explainability.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
