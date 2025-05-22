from explainability.explainability_llm import QueryExplanation_LLM, InnerSpeechExplanation_LLM
from common_msgs.msg import QueryOutput, InnerSpeech
from std_msgs.msg import String
from rclpy.node import Node
import rclpy
import json


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
        
        # self.listener_clingo_explanation = self.create_subscription(
        #     String,
        #     self.clingo_explanation_topic,
        #     self.clingo_explanation_callback,
        #     10)
        
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

        self.QueryEXP_LLM = QueryExplanation_LLM(node_name=self.node_name)
        self.ISEXP_LLM = InnerSpeechExplanation_LLM(node_name=self.node_name)

        # # CLINGO EXPLAINABILITY LLM VARIABLES
        # self.clingo_instructions = "Tu sei un Robot di nome Pepper e devi supportare un utente nel seguire un corretto piano alimentare basato sui suoi bisogni e preferenze. A questo punto del processo abbiamo escluso già i piatti non adatti allo stile alimentare dell'utente e, in questo step, abbiamo generato diverse combinazioni di piatti in grado di soddisfare i vincoli di calorie e macronutrienti rimanenti. Data una una lista di combinazioni di piatti, il tuo compito è spiegare all'utente come sono stati scelti. Il numero di piatti in ogni risposta può essere 1, N, o 0 dipendentemente dai requisiti."
        # self.clingo_suffix = "Rispondini in linguaggio naturale in lingua Italiana in modo sintetico."
        # self.clingo_example_template = """User Input: {results}\nExplanation: {explanation}"""

        # with open(os.path.join(self.source_dir, 'fewshot_examples/FewShot_clingo_explanation.json'), 'r') as f:
        #     self.examples['clingo'] = json.load(f)

        
        
        # self.llm_response = (
        #     self.llm.bind()
        #     | StrOutputParser()
        # )


    def query_explanation_callback(self, msg):
        action_name = msg.action_name
        self.get_logger().info('Received: "%s" query_explanation_callback\n' % action_name)

        print(f"\033[34m{msg.user_input=}, {msg.queries=}, {msg.results=}\033[0m")
        
        explanation = self.QueryEXP_LLM.get_LLM_response(msg.user_input, msg.action_name, msg.queries, msg.results)

        response_dict = {'question':msg.user_input, 'response':explanation}
        response_string = json.dumps(response_dict)
        self.publisher_user_input.publish(String(data=response_string))
        self.get_logger().info('Published: "%s"' % response_string)

        # Output the explanation
        print(f"\033[1;34mExplanation:\033[0m")
        print(f"\033[1;32m{explanation}\033[0m")


    # def clingo_explanation_callback(self, msg):
    #     self.get_logger().info('Received: "%s" __ clingo_explanation_callback\n' % msg.data)
    #     msg_dict = json.loads(msg.data)

    #     few_shot_prompt = prepare_few_shot_prompt(
    #                                                 instructions=self.clingo_instructions,
    #                                                 suffix=self.clingo_suffix, 
    #                                                 examples=self.examples['clingo'],
    #                                                 example_variables=["results", "explanation"],
    #                                                 example_template=self.clingo_example_template,
    #                                                 input_variables=["results"],
    #                                                 )

    #     formatted_prompt = few_shot_prompt.format(results = msg_dict['results'])
        
    #     explanation = self.llm_response.invoke(formatted_prompt)
    #     response_dict = {'question':msg.user_input, 'response':explanation}
    #     response_string = json.dumps(response_dict)
    #     self.publisher_user_input.publish(String(data=response_string))
    #     self.get_logger().info('Published: "%s"' % response_string)

    #     # Output the explanation
    #     print(f"\033[1;34mExplanation:\033[0m")
    #     print(f"\033[1;32m{explanation}\033[0m")

    def inner_speech_explanation_callback(self, msg):
        self.get_logger().info('Received: "%s" __ inner_speech_explanation_callback\n' % msg)
        user_input = msg.user_input
        parameters = msg.parameters
        action_name = msg.action_name
        action_description = msg.action_description
        missing_parameters = msg.missing_parameters
        inner_speech = msg.inner_speech

        response = self.ISEXP_LLM.get_LLM_response(
                                                    user_input, 
                                                    action_name, 
                                                    action_description, 
                                                    parameters, 
                                                    missing_parameters,
                                                    inner_speech
                                                    )

        response_dict = {'question':user_input, 'response':response}
        response_string = json.dumps(response_dict)
        self.publisher_user_input.publish(String(data=response_string))
        self.get_logger().info('Published: "%s"' % response_string)

        # Output the explanation
        print(f"\033[1;34mExplanation:\033[0m")
        print(f"\033[1;32m{response}\033[0m")

def main(args=None):
    rclpy.init(args=args)
    explainability = Explainability()
    rclpy.spin(explainability)
    explainability.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
