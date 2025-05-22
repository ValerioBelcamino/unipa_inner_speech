from query_generation.query_generation_llm import QueryGeneration_LLM
# from .export_query_results import generate_pl_file, generate_csv_file
from common_msgs.msg import Intent, QueryOutput
from std_msgs.msg import Bool
from rclpy.node import Node
import rclpy
import json


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

        self.QueryGen_LLM = QueryGeneration_LLM(node_name = self.node_name)

        
    def query_generation_callback(self, intent_msg):
        self.get_logger().info('Received: "%s" __ query_generation_callback\n')

        user_input = intent_msg.user_input
        action_name = intent_msg.action_name
        parameters = json.loads(intent_msg.parameters)

        print(f"\033[34m{user_input=}\n {action_name=}\n{parameters=}\033[0m")

        llm_response = self.QueryGen_LLM.get_LLM_response(user_input, action_name, parameters)

        queries = getattr(llm_response, 'query')

        if type(queries) == str:
            queries = [queries]

        # If the key is not in the dict let's go with the default db
        self.db = self.QueryGen_LLM.db_dict.get(action_name, self.QueryGen_LLM.db_dict['default'])

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
    
    
    def query_execution(self, query_list):
        query_results = []
        for query in query_list:
            query_results.append(self.db.execute_query(query))
        return query_results
    

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
