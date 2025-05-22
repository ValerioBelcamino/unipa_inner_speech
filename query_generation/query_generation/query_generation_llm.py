from shared_utils.customization_helpers import load_all_scenario_dbs, load_all_query_examples, load_all_query_models
from shared_utils.fewshot_helpers import prepare_few_shot_prompt
from shared_utils.llm_helpers import LLM_Initializer
from groq import BadRequestError
import time
import os


class QueryGeneration_LLM(LLM_Initializer):

    def __init__(self, node_name:str):
        super().__init__(node_name)

        # Load all the pydantic query generation tools
        self.dynamic_query_tools_dict = load_all_query_models(self.scenario)
        print(f"\033[1;38;5;207mLoaded {len(self.dynamic_query_tools_dict.values())} intent_tool(s).\033[0m")

        # Load a dictionary of DBs
        self.default_db_type = os.getenv("DB_TYPE")
        self.db_dict, self.schemas_dict, self.instructions_dict = load_all_scenario_dbs(self.scenario, self.default_db_type)
        print(f"\033[34mDB Dict: {self.db_dict}!\033[0m")

        # Load examples
        self.examples = load_all_query_examples(self.scenario)
        print(f"\033[1;38;5;207mLoaded {len(self.examples.keys())} example file(s).\033[0m")
        print()

        # Prepare template prompts for fewshot
        self.example_template = "User asks: {question}\nParameters: {parameters}\nQueries: {queries}"
        self.suffix = "User asks: {question}\nParameters: {parameters}\nQuery: "


    # Redefine abstractmethod from the parent class with more parameters (must be Noneable by default)
    def get_LLM_response(self, user_input, action_name=None, parameters=None, return_time=False):
        """
        Function to get the LLM response for a given user input.
        """

        # If the key is not in the dict let's go with the default db instructions
        instructions = self.instructions_dict.get(action_name, self.instructions_dict['default'])

        few_shot_prompt = prepare_few_shot_prompt(
                                                    instructions=instructions,
                                                    suffix=self.suffix, 
                                                    examples=self.examples[action_name],
                                                    example_variables=["question", "parameters", "queries"],
                                                    example_template=self.example_template,
                                                    input_variables=["question", "parameters"],
                                                    )
        llm_with_query = self._llm.with_structured_output(self.dynamic_query_tools_dict[action_name])
        llm_cypher_chain = few_shot_prompt | llm_with_query

        try: 
            print(user_input)
            print(parameters)
            initial_time = time.time()
            llm_response = llm_cypher_chain.invoke({"question": user_input, "parameters": parameters})
            llm_response_time = initial_time - time.time()

        except BadRequestError as e:
            print(f"\033[31mError: {e}\033[0m")
            llm_response = e
            llm_response_time = -1

        if return_time:
            return llm_response, llm_response_time
        else:
            return llm_response