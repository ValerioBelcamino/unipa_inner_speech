from shared_utils.customization_helpers import load_all_explainability_examples
from shared_utils.fewshot_helpers import prepare_few_shot_prompt
from shared_utils.llm_helpers import LLM_Initializer
from pydantic import BaseModel, Field
from groq import BadRequestError
import textwrap
import time


class ExplanationOutputFormat(BaseModel):
    """Output format for explanation generation."""
    explanation: str = Field(description="The explanation text for the user")



class QueryExplanation_LLM(LLM_Initializer):

    def __init__(self, node_name:str):
        super().__init__(node_name)

        # Create a dictionary of descriptions for each pydantic intent_tool
        self.examples = load_all_explainability_examples(self.scenario)
        print(f"\033[1;38;5;207mLoaded {len(self.examples.keys())} example file(s).\033[0m")
        print()



    # Redefine abstractmethod from the parent class with more parameters (must be Noneable by default)
    def get_LLM_response(self, user_input, action_name=None, queries=None, results=None, return_time=False, return_tokens=False):
        """
        Function to get the LLM response for a given user input.
        """

        # LLM prompts and template
        query_instructions = f"{self.context_scenario}. Data una richiesta e la sua traduzione in query con i relativi risultati, devi spiegare all'utente il processo decisionale ed il risulato."
        query_suffix = "Rispondini in linguaggio naturale in lingua Italiana in modo sintetico.\nUser Input: {user_input}\nQueries: {queries}\nQuery Results: {results}\nExplanation: "
        query_example_template = "User Input: {user_input}\nQueries: {queries}\nQuery Results: {results}\nExplanation: {explanation}"

        few_shot_prompt = prepare_few_shot_prompt(
                                                    instructions=query_instructions,
                                                    suffix=query_suffix, 
                                                    examples=self.examples[action_name],
                                                    example_variables=["user_input", "queries", "results", "explanation"],
                                                    example_template=query_example_template,
                                                    input_variables=["user_input", "queries", "results"],
                                                    )

        formatted_prompt = few_shot_prompt.format(
                                        user_input = user_input, 
                                        queries = queries, 
                                        results = results
                                        )
        
        # Initialize token counts (used when BadRequestError occurs)
        prompt_tokens = 0
        completion_tokens = 0
        total_tokens = 0
        
        try:
            initial_time = time.time()
            llm_response = self._llm.invoke(formatted_prompt)
            llm_response_content = llm_response.content
            llm_response_time = llm_response.response_metadata['token_usage']['total_time']
            prompt_tokens = llm_response.response_metadata['token_usage'].get('prompt_tokens', 0)
            completion_tokens = llm_response.response_metadata['token_usage'].get('completion_tokens', 0)
            total_tokens = llm_response.response_metadata['token_usage'].get('total_tokens', 0)

        except BadRequestError as e:
            # Use shared error handling to parse failed_generation and fix parameter types
            _, fixed_args = self._handle_bad_request_error(e, ExplanationOutputFormat)
            
            if fixed_args:
                # Successfully parsed and fixed the parameters
                llm_response_time = time.time() - initial_time
                llm_response_content = fixed_args.get('explanation', 'Mi dispiace, si è verificato un errore.')
                prompt_tokens = self._llm.get_num_tokens(formatted_prompt)
                completion_tokens = self._llm.get_num_tokens(llm_response_content)
                total_tokens = prompt_tokens + completion_tokens
            else:
                # Could not parse or fix the error
                llm_response_content = "Mi dispiace, si è verificato un errore nel generare la risposta."
                llm_response_time = time.time() - initial_time
        
        if return_tokens:
            return llm_response_content, llm_response_time, prompt_tokens, completion_tokens, total_tokens
        elif return_time:
            return llm_response_content, llm_response_time
        else:
            return llm_response_content


    def change_scenario(self, new_scenario):
        '''Updates the llm class to handle a different scenario'''

        # Update the scenario and its description
        self.update_scenario(new_scenario)

        # Reload examples 
        self.examples = load_all_explainability_examples(self.scenario)
        print(f"\033[1;38;5;207mLoaded {len(self.examples.keys())} example file(s).\033[0m")
        print()




class InnerSpeechExplanation_LLM(LLM_Initializer):

    def __init__(self, node_name:str):
        super().__init__(node_name)


    # Redefine abstractmethod from the parent class with more parameters (must be Noneable by default)
    def get_LLM_response(self, user_input, action_name=None, action_description=None, parameters=None, missing_parameters=None, return_time=False, return_tokens=False):
        """
        Function to get the LLM response for a given user input.
        """

        prompt = textwrap.dedent(
            f"""Tu sei un Robot di nome Pepper e devi supportare i tuoi utenti.
            Chiedi all'utente maggiori dettagli per completare l'azione {action_name}: {action_description}.

            La sua domanda è: {user_input}.
            Il riconoscimento dell'intento ha estratto i seguenti parametri: {parameters}.
            L'azione non può essere completata perché mancano i seguenti parametri: {missing_parameters}.
            Chiedi all'utente di fornire i parametri mancanti con una domanda formulata in linguaggio naturale se è relevante alla sistema.
            Oppure, se non è rilevante, chiedi all'utente di riformulare la domanda in modo che il sistema possa fornire una risposta.

            Formula la tua risposta in italiano:""")
        
        # Initialize token counts (used when BadRequestError occurs)
        prompt_tokens = 0
        completion_tokens = 0
        total_tokens = 0
        
        try:
            initial_time = time.time()
            llm_response = self._llm.invoke(prompt)
            llm_response_content = llm_response.content
            llm_response_time = llm_response.response_metadata['token_usage']['total_time']
            prompt_tokens = llm_response.response_metadata['token_usage'].get('prompt_tokens', 0)
            completion_tokens = llm_response.response_metadata['token_usage'].get('completion_tokens', 0)
            total_tokens = llm_response.response_metadata['token_usage'].get('total_tokens', 0)

        except BadRequestError as e:
            # Use shared error handling to parse failed_generation and fix parameter types
            _, fixed_args = self._handle_bad_request_error(e, ExplanationOutputFormat)
            
            if fixed_args:
                # Successfully parsed and fixed the parameters
                llm_response_time = time.time() - initial_time
                llm_response_content = fixed_args.get('explanation', 'Mi dispiace, si è verificato un errore.')
            else:
                # Could not parse or fix the error
                llm_response_content = "Mi dispiace, si è verificato un errore nel generare la risposta."
                llm_response_time = time.time() - initial_time
        
        if return_time and return_tokens:
            return llm_response_content, llm_response_time, prompt_tokens, completion_tokens, total_tokens
        elif return_time:
            return llm_response_content, llm_response_time
        else:
            return llm_response_content

