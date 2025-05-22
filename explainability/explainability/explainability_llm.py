from shared_utils.customization_helpers import load_all_explainability_examples
from shared_utils.fewshot_helpers import prepare_few_shot_prompt
from shared_utils.llm_helpers import LLM_Initializer
from groq import BadRequestError
import textwrap



class QueryExplanation_LLM(LLM_Initializer):

    def __init__(self, node_name:str):
        super().__init__(node_name)

        # Create a dictionary of descriptions for each pydantic intent_tool
        self.examples = load_all_explainability_examples(self.scenario)
        print(f"\033[1;38;5;207mLoaded {len(self.examples.keys())} example file(s).\033[0m")
        print()

        # QUERY EXPLAINABILITY LLM VARIABLES
        self.query_instructions = f"{self.context_scenario}. Data una richiesta e la sua traduzione in query con i relativi risultati, devi spiegare all'utente il processo decisionale ed il risulato."
        self.query_suffix = "Rispondini in linguaggio naturale in lingua Italiana in modo sintetico.\nUser Input: {user_input}\nQueries: {queries}\nQuery Results: {results}\nExplanation: "
        self.query_example_template = "User Input: {user_input}\nQueries: {queries}\nQuery Results: {results}\nExplanation: {explanation}"


    # Redefine abstractmethod from the parent class with more parameters (must be Noneable by default)
    def get_LLM_response(self, user_input, action_name=None, queries=None, results=None):
        """
        Function to get the LLM response for a given user input.
        """
        few_shot_prompt = prepare_few_shot_prompt(
                                                    instructions=self.query_instructions,
                                                    suffix=self.query_suffix, 
                                                    examples=self.examples[action_name],
                                                    example_variables=["user_input", "queries", "results", "explanation"],
                                                    example_template=self.query_example_template,
                                                    input_variables=["user_input", "queries", "results"],
                                                    )
        
        formatted_prompt = few_shot_prompt.format(
                                        user_input = user_input, 
                                        queries = queries, 
                                        results = results
                                        )
        
        try:
            llm_response = self._llm.invoke(formatted_prompt).content

        except BadRequestError as e:
            print(f"\033[31mError: {e}\033[0m")
            return e
        
        return llm_response


class InnerSpeechExplanation_LLM(LLM_Initializer):

    def __init__(self, node_name:str):
        super().__init__(node_name)


    # Redefine abstractmethod from the parent class with more parameters (must be Noneable by default)
    def get_LLM_response(self, user_input, action_name=None, action_description=None, parameters=None, missing_parameters=None):
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
        
        try:
            llm_response = self._llm.invoke(prompt).content

        except BadRequestError as e:
            print(f"\033[31mError: {e}\033[0m")
            return e
        
        return llm_response

