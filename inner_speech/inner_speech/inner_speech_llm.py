from shared_utils.customization_helpers import list_required_parameters_by_tool
from langchain_core.messages import SystemMessage, HumanMessage
from shared_utils.llm_helpers import LLM_Initializer
from pydantic import BaseModel, Field
from groq import BadRequestError
import textwrap



class InnerSeechOutputFormat(BaseModel):
    """ Dato un prompt di un utente, l'azione ed i parametri estratti dal riconoscimento dell'intento devi elaborare un discorso interiore che spieghi se l'azione può essere portata a termine oppure no."""

    inner_speech: str = Field(description="Il tuo ragionamento")
    can_proceed: bool = Field(description="Se la richiesta dell'utente può essere accolta")



class InnerSpeech_LLM(LLM_Initializer):

    def __init__(self, node_name:str):
        super().__init__(node_name, use_intent_tools = True)

        # Bind pydantic class for strict output format
        self._llm = self._llm.with_structured_output(InnerSeechOutputFormat)

        # Use parent list of pydantic intent_tools to list the required parameters
        self.action_name_to_required_parameters = list_required_parameters_by_tool(self._dynamic_intent_tools_dict)
        self.action_name_to_required_parameters['OutOfScope'] = []

        # Create a dictionary of descriptions for each pydantic intent_tool
        self.action_name_to_description = {k:v.__doc__ for k,v in self._dynamic_intent_tools_dict.items()}
        self.action_name_to_description['OutOfScope'] = 'L\'azione non è rilevante per il sistema, quindi il sistema non è in grado di fornire una risposta all\'utente.'


    # Redefine abstractmethod from the parent class with more parameters (must be Noneable by default)
    def get_LLM_response(self, user_input, action_name=None, parameters=None, missing_parameters=None):
        """
        Function to get the LLM response for a given user input.
        """

        if action_name is None or parameters is None or missing_parameters is None:
            print("\033[31mOne or more parameters are missing!!!\033[0m")

        # Define a prompt for out LLM call
        prompt = [  
            SystemMessage(content=
                textwrap.dedent(
                    f"""{self.context_scenario}. 
                    Devi impedire l'esecuzione di domande non pertinenti al tuo scopo
                    Devi impedire l'esecuzione di domande con parametri obbligatori mancanti. 
                    Devi filtrare domande relative al tuo argomento ma troppo vaghe."""
                )),
            HumanMessage(content=
                textwrap.dedent(
                    f"""La domanda dell'utente è: {user_input}.
                    Il riconoscimento dell'intento ha assegnato la seguente funzione: {action_name}.
                    Con i seguenti parametri: {parameters}.
                    Parametri obbligatori mancanti: {missing_parameters}"""
                ))
        ]
        try:
            llm_response = self._llm.invoke(prompt)
            print(f'\033[91m{llm_response}\033[0m')
            
            result = {}
            result['question'] = user_input
            result['inner_speech'] = llm_response.inner_speech
            result['can_proceed'] = llm_response.can_proceed

        except BadRequestError as e:
            print(f"\033[31mError: {e}\033[0m")
            result = {}
            result['error'] = e
            return e

        return result

