from shared_utils.customization_helpers import list_required_parameters_by_tool, load_all_intent_models
from langchain_core.messages import SystemMessage, HumanMessage
from shared_utils.llm_helpers import LLM_Initializer
from pydantic import BaseModel, Field
from groq import BadRequestError
import textwrap
import time
import json


class InnerSeechOutputFormat(BaseModel):
    """ Dato un prompt di un utente, l'azione ed i parametri estratti dal riconoscimento dell'intento devi elaborare un discorso interiore che spieghi se l'azione può essere portata a termine oppure no."""

    inner_speech: str = Field(description="Il tuo ragionamento")
    can_proceed: bool = Field(description="Se la richiesta dell'utente può essere accolta")



class InnerSpeech_LLM(LLM_Initializer):

    def __init__(self, node_name:str):
        super().__init__(node_name)

        # Bind pydantic class for strict output format
        self._llm_with_output = self._llm.with_structured_output(InnerSeechOutputFormat)

        # Load all pydantic intent tools
        self.dynamic_intent_tools_dict = load_all_intent_models(self.scenario)
        print(f"\033[34mLoaded {self.dynamic_intent_tools_dict} intent_tool(s).\033[0m")
        self._dynamic_intent_toolnames = [dit.__name__ for dit in self.dynamic_intent_tools_dict.values()]
        print(f"\033[1;38;5;207mLoaded {len(self._dynamic_intent_toolnames)} intent_tool(s).\033[0m")

        # Use the list of pydantic intent_tools to list the required parameters
        self.action_name_to_required_parameters = list_required_parameters_by_tool(self.dynamic_intent_tools_dict)
        self.action_name_to_required_parameters['OutOfScope'] = []

        # Create a dictionary of descriptions for each pydantic intent_tool
        self.action_name_to_description = {k:v.__doc__ for k,v in self.dynamic_intent_tools_dict.items()}
        self.action_name_to_description['OutOfScope'] = 'L\'azione non è rilevante per il sistema, quindi il sistema non è in grado di fornire una risposta all\'utente.'


    def _get_full_context_tokens(self, prompt):
        """
        Get accurate token count including both the prompt messages and tool definitions.
        When tools are bound to the LLM, they are serialized and sent along with the messages.
        """
        # Get tokens from the prompt messages
        prompt_str = str(prompt)
        message_tokens = self._llm.get_num_tokens(prompt_str)
        
        # Get tokens from tool definitions (InnerSeechOutputFormat schema)
        # The structured output is converted to JSON schema format when sent to the API
        tool_schema = {
            "functions": [{
                "name": InnerSeechOutputFormat.__name__,
                "description": InnerSeechOutputFormat.__doc__ or "",
                "parameters": InnerSeechOutputFormat.model_json_schema()
            }]
        }
        
        tools_str = json.dumps(tool_schema)
        tool_tokens = self._llm.get_num_tokens(tools_str)
        
        return message_tokens + tool_tokens


    # Redefine abstractmethod from the parent class with more parameters (must be Noneable by default)
    def get_LLM_response(self, user_input, action_name=None, parameters=None, missing_parameters=None, memory= None, return_time=False, return_tokens=False):
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
                    Hai a disposizione anche una memoria a breve termine con interazioni passate.
                    Devi impedire l'esecuzione di domande non pertinenti al tuo scopo
                    Devi impedire l'esecuzione di domande con parametri obbligatori mancanti. 
                    Devi filtrare domande relative al tuo argomento ma troppo vaghe."""
                )),
            HumanMessage(content=
                textwrap.dedent(
                    f"""
                    Memoria: {memory}
                    La domanda dell'utente è: {user_input}.
                    Il riconoscimento dell'intento ha assegnato la seguente funzione: {action_name}.
                    Action description:{self.action_name_to_description[action_name]}
                    Con i seguenti parametri: {parameters}.
                    Parametri obbligatori mancanti: {missing_parameters}"""
                ))
        ]
        
        # Initialize token counts (used when BadRequestError occurs)
        prompt_tokens = 0
        completion_tokens = 0
        total_tokens = 0
        
        try:
            initial_time = time.time()
            llm_response = self._llm_with_output.invoke(prompt)
            llm_response_time = time.time() - initial_time
            print(f'\033[91m{llm_response}\033[0m')
            
            # Extract token usage from response metadata
            if hasattr(llm_response, 'response_metadata') and 'token_usage' in llm_response.response_metadata:
                prompt_tokens = llm_response.response_metadata['token_usage'].get('prompt_tokens', 0)
                completion_tokens = llm_response.response_metadata['token_usage'].get('completion_tokens', 0)
                total_tokens = llm_response.response_metadata['token_usage'].get('total_tokens', 0)
            
            result = {}
            result['question'] = user_input
            result['inner_speech'] = llm_response.inner_speech
            result['can_proceed'] = llm_response.can_proceed

        except BadRequestError as e:
            # Use shared error handling to parse failed_generation and fix parameter types
            _, fixed_args = self._handle_bad_request_error(e, InnerSeechOutputFormat)
            
            if fixed_args:
                # Successfully parsed and fixed the parameters
                llm_response_time = time.time() - initial_time
                result = {}
                result['question'] = user_input
                result['inner_speech'] = fixed_args.get('inner_speech', '')
                result['can_proceed'] = fixed_args.get('can_proceed', False)
                # Estimate tokens including tool definitions
                prompt_tokens = self._get_full_context_tokens(prompt)
                completion_tokens = self._llm.get_num_tokens(result['inner_speech'])
                total_tokens = prompt_tokens + completion_tokens
            else:
                # Could not parse or fix the error
                result = {}
                result['error'] = e
                llm_response_time = -1

        if return_tokens:
            return result, llm_response_time, prompt_tokens, completion_tokens, total_tokens
        elif return_time:
            return result, llm_response_time
        else:
            return result


    def change_scenario(self, new_scenario):
        '''Updates the llm class to handle a different scenario'''

        # Update the scenario and its description
        self.update_scenario(new_scenario)

        # Reload intent tools and required parameters
        
        # Load all pydantic intent tools
        self.dynamic_intent_tools_dict = load_all_intent_models(self.scenario)
        print(f"\033[34mLoaded {self.dynamic_intent_tools_dict} intent_tool(s).\033[0m")
        self._dynamic_intent_toolnames = [dit.__name__ for dit in self.dynamic_intent_tools_dict.values()]
        print(f"\033[1;38;5;207mLoaded {len(self._dynamic_intent_toolnames)} intent_tool(s).\033[0m")

        # Use the list of pydantic intent_tools to list the required parameters
        self.action_name_to_required_parameters = list_required_parameters_by_tool(self.dynamic_intent_tools_dict)
        self.action_name_to_required_parameters['OutOfScope'] = []

        # Create a dictionary of descriptions for each pydantic intent_tool
        self.action_name_to_description = {k:v.__doc__ for k,v in self.dynamic_intent_tools_dict.items()}
        self.action_name_to_description['OutOfScope'] = 'L\'azione non è rilevante per il sistema, quindi il sistema non è in grado di fornire una risposta all\'utente.'