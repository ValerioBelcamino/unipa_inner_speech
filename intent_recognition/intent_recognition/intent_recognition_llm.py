from shared_utils.customization_helpers import load_all_intent_models
from langchain_core.messages import SystemMessage, HumanMessage
from intent_post_processing.loader import load_plugins
from shared_utils.llm_helpers import LLM_Initializer
from typing import Any, get_origin, get_args, Union
from groq import BadRequestError
import textwrap
import ast
import re



def get_default_value(t: Any):
    try:
        origin = get_origin(t)
            
        if origin is Union:
                # Filter out NoneType and keep the first non-None type
            non_none_args = [arg for arg in get_args(t) if arg is not type(None)]
            if non_none_args:
                t = non_none_args[0]
            
        origin = get_origin(t) or t
        value = origin()
            
        if callable(value):
            return value
        return value
    except Exception:
        return None

def check_undeclared_parameters(tool_class, tool_result):
    parameter_list = [(k, get_default_value(v.annotation)) for k,v in tool_class.model_fields.items()]
    for parameter, default_value in parameter_list:
        if parameter not in tool_result or tool_result[parameter] is None:
            print(f"\033[33mParameter {parameter} not found in tool result. Setting default value: {default_value}\033[0m")
            tool_result[parameter] = default_value
    return tool_result



class IntentRecognition_LLM(LLM_Initializer):

    def __init__(self, node_name:str):
        super().__init__(node_name)

        # Load all pydantic intent tools
        self.dynamic_intent_tools_dict = load_all_intent_models(self.scenario)
        print(f"\033[34mLoaded {self.dynamic_intent_tools_dict} intent_tool(s).\033[0m")
        self._dynamic_intent_toolnames = [dit.__name__ for dit in self.dynamic_intent_tools_dict.values()]
        print(f"\033[1;38;5;207mLoaded {len(self._dynamic_intent_toolnames)} intent_tool(s).\033[0m")

        # Bind the tools to the LLM call
        self._llm = self._llm.bind_tools(self.dynamic_intent_tools_dict.values())
        print(self._llm.get_input_schema())
        print(f"\033[1;38;5;207mBound the tools to the LLM.\033[0m")
        print()

        # Load plugins dynamically from the config file
        self._plugins = load_plugins(self.scenario)
        print(f"\033[1;38;5;208mLoaded {len(self._plugins)} processing plugin(s).\033[0m")


    def execute_plugin_pipeline(self, action_name, intent_parameters):
        """
        Function to execute the loaded plugins with the appropriate parameters.
        """

        context = {
            "db_adapter": self._db,  # Pass the adapter instead of the driver
            "action_name": action_name,
            "intent_parameters": intent_parameters  # Adding the dynamic parameters extracted after LLM computation
        }
            # Iterate over each loaded plugin (each wrapped function)
        for plugin_function in self._plugins:
            try:
                    # Call the plugin with the context
                context['intent_parameters'] = plugin_function(context)

            except Exception as e:
                print(f"Error executing plugin: {e}")


    # Redefine abstractmethod from the parent class
    def get_LLM_response(self, user_input):
        """
        Function to get the LLM response for a given user input.
        """
        prompt = [  
            SystemMessage(content=
                    textwrap.dedent(
                        """"You are tasked with identifying the correct intent from a set of available tools and extracting only the parameters explicitly provided by the user.
                        You must not use external knowledge, assumptions, or inference to guess or complete missing information.
                        If the user input is not relevant to any of the available tools, do not respond or assign an intent.
                        Only fill tool parameters when the necessary information is clearly and explicitly included in the user input.
                        Do not hallucinate.
                        Do not fill gaps, or rephrase missing data.
                        If a question seems to be correlated to the current topic, but it is too vague and doesn't directly refer to a tool, don't answer!
                        If a parameter is missing, ambiguous, or incomplete, leave it blank and do not attempt to infer or complete it.
                        Follow these constraints strictly to ensure reliability and factual accuracy in tool usage."""
                    )),
            HumanMessage(content=user_input)
        ]

        try:
            llm_response = self._llm.invoke(prompt)
            print(llm_response)
            tool_calls = llm_response.tool_calls

        except BadRequestError as e:
            print(f"\033[31mError: {e}\033[0m")
            llm_response = re.findall(r"<tool-use>(.*)</tool-use>", str(e))[0]
            llm_response = ast.literal_eval(llm_response)
            tool_calls = llm_response['tool_calls']

        tool_calls = [tool_call for tool_call in tool_calls if tool_call['name'] in self._dynamic_intent_toolnames]

        if tool_calls == []: # no tool called -> out of scope
            tool_result = {}
            tool_name = 'OutOfScope'
        else:
            tool_name = tool_calls[0]['name']
            tool_result = tool_calls[0]['args']

                # defaults missing parameters to '' or None
            tool_result = check_undeclared_parameters(self.dynamic_intent_tools_dict[tool_name], tool_result)

                # execute post processing plugin pipeline 
            self.execute_plugin_pipeline(tool_name, tool_result)

        return tool_name, tool_result

