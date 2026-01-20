from shared_utils.customization_helpers import load_all_intent_models
from langchain_core.messages import SystemMessage, HumanMessage
from intent_post_processing.loader import load_plugins
from shared_utils.llm_helpers import LLM_Initializer
from typing import Any, get_origin, get_args, Union, List
from groq import BadRequestError
import textwrap
import json
import time
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


    def _fix_parameter_types(self, tool_class, tool_args: dict) -> dict:
        """
        Fix parameter types based on the tool schema.
        Converts empty strings to proper default values based on expected types.
        Also converts string representations to proper types (e.g., "2500" -> 2500).
        Handles stringified JSON arrays (e.g., '["glutine"]' -> ["glutine"]).
        """
        fixed_args = tool_args.copy()
        
        for field_name, field_info in tool_class.model_fields.items():
            if field_name not in fixed_args:
                continue
                
            value = fixed_args[field_name]
            field_type = field_info.annotation
            
            # Get the origin type (e.g., list, Optional, etc.)
            origin = get_origin(field_type)
            
            # Handle Optional types
            if origin is Union:
                args = get_args(field_type)
                non_none_args = [arg for arg in args if arg is not type(None)]
                if non_none_args:
                    field_type = non_none_args[0]
                    origin = get_origin(field_type) or field_type
            
            # Fix empty strings based on expected type
            if value == "" or value is None:
                if origin is list or origin is List or field_type is list:
                    fixed_args[field_name] = []
                elif field_type is bool:
                    fixed_args[field_name] = False
                elif field_type is int:
                    fixed_args[field_name] = 0
                elif field_type is float:
                    fixed_args[field_name] = 0.0
                elif field_type is str:
                    fixed_args[field_name] = ""
                else:
                    fixed_args[field_name] = get_default_value(field_type)
            # Convert string numbers to int
            elif field_type is int:
                if isinstance(value, str):
                    try:
                        fixed_args[field_name] = int(value)
                    except ValueError:
                        fixed_args[field_name] = 0
                elif isinstance(value, float):
                    fixed_args[field_name] = int(value)
                elif not isinstance(value, int):
                    fixed_args[field_name] = 0
            # Convert string numbers to float
            elif field_type is float:
                if isinstance(value, str):
                    try:
                        fixed_args[field_name] = float(value)
                    except ValueError:
                        fixed_args[field_name] = 0.0
                elif isinstance(value, int):
                    fixed_args[field_name] = float(value)
                elif not isinstance(value, float):
                    fixed_args[field_name] = 0.0
            # Fix string "true"/"false" for booleans
            elif field_type is bool:
                if isinstance(value, str):
                    fixed_args[field_name] = value.lower() in ('true', '1', 'yes')
                elif not isinstance(value, bool):
                    fixed_args[field_name] = bool(value)
            # Fix string arrays (when a string is passed instead of a list)
            elif (origin is list or origin is List or field_type is list):
                if isinstance(value, str):
                    # First, try to parse as JSON (handles '["glutine"]' case)
                    if value.strip().startswith('['):
                        try:
                            parsed = json.loads(value)
                            if isinstance(parsed, list):
                                fixed_args[field_name] = parsed
                            else:
                                fixed_args[field_name] = [parsed] if parsed else []
                        except json.JSONDecodeError:
                            # If JSON parsing fails, treat as a single element
                            if value.strip():
                                fixed_args[field_name] = [value]
                            else:
                                fixed_args[field_name] = []
                    elif value.strip():
                        fixed_args[field_name] = [value]
                    else:
                        fixed_args[field_name] = []
                elif not isinstance(value, list):
                    fixed_args[field_name] = []
        
        return fixed_args


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
    def get_LLM_response(self, user_input, memory, return_time=False):
        """
        Function to get the LLM response for a given user input.
        """
        prompt = [  
            SystemMessage(content=
                    textwrap.dedent(
                        """You are tasked with identifying the correct intent from a set of available tools and extracting only the parameters explicitly provided by the user.
                        You must not use external knowledge, assumptions, or inference to guess or complete missing information.
                        You will also receive a short term memory with additional information on past interactions.
                        If the user input is not relevant to any of the available tools, do not respond or assign an intent.
                        Only fill tool parameters when the necessary information is clearly and explicitly included in the user input.
                        Do not hallucinate.
                        Do not fill gaps, or rephrase missing data.
                        If a question seems to be correlated to the current topic, but it is too vague and doesn't directly refer to a tool, don't answer!
                        If a parameter is missing, ambiguous, or incomplete, leave it blank and do not attempt to infer or complete it.
                        Follow these constraints strictly to ensure reliability and factual accuracy in tool usage."""
                    )),
            HumanMessage(content=textwrap.dedent(
                        f"""Memory: {memory}
                        User Input: {user_input}"""
                    ))
        ]

        try:
            start_time = time.perf_counter()
            llm_response = self._llm.invoke(prompt)
            llm_response_time = llm_response.response_metadata['token_usage']['total_time']
            print(llm_response)
            tool_calls = llm_response.tool_calls

        except BadRequestError as e:
            llm_response_time = time.perf_counter() - start_time  # Calculate elapsed time
            print(f"\033[31mBadRequestError: {e}\033[0m")
            tool_calls = []
            
            # Try to extract failed_generation from the error message
            try:
                error_str = str(e)
                # Look for the failed_generation JSON in the error (greedy to capture full JSON array)
                failed_gen_match = re.search(r"'failed_generation':\s*'(\[.*\])'", error_str, re.DOTALL)
                if failed_gen_match:
                    failed_gen_str = failed_gen_match.group(1)
                    # Unescape the JSON string step by step
                    # First handle escaped newlines
                    failed_gen_str = failed_gen_str.replace('\\n', '\n')
                    # Handle double-escaped quotes (\\\" -> ")
                    failed_gen_str = failed_gen_str.replace('\\"', '"')
                    # Handle remaining escaped backslashes
                    failed_gen_str = failed_gen_str.replace('\\\\', '\\')
                    
                    print(f"\033[33mParsing failed_generation: {failed_gen_str}\033[0m")
                    failed_gen = json.loads(failed_gen_str)
                    
                    if failed_gen and isinstance(failed_gen, list) and len(failed_gen) > 0:
                        tool_call = failed_gen[0]
                        tool_name = tool_call.get('name', '')
                        tool_args = tool_call.get('parameters', {})
                        tool_args = tool_call.get('parameters', {})
                        
                        # Fix invalid parameter types based on the tool schema
                        if tool_name in self.dynamic_intent_tools_dict:
                            tool_class = self.dynamic_intent_tools_dict[tool_name]
                            tool_args = self._fix_parameter_types(tool_class, tool_args)
                        
                        tool_calls = [{'name': tool_name, 'args': tool_args}]
            except Exception as parse_error:
                print(f"\033[31mFailed to parse error response: {parse_error}\033[0m")

        tool_calls = [tool_call for tool_call in tool_calls if tool_call['name'] in self._dynamic_intent_toolnames]

        if tool_calls == []: # no tool called -> out of scope
            tool_result = {}
            tool_name = 'OutOfScope'
        else:
            tool_name = tool_calls[0]['name']
            tool_result = tool_calls[0]['args']

                # Fix parameter types based on the tool schema
            tool_result = self._fix_parameter_types(self.dynamic_intent_tools_dict[tool_name], tool_result)

                # defaults missing parameters to '' or None
            tool_result = check_undeclared_parameters(self.dynamic_intent_tools_dict[tool_name], tool_result)

                # execute post processing plugin pipeline 
            self.execute_plugin_pipeline(tool_name, tool_result)

        if return_time:
            return tool_name, tool_result, llm_response_time
        else:
            return tool_name, tool_result


    def change_scenario(self, new_scenario):
        '''Updates the llm class to handle a different scenario'''

        # Update the scenario and its description
        self.update_scenario(new_scenario)

        # Reload intent tools, binds them again and update the plugin pipeline 
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