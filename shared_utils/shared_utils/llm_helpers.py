from db_adapters import DBFactory  
from abc import ABC, abstractmethod
from langchain.chat_models import init_chat_model
from shared_utils.customization_helpers import load_all_intent_models, get_scenario_description
from typing import Any, get_origin, get_args, Union, List, Type
from pydantic import BaseModel
from dotenv import load_dotenv
from groq import BadRequestError
import json
import ast
import os
import re


def get_default_value(t: Any):
    """Get default value for a given type."""
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


class LLM_Initializer(ABC):
    def __init__(
                self, 
                node_name:str,
                use_db_adapter:bool=True,
                use_scenario_description:bool=True,
                ):
        
        self.node_name = node_name

        # Load environment variables from .env and .config files
        self.CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
        self.BASE_DIR = os.path.abspath(os.path.join(self.CURRENT_DIR, "../../../../../../unipa_inner_speech"))

        self.dotenv_path = os.path.join(self.BASE_DIR, ".env")
        load_dotenv(self.dotenv_path, override=True)
        self.dotconfig_path = os.path.join(self.BASE_DIR, ".config")
        load_dotenv(self.dotconfig_path)

        if use_db_adapter:
            # Create default DB adapter
            self.db_type = os.getenv("DB_TYPE") 
            self._db = DBFactory.create_adapter(self.db_type)

        self.llm_config = ast.literal_eval(os.getenv("LLM_CONFIG"))[self.node_name]
        self._llm = init_chat_model(
                                    model=self.llm_config['model_name'], 
                                    model_provider=self.llm_config['model_provider'], 
                                    temperature=self.llm_config['temperature'], 
                                    api_key=os.getenv("GROQ_API_KEY")
                                )

        if use_scenario_description:
            print()
            self.scenario = os.getenv("SCENARIO")
            print(f"\033[34mUsing {self.scenario}!\033[0m")
            self.context_scenario = get_scenario_description(self.scenario)
            print(f"\033[34mDesciription: {self.context_scenario}\033[0m")


    @abstractmethod
    def get_LLM_response(self, user_input):
        try:
            self._llm.invoke(user_input)
        except BadRequestError as e:
            print(f"\033[31mError: {e}\033[0m")


    def update_scenario(self, new_scenario):
        # Update the environmental variable
        os.environ['SCENARIO'] = new_scenario
        # Update the scenario and its description
        self.scenario = os.getenv("SCENARIO")
        print(f"\033[34mChanging scenario to: {self.scenario}!\033[0m")
        self.context_scenario = get_scenario_description(self.scenario)
        print(f"\033[34mDesciription: {self.context_scenario}\033[0m")


    def _fix_parameter_types(self, tool_class: Type[BaseModel], tool_args: dict) -> dict:
        """
        Fix parameter types based on the tool schema.
        Converts empty strings to proper default values based on expected types.
        Also converts string representations to proper types (e.g., "2500" -> 2500, "false" -> False).
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


    def _handle_bad_request_error(self, error: BadRequestError, tool_class: Type[BaseModel]) -> tuple:
        """
        Handle BadRequestError from Groq API by parsing the failed_generation and fixing parameter types.
        
        Args:
            error: The BadRequestError exception from Groq
            tool_class: The Pydantic model class defining the expected schema
            
        Returns:
            tuple: (tool_name, fixed_args) extracted from the failed_generation, or (None, {}) if parsing fails
        """
        print(f"\033[31mBadRequestError: {error}\033[0m")
        
        try:
            error_str = str(error)
            # Look for the failed_generation JSON in the error (greedy to capture full JSON array)
            failed_gen_match = re.search(r"'failed_generation':\s*'(\[.*\])'", error_str, re.DOTALL)
            if failed_gen_match:
                failed_gen_str = failed_gen_match.group(1)
                # Unescape the JSON string step by step
                # First handle \' (escaped single quotes in Python repr) -> '
                failed_gen_str = failed_gen_str.replace("\\'", "'")
                # Handle escaped newlines
                failed_gen_str = failed_gen_str.replace('\\n', '\n')
                # Handle double-escaped unicode \\u -> \u
                failed_gen_str = failed_gen_str.replace('\\\\u', '\\u')
                # Handle double-escaped quotes (\\\" -> ")
                failed_gen_str = failed_gen_str.replace('\\"', '"')
                # Handle remaining escaped backslashes
                failed_gen_str = failed_gen_str.replace('\\\\', '\\')
                
                print(f"\033[33mParsing failed_generation: {failed_gen_str}\033[0m")
                failed_gen = json.loads(failed_gen_str)
                
                if failed_gen and isinstance(failed_gen, list) and len(failed_gen) > 0:
                    tool_call = failed_gen[0]
                    tool_name = tool_call.get('name', None)
                    tool_args = tool_call.get('parameters', {})
                    
                    # Fix invalid parameter types based on the tool schema
                    fixed_args = self._fix_parameter_types(tool_class, tool_args)
                    print(f"\033[32mFixed parameters: {fixed_args}\033[0m")
                    return (tool_name, fixed_args)
        except Exception as parse_error:
            print(f"\033[31mFailed to parse error response: {parse_error}\033[0m")
        
        return (None, {})
