import importlib
import pkgutil
import inspect
from pydantic import BaseModel
from typing import Type, Dict, get_origin, get_args, Union
from shared_utils.fewshot_helpers import queries_to_query_list, escape_curly_braces
import json
import os 



def load_all_intent_models() -> Dict[str, Type[BaseModel]]:
    return dynamic_iterative_load('intent')

def load_all_query_models() -> Dict[str, Type[BaseModel]]:
    return dynamic_iterative_load('query')

def dynamic_iterative_load(filename:str) -> Dict[str, Type[BaseModel]]:
    tool_dict = {}
    package = "scenario_customization"  # Top-level package

    # Discover all submodules (task folders)
    for finder, task_name, ispkg in pkgutil.iter_modules(__import__(package).__path__):
        # print(finder, task_name)
        try:
            # Try to import task_name.intent
            intent_module_path = f"{package}.{task_name}.{filename}"
            intent_module = importlib.import_module(intent_module_path)
            
            # Inspect classes inside intent.py
            for name, obj in inspect.getmembers(intent_module, inspect.isclass):
                if issubclass(obj, BaseModel) and obj is not BaseModel:
                    # models.append(obj)
                    print(f"\033[95mLoaded {obj.__name__} from {task_name}\033[0m")
                    tool_dict[task_name] = obj
        except ModuleNotFoundError:
            # No intent.py or broken module, ignore
            pass

    return tool_dict

def is_optional(annotation):
    return get_origin(annotation) is Union and type(None) in get_args(annotation)

def list_required_parameters_by_tool(tool_dict):
    required_parameters = {}
    for tool, tool_class in tool_dict.items():
        tool_required_parameters = []
        for field_name, field_info in tool_class.model_fields.items():
            if not is_optional(field_info.annotation):
                tool_required_parameters.append(field_name)
        required_parameters[tool] = tool_required_parameters
    return required_parameters    

def load_all_query_examples() -> Dict[str, Type[BaseModel]]:
    return dynamic_iterative_load_examples('query')

def load_all_explainability_examples() -> Dict[str, Type[BaseModel]]:
    return dynamic_iterative_load_examples('explainability')

def dynamic_iterative_load_examples(modality: str) -> Dict[str, list]:
    examples = {}
    package = "scenario_customization"
    base_path = os.path.join(os.path.expanduser('~'), 'unipa', 'src', 'install', 'scenario_customization', 'share', 'scenario_customization', 'examples')

    # Ensure modality is either 'query' or 'explainability'
    if modality not in ['query', 'explainability']:
        raise ValueError("Modality must be either 'query' or 'explainability'")

    for finder, task_name, ispkg in pkgutil.iter_modules(__import__(package).__path__):
        # print(f"Found task: {task_name}")
        task_path = os.path.join(base_path, task_name)
        examples_path = os.path.join(task_path, f"{modality}_examples")  # Dynamic modality path

        if os.path.isdir(examples_path):
            for filename in os.listdir(examples_path):
                if filename.endswith(".json"):
                    file_path = os.path.join(examples_path, filename)
                    with open(file_path, "r", encoding="utf-8") as f:
                        content = json.load(f)
                        for j, ex in enumerate(content):
                            for k,v in ex.items():
                                ex[k] = escape_curly_braces(v)
                            # TODO: uniform the examples for query gen e explainability and remove this if statement
                            if modality == 'query':
                                content[j] = queries_to_query_list(ex)
                        task_examples = content

            if task_examples:
                print(f"\033[96mLoaded {len(task_examples)} example(s) from {task_name} ({modality} examples)\033[0m")
                examples[task_name] = task_examples

    return examples

