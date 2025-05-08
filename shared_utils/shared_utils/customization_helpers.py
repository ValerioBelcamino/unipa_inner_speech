import importlib
import pkgutil
import inspect
from pathlib import Path
from pydantic import BaseModel
from typing import Type, Dict, get_origin, get_args, Union
from shared_utils.fewshot_helpers import queries_to_query_list, escape_curly_braces
import json
import os 



def load_all_intent_models(scenario:str) -> Dict[str, Type[BaseModel]]:
    '''returns all intent tools for a given scenario'''
    return dynamic_iterative_load(scenario, 'intent')

def load_all_query_models(scenario) -> Dict[str, Type[BaseModel]]:
    '''returns all query generation tools for a given scenario'''
    return dynamic_iterative_load(scenario, 'query')

def dynamic_iterative_load(scenario: str, filename: str) -> Dict[str, Type[BaseModel]]:
    tool_dict = {}
    package = f'scenario_customization.{scenario}'

    try:
        scenario_module = __import__(package, fromlist=[""])
    except ModuleNotFoundError:
        print(f"Scenario module '{package}' not found.")
        return tool_dict

    for finder, task_name, ispkg in pkgutil.iter_modules(scenario_module.__path__):
        try:
            intent_module_path = f"{package}.{task_name}.{filename}"
            intent_module = importlib.import_module(intent_module_path)

            for name, obj in inspect.getmembers(intent_module, inspect.isclass):
                if issubclass(obj, BaseModel) and obj is not BaseModel:
                    print(f"\033[95mLoaded {obj.__name__} from {task_name}\033[0m")
                    tool_dict[task_name] = obj
        except ModuleNotFoundError:
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

def load_all_query_examples(scenario:str) -> Dict[str, Type[BaseModel]]:
    '''returns all query examples for a given scenario'''
    return dynamic_iterative_load_examples(scenario, 'query')

def load_all_explainability_examples(scenario:str) -> Dict[str, Type[BaseModel]]:
    '''returns all explainability examples for a given scenario'''
    return dynamic_iterative_load_examples(scenario, 'explainability')

def dynamic_iterative_load_examples(scenario: str, modality: str) -> Dict[str, list]:
    examples = {}

    if modality not in ['query', 'explainability']:
        raise ValueError("Modality must be either 'query' or 'explainability'")

    # Locate the installation path of the scenario_customization package
    spec = importlib.util.find_spec("scenario_customization")
    if spec is None or not spec.submodule_search_locations:
        raise ImportError("Cannot locate scenario_customization package.")
    package_dir = spec.submodule_search_locations[0]

    # print(package_dir)
    # go from code installation path in lib to file installation folder in share

    # from this 
    # /home/belca/Desktop/ros2_humble_ws/src/install/scenario_customization/lib/python3.10/site-packages/scenario_customization
    # to this 
    # /home/belca/Desktop/ros2_humble_ws/src/install/scenario_customization/share/scenario_customization/scenarios/{SCENARIO}
    base_path = Path(package_dir).parent.parent.parent.parent / 'share' / 'scenario_customization' / 'scenarios' / scenario

    if not os.path.isdir(base_path):
        raise FileNotFoundError(f"Scenario folder not found: {base_path}")

    for task_name in os.listdir(base_path):
        task_path = os.path.join(base_path, task_name)
        examples_path = os.path.join(task_path, f"{modality}_examples")

        task_examples = []

        if os.path.isdir(examples_path):
            for filename in os.listdir(examples_path):
                if filename.endswith(".json"):
                    file_path = os.path.join(examples_path, filename)
                    with open(file_path, "r", encoding="utf-8") as f:
                        content = json.load(f)
                        for j, ex in enumerate(content):
                            for k, v in ex.items():
                                ex[k] = escape_curly_braces(v)
                            if modality == 'query':
                                content[j] = queries_to_query_list(ex)
                        task_examples.extend(content)

        if task_examples:
            print(f"\033[96mLoaded {len(task_examples)} example(s) from {task_name} ({modality} examples)\033[0m")
            examples[task_name] = task_examples

    return examples
