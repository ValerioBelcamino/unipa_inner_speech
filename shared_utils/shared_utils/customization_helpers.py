import importlib
import pkgutil
import inspect
from pathlib import Path
from pydantic import BaseModel
from db_adapters import DBFactory
from typing import Type, Dict, get_origin, get_args, Union
from shared_utils.fewshot_helpers import queries_to_query_list, escape_curly_braces
import json
import os 

def create_scenario_tools():
    scenarios = get_all_scenario_descriptions()
    scenario_tools = {}
    
    for name, doc in scenarios.items():
        supported_tasks = load_all_intent_models(name)

        doc += '\nTask supportati:\n'
        for task_name, task_class in supported_tasks.items():
            doc += f'-{task_name}: {task_class.__doc__}'

        cls = type(name, (BaseModel,), {'__doc__': doc})
        scenario_tools[name] = cls
    return scenario_tools

def get_scenarios():
    spec = importlib.util.find_spec("scenario_customization")
    if spec is None or not spec.submodule_search_locations:
        raise ImportError("Cannot locate scenario_customization package.")
    package_dir = spec.submodule_search_locations[0]

    base_path = Path(package_dir).parent.parent.parent.parent / 'share' / 'scenario_customization' / 'scenarios'
    return os.listdir(base_path)    

def get_all_scenario_descriptions():
    scenarios = get_scenarios()
    scenario_to_description = {}
    for scen in scenarios:
        scenario_to_description[scen] = get_scenario_description(scen)
    return scenario_to_description 

def get_scenario_description(scenario:str):
    spec = importlib.util.find_spec("scenario_customization")
    if spec is None or not spec.submodule_search_locations:
        raise ImportError("Cannot locate scenario_customization package.")
    package_dir = spec.submodule_search_locations[0]

    base_path = Path(package_dir).parent.parent.parent.parent / 'share' / 'scenario_customization' / 'scenarios' / scenario

    if not os.path.isfile(base_path / 'scenario_description.txt'):
        raise FileNotFoundError(f"Scenario description file not found: {base_path / 'scenario_description.txt'}")
    
    description = ''
    with open((base_path / 'scenario_description.txt'), 'r') as f:
        description = f.readlines()
    
    return '\n'.join(description)


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


def get_db_by_task(scenario: str, default_db: str) -> Dict[str, Type[BaseModel]]:
    tool_dict = {}
    package = f'scenario_customization.{scenario}'

    try:
        scenario_module = __import__(package, fromlist=[""])
    except ModuleNotFoundError:
        print(f"Scenario module '{package}' not found.")
        return tool_dict

    for finder, task_name, ispkg in pkgutil.iter_modules(scenario_module.__path__):
        try:
            intent_module_path = f"{package}.{task_name}.query"
            intent_module = importlib.import_module(intent_module_path)

            for name, obj in inspect.getmembers(intent_module, inspect.isclass):
                if issubclass(obj, BaseModel) and obj is not BaseModel:
                    print(f"\033[95mLoaded {obj.__name__} from {task_name}\033[0m")

                    if hasattr(obj, '_DB'):
                        tool_dict[task_name] = obj.__private_attributes__['_DB'].default
        except ModuleNotFoundError:
            pass

    tool_dict['default'] = default_db
    return tool_dict

def load_all_scenario_dbs(scenario:str, default_db: str):
    db_dict_names = get_db_by_task(scenario, default_db)
        
    db_dict = {}
    schemas_dict = {}
    instructions_dict = {}

    for task, db_name in db_dict_names.items():
        db_dict[task] = DBFactory.create_adapter(db_name)
        schemas_dict[task] = db_dict[task].get_schema()
        instructions_dict[task] = db_dict[task].get_prompt()
    return db_dict, schemas_dict, instructions_dict


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
