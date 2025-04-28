import importlib
import pkgutil
import inspect
from pydantic import BaseModel
from typing import Type, Dict

def load_all_intent_models() -> Dict[str, Type[BaseModel]]:
    return dynamic__iterative_load('intent')

def load_all_query_models() -> Dict[str, Type[BaseModel]]:
    return dynamic__iterative_load('query')

def dynamic__iterative_load(filename:str) -> Dict[str, Type[BaseModel]]:
    models = {}
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
                    models[task_name] = obj
        except ModuleNotFoundError:
            # No intent.py or broken module, ignore
            pass

    return models


