import yaml
import importlib
import os
from ament_index_python.packages import get_package_share_directory
from .wrapper import make_callable_from_context


def load_plugins(scenario:str):
    package_share = get_package_share_directory("scenario_customization")
    PLUGIN_CONFIG_PATH = os.path.abspath(os.path.join(package_share, f'./scenarios/{scenario}/plugin_config.yaml'))
    
    with open(PLUGIN_CONFIG_PATH, "r") as f:
        config = yaml.safe_load(f) or {}
    
    plugins = config.get('plugins', [])
    
    loaded_plugins = []
    for plugin in plugins:
        module_name = plugin.get('module')
        function_name = plugin.get('function')

        # Dynamically import the module
        try:
            module = importlib.import_module(module_name)
            function = getattr(module, function_name)

            # Wrap the function with make_callable_from_context
            wrapped_function = make_callable_from_context(function)

            loaded_plugins.append(wrapped_function)
            print(f"\033[38;5;208mLoaded and wrapped {function_name} from {module_name}\033[0m")
        except (ModuleNotFoundError, AttributeError) as e:
            print(f"Error loading {module_name}.{function_name}: {e}")

    return loaded_plugins