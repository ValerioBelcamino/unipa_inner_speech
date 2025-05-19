import pytest, os, json, random, ast
from langsmith import testing as t
from dotenv import load_dotenv
from shared_utils.customization_helpers import load_all_intent_models
from intent_post_processing.loader import load_plugins
from langchain.chat_models import init_chat_model
from typing import Any, get_origin, get_args, Union
from db_adapters import DBFactory
from langchain.evaluation.parsing.base import JsonEqualityEvaluator
import asyncio

os.environ["LANGSMITH_TRACING"] = "true"
os.environ["LANGSMITH_ENDPOINT"] = "https://api.smith.langchain.com"
os.environ["LANGSMITH_PROJECT"] = "advisor"

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
BASE_DIR = os.path.abspath(os.path.join(CURRENT_DIR, "../../../unipa_inner_speech"))
dotenv_path = os.path.join(BASE_DIR, ".env")
load_dotenv(dotenv_path)
dotconfig_path = os.path.join(BASE_DIR, ".config")
load_dotenv(dotconfig_path)

os.environ["LANGSMITH_API_KEY"] = os.getenv("LANGSMITH_API_KEY")

llm_config = ast.literal_eval(os.getenv("LLM_CONFIG"))['intent_recognition']

llm = init_chat_model(
        model=llm_config['model_name'], 
        model_provider=llm_config['model_provider'], 
        temperature=llm_config['temperature'], 
        api_key=os.getenv("GROQ_API_KEY")
    )

scenario = os.getenv("SCENARIO")
print(f"\033[34mUsing {scenario}!\033[0m")
dynamic_intent_tools_dict = load_all_intent_models(scenario)
print(f"\033[34mLoaded {dynamic_intent_tools_dict} intent_tool(s).\033[0m")
dynamic_intent_toolnames = [dit.__name__ for dit in dynamic_intent_tools_dict.values()]

db_type = os.getenv("DB_TYPE") 
db = DBFactory.create_adapter(db_type)

llm_with_tools = llm.bind_tools(dynamic_intent_tools_dict.values())
print(llm_with_tools.get_input_schema())
plugins = load_plugins(scenario)
print(f"\033[1;38;5;208mLoaded {len(plugins)} processing plugin(s).\033[0m")


def execute_plugin_pipeline(db_adapter, action_name, intent_parameters):
        """
        Function to execute the loaded plugins with the appropriate parameters.
        """

        context = {
            "db_adapter": db_adapter,  # Pass the adapter instead of the driver
            "action_name": action_name,
            "intent_parameters": intent_parameters  # Adding the dynamic parameters extracted after LLM computation
        }
        # Iterate over each loaded plugin (each wrapped function)
        for plugin_function in plugins:
            try:
                # Call the plugin with the context
                context['intent_parameters'] = plugin_function(context)

            except Exception as e:
                print(f"Error executing plugin: {e}")

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

def get_LLM_response(user_input):
    """
    Function to get the LLM response for a given user input.
    """
    # start_time = time.time()
    llm_response = llm_with_tools.invoke(
                [
                    ("system", """
You are tasked with identifying the correct intent from a set of available tools and extracting only the parameters explicitly provided by the user.
You must not use external knowledge, assumptions, or inference to guess or complete missing information.
If the user input is not relevant to any of the available tools, do not respond or assign an intent.
Only fill tool parameters when the necessary information is clearly and explicitly included in the user input.
Do not hallucinate or fabricate any information.
Do not fill gaps, or rephrase missing data.
If a parameter is missing, ambiguous, or incomplete, leave it blank and do not attempt to infer or complete it.
Follow these constraints strictly to ensure reliability and factual accuracy in tool usage."""),
                    ("human", user_input),
                ]
            )
    print(llm_response)
    tool_calls = llm_response.tool_calls
    print(f"\033[1;38;5;208mTool calls: '{tool_calls}'\033[0m")
    if not tool_calls:
        tool_name = "OutOfScope"
        tool_result = {}
    else:
        tool_name = tool_calls[0]["name"]
        tool_result = tool_calls[0]['args']
        tool_result = check_undeclared_parameters(dynamic_intent_tools_dict[tool_name], tool_result)
        execute_plugin_pipeline(db, tool_name, tool_result)
    
    return {
        "action_name": tool_name,
        "parameters": tool_result
    }

def extract_examples(filename='examples.json'):
    dir_path = os.path.dirname(os.path.realpath(__file__))
    full_path = os.path.join(dir_path, filename)

    with open(full_path, 'r') as file:
        data = json.load(file)
    processed_data = [(example["question"], example["action_name"], example["parameters"]) for example in data]
    return processed_data

def get_examples():
    """Helper function to get examples for parameterized tests"""
    scenario = os.getenv("SCENARIO")
    example_filename = "examples.json" if scenario is None else f"examples_{scenario}.json"
    examples = extract_examples(filename=example_filename)
    print(f"Found {len(examples)} examples in the dataset.")
    inputs, intents, parameters = zip(*examples)
    input2intents = dict(zip(inputs, intents))
    input2parameters = dict(zip(inputs, parameters))
    return inputs, input2intents, input2parameters

inputs, input2intents, input2parameters = get_examples()

@pytest.mark.parametrize("question", inputs)
@pytest.mark.langsmith  # Enables tracking in LangSmith
def test_my_groq_chain(question):
    expected_intent = input2intents[question]
    expected_parameters = input2parameters[question]

    # Log to LangSmith
    t.log_reference_outputs({
        "action_name": expected_intent,
        "parameters": expected_parameters
    })

    # Call your Groq chain
    outputs = get_LLM_response(question)
    
    actual_intent = outputs["action_name"]
    actual_parameters = outputs["parameters"]

    t.log_outputs({
        "action_name": actual_intent,
        "parameters": actual_parameters
    })

    # Assert the intent name
    assert actual_intent == expected_intent, f"Expected {expected_intent}, got {actual_intent}"

    # Assert the parameters
    if expected_parameters:
        for key, expected in expected_parameters.items():
            assert actual_parameters.get(key) == expected, f"Expected {key} : {expected}, got {actual_parameters.get(key)}"

# to run:
# LANGSMITH_TEST_SUITE="Groq LLM Intent Tests" pytest /home/kimary/unipa/src/unipa_inner_speech/intent_recognition/test/test_intent_LLM.py