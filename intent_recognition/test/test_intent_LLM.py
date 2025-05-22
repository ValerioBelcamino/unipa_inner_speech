from intent_recognition.intent_recognition_llm import IntentRecognition_LLM
from langsmith import testing as t
import pytest, os, json
import ast

node_name = "intent_recognition" 
IR_LLM = IntentRecognition_LLM(node_name)

os.environ["LANGSMITH_TRACING"] = "true"
os.environ["LANGSMITH_ENDPOINT"] = "https://api.smith.langchain.com"
os.environ["LANGSMITH_PROJECT"] = f'{os.getenv("SCENARIO")}:{ast.literal_eval(os.getenv("LLM_CONFIG"))[node_name]["model_name"]}'
os.environ["LANGSMITH_API_KEY"] = os.getenv("LANGSMITH_API_KEY")


def f1_measure(expected, actual):
    tp = sum(1 for key in expected if actual.get(key) == expected[key])
    precision = tp / len(actual) if actual else 0
    recall = tp / len(expected) if expected else 0
    f1_score = (2 * precision * recall) / (precision + recall) if (precision + recall) > 0 else 0
    return f1_score

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
    actual_intent, actual_parameters, total_time = IR_LLM.get_LLM_response(question, return_time=True)

    t.log_outputs({
        "action_name": actual_intent,
        "parameters": actual_parameters
    })

    t.log_feedback(key="total_time", score=round(total_time, 3))

    t.log_feedback(
        key="Intent Accuracy",
        score=1 if actual_intent == expected_intent else 0
    )
    
    # Calculate F1 measure for parameters
    if expected_parameters:
        
        f1_score = f1_measure(expected_parameters, actual_parameters)
        t.log_feedback(
            key="Entities F1 Score",
            score=round(f1_score, 3)
        )

    # Assert the intent name
    assert actual_intent == expected_intent, f"Expected {expected_intent}, got {actual_intent}"

    # Assert the parameters
    if expected_parameters:
        for key, expected in expected_parameters.items():
            assert actual_parameters.get(key) == expected, f"Expected {key} : {expected}, got {actual_parameters.get(key)}"

# to run:
# LANGSMITH_TEST_SUITE="Groq LLM Intent Tests" pytest /home/kimary/unipa/src/unipa_inner_speech/intent_recognition/test/test_intent_LLM.py
# LANGSMITH_TEST_SUITE="Intent Recognition" pytest /home/belca/Desktop/ros2_humble_ws/src/unipa_inner_speech/intent_recognition/test/test_intent_LLM.py