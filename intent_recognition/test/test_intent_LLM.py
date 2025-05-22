import pytest, os, json, sys
from langsmith import testing as t
filepath = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, filepath)
from intent_recognition.llm import get_LLM_response

os.environ["LANGSMITH_TRACING"] = "true"
os.environ["LANGSMITH_ENDPOINT"] = "https://api.smith.langchain.com"
os.environ["LANGSMITH_PROJECT"] = "advisor"
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
    actual_intent, actual_parameters = get_LLM_response(question)

    t.log_outputs({
        "action_name": actual_intent,
        "parameters": actual_parameters
    })

    t.log_feedback(
        key="Intent Accuracy",
        score=1 if actual_intent == expected_intent else 0
    )
    
    # Calculate F1 measure for parameters
    if expected_parameters:
        
        f1_score = f1_measure(expected_parameters, actual_parameters)
        t.log_feedback(
            key="Entities F1 Score",
            score=f1_score
        )

    # Assert the intent name
    assert actual_intent == expected_intent, f"Expected {expected_intent}, got {actual_intent}"

    errors = []

    # Assert the parameters
    # Compare expected and actual parameters, collect differences
    for key in expected_parameters:
        expected_value = expected_parameters.get(key)
        actual_value = actual_parameters.get(key)
        if key not in actual_parameters:
            errors.append(f"{key}: expected {expected_value}, got <missing>")
        elif actual_value != expected_value:
            errors.append(f"{key}: expected {expected_value}, got {actual_value}")

    # Check for extra parameters in actual that are not expected
    for key in actual_parameters:
        if key not in expected_parameters:
            actual_value = actual_parameters.get(key)
            errors.append(f"{key}: expected <not present>, got {actual_value}")

    if errors:
        t.log_feedback(
            key="Parameter Errors",
            value="\n".join(errors)
        )
        assert False, f"Errors in parameters:\n{errors}"
# to run:
# LANGSMITH_TEST_SUITE="Groq LLM Intent Tests" pytest /home/kimary/unipa/src/unipa_inner_speech/intent_recognition/test/test_intent_LLM.py