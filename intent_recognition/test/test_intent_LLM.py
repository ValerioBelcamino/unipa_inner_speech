from intent_recognition.intent_recognition_llm import IntentRecognition_LLM
from langsmith import testing as t
import pytest, os, json
import ast
import random

# Set seed for reproducibility (change or remove for different shuffles each run)
SHUFFLE_SEED = int(os.getenv("TEST_SHUFFLE_SEED", "42"))

node_name = "intent_recognition" 
IR_LLM = IntentRecognition_LLM(node_name)

IR_LLM = IntentRecognition_LLM('intent_recognition')

os.environ["LANGSMITH_TRACING"] = "true"
os.environ["LANGSMITH_ENDPOINT"] = "https://api.smith.langchain.com"
os.environ["LANGSMITH_PROJECT"] = f'{os.getenv("SCENARIO")}:{ast.literal_eval(os.getenv("LLM_CONFIG"))[node_name]["model_name"]}'
os.environ["LANGSMITH_API_KEY"] = os.getenv("LANGSMITH_API_KEY")
os.environ["LANGSMITH_TEST_SUITE"] = "Intent Recognition"

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
    processed_data = [(example["question"], example["action_name"], example["parameters"], example["question_en"]) for example in data]
    return processed_data

def get_examples():
    """Helper function to get examples for parameterized tests"""
    scenario = os.getenv("SCENARIO")
    example_filename = "examples.json" if scenario is None else f"examples_{scenario}.json"
    examples = extract_examples(filename=example_filename)
    
    # Shuffle examples with a seed for reproducibility
    random.seed(SHUFFLE_SEED)
    random.shuffle(examples)
    
    return examples

examples = get_examples()
inputs, intents, parameters, questions_en = zip(*examples)
input2intents = dict(zip(inputs, intents))
input2parameters = dict(zip(inputs, parameters))
input2questions_en = dict(zip(inputs, questions_en))

@pytest.mark.parametrize("question", inputs)
@pytest.mark.langsmith  # Enables tracking in LangSmith
def test_my_groq_chain(question):
    expected_intent = input2intents[question]
    expected_parameters = input2parameters[question]
    question_en = input2questions_en[question]

    # Log inputs to LangSmith
    t.log_inputs({
        "question": question,
        "question_en": question_en
    })

    # Log expected outputs to LangSmith
    t.log_reference_outputs({
        "action_name": expected_intent,
        "parameters": expected_parameters
    })

    # Call your Groq chain
    actual_intent, actual_parameters, total_time, prompt_tokens, completion_tokens, total_tokens = IR_LLM.get_LLM_response(question, memory="", return_time=True, return_tokens=True)

    t.log_outputs({
        "action_name": actual_intent,
        "parameters": actual_parameters
    })

    t.log_feedback(key="total_time", score=round(total_time, 3))
    t.log_feedback(key="prompt_tokens", score=prompt_tokens)
    t.log_feedback(key="completion_tokens", score=completion_tokens)
    t.log_feedback(key="total_tokens", score=total_tokens)

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
# pytest /home/kimary/unipa/src/unipa_inner_speech/intent_recognition/test/test_intent_LLM.py
