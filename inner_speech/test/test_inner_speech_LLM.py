from inner_speech.inner_speech_llm import InnerSpeech_LLM
from langsmith import testing as t
import pytest, os, json
import evaluate

IS_LLM = InnerSpeech_LLM('inner_speech')

# # Load metrics once
bertscore = evaluate.load("bertscore")

def compute_metrics(prediction: str, reference: str):
    bertscore_result = bertscore.compute(predictions=[prediction], references=[reference], lang="it")
    bert_f1 = bertscore_result["f1"][0]  # F1 score from BERTScore

    return {
        "bert_f1": bert_f1
    }


os.environ["LANGSMITH_TRACING"] = "true"
os.environ["LANGSMITH_ENDPOINT"] = "https://api.smith.langchain.com"
os.environ["LANGSMITH_PROJECT"] = "advisor"
os.environ["LANGSMITH_API_KEY"] = os.getenv("LANGSMITH_API_KEY")


IS_LLM = InnerSpeech_LLM('inner_speech')


def extract_examples(filename='examples.json'):
    dir_path = os.path.dirname(os.path.realpath(__file__))
    full_path = os.path.join(dir_path, filename)

    with open(full_path, 'r') as file:
        data = json.load(file)

    return data


def get_examples():
    """Helper function to get examples for parameterized tests"""
    scenario = os.getenv("SCENARIO")
    example_filename = "examples.json" if scenario is None else f"examples_{scenario}.json"
    examples = extract_examples(filename=example_filename)
    return examples

examples = get_examples()
inputs = [example["question"] for example in examples]
input2params = {example["question"]: {
    "action_name": example["action_name"], 
    "parameters": example["parameters"],
    "missing_parameters": example["missing_parameters"]} for example in examples}
input2output = {example["question"]: {
    "inner_speech": example["inner_speech"], 
    "can_proceed": example["can_proceed"]} for example in examples}


@pytest.mark.parametrize("question", inputs)
@pytest.mark.langsmith  # Enables tracking in LangSmith
def test_my_groq_chain(question):
    expected_inner_speech = input2output[question]["inner_speech"]
    expected_can_proceed = input2output[question]["can_proceed"]

    # Log to LangSmith
    t.log_reference_outputs({
        "inner_speech": expected_inner_speech,
        "can_proceed": expected_can_proceed
    })

    # Call your Groq chain w/ question, action_name, parameters, missing_parameters
    action_name = input2params[question]["action_name"]
    parameters = input2params[question]["parameters"]
    missing_parameters = input2params[question]["missing_parameters"]
    outputs, total_time = IS_LLM.get_LLM_response(question, action_name, parameters, missing_parameters, return_time=True)

    t.log_feedback(key="total_time", score=round(total_time, 3))
    
    actual_inner_speech = outputs["inner_speech"]
    actual_can_proceed = outputs["can_proceed"]

    t.log_outputs({
        "inner_speech": actual_inner_speech,
        "can_proceed": actual_can_proceed,
    })

    metrics = compute_metrics(actual_inner_speech, expected_inner_speech)

    t.log_feedback(key="bert_f1", score=round(metrics["bert_f1"], 3))

    # Also check can_proceed match
    assert actual_can_proceed == expected_can_proceed

# to run:
# pytest /home/kimary/unipa/src/unipa_inner_speech/inner_speech/test/test_inner_speech_LLM.py