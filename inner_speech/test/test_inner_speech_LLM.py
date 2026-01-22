from inner_speech.inner_speech_llm import InnerSpeech_LLM
from langsmith import testing as t
import pytest, os, json
import evaluate
import ast
from deep_translator import GoogleTranslator


# # Load metrics once
bertscore = evaluate.load("bertscore")

# Initialize translator (Italian to English)
translator = GoogleTranslator(source='it', target='en')

def compute_metrics(prediction: str, reference: str):
    bertscore_result = bertscore.compute(predictions=[prediction], references=[reference], lang="it")
    bert_f1 = bertscore_result["f1"][0]  # F1 score from BERTScore

    return {
        "bert_f1": bert_f1
    }

node_name = "inner_speech" 
IS_LLM = InnerSpeech_LLM(node_name)

os.environ["LANGSMITH_TRACING"] = "true"
os.environ["LANGSMITH_ENDPOINT"] = "https://api.smith.langchain.com"
os.environ["LANGSMITH_PROJECT"] = f'{os.getenv("SCENARIO")}:{ast.literal_eval(os.getenv("LLM_CONFIG"))[node_name]["model_name"]}'
os.environ["LANGSMITH_API_KEY"] = os.getenv("LANGSMITH_API_KEY")
os.environ["LANGSMITH_TEST_SUITE"] = "Inner Speech"



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
questions_en = [example.get("question_en", "") for example in examples]
input2questions_en = dict(zip(inputs, questions_en))
input2params = {example["question"]: {
    "action_name": example["action_name"], 
    "parameters": example["parameters"],
    "missing_parameters": example["missing_parameters"]} for example in examples}
input2output = {example["question"]: {
    "inner_speech": example["inner_speech"],
    "inner_speech_en": example.get("inner_speech_en", ""),
    "can_proceed": example["can_proceed"]} for example in examples}


@pytest.mark.parametrize("question", inputs)
@pytest.mark.langsmith  # Enables tracking in LangSmith
def test_my_groq_chain(question):
    expected_inner_speech = input2output[question]["inner_speech"]
    expected_inner_speech_en = input2output[question]["inner_speech_en"]
    expected_can_proceed = input2output[question]["can_proceed"]
    question_en = input2questions_en[question]

    # Log inputs to LangSmith (including English translation)
    t.log_inputs({
        "question": question,
        "question_en": question_en
    })

    # Log reference outputs to LangSmith (including English translation)
    t.log_reference_outputs({
        "inner_speech": expected_inner_speech,
        "inner_speech_en": expected_inner_speech_en,
        "can_proceed": expected_can_proceed
    })

    # Call your Groq chain w/ question, action_name, parameters, missing_parameters
    action_name = input2params[question]["action_name"]
    parameters = input2params[question]["parameters"]
    missing_parameters = input2params[question]["missing_parameters"]
    outputs, total_time, prompt_tokens, completion_tokens, total_tokens = IS_LLM.get_LLM_response(question, action_name, parameters, missing_parameters, return_time=True, return_tokens=True)

    t.log_feedback(key="total_time", score=round(total_time, 3))
    t.log_feedback(key="prompt_tokens", score=prompt_tokens)
    t.log_feedback(key="completion_tokens", score=completion_tokens)
    t.log_feedback(key="total_tokens", score=total_tokens)
    
    actual_inner_speech = outputs["inner_speech"]
    actual_can_proceed = outputs["can_proceed"]
    
    # Translate actual inner speech to English
    try:
        actual_inner_speech_en = translator.translate(actual_inner_speech)
    except Exception:
        actual_inner_speech_en = ""

    t.log_outputs({
        "inner_speech": actual_inner_speech,
        "inner_speech_en": actual_inner_speech_en,
        "can_proceed": actual_can_proceed,
    })

    metrics = compute_metrics(actual_inner_speech, expected_inner_speech)

    t.log_feedback(key="bert_f1", score=round(metrics["bert_f1"], 3))

    # Also check can_proceed match
    assert actual_can_proceed == expected_can_proceed

# to run:
# pytest /home/kimary/unipa/src/unipa_inner_speech/inner_speech/test/test_inner_speech_LLM.py