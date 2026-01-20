import importlib
import sys
from pathlib import Path
import ast
import json
import os
import pytest
import time

from dotenv import load_dotenv
from langsmith import testing as t
from pydantic import BaseModel, Field

try:
    import evaluate
except Exception:
    evaluate = None

if not hasattr(t, "log_question_en"):
    def _log_question_en_stub(*args, **kwargs):
        return None
    t.log_question_en = _log_question_en_stub

REPO_ROOT = Path(__file__).resolve().parents[2]
EXTRA_PATHS = [
    str(REPO_ROOT / "scope_detection"),
    str(REPO_ROOT / "shared_utils"),
    str(REPO_ROOT / "db_adapters"),
    str(REPO_ROOT),
]
rest = [p for p in sys.path if p not in EXTRA_PATHS]
sys.path = EXTRA_PATHS + rest
importlib.invalidate_caches()

load_dotenv(REPO_ROOT / ".env", override=True)
load_dotenv(REPO_ROOT / ".config", override=True)
os.environ.setdefault("SCENARIO", "ADVISOR")

from scope_detection.scope_detection_llm import ScopeDetection_LLM
from scope_detection.domain_examples.ita import domain_descriptions


def create_scenario_tools(domain_descriptions: list):
    scenario_tools = {}

    for entry in domain_descriptions:
        # Split the domain name and its docstring
        if ':' not in entry:
            continue  # Skip malformed entries
        name, doc = entry.split(':', maxsplit=1)
        name = name.strip()
        doc = doc.strip()

        # Define fields and annotations
        fields = {
            '__doc__': doc,
            '__annotations__': {
                'reason': str
            },
            'reason': Field(
                description="Il tuo ragionamento. Devi spiegare perché questo tool è adeguato alla domanda dell'utente."
            ),
        }

        # Dynamically create the class
        cls = type(name, (BaseModel,), fields)
        scenario_tools[name] = cls

    return scenario_tools


with open('/home/belca/Desktop/ros2_ws/src/unipa_inner_speech/scope_detection/scope_detection/selected_domain_combinations.json', 'r') as f:
    combinations = json.load(f)

idx = 5  # choose which combo to run
selected_combo = combinations[idx]
combo_name = selected_combo.get("name", f"combo_{idx}")
combo = list(selected_combo["labels"])
combo.append('OutOfScope')
print(f"Using combo {combo_name}: {combo}")

# # Load metrics once
bertscore = None
if evaluate:
    try:
        bertscore = evaluate.load("bertscore")
    except Exception as exc:
        print(f"Warning: failed to load bertscore metric: {exc}")
        bertscore = None

def compute_metrics(prediction: str, reference: str):
    global bertscore
    if bertscore is None and evaluate:
        try:
            bertscore = evaluate.load("bertscore")
        except Exception as exc:
            raise RuntimeError(f"BERTScore unavailable: {exc}")
    if bertscore is None:
        raise RuntimeError("BERTScore unavailable: evaluate not installed or failed to load.")
    bertscore_result = bertscore.compute(predictions=[prediction], references=[reference], lang="it")
    bert_f1 = bertscore_result["f1"][0]  # F1 score from BERTScore
    return {"bert_f1": bert_f1}

# let's make up some fake tools to handle all the scenarios
testing_tools = create_scenario_tools(domain_descriptions)
print(testing_tools)

testing_tools_new = {k:v for k,v in testing_tools.items() if k in combo}
testing_tools = testing_tools_new

node_name = "scope_detection" 
SD_LLM = ScopeDetection_LLM(
    node_name,
    testing_tools,
    use_db_adapter=False,
    use_scenario_description=False,
)

os.environ["LANGSMITH_TRACING"] = "true"
os.environ["LANGSMITH_ENDPOINT"] = "https://api.smith.langchain.com"
os.environ["LANGSMITH_PROJECT"] = f'{combo_name}:{ast.literal_eval(os.getenv("LLM_CONFIG"))[node_name]["model_name"]}'
os.environ["LANGSMITH_API_KEY"] = os.getenv("LANGSMITH_API_KEY")
os.environ["LANGSMITH_TEST_SUITE"] = "_Scope Detection"



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
new_examples = []
for i in range(len(examples)):
    if examples[i]['scenario'] in combo:
        new_examples.append(examples[i])  
examples = new_examples

inputs = [example["question"] for example in examples]
input2params = {example["question"]: {
    "inner_speech": example["inner_speech"], "question_en": example["question_en"]} for example in examples}
input2output = {example["question"]: {
    "reason": example["reason"], 
    "scenario": example["scenario"]} for example in examples}

# print(examples)
print(len(examples))
# exit()


@pytest.mark.parametrize("question", inputs)
@pytest.mark.langsmith  # Enables tracking in LangSmith
def test_my_groq_chain(question):
    expected_reason = input2output[question]["reason"]
    expected_scenario = input2output[question]["scenario"]

    # t.log_question_en({
    #     "question_en": input2params[question]["question_en"]
    # })
    t.log_inputs({
        "question": question,
        "question_en": input2params[question]["question_en"]
    })

    # Log to LangSmith
    t.log_reference_outputs({
        "reason": expected_reason,
        "scenario": expected_scenario
    })
    # print('ciao')

    # Call your Groq chain w/ question, action_name, parameters, missing_parameters
    inner_speech = input2params[question]["inner_speech"]
    outputs, total_time = SD_LLM.get_LLM_response(question, inner_speech, return_time=True)

    t.log_feedback(key="total_time", score=round(total_time, 3))
    
    actual_reason = outputs["reason"]
    actual_scenario = outputs["scenario"]

    t.log_outputs({
        "reason": actual_reason,
        "scenario": actual_scenario,
    })

    try:
        metrics = compute_metrics(actual_reason, expected_reason)
        bert_f1 = metrics.get("bert_f1")
        print(f"bert_f1: {bert_f1}")
        if bert_f1 is not None:
            t.log_feedback(key="bert_f1", score=round(bert_f1, 3))
    except Exception as exc:
        print(f"Warning: skipping bert_f1 logging: {exc}")

    # Also check can_proceed match
    assert actual_scenario == expected_scenario

    time.sleep(30)

# to run:
# pytest /home/kimary/unipa/src/unipa_inner_speech/inner_speech/test/test_scope_detection_LLM.py
# python3 -m pytest /home/belca/Desktop/ros2_ws/src/unipa_inner_speech/scope_detection/test/test_scope_detection_LLM.py
