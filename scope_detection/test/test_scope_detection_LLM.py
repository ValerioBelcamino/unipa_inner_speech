from scope_detection.scope_detection_llm import ScopeDetection_LLM
from scope_detection.domain_examples.ita import domain_descriptions
from pydantic import BaseModel, Field
from langsmith import testing as t
from typing import Optional
import pytest, os, json
import evaluate
import json
import ast

import time


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
combo_dict = {}
for it in combinations:
    combo_dict[it['cosine_avg']] = it['labels']

# exit()

idx = 1
idkk = list(combo_dict.keys())[idx]
combo = combo_dict[idkk]
combo.append('OutOfScope')
print(combo)

# # Load metrics once
bertscore = evaluate.load("bertscore")

def compute_metrics(prediction: str, reference: str):
    bertscore_result = bertscore.compute(predictions=[prediction], references=[reference], lang="it")
    bert_f1 = bertscore_result["f1"][0]  # F1 score from BERTScore

    return {
        "bert_f1": bert_f1
    }

# let's make up some fake tools to handle all the scenarios
testing_tools = create_scenario_tools(domain_descriptions)
print(testing_tools)

testing_tools_new = {k:v for k,v in testing_tools.items() if k in combo}
testing_tools = testing_tools_new

node_name = "scope_detection" 
SD_LLM = ScopeDetection_LLM(node_name, testing_tools)

os.environ["LANGSMITH_TRACING"] = "true"
os.environ["LANGSMITH_ENDPOINT"] = "https://api.smith.langchain.com"
os.environ["LANGSMITH_PROJECT"] = f'{combo}:{ast.literal_eval(os.getenv("LLM_CONFIG"))[node_name]["model_name"]}'
os.environ["LANGSMITH_API_KEY"] = os.getenv("LANGSMITH_API_KEY")
os.environ["LANGSMITH_TEST_SUITE"] = "Scope Detection stupid"



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
    "inner_speech": example["inner_speech"]} for example in examples}
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

    metrics = compute_metrics(actual_reason, expected_reason)

    t.log_feedback(key="bert_f1", score=round(metrics["bert_f1"], 3))

    # Also check can_proceed match
    assert actual_scenario == expected_scenario

    time.sleep(59)

# to run:
# pytest /home/kimary/unipa/src/unipa_inner_speech/inner_speech/test/test_scope_detection_LLM.py
# python3 -m pytest /home/belca/Desktop/ros2_ws/src/unipa_inner_speech/scope_detection/test/test_scope_detection_LLM.py