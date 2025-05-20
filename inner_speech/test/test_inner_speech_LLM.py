import pytest, os, json, random, ast
from langsmith import testing as t
from dotenv import load_dotenv
from shared_utils.customization_helpers import load_all_intent_models, get_scenario_description
from intent_post_processing.loader import load_plugins
from langchain.chat_models import init_chat_model
from typing import Any, get_origin, get_args, Union
from db_adapters import DBFactory
from pydantic import BaseModel, Field
from langchain.evaluation.parsing.base import JsonEqualityEvaluator
from langchain_core.messages import SystemMessage, HumanMessage
import evaluate
from transformers import AutoTokenizer, AutoModel
import torch
import torch.nn.functional as F
import asyncio

# # Load metrics once
# rouge = evaluate.load("rouge")
# bleu = evaluate.load("bleu")
# bertscore_tokenizer = AutoTokenizer.from_pretrained("bert-base-uncased")
# bertscore_model = AutoModel.from_pretrained("bert-base-uncased")
bleurt = evaluate.load("bleurt", config_name="bleurt-base-128")
bertscore = evaluate.load("bertscore")

class InnerSeechOutputFormat(BaseModel):
    """ Dato un prompt di un utente, l'azione ed i parametri estratti dal riconoscimento dell'intento devi elaborare un discorso interiore che spieghi se l'azione può essere portata a termine oppure no."""

    inner_speech: str = Field(description="Il tuo ragionamento")
    can_proceed: bool = Field(description="Se la richiesta dell'utente può essere accolta")


# def embed_text(text):
#     inputs = bertscore_tokenizer(text, return_tensors="pt", truncation=True, max_length=512)
#     with torch.no_grad():
#         outputs = bertscore_model(**inputs)
#     # Use mean pooling
#     return outputs.last_hidden_state.mean(dim=1).squeeze()

def compute_metrics(prediction: str, reference: str):
    # rouge_result = rouge.compute(predictions=[prediction], references=[reference])
    # bleu_result = bleu.compute(predictions=[prediction], references=[[reference]])

    # pred_embedding = embed_text(prediction)
    # ref_embedding = embed_text(reference)
    # cosine_sim = F.cosine_similarity(pred_embedding, ref_embedding, dim=0).item()

     # BERTScore: Uses BERT model to compute semantic similarity between prediction and reference.
    bertscore_result = bertscore.compute(predictions=[prediction], references=[reference], lang="en")
    bert_f1 = bertscore_result["f1"][0]  # F1 score from BERTScore

    # # BLEURT: Pretrained model trained on human ratings to score similarity (requires large model download).
    bleurt_result = bleurt.compute(predictions=[prediction], references=[reference])
    bleurt_score = bleurt_result["scores"][0]

    # return {
    #     "rouge1": rouge_result["rouge1"],
    #     "rouge2": rouge_result["rouge2"],
    #     "rougeL": rouge_result["rougeL"],
    #     "bleu": bleu_result["bleu"],
    #     "cosine_similarity": cosine_sim
    # }

    return {
        "bert_f1": bert_f1,
        "bleurt": bleurt_score
    }


os.environ["LANGSMITH_TRACING"] = "true"
os.environ["LANGSMITH_ENDPOINT"] = "https://api.smith.langchain.com"
os.environ["LANGSMITH_PROJECT"] = "advisor"

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
BASE_DIR = os.path.abspath(os.path.join(CURRENT_DIR, "../../../unipa_inner_speech"))
dotenv_path = os.path.join(BASE_DIR, ".env")
load_dotenv(dotenv_path, override=True)
dotconfig_path = os.path.join(BASE_DIR, ".config")
load_dotenv(dotconfig_path)

os.environ["LANGSMITH_API_KEY"] = os.getenv("LANGSMITH_API_KEY")

llm_config = ast.literal_eval(os.getenv("LLM_CONFIG"))['inner_speech']

llm = init_chat_model(
        model=llm_config['model_name'], 
        model_provider=llm_config['model_provider'], 
        temperature=llm_config['temperature'], 
        api_key=os.getenv("GROQ_API_KEY")
    )

scenario = os.getenv("SCENARIO")
print(f"\033[34mUsing {scenario}!\033[0m")
context_scenario = get_scenario_description(scenario)
print(f"\033[34mDesciription: {context_scenario}\033[0m")
dynamic_intent_tools_dict = load_all_intent_models(scenario)
print(f"\033[34mLoaded {dynamic_intent_tools_dict} intent_tool(s).\033[0m")
dynamic_intent_toolnames = [dit.__name__ for dit in dynamic_intent_tools_dict.values()]

db_type = os.getenv("DB_TYPE") 
db = DBFactory.create_adapter(db_type)

structured_llm = llm.with_structured_output(InnerSeechOutputFormat)
print(f"\033[1;38;5;208mLoaded {structured_llm}.\033[0m")



def get_LLM_response(question, action_name, parameters, missing_parameters):
    """
    Function to get the LLM response for a given user input.
    """
    # start_time = time.time()
    prompt = [  
            SystemMessage(content=f"""{context_scenario}. 
                Devi impedire l'esecuzione di domande non pertinenti al tuo scopo
                Devi impedire l'esecuzione di domande con parametri obbligatori mancanti. 
                Devi filtrare domande relative al tuo argomento ma troppo vaghe."""),
            HumanMessage(content=f"""La domanda dell'utente è: {question}.
                Il riconoscimento dell'intento ha assegnato la seguente funzione: {action_name}.
                Con i seguenti parametri: {parameters}.
                Parametri obbligatori mancanti: {missing_parameters}""") 
        ]
    
    llm_response = structured_llm.invoke(prompt)
    print(f'\033[91m{llm_response}\033[0m')

    return {
        "inner_speech": llm_response.inner_speech,
        "can_proceed": llm_response.can_proceed
    }

def extract_examples(filename='examples.json'):
    dir_path = os.path.dirname(os.path.realpath(__file__))
    full_path = os.path.join(dir_path, filename)

    with open(full_path, 'r') as file:
        data = json.load(file)
    processed_data = [(example["question"], example["action_name"], example["parameters"], example["missing_parameters"], example["inner_speech"], example["can_proceed"]) for example in data]
    return processed_data

def get_examples():
    """Helper function to get examples for parameterized tests"""
    scenario = os.getenv("SCENARIO")
    example_filename = "examples.json" if scenario is None else f"examples_{scenario}.json"
    examples = extract_examples(filename=example_filename)
    return examples

# Order is 
# question, action_name, parameters, missing_parameters, inner_speech, can_proceed
examples = get_examples()
print(examples)


@pytest.mark.parametrize("examples_input", examples)
@pytest.mark.langsmith  # Enables tracking in LangSmith
def test_my_groq_chain(examples_input):
    expected_inner_speech = examples_input[-2]
    expected_can_proceed = examples_input[-1]

    # Log to LangSmith
    t.log_reference_outputs({
        "inner_speech": expected_inner_speech,
        "can_proceed": expected_can_proceed
    })

    # Call your Groq chain w/ question, action_name, parameters, missing_parameters
    outputs = get_LLM_response(examples_input[0], examples_input[1], examples_input[2], examples_input[3])
    
    actual_inner_speech = outputs["inner_speech"]
    actual_can_proceed = outputs["can_proceed"]

    metrics = compute_metrics(actual_inner_speech, expected_inner_speech)

    t.log_outputs({
        "inner_speech": actual_inner_speech,
        "can_proceed": actual_can_proceed,
    })


    t.log_feedback(key="bert_f1", score=round(metrics["bert_f1"], 3))
    t.log_feedback(key="bleurt", score=round(metrics["bleurt"], 3))
    # t.log_feedback(key="rougeL", score=round(metrics["rougeL"], 3))
    # t.log_feedback(key="bleu", score=round(metrics["bleu"], 3))
    # t.log_feedback(key="cosine_similarity", score=round(metrics["cosine_similarity"], 3))

    # # "rouge1": metrics["rouge1"],
    # # "rouge2": metrics["rouge2"],
    # # "rougeL": metrics["rougeL"],
    # # "bleu": metrics["bleu"],
    # # "cosine_similarity": metrics["cosine_similarity"]

    # Also check can_proceed match
    assert actual_can_proceed == expected_can_proceed

# to run:
# LANGSMITH_TEST_SUITE="Groq LLM Intent Tests" pytest /home/belca/Desktop/ros2_humble_ws/src/unipa_inner_speech/inner_speech/test/test_inner_speech_LLM.py