from explainability.explainability_llm import QueryExplanation_LLM
from langsmith import testing as t
# import torch.nn.functional as F
import pytest, os, json
import evaluate
import ast
from deep_translator import GoogleTranslator

# Initialize translator (Italian to English)
translator = GoogleTranslator(source='it', target='en')


# # Load metrics once
# rouge = evaluate.load("rouge")
# bleu = evaluate.load("bleu")
# bertscore_tokenizer = AutoTokenizer.from_pretrained("bert-base-uncased")
# bertscore_model = AutoModel.from_pretrained("bert-base-uncased")
# bleurt = evaluate.load("bleurt", config_name="bleurt-base-128")
bertscore = evaluate.load("bertscore")


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
    # bleurt_result = bleurt.compute(predictions=[prediction], references=[reference])
    # bleurt_score = bleurt_result["scores"][0]

    # return {
    #     "rouge1": rouge_result["rouge1"],
    #     "rouge2": rouge_result["rouge2"],
    #     "rougeL": rouge_result["rougeL"],
    #     "bleu": bleu_result["bleu"],
    #     "cosine_similarity": cosine_sim
    # }

    return {
        "bert_f1": bert_f1,
        # "bleurt": bleurt_score
    }


node_name = "explainability"
IS_LLM = QueryExplanation_LLM(node_name)

os.environ["LANGSMITH_TRACING"] = "true"
os.environ["LANGSMITH_ENDPOINT"] = "https://api.smith.langchain.com"
os.environ["LANGSMITH_PROJECT"] = f'{os.getenv("SCENARIO")}:{ast.literal_eval(os.getenv("LLM_CONFIG"))[node_name]["model_name"]}'
os.environ["LANGSMITH_API_KEY"] = os.getenv("LANGSMITH_API_KEY")
os.environ["LANGSMITH_TEST_SUITE"] = "Explainability"

def extract_examples(filename='examples.json'):
    dir_path = os.path.dirname(os.path.realpath(__file__))
    full_path = os.path.join(dir_path, filename)

    with open(full_path, 'r') as file:
        data = json.load(file)
    processed_data = [(
                        example["question"], 
                        example.get("question_en", ""),
                        example["action_name"], 
                        example["queries"], 
                        example["results"], 
                        example["inner_speech"],
                        example["explanation"],
                        example.get("explanation_en", "")) for example in data]
    return processed_data


def get_examples():
    """Helper function to get examples for parameterized tests"""
    scenario = os.getenv("SCENARIO")
    example_filename = "examples.json" if scenario is None else f"examples_{scenario}.json"
    examples = extract_examples(filename=example_filename)
    return examples

    query_examples, is_examples = [], []

    for e in examples:
        # Index 3 is queries (previously index 2)
        if len(e[3]) == 0 and len(e[3]) == 0:
            is_examples.append(e)
        else:
            query_examples.append(e)
    return query_examples, is_examples

# Order is 
# question, question_en, action_name, queries, results, inner_speech, explanation, explanation_en
# query_examples, is_examples = get_examples()
examples = get_examples()

# Create lookup dictionaries for test data (keyed by question)
inputs = [e[0] for e in examples]  # List of questions only
input2question_en = {e[0]: e[1] for e in examples}
input2params = {e[0]: {"action_name": e[2], "queries": e[3], "results": e[4]} for e in examples}
input2output = {e[0]: {"explanation": e[6], "explanation_en": e[7]} for e in examples}



@pytest.mark.parametrize("question", inputs)
@pytest.mark.langsmith  # Enables tracking in LangSmith
def test_my_groq_chain(question):
    # Look up data from dictionaries
    question_en = input2question_en[question]
    action_name = input2params[question]["action_name"]
    queries = input2params[question]["queries"]
    results = input2params[question]["results"]
    expected_explanation = input2output[question]["explanation"]
    expected_explanation_en = input2output[question]["explanation_en"]

    # Log to LangSmith - include English translation in inputs
    t.log_inputs({
        "question": question,
        "question_en": question_en,
    })

    t.log_reference_outputs({
        "explanation": expected_explanation,
        "explanation_en": expected_explanation_en,
    })

    # Call your Groq chain w/ question, action_name, queries, results
    actual_explanation, total_time, prompt_tokens, completion_tokens, total_tokens = IS_LLM.get_LLM_response(question, action_name, queries, results, return_time=True, return_tokens=True)
    
    metrics = compute_metrics(actual_explanation, expected_explanation)
    
    # Translate actual inner speech to English
    try:
        actual_explanation_en = translator.translate(actual_explanation)
    except Exception:
        actual_explanation_en = ""

    t.log_outputs({
        "explanation": actual_explanation,
        "explanation_en": actual_explanation_en,
    })

    t.log_feedback(key="total_time", score=round(total_time, 3))

    t.log_feedback(key="bert_f1", score=round(metrics["bert_f1"], 3))
    
    t.log_feedback(key="prompt_tokens", score=prompt_tokens)
    t.log_feedback(key="completion_tokens", score=completion_tokens)
    t.log_feedback(key="total_tokens", score=total_tokens)
    # t.log_feedback(key="bleurt", score=round(metrics["bleurt"], 3))
    # t.log_feedback(key="rougeL", score=round(metrics["rougeL"], 3))
    # t.log_feedback(key="bleu", score=round(metrics["bleu"], 3))
    # t.log_feedback(key="cosine_similarity", score=round(metrics["cosine_similarity"], 3))

    # # "rouge1": metrics["rouge1"],
    # # "rouge2": metrics["rouge2"],
    # # "rougeL": metrics["rougeL"],
    # # "bleu": metrics["bleu"],
    # # "cosine_similarity": metrics["cosine_similarity"]


# to run:
# LANGSMITH_TEST_SUITE="Explainability" pytest /home/belca/Desktop/ros2_humble_ws/src/unipa_inner_speech/explainability/test/test_explainability_LLM.py
# python3 -m pytest /home/mary/src/unipa_inner_speech/explainability/test/test_explainability_LLM.py