from explainability.explainability_llm import QueryExplanation_LLM
from langsmith import testing as t
import torch.nn.functional as F
import pytest, os, json
import evaluate


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


os.environ["LANGSMITH_TRACING"] = "true"
os.environ["LANGSMITH_ENDPOINT"] = "https://api.smith.langchain.com"
os.environ["LANGSMITH_PROJECT"] = "advisor"

os.environ["LANGSMITH_API_KEY"] = os.getenv("LANGSMITH_API_KEY")


IS_LLM = QueryExplanation_LLM('explainability')


def get_LLM_response_wrap(question, action_name, queries, results):
    """
    Function to get the LLM response for a given user input.
    """
    return IS_LLM.get_LLM_response(question, action_name, queries, results)


def extract_examples(filename='examples.json'):
    dir_path = os.path.dirname(os.path.realpath(__file__))
    full_path = os.path.join(dir_path, filename)

    with open(full_path, 'r') as file:
        data = json.load(file)
    processed_data = [(
                        example["question"], 
                        example["action_name"], 
                        example["queries"], 
                        example["results"], 
                        example["inner_speech"],
                        example["explanation"]) for example in data]
    return processed_data


def get_examples():
    """Helper function to get examples for parameterized tests"""
    scenario = os.getenv("SCENARIO")
    example_filename = "examples.json" if scenario is None else f"examples_{scenario}.json"
    examples = extract_examples(filename=example_filename)

    query_examples, is_examples = [], []

    for e in examples:
        if len(e[2]) == 0 and len(e[2]) == 0:
            is_examples.append(e)
        else:
            query_examples.append(e)
    return query_examples, is_examples

# Order is 
# question, action_name, queries, results, inner_speech, explanation
query_examples, is_examples = get_examples()
print(query_examples)
print(len(query_examples))
print(is_examples)
print(len(is_examples))

exit()


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

    # Call your Groq chain w/ question, action_name, queries, results
    outputs = get_LLM_response_wrap(examples_input[0], examples_input[1], examples_input[2], examples_input[3])
    
    actual_inner_speech = outputs["inner_speech"]
    actual_can_proceed = outputs["can_proceed"]

    metrics = compute_metrics(actual_inner_speech, expected_inner_speech)

    t.log_outputs({
        "inner_speech": actual_inner_speech,
        "can_proceed": actual_can_proceed,
    })


    t.log_feedback(key="bert_f1", score=round(metrics["bert_f1"], 3))
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
# LANGSMITH_TEST_SUITE="Groq LLM Intent Tests" pytest /home/belca/Desktop/ros2_humble_ws/src/unipa_inner_speech/inner_speech/test/test_inner_speech_LLM.py