from __future__ import annotations

from evaluation.grounding_benchmark import (
    format_direct_plan_prompt,
    format_janus_query_prompt,
)
from evaluation.grounding_metrics import evidence_records, score_grounding_record


def _record(query_results, answer, *, expected_evidence=None, targets=None):
    if expected_evidence is None:
        expected_evidence = [{"dish": "tiramisu", "calories": 420}]
    if targets is None:
        targets = ["calories"]
    return {
        "architecture": "test",
        "case_id": "case",
        "category": "dish_properties",
        "expected": {
            "decision": "execute",
            "action": "DishInfo",
            "parameters": {
                "nome_utente": "",
                "nome_piatto": "tiramisu",
                "controllo_ingredienti": ["calorie"],
            },
        },
        "reference": {
            "expected_evidence": expected_evidence,
            "answer_targets": targets,
            "expect_abstention": not expected_evidence,
        },
        "prediction": {
            "controller": {
                "decision": "execute",
                "action": "DishInfo",
                "parameters": {
                    "nome_utente": "",
                    "nome_piatto": "tiramisu",
                    "controllo_ingredienti": ["calorie"],
                },
            },
            "controller_structured_output_failure": False,
            "queries": ["MATCH ..."],
            "query_results": query_results,
            "answer_attempted": True,
            "answer": answer,
        },
        "structured_output_failure": False,
    }


def test_alias_invariant_grounded_success():
    record = _record(
        [[{"piatto": "Tiramisu", "calorie": 420}]],
        {
            "abstain": False,
            "claims": [
                {"entity": "tiramisu", "field": "calorie", "value_json": "420"}
            ],
            "answer": "Il tiramisu ha 420 calorie.",
        },
    )
    score = score_grounding_record(record)
    assert score["retrieval_correct"] is True
    assert score["supported_claim_rate"] == 1.0
    assert score["grounded_task_success"] is True


def test_broad_query_is_penalized_by_entity_precision():
    record = _record(
        [[
            {"dish": "tiramisu", "calories": 420},
            {"dish": "lasagna", "calories": 500},
        ]],
        {
            "abstain": False,
            "claims": [
                {"entity": "tiramisu", "field": "calories", "value_json": "420"}
            ],
            "answer": "Il tiramisu ha 420 calorie.",
        },
    )
    score = score_grounding_record(record)
    assert score["retrieval_target_recall"] == 1.0
    assert score["entity_precision"] == 0.5
    assert score["retrieval_correct"] is False


def test_compatibility_is_derived_from_empty_risks():
    records = evidence_records(
        [[{"piatto": "bistecca", "rischi": [], "allergie_utente": ["lattosio"]}]]
    )
    assert records[0]["compatible"] is True


def test_collected_and_row_wise_values_score_equally():
    expected = [
        {
            "dish": "tiramisu",
            "ingredients": ["caffe", "mascarpone", "uova", "zucchero"],
        }
    ]
    generated = [[
        {"piatto": "tiramisu", "ingredienti": "caffe"},
        {"piatto": "tiramisu", "ingredienti": "mascarpone"},
        {"piatto": "tiramisu", "ingredienti": "uova"},
        {"piatto": "tiramisu", "ingredienti": "zucchero"},
    ]]
    record = _record(
        generated,
        {
            "abstain": False,
            "claims": [
                {
                    "entity": "tiramisu",
                    "field": "ingredients",
                    "value_json": '["caffe","mascarpone","uova","zucchero"]',
                }
            ],
            "answer": "Il tiramisu contiene gli ingredienti elencati.",
        },
        expected_evidence=expected,
        targets=["ingredients"],
    )
    score = score_grounding_record(record)
    assert score["retrieval_target_recall"] == 1.0
    assert score["retrieval_target_precision"] == 1.0
    assert score["answer_target_recall"] == 1.0


def test_correct_abstention_on_empty_evidence():
    record = _record(
        [[]],
        {"abstain": True, "claims": [], "answer": "Non posso verificarlo."},
        expected_evidence=[],
        targets=[],
    )
    score = score_grounding_record(record)
    assert score["retrieval_correct"] is True
    assert score["abstention_correct"] is True
    assert score["grounded_task_success"] is True


def test_prompts_use_action_specific_vs_all_relevant_demonstrations():
    janus = format_janus_query_prompt(
        action="DishInfo",
        question="Quante calorie ha il tiramisu?",
        parameters={"nome_piatto": "tiramisu"},
        database_schema="Dish {name: STRING}",
    )
    direct_system, direct_user = format_direct_plan_prompt(
        {"user_input": "Quante calorie ha il tiramisu?", "memory": []},
        "Dish {name: STRING}",
    )
    assert janus.count("User asks:") == 6  # five demonstrations plus target
    assert direct_system.count("Action: DishInfo") == 5
    assert direct_system.count("Action: SubstituteDish") == 5
    assert "Quante calorie ha il tiramisu?" in direct_user
