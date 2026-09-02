from __future__ import annotations

from evaluation.controllers import infer_intent, run_direct, run_factored_and_rule
from evaluation.llm_client import CompletionTrace, _retry_after_seconds
from evaluation.metrics import aggregate, score_controller_record, score_readiness_record
from evaluation.paired_stats import exact_mcnemar, wilson_interval
from evaluation.task_spec import missing_parameters, normalize_parameters


class FakeClient:
    def __init__(self, responses):
        self.responses = iter(responses)

    def complete_json(self, **_kwargs):
        parsed = next(self.responses)
        return CompletionTrace(
            parsed=parsed,
            raw_text="{}",
            latency_seconds=0.1,
            prompt_tokens=10,
            completion_tokens=5,
            total_tokens=15,
            attempts=1,
            error=None,
        )

    def complete_tool_call(self, **_kwargs):
        return self.complete_json()


CASE = {
    "id": "valid-dish",
    "category": "complete_valid",
    "user_input": "Quanti grassi ha il tiramisu?",
    "expected": {
        "decision": "execute",
        "action": "DishInfo",
        "parameters": {
            "nome_utente": "",
            "nome_piatto": "tiramisu",
            "controllo_ingredienti": ["grassi"],
        },
    },
}


def test_false_is_not_missing_for_boolean_context():
    assert missing_parameters(
        "SubstituteDish",
        {
            "nome_utente": "luca",
            "giorno": "lunedi",
            "pasto": "cena",
            "ha_piano_settimanale": False,
        },
    ) == []


def test_parameter_normalization():
    assert normalize_parameters(
        "DishInfo",
        {"nome_piatto": "Panna Cotta", "controllo_ingredienti": ["Lattosio"]},
    ) == {
        "nome_utente": "",
        "nome_piatto": "panna cotta",
        "controllo_ingredienti": ["lattosio"],
    }


def test_paired_factored_and_rule_share_intent():
    client = FakeClient(
        [
            {
                "action": "DishInfo",
                "parameters": {"nome_piatto": "tiramisu", "controllo_ingredienti": ["grassi"]},
            },
            {"can_proceed": True, "reason": "complete"},
        ]
    )
    records = run_factored_and_rule(
        client,
        CASE,
        repeat=0,
        intent_temperature=0.0,
        gate_temperature=0.2,
        include_factored=True,
        include_rule=True,
    )
    assert {record["architecture"] for record in records} == {
        "janus_factored",
        "janus_rule_gate",
    }
    assert all(record["prediction"]["decision"] == "execute" for record in records)
    assert records[0]["traces"][0] == records[1]["traces"][0]


def test_direct_and_aggregation():
    client = FakeClient(
        [
            {
                "decision": "execute",
                "action": "DishInfo",
                "parameters": {"nome_piatto": "tiramisu", "controllo_ingredienti": ["grassi"]},
                "reason": "complete",
                "clarification": "",
            }
        ]
    )
    record = run_direct(client, CASE, repeat=0, temperature=0.0)
    summary = aggregate([record], "controller")
    overall = next(row for row in summary if row["category"] == "ALL")
    assert overall["operational_task_success"] == 1.0
    assert overall["strict_state_match"] == 1.0
    assert overall["parameter_micro_f1"] == 1.0


def test_structured_failure_cannot_match_conservative_fallback():
    controller_record = {
        "expected": {"decision": "reject", "action": "OutOfScope", "parameters": {}},
        "prediction": {"decision": "reject", "action": "OutOfScope", "parameters": {}},
        "structured_output_failure": True,
    }
    readiness_record = {
        "expected": {"can_proceed": False},
        "prediction": {"can_proceed": False},
        "structured_output_failure": True,
    }
    assert score_controller_record(controller_record)["operational_success"] is False
    assert score_readiness_record(readiness_record)["readiness_correct"] is False


def test_groq_retry_delay_is_parsed():
    error = RuntimeError("Rate limit reached. Please try again in 570ms.")
    assert _retry_after_seconds(error) == 0.82


def test_clarification_does_not_require_exact_partial_state_for_operational_success():
    record = {
        "expected": {
            "decision": "clarify",
            "action": "DishInfo",
            "parameters": {"nome_piatto": "", "controllo_ingredienti": ["calorie"]},
        },
        "prediction": {
            "decision": "clarify",
            "action": "DishInfo",
            "parameters": {},
        },
        "structured_output_failure": False,
    }
    score = score_controller_record(record)
    assert score["operational_success"] is True
    assert score["strict_state_match"] is False


def test_exact_paired_statistics():
    assert exact_mcnemar(11, 1) == 0.00634765625
    lower, upper = wilson_interval(25, 29)
    assert lower < 25 / 29 < upper


def test_native_intent_uses_tool_call_and_frozen_db_context():
    client = FakeClient(
        [
            {
                "name": "SubstituteDish",
                "arguments": {
                    "nome_utente": "Luca",
                    "giorno": "martedì",
                    "pasto": "cena",
                    "ha_piano_settimanale": False,
                },
            }
        ]
    )
    case = {
        "user_input": "Sono Luca, cosa posso mangiare martedì a cena?",
        "memory": [],
        "tool_context": {"ha_piano_settimanale": True},
    }
    prediction = infer_intent(client, case, temperature=0, interface="native_tools")
    assert prediction.action == "SubstituteDish"
    assert prediction.parameters["giorno"] == "martedi"
    assert prediction.parameters["ha_piano_settimanale"] is True
