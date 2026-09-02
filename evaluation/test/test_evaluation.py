from __future__ import annotations

from types import SimpleNamespace

from evaluation.controllers import infer_intent, run_direct, run_factored_and_rule
from evaluation.llm_client import (
    CompletionTrace,
    JsonLLMClient,
    ProviderRateLimitError,
    _retry_after_seconds,
)
from evaluation.metrics import (
    aggregate,
    score_controller_record,
    score_readiness_record,
)
from evaluation.multidomain import (
    run_multidomain_direct,
    run_multidomain_factored_and_rule,
)
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

    def complete_structured_tool_call(self, **_kwargs):
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
    assert (
        missing_parameters(
            "SubstituteDish",
            {
                "nome_utente": "luca",
                "giorno": "lunedi",
                "pasto": "cena",
                "ha_piano_settimanale": False,
            },
        )
        == []
    )


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
                "parameters": {
                    "nome_piatto": "tiramisu",
                    "controllo_ingredienti": ["grassi"],
                },
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
                "parameters": {
                    "nome_piatto": "tiramisu",
                    "controllo_ingredienti": ["grassi"],
                },
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


def test_explicit_millisecond_message_wins_over_ambiguous_header():
    error = RuntimeError("Rate limit reached. Please try again in 439ms.")
    error.response = SimpleNamespace(headers={"retry-after": "439"})
    assert _retry_after_seconds(error) == 0.6890000000000001


def test_provider_rate_limit_carries_retry_delay():
    error = ProviderRateLimitError("quota exhausted", 381.5)
    assert error.retry_after_seconds == 381.5


def test_length_truncated_native_call_is_not_out_of_scope():
    response = SimpleNamespace(
        usage=None,
        choices=[
            SimpleNamespace(
                finish_reason="length",
                message=SimpleNamespace(content="unfinished reasoning", tool_calls=[]),
            )
        ],
    )
    client = object.__new__(JsonLLMClient)
    client.model = "fake"
    client.base_url = "https://example.invalid/v1"
    client.max_attempts = 1
    client.max_completion_tokens = 32
    client.request_delay = 0.0
    client.max_retry_wait = 60.0
    client._last_request_finished = 0.0
    client._client = SimpleNamespace(
        chat=SimpleNamespace(
            completions=SimpleNamespace(create=lambda **_kwargs: response)
        )
    )

    trace = client.complete_tool_call(
        system="route",
        user="request",
        tools=[],
        temperature=0.0,
    )

    assert trace.parsed is None
    assert "completion-token cap" in (trace.error or "")


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


def test_multidomain_factored_routes_before_intent_and_gate():
    client = FakeClient(
        [
            {"name": "MOVIES", "arguments": {}},
            {"name": "MovieInfo", "arguments": {"title": "Inception"}},
            {"can_proceed": True, "reason": "clear movie request"},
        ]
    )
    case = {
        "id": "movie-route",
        "category": "complete_valid",
        "user_input": "Chi ha diretto Inception?",
        "memory": [],
        "expected": {
            "domain": "MOVIES",
            "decision": "execute",
            "action": "MovieInfo",
            "parameters": {
                "title": "inception",
                "director": "",
                "genres": [],
                "year": 0,
                "actors": [],
                "descriptive_movie_facts": [],
            },
        },
    }
    records = run_multidomain_factored_and_rule(
        client,
        case,
        repeat=0,
        scope_temperature=0.0,
        intent_temperature=0.0,
        gate_temperature=0.2,
        include_factored=True,
        include_rule=True,
        structured_interface="native_tools",
    )
    assert all(record["prediction"]["domain"] == "MOVIES" for record in records)
    assert all(record["prediction"]["action"] == "MovieInfo" for record in records)
    assert all(
        score_controller_record(record)["operational_success"] for record in records
    )


def test_wrong_multidomain_route_cannot_receive_operational_credit():
    record = {
        "expected": {
            "domain": "MOVIES",
            "decision": "execute",
            "action": "MovieInfo",
            "parameters": {"title": "dune"},
        },
        "prediction": {
            "domain": "ADVISOR",
            "decision": "execute",
            "action": "MovieInfo",
            "parameters": {"title": "dune"},
        },
        "structured_output_failure": False,
    }
    score = score_controller_record(record)
    assert score["domain_correct"] is False
    assert score["operational_success"] is False


def test_multidomain_direct_uses_one_typed_decision():
    client = FakeClient(
        [
            {
                "domain": "I-TROPHYTS",
                "decision": "execute",
                "action": "ExerciseInformation",
                "parameters": {"exercise_name": "Ponte", "step_number": 2},
                "reason": "clear rehabilitation request",
                "clarification": "",
            }
        ]
    )
    case = {
        "id": "rehab-direct",
        "category": "complete_valid",
        "user_input": "Qual è il secondo passaggio del ponte?",
        "memory": [],
        "expected": {
            "domain": "I-TROPHYTS",
            "decision": "execute",
            "action": "ExerciseInformation",
            "parameters": {
                "exercise_name": "ponte",
                "user_name": "",
                "step_number": 2,
                "giorno": "",
            },
        },
    }
    record = run_multidomain_direct(
        client,
        case,
        repeat=0,
        temperature=0.0,
        structured_interface="native_tools",
    )
    assert record["logical_llm_stages"] == 1
    assert score_controller_record(record)["operational_success"] is True
