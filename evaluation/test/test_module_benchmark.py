from __future__ import annotations

from evaluation.module_benchmark import (
    MODULES,
    _selected_scope_configs,
    load_module_cases,
    query_result_overlap,
)
from evaluation.module_metrics import aggregate_module_records, percentile


def test_manuscript_module_case_counts():
    cases = load_module_cases(list(MODULES), _selected_scope_configs([]))
    counts = {}
    for case in cases:
        key = (case["module"], case["configuration"])
        counts[key] = counts.get(key, 0) + 1

    assert counts == {
        ("scope", "MHS"): 24,
        ("scope", "HSR"): 24,
        ("scope", "ATT"): 24,
        ("scope", "TEF"): 24,
        ("scope", "TER"): 24,
        ("scope", "ALL"): 66,
        ("intent", "ADVISOR"): 51,
        ("inner", "ADVISOR"): 40,
        ("query", "ADVISOR-legacy"): 30,
        ("outer", "ADVISOR"): 31,
    }


def test_query_overlap_accepts_legacy_subset_match_and_penalizes_errors():
    reference = [[{"dish": "tiramisu", "calories": 420}]]
    generated = [[{"dish": "tiramisu"}]]
    assert query_result_overlap(reference, generated) == 1.0
    assert query_result_overlap(reference, ["ERROR::invalid Cypher"]) == 0.0


def test_module_summary_counts_failures_as_incorrect():
    base = {
        "module": "inner",
        "configuration": "ADVISOR",
        "api_latency_seconds": 1.0,
        "wall_latency_seconds": 1.25,
        "prompt_tokens": 10,
        "completion_tokens": 5,
        "total_tokens": 15,
        "attempts": 1,
    }
    rows = aggregate_module_records(
        [
            {**base, "failure": False, "scores": {"readiness_correct": True}},
            {
                **base,
                "failure": True,
                "scores": {"readiness_correct": False},
                "api_latency_seconds": 3.0,
            },
        ]
    )
    assert rows[0]["execution_readiness_accuracy"] == 0.5
    assert rows[0]["failure_rate"] == 0.5
    assert rows[0]["api_latency_mean_seconds"] == 2.0
    assert percentile([1.0, 3.0], 0.5) == 2.0
