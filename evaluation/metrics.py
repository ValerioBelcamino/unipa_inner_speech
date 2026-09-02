"""Deterministic aggregation for reviewer benchmark JSONL records."""

from __future__ import annotations

import csv
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path
from typing import Any, Iterable

from .task_spec import normalize_action, normalize_decision, normalize_parameters


def _percentile(values: list[float], percentile: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * percentile
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def _rate(numerator: int, denominator: int) -> float | None:
    return numerator / denominator if denominator else None


def _parameter_items(parameters: dict[str, Any]) -> set[tuple[str, str]]:
    return {
        (key, json.dumps(value, ensure_ascii=False, sort_keys=True))
        for key, value in parameters.items()
    }


def score_controller_record(record: dict[str, Any]) -> dict[str, Any]:
    expected = record["expected"]
    predicted = record["prediction"]
    multidomain = "domain" in expected
    expected_action = (
        str(expected.get("action", ""))
        if multidomain
        else normalize_action(expected.get("action"))
    )
    predicted_action = (
        str(predicted.get("action", ""))
        if multidomain
        else normalize_action(predicted.get("action"))
    )
    expected_decision = normalize_decision(expected.get("decision"))
    predicted_decision = normalize_decision(predicted.get("decision"))
    if multidomain:
        expected_parameters = expected.get("parameters", {})
        predicted_parameters = predicted.get("parameters", {})
    else:
        expected_parameters = normalize_parameters(expected_action, expected.get("parameters", {}))
        predicted_parameters = normalize_parameters(predicted_action, predicted.get("parameters", {}))

    expected_items = _parameter_items(expected_parameters)
    predicted_items = _parameter_items(predicted_parameters)
    true_positive = len(expected_items & predicted_items)
    # A parser/API failure must never receive credit merely because the
    # controller's conservative fallback happens to match the gold label.
    valid_prediction = not bool(record.get("structured_output_failure", False))
    decision_correct = valid_prediction and expected_decision == predicted_decision
    action_correct = valid_prediction and expected_action == predicted_action
    domain_correct = valid_prediction and (
        not multidomain or expected.get("domain") == predicted.get("domain")
    )
    parameters_exact = valid_prediction and expected_parameters == predicted_parameters
    return {
        "decision_correct": decision_correct,
        "domain_correct": domain_correct,
        "action_correct": action_correct,
        "parameters_exact": parameters_exact,
        "strict_state_match": (
            decision_correct and domain_correct and action_correct and parameters_exact
        ),
        "operational_success": (
            decision_correct
            and domain_correct
            and action_correct
            and (expected_decision != "execute" or parameters_exact)
        ),
        "parameter_tp": true_positive,
        "parameter_predicted": len(predicted_items),
        "parameter_expected": len(expected_items),
        "premature_execution": expected_decision != "execute" and predicted_decision == "execute",
        "premature_execution_eligible": expected_decision != "execute",
        "unnecessary_clarification": expected_decision == "execute" and predicted_decision == "clarify",
        "unnecessary_clarification_eligible": expected_decision == "execute",
        "false_rejection": expected_decision != "reject" and predicted_decision == "reject",
        "false_rejection_eligible": expected_decision != "reject",
    }


def score_readiness_record(record: dict[str, Any]) -> dict[str, Any]:
    expected = bool(record["expected"]["can_proceed"])
    predicted = bool(record["prediction"]["can_proceed"])
    valid_prediction = not bool(record.get("structured_output_failure", False))
    return {
        "readiness_correct": valid_prediction and expected == predicted,
        "premature_proceed": not expected and predicted,
        "premature_proceed_eligible": not expected,
        "unnecessary_block": expected and not predicted,
        "unnecessary_block_eligible": expected,
    }


def _aggregate_group(records: list[dict[str, Any]], suite: str) -> dict[str, Any]:
    latency = [float(record["wall_latency_seconds"]) for record in records]
    llm_latency = [float(record["llm_latency_seconds"]) for record in records]
    total_tokens = [int(record["total_tokens"]) for record in records]
    calls = [int(record["llm_calls"]) for record in records]
    result: dict[str, Any] = {
        "n": len(records),
        "latency_p50_seconds": _percentile(latency, 0.50),
        "latency_p95_seconds": _percentile(latency, 0.95),
        "llm_latency_p50_seconds": _percentile(llm_latency, 0.50),
        "llm_latency_p95_seconds": _percentile(llm_latency, 0.95),
        "total_tokens_median": statistics.median(total_tokens) if total_tokens else math.nan,
        "llm_calls_mean": statistics.mean(calls) if calls else math.nan,
        "structured_output_failure_rate": _rate(
            sum(bool(record["structured_output_failure"]) for record in records), len(records)
        ),
    }

    if suite in {"controller", "multidomain"}:
        scores = [score_controller_record(record) for record in records]
        result.update(
            {
                "decision_accuracy": _rate(sum(s["decision_correct"] for s in scores), len(scores)),
                "action_accuracy": _rate(sum(s["action_correct"] for s in scores), len(scores)),
                "parameter_exact_match": _rate(sum(s["parameters_exact"] for s in scores), len(scores)),
                "strict_state_match": _rate(
                    sum(s["strict_state_match"] for s in scores), len(scores)
                ),
                "operational_task_success": _rate(
                    sum(s["operational_success"] for s in scores), len(scores)
                ),
            }
        )
        if suite == "multidomain":
            result["domain_accuracy"] = _rate(
                sum(s["domain_correct"] for s in scores), len(scores)
            )
        decision_recalls: list[float] = []
        for decision_name in ("execute", "clarify", "reject"):
            eligible = [
                (
                    normalize_decision(record["expected"].get("decision")),
                    normalize_decision(record["prediction"].get("decision")),
                    not bool(record.get("structured_output_failure", False)),
                )
                for record in records
                if normalize_decision(record["expected"].get("decision")) == decision_name
            ]
            recall = _rate(
                sum(valid and expected == predicted for expected, predicted, valid in eligible),
                len(eligible),
            )
            result[f"decision_recall_{decision_name}"] = recall
            if recall is not None:
                decision_recalls.append(recall)
        result["decision_macro_recall"] = (
            statistics.mean(decision_recalls) if decision_recalls else None
        )
        tp = sum(s["parameter_tp"] for s in scores)
        predicted = sum(s["parameter_predicted"] for s in scores)
        expected = sum(s["parameter_expected"] for s in scores)
        precision = _rate(tp, predicted) or 0.0
        recall = _rate(tp, expected) or 0.0
        result["parameter_micro_precision"] = precision
        result["parameter_micro_recall"] = recall
        result["parameter_micro_f1"] = (
            2 * precision * recall / (precision + recall) if precision + recall else 0.0
        )
        for name in ("premature_execution", "unnecessary_clarification", "false_rejection"):
            numerator = sum(s[name] for s in scores)
            denominator = sum(s[f"{name}_eligible"] for s in scores)
            result[f"{name}_count"] = numerator
            result[f"{name}_denominator"] = denominator
            result[f"{name}_rate"] = _rate(numerator, denominator)
    else:
        scores = [score_readiness_record(record) for record in records]
        result["readiness_accuracy"] = _rate(
            sum(s["readiness_correct"] for s in scores), len(scores)
        )
        proceed_records = [
            record for record in records if bool(record["expected"]["can_proceed"])
        ]
        block_records = [
            record for record in records if not bool(record["expected"]["can_proceed"])
        ]
        proceed_recall = _rate(
            sum(
                not bool(record.get("structured_output_failure", False))
                and bool(record["prediction"]["can_proceed"])
                for record in proceed_records
            ),
            len(proceed_records),
        )
        block_recall = _rate(
            sum(
                not bool(record.get("structured_output_failure", False))
                and not bool(record["prediction"]["can_proceed"])
                for record in block_records
            ),
            len(block_records),
        )
        result["proceed_recall"] = proceed_recall
        result["block_recall"] = block_recall
        recalls = [value for value in (proceed_recall, block_recall) if value is not None]
        result["readiness_balanced_accuracy"] = statistics.mean(recalls) if recalls else None
        for name in ("premature_proceed", "unnecessary_block"):
            numerator = sum(s[name] for s in scores)
            denominator = sum(s[f"{name}_eligible"] for s in scores)
            result[f"{name}_count"] = numerator
            result[f"{name}_denominator"] = denominator
            result[f"{name}_rate"] = _rate(numerator, denominator)
    return result


def aggregate(records: Iterable[dict[str, Any]], suite: str) -> list[dict[str, Any]]:
    """Aggregate overall and per-category metrics for each architecture."""
    grouped: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
    for record in records:
        grouped[(record["architecture"], "ALL")].append(record)
        grouped[(record["architecture"], record["category"])].append(record)
    rows: list[dict[str, Any]] = []
    for (architecture, category), group in sorted(grouped.items()):
        row = {"architecture": architecture, "category": category}
        row.update(_aggregate_group(group, suite))
        rows.append(row)
    return rows


def write_summary(records: list[dict[str, Any]], suite: str, output_dir: Path) -> list[dict[str, Any]]:
    rows = aggregate(records, suite)
    (output_dir / "summary.json").write_text(
        json.dumps(rows, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    fieldnames = sorted({key for row in rows for key in row})
    with (output_dir / "summary.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    return rows
