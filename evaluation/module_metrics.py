"""Aggregation for the manuscript-compatible isolated module benchmark."""

from __future__ import annotations

import csv
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path
from typing import Any, Iterable


def percentile(values: list[float], quantile: float) -> float:
    """Return a linearly interpolated quantile, matching the reviewer metrics."""
    if not values:
        return math.nan
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * quantile
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def _mean(values: list[float]) -> float | None:
    return statistics.mean(values) if values else None


def _rate(numerator: int, denominator: int) -> float | None:
    return numerator / denominator if denominator else None


def _aggregate_group(records: list[dict[str, Any]]) -> dict[str, Any]:
    api_latency = [float(record["api_latency_seconds"]) for record in records]
    provider_latency = [
        float(record["provider_total_time_seconds"])
        for record in records
        if record.get("provider_total_time_seconds") is not None
    ]
    wall_latency = [float(record["wall_latency_seconds"]) for record in records]
    prompt_tokens = [int(record["prompt_tokens"]) for record in records]
    completion_tokens = [int(record["completion_tokens"]) for record in records]
    total_tokens = [int(record["total_tokens"]) for record in records]
    attempts = [int(record["attempts"]) for record in records]
    result: dict[str, Any] = {
        "n": len(records),
        "api_latency_mean_seconds": _mean(api_latency),
        "api_latency_p50_seconds": percentile(api_latency, 0.50),
        "api_latency_p95_seconds": percentile(api_latency, 0.95),
        "provider_total_time_mean_seconds": _mean(provider_latency),
        "provider_total_time_p50_seconds": (
            percentile(provider_latency, 0.50) if provider_latency else None
        ),
        "provider_total_time_p95_seconds": (
            percentile(provider_latency, 0.95) if provider_latency else None
        ),
        "wall_latency_mean_seconds": _mean(wall_latency),
        "wall_latency_p50_seconds": percentile(wall_latency, 0.50),
        "wall_latency_p95_seconds": percentile(wall_latency, 0.95),
        "total_tokens_mean": _mean(total_tokens),
        "prompt_tokens_p50": percentile(prompt_tokens, 0.50),
        "completion_tokens_p50": percentile(completion_tokens, 0.50),
        "total_tokens_p50": percentile(total_tokens, 0.50),
        "total_tokens_p95": percentile(total_tokens, 0.95),
        "total_tokens_sum": sum(total_tokens),
        "attempts_mean": _mean(attempts),
        "failure_rate": _rate(
            sum(bool(record["failure"]) for record in records), len(records)
        ),
    }

    module = records[0]["module"]
    valid = [record for record in records if not record["failure"]]
    if module == "scope":
        result["domain_selection_accuracy"] = _rate(
            sum(bool(record["scores"]["domain_correct"]) for record in valid),
            len(records),
        )
    elif module == "intent":
        result["task_selection_accuracy"] = _rate(
            sum(bool(record["scores"]["task_correct"]) for record in valid),
            len(records),
        )
        result["parameter_exact_match"] = _rate(
            sum(bool(record["scores"]["parameters_exact"]) for record in valid),
            len(records),
        )
        parameter_f1 = [
            float(record["scores"]["parameter_f1"])
            for record in valid
            if record["scores"]["parameter_f1"] is not None
        ]
        result["parameter_extraction_f1"] = _mean(parameter_f1)
    elif module == "inner":
        result["execution_readiness_accuracy"] = _rate(
            sum(bool(record["scores"]["readiness_correct"]) for record in valid),
            len(records),
        )
    elif module == "query":
        valid_queries = sum(int(record["scores"]["valid_queries"]) for record in valid)
        generated_queries = sum(
            int(record["scores"]["generated_query_count"]) for record in valid
        )
        result["query_validity_rate"] = _rate(valid_queries, generated_queries)
        overlap = [float(record["scores"]["result_overlap"]) for record in valid]
        result["result_overlap"] = _mean(overlap)
        result["query_case_success"] = _rate(
            sum(bool(record["scores"]["all_queries_valid"]) for record in valid),
            len(records),
        )
    bert_f1 = [
        float(record["scores"]["bert_f1"])
        for record in valid
        if record["scores"].get("bert_f1") is not None
    ]
    if bert_f1:
        result["bert_f1_mean"] = _mean(bert_f1)
    return result


def aggregate_module_records(records: Iterable[dict[str, Any]]) -> list[dict[str, Any]]:
    """Aggregate records by module and manuscript configuration."""
    grouped: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
    for record in records:
        grouped[(record["module"], record["configuration"])].append(record)
    rows: list[dict[str, Any]] = []
    for (module, configuration), group in sorted(grouped.items()):
        row = {"module": module, "configuration": configuration}
        row.update(_aggregate_group(group))
        rows.append(row)
    return rows


def write_module_summary(
    records: list[dict[str, Any]], output_dir: Path
) -> list[dict[str, Any]]:
    """Write JSON and CSV summaries and return the aggregate rows."""
    rows = aggregate_module_records(records)
    (output_dir / "summary.json").write_text(
        json.dumps(rows, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    fieldnames = sorted({field for row in rows for field in row})
    with (output_dir / "summary.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    return rows
