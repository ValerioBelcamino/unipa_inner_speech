"""Deterministic, alias-invariant metrics for the grounding benchmark."""

from __future__ import annotations

import csv
import json
import math
import statistics
import unicodedata
from collections import defaultdict
from pathlib import Path
from typing import Any, Iterable

from .metrics import score_controller_record


FIELD_ALIASES = {
    "dish": "dish",
    "piatto": "dish",
    "nome_piatto": "dish",
    "user": "user",
    "utente": "user",
    "nome_utente": "user",
    "calories": "calories",
    "calorie": "calories",
    "proteins": "proteins",
    "proteine": "proteins",
    "carbs": "carbs",
    "carboidrati": "carbs",
    "fats": "fats",
    "grassi": "fats",
    "ingredients": "ingredients",
    "ingredienti": "ingredients",
    "allergens": "allergens",
    "allergeni": "allergens",
    "dish_allergens": "dish_allergens",
    "allergeni_piatto": "dish_allergens",
    "user_allergens": "user_allergens",
    "user_allergies": "user_allergens",
    "allergie_utente": "user_allergens",
    "risks": "risks",
    "risks_for_user": "risks",
    "rischi": "risks",
    "compatible": "compatible",
    "compatibile": "compatible",
    "already_assigned": "already_assigned",
    "gia_assegnato": "already_assigned",
}


def _plain_text(value: str) -> str:
    value = unicodedata.normalize("NFKD", value.strip().casefold())
    return "".join(char for char in value if not unicodedata.combining(char))


def normalize_value(value: Any) -> Any:
    if isinstance(value, str):
        stripped = value.strip()
        try:
            decoded = json.loads(stripped)
        except (json.JSONDecodeError, TypeError):
            return _plain_text(stripped)
        if not isinstance(decoded, str):
            return normalize_value(decoded)
        return _plain_text(decoded)
    if isinstance(value, dict):
        return {
            str(key): normalize_value(item)
            for key, item in sorted(value.items())
        }
    if isinstance(value, (list, tuple, set)):
        normalized = [normalize_value(item) for item in value]
        return sorted(normalized, key=_value_key)
    if isinstance(value, float) and value.is_integer():
        return int(value)
    return value


def _value_key(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"))


def normalize_field(value: Any) -> str:
    field = _plain_text(str(value)).replace(" ", "_")
    return FIELD_ALIASES.get(field, field)


def normalize_record(record: dict[str, Any]) -> dict[str, Any]:
    normalized: dict[str, Any] = {}
    for field, value in record.items():
        canonical_field = normalize_field(field)
        canonical_value = normalize_value(value)
        if canonical_field not in normalized:
            normalized[canonical_field] = canonical_value
    # The submitted DishInfo examples retrieve the risk intersection rather
    # than an explicit boolean. Compatibility is deterministically implied.
    if "compatible" not in normalized and "risks" in normalized:
        risks = normalized["risks"]
        if isinstance(risks, list):
            normalized["compatible"] = len(risks) == 0
    return normalized


def evidence_records(value: Any) -> list[dict[str, Any]]:
    """Flatten per-query outputs while keeping only returned record objects."""
    records: list[dict[str, Any]] = []

    def visit(item: Any) -> None:
        if isinstance(item, dict):
            records.append(normalize_record(item))
        elif isinstance(item, (list, tuple)):
            for child in item:
                visit(child)

    visit(value)
    return records


def query_errors(value: Any) -> list[str]:
    errors: list[str] = []

    def visit(item: Any) -> None:
        if isinstance(item, str) and item.startswith("ERROR::"):
            errors.append(item)
        elif isinstance(item, (list, tuple)):
            for child in item:
                visit(child)

    visit(value)
    return errors


def _entity(record: dict[str, Any]) -> str:
    value = record.get("dish", record.get("user", ""))
    return str(normalize_value(value)) if value not in (None, "") else ""


def _fact_values(value: Any) -> list[str]:
    """Represent collections atomically so COLLECT and row-wise queries are equal."""
    normalized = normalize_value(value)
    if isinstance(normalized, list) and normalized:
        return [_value_key(item) for item in normalized]
    return [_value_key(normalized)]


def fact_set(records: list[dict[str, Any]], fields: Iterable[str]) -> set[tuple[str, str, str]]:
    requested = {normalize_field(field) for field in fields}
    facts: set[tuple[str, str, str]] = set()
    for record in records:
        entity = _entity(record)
        for field in requested:
            if field in record:
                for value in _fact_values(record[field]):
                    facts.add((entity, field, value))
    return facts


def claim_set(claims: Any) -> set[tuple[str, str, str]]:
    facts: set[tuple[str, str, str]] = set()
    if not isinstance(claims, list):
        return facts
    for claim in claims:
        if not isinstance(claim, dict) or "field" not in claim or "value_json" not in claim:
            continue
        entity = str(normalize_value(claim.get("entity", "")))
        field = normalize_field(claim["field"])
        value = normalize_value(claim["value_json"])
        for fact_value in _fact_values(value):
            facts.add((entity, field, fact_value))
    return facts


def _fact_matches(candidate: tuple[str, str, str], reference: tuple[str, str, str]) -> bool:
    candidate_entity, candidate_field, candidate_value = candidate
    reference_entity, reference_field, reference_value = reference
    return (
        candidate_field == reference_field
        and candidate_value == reference_value
        and (
            not candidate_entity
            or not reference_entity
            or candidate_entity == reference_entity
        )
    )


def _matched_count(
    candidates: set[tuple[str, str, str]], references: set[tuple[str, str, str]]
) -> int:
    return sum(any(_fact_matches(candidate, ref) for ref in references) for candidate in candidates)


def _reference_match_count(
    references: set[tuple[str, str, str]], candidates: set[tuple[str, str, str]]
) -> int:
    return sum(any(_fact_matches(candidate, ref) for candidate in candidates) for ref in references)


def _ratio(numerator: int, denominator: int) -> float:
    return numerator / denominator if denominator else 1.0


def score_grounding_record(record: dict[str, Any]) -> dict[str, Any]:
    expected_evidence = evidence_records(record["reference"]["expected_evidence"])
    generated_evidence = evidence_records(record["prediction"].get("query_results", []))
    targets = record["reference"].get("answer_targets", [])
    expected_facts = fact_set(expected_evidence, targets)
    generated_facts = fact_set(generated_evidence, targets)
    retrieved_expected = _reference_match_count(expected_facts, generated_facts)
    correct_generated = _matched_count(generated_facts, expected_facts)

    expected_entities = {
        record["dish"] for record in expected_evidence if record.get("dish")
    }
    generated_entities = {
        record["dish"] for record in generated_evidence if record.get("dish")
    }
    entity_matches = len(expected_entities & generated_entities)

    answer = record["prediction"].get("answer", {})
    answer_claims = claim_set(answer.get("claims", [])) if isinstance(answer, dict) else set()
    all_evidence_fields = set(FIELD_ALIASES.values())
    support_facts = fact_set(generated_evidence, all_evidence_fields)
    supported_claims = _matched_count(answer_claims, support_facts)
    correct_answer_facts = _reference_match_count(expected_facts, answer_claims)
    expect_abstention = bool(record["reference"].get("expect_abstention"))
    predicted_abstention = bool(answer.get("abstain", False)) if isinstance(answer, dict) else False
    answer_available = bool(record["prediction"].get("answer_attempted", False))

    query_list = record["prediction"].get("queries", [])
    errors = query_errors(record["prediction"].get("query_results", []))
    query_valid = bool(query_list) and not errors
    evidence_empty = not generated_evidence
    retrieval_recall = _ratio(retrieved_expected, len(expected_facts))
    retrieval_precision = _ratio(correct_generated, len(generated_facts))
    entity_recall = _ratio(entity_matches, len(expected_entities))
    entity_precision = _ratio(entity_matches, len(generated_entities))
    answer_recall = _ratio(correct_answer_facts, len(expected_facts))
    supported_claim_rate = _ratio(supported_claims, len(answer_claims))

    controller_score = score_controller_record(
        {
            "expected": record["expected"],
            "prediction": record["prediction"]["controller"],
            "structured_output_failure": record["prediction"].get(
                "controller_structured_output_failure", False
            ),
        }
    )
    if record["category"] == "substitution":
        answer_coverage = correct_answer_facts >= 1
    else:
        answer_coverage = correct_answer_facts == len(expected_facts)
    abstention_correct = answer_available and predicted_abstention == expect_abstention
    retrieval_correct = (
        query_valid
        and retrieval_recall == 1.0
        and retrieval_precision == 1.0
        and entity_recall == 1.0
        and entity_precision == 1.0
        and (evidence_empty == expect_abstention)
    )
    answer_correct = (
        answer_available
        and abstention_correct
        and supported_claim_rate == 1.0
        and (expect_abstention or answer_coverage)
        and (not expect_abstention or not answer_claims)
    )
    return {
        "decision_correct": controller_score["decision_correct"],
        "action_correct": controller_score["action_correct"],
        "parameters_exact": controller_score["parameters_exact"],
        "query_valid": query_valid,
        "retrieval_correct": retrieval_correct,
        "retrieval_target_recall": retrieval_recall,
        "retrieval_target_precision": retrieval_precision,
        "entity_recall": entity_recall,
        "entity_precision": entity_precision,
        "answer_attempted": answer_available,
        "abstention_correct": abstention_correct,
        "answer_target_recall": answer_recall,
        "supported_claim_rate": supported_claim_rate,
        "unsupported_claims": len(answer_claims) - supported_claims,
        "answer_claims": len(answer_claims),
        "grounded_task_success": (
            controller_score["decision_correct"]
            and controller_score["action_correct"]
            and retrieval_correct
            and answer_correct
            and not bool(record.get("structured_output_failure", False))
        ),
        "expected_target_facts": len(expected_facts),
        "retrieved_expected_facts": retrieved_expected,
        "generated_target_facts": len(generated_facts),
        "correct_generated_target_facts": correct_generated,
        "answer_correct_target_facts": correct_answer_facts,
    }


def _percentile(values: list[float], quantile: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * quantile
    low = math.floor(position)
    high = math.ceil(position)
    if low == high:
        return ordered[low]
    return ordered[low] + (ordered[high] - ordered[low]) * (position - low)


def _aggregate_group(records: list[dict[str, Any]]) -> dict[str, Any]:
    scores = [score_grounding_record(record) for record in records]
    n = len(records)
    expected_facts = sum(score["expected_target_facts"] for score in scores)
    retrieved = sum(score["retrieved_expected_facts"] for score in scores)
    generated = sum(score["generated_target_facts"] for score in scores)
    correct_generated = sum(
        score["correct_generated_target_facts"] for score in scores
    )
    answer_claims = sum(score["answer_claims"] for score in scores)
    unsupported = sum(score["unsupported_claims"] for score in scores)
    precision = _ratio(correct_generated, generated)
    recall = _ratio(retrieved, expected_facts)
    return {
        "n": n,
        "decision_accuracy": sum(score["decision_correct"] for score in scores) / n,
        "action_accuracy": sum(score["action_correct"] for score in scores) / n,
        "parameter_exact_match": sum(score["parameters_exact"] for score in scores) / n,
        "query_validity_rate": sum(score["query_valid"] for score in scores) / n,
        "exact_retrieval_success": sum(score["retrieval_correct"] for score in scores) / n,
        "evidence_micro_precision": precision,
        "evidence_micro_recall": recall,
        "evidence_micro_f1": (
            2 * precision * recall / (precision + recall) if precision + recall else 0.0
        ),
        "answer_target_recall": _ratio(
            sum(score["answer_correct_target_facts"] for score in scores),
            expected_facts,
        ),
        "evidence_supported_claim_rate": _ratio(
            answer_claims - unsupported, answer_claims
        ),
        "unsupported_claim_count": unsupported,
        "abstention_accuracy": sum(score["abstention_correct"] for score in scores) / n,
        "grounded_task_success": sum(score["grounded_task_success"] for score in scores) / n,
        "structured_output_failure_rate": sum(
            bool(record.get("structured_output_failure", False)) for record in records
        )
        / n,
        "wall_latency_p50_seconds": _percentile(
            [float(record["wall_latency_seconds"]) for record in records], 0.5
        ),
        "wall_latency_p95_seconds": _percentile(
            [float(record["wall_latency_seconds"]) for record in records], 0.95
        ),
        "total_tokens_median": statistics.median(
            int(record["total_tokens"]) for record in records
        ),
        "llm_calls_mean": statistics.mean(int(record["llm_calls"]) for record in records),
    }


def aggregate_grounding(records: Iterable[dict[str, Any]]) -> list[dict[str, Any]]:
    grouped: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
    for record in records:
        grouped[(record["architecture"], "ALL")].append(record)
        grouped[(record["architecture"], record["category"])].append(record)
    rows: list[dict[str, Any]] = []
    for (architecture, category), group in sorted(grouped.items()):
        rows.append(
            {
                "architecture": architecture,
                "category": category,
                **_aggregate_group(group),
            }
        )
    return rows


def write_grounding_summary(
    records: list[dict[str, Any]], output_dir: Path
) -> list[dict[str, Any]]:
    rows = aggregate_grounding(records)
    (output_dir / "summary.json").write_text(
        json.dumps(rows, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    fieldnames = sorted({field for row in rows for field in row})
    with (output_dir / "summary.csv").open(
        "w", encoding="utf-8", newline=""
    ) as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    return rows
