"""Reference execution and validation for the grounding benchmark.

The gold evidence is never produced by an LLM.  Every reference kind maps to a
fixed, parameterized, read-only Cypher query and is checked against the exact
Neo4j snapshot before a model run is allowed to start.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import unicodedata
from pathlib import Path
from typing import Any


DEFAULT_DATASET = Path(__file__).parent / "datasets" / "grounding_v1.json"
REPO_ROOT = Path(__file__).resolve().parents[1]
ALLOWED_DISH_FIELDS = {"calories", "proteins", "carbs", "fats"}
PROPERTY_TO_PARAMETER = {
    "calories": "calorie",
    "proteins": "proteine",
    "carbs": "carboidrati",
    "fats": "grassi",
}
EXPECTED_TARGETS = {
    "dish_composition": {"ingredients", "allergens"},
    "compatibility": {"compatible", "risks"},
    "substitution": {"dish"},
}


DISH_COMPOSITION_QUERY = """
MATCH (d:Dish {name: $dish})
OPTIONAL MATCH (d)-[:CONTAINS]->(i:Ingredient)
OPTIONAL MATCH (i)-[:CONTAINS]->(a:Allergen)
RETURN d.name AS dish,
       apoc.coll.sort(collect(DISTINCT i.name)) AS ingredients,
       apoc.coll.sort(collect(DISTINCT a.name)) AS allergens
"""


COMPATIBILITY_QUERY = """
MATCH (p:Person {name: $user}), (d:Dish {name: $dish})
OPTIONAL MATCH (p)-[:IS_ALLERGIC_TO]->(ua:Allergen)
WITH p, d, apoc.coll.sort(collect(DISTINCT ua.name)) AS user_allergens
OPTIONAL MATCH (d)-[:CONTAINS]->(:Ingredient)-[:CONTAINS]->(da:Allergen)
WITH p, d, user_allergens,
     apoc.coll.sort(collect(DISTINCT da.name)) AS dish_allergens
WITH p, d, user_allergens, dish_allergens,
     apoc.coll.sort(
       apoc.coll.intersection(user_allergens, dish_allergens)
     ) AS risks
RETURN p.name AS user, d.name AS dish, user_allergens, dish_allergens, risks,
       size(risks) = 0 AS compatible
"""


SUBSTITUTION_QUERY = """
MATCH (p:Person {name: $user})
MATCH (d:Dish)
WHERE all(wanted IN $include WHERE EXISTS {
  MATCH (d)-[:CONTAINS]->(wi:Ingredient) WHERE wi.name = wanted
})
AND none(blocked IN $exclude WHERE EXISTS {
  MATCH (d)-[:CONTAINS]->(bi:Ingredient) WHERE bi.name = blocked
})
AND none(blocked IN $exclude WHERE EXISTS {
  MATCH (d)-[:CONTAINS]->(:Ingredient)-[:CONTAINS]->(ba:Allergen)
  WHERE ba.name = blocked
})
AND (
  size($exclusive) = 0 OR (
    all(required IN $exclusive WHERE EXISTS {
      MATCH (d)-[:CONTAINS]->(ei:Ingredient) WHERE ei.name = required
    })
    AND NOT EXISTS {
      MATCH (d)-[:CONTAINS]->(other:Ingredient)
      WHERE NOT other.name IN $exclusive
    }
  )
)
AND NOT EXISTS {
  MATCH (p)-[:IS_ALLERGIC_TO]->(a:Allergen)
        <-[:CONTAINS]-(:Ingredient)<-[:CONTAINS]-(d)
}
OPTIONAL MATCH (p)-[meal:SHOULD_EAT {day: $day, meal: $meal}]->(d)
RETURN d.name AS dish, d.calories AS calories, d.proteins AS proteins,
       d.carbs AS carbs, d.fats AS fats,
       meal IS NOT NULL AS already_assigned
ORDER BY dish
"""


def _json_key(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"))


def canonicalize(value: Any) -> Any:
    """Normalize Neo4j values and unordered result/list collections."""
    if isinstance(value, dict):
        return {str(key): canonicalize(item) for key, item in sorted(value.items())}
    if isinstance(value, (list, tuple, set)):
        normalized = [canonicalize(item) for item in value]
        return sorted(normalized, key=_json_key)
    if hasattr(value, "items"):
        return canonicalize(dict(value.items()))
    return value


def load_dataset(path: Path = DEFAULT_DATASET) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _normalize_prompt(value: str) -> str:
    value = unicodedata.normalize("NFKD", value.casefold())
    value = "".join(char for char in value if not unicodedata.combining(char))
    return " ".join(re.sub(r"[^a-z0-9]+", " ", value).split())


def _prompt_values(value: Any, source: Path) -> list[tuple[str, Path]]:
    found: list[tuple[str, Path]] = []
    if isinstance(value, dict):
        for key, item in value.items():
            if key in {"question", "question_en", "user_input"} and isinstance(
                item, str
            ):
                found.append((item, source))
            else:
                found.extend(_prompt_values(item, source))
    elif isinstance(value, list):
        for item in value:
            found.extend(_prompt_values(item, source))
    return found


def prior_prompt_paths(dataset_path: Path = DEFAULT_DATASET) -> list[Path]:
    """Return existing few-shot, test, and evaluation prompt sources."""
    candidates: set[Path] = set()
    for pattern in (
        "**/few_shot_examples/*.json",
        "**/fewshot_examples/*.json",
        "**/test/*.json",
        "evaluation/datasets/*.json",
    ):
        candidates.update(REPO_ROOT.glob(pattern))
    resolved_dataset = dataset_path.resolve()
    return sorted(
        path
        for path in candidates
        if path.is_file() and path.resolve() != resolved_dataset
    )


def audit_prompt_overlap(
    dataset: dict[str, Any], source_paths: list[Path]
) -> dict[str, Any]:
    """Detect normalized exact prompt copies in pre-existing benchmark material."""
    prior: dict[str, list[dict[str, str]]] = {}
    source_prompt_count = 0
    parse_errors: list[str] = []
    for path in source_paths:
        try:
            content = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            parse_errors.append(f"{path}: {exc}")
            continue
        for prompt, source in _prompt_values(content, path):
            source_prompt_count += 1
            normalized = _normalize_prompt(prompt)
            if normalized:
                try:
                    source_label = str(source.relative_to(REPO_ROOT))
                except ValueError:
                    source_label = str(source)
                prior.setdefault(normalized, []).append(
                    {"prompt": prompt, "source": source_label}
                )

    overlaps: list[dict[str, Any]] = []
    for case in dataset.get("cases", []):
        normalized = _normalize_prompt(str(case.get("user_input", "")))
        if normalized in prior:
            overlaps.append(
                {
                    "case_id": case.get("id"),
                    "user_input": case.get("user_input"),
                    "matches": prior[normalized],
                }
            )
    return {
        "source_files": len(source_paths),
        "source_prompts": source_prompt_count,
        "normalized_unique_source_prompts": len(prior),
        "normalized_exact_overlaps": overlaps,
        "parse_errors": parse_errors,
        "valid": not overlaps and not parse_errors,
    }


class ReferenceDatabase:
    """Read-only access to the frozen Neo4j reference graph."""

    def __init__(self, uri: str, username: str, password: str) -> None:
        try:
            from neo4j import GraphDatabase
        except ImportError as exc:  # pragma: no cover - installation failure.
            raise RuntimeError("install the neo4j Python package") from exc
        self.driver = GraphDatabase.driver(uri, auth=(username, password))
        self.driver.verify_connectivity()

    def close(self) -> None:
        self.driver.close()

    def _run(self, query: str, parameters: dict[str, Any]) -> list[dict[str, Any]]:
        with self.driver.session() as session:
            records = [dict(record) for record in session.run(query, parameters)]
        return canonicalize(records)

    def execute_reference(self, reference: dict[str, Any]) -> list[dict[str, Any]]:
        kind = str(reference["kind"])
        arguments = dict(reference["arguments"])
        if kind == "dish_properties":
            fields = list(arguments.pop("fields"))
            if not fields or not set(fields) <= ALLOWED_DISH_FIELDS:
                raise ValueError(f"invalid dish property fields: {fields}")
            projection = ", ".join(
                ["d.name AS dish", *(f"d.{field} AS {field}" for field in fields)]
            )
            query = f"MATCH (d:Dish {{name: $dish}}) RETURN {projection}"
        elif kind == "dish_composition":
            query = DISH_COMPOSITION_QUERY
        elif kind == "compatibility":
            query = COMPATIBILITY_QUERY
        elif kind == "substitution":
            query = SUBSTITUTION_QUERY
        else:
            raise ValueError(f"unknown reference kind: {kind}")
        return self._run(query, arguments)

    def fingerprint(self) -> str:
        """Hash all nodes and relationships, including plan edge properties."""
        nodes = self._run(
            "MATCH (n) RETURN labels(n) AS labels, properties(n) AS properties",
            {},
        )
        relationships = self._run(
            """
            MATCH (start)-[r]->(end)
            RETURN labels(start) AS start_labels,
                   properties(start) AS start_properties,
                   type(r) AS relationship_type,
                   properties(r) AS relationship_properties,
                   labels(end) AS end_labels,
                   properties(end) AS end_properties
            """,
            {},
        )
        payload = _json_key({"nodes": nodes, "relationships": relationships})
        return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def _expected_parameter(reference: dict[str, Any], name: str) -> Any:
    return reference["arguments"].get(name)


def validate_case_alignment(case: dict[str, Any]) -> list[str]:
    """Check that a case's controller gold and DB reference describe one task."""
    errors: list[str] = []
    expected = case.get("expected", {})
    action = expected.get("action")
    parameters = expected.get("parameters", {})
    reference = case.get("reference", {})
    kind = reference.get("kind")
    if expected.get("decision") != "execute":
        errors.append("grounding cases must reach an execute-or-query decision")
    if kind in {"dish_properties", "dish_composition"}:
        if action != "DishInfo":
            errors.append(f"{kind} requires DishInfo")
        if parameters.get("nome_piatto") != _expected_parameter(reference, "dish"):
            errors.append("nome_piatto does not match reference dish")
        if kind == "dish_properties":
            fields = reference.get("arguments", {}).get("fields", [])
            requested = parameters.get("controllo_ingredienti", [])
            represented = {PROPERTY_TO_PARAMETER[field] for field in fields}
            if requested and set(requested) != represented:
                errors.append("requested nutrients do not match reference fields")
    elif kind == "compatibility":
        if action != "DishInfo":
            errors.append("compatibility requires DishInfo")
        if parameters.get("nome_piatto") != _expected_parameter(reference, "dish"):
            errors.append("nome_piatto does not match reference dish")
        if parameters.get("nome_utente") != _expected_parameter(reference, "user"):
            errors.append("nome_utente does not match reference user")
    elif kind == "substitution":
        if action != "SubstituteDish":
            errors.append("substitution reference requires SubstituteDish")
        mapping = {
            "nome_utente": "user",
            "ingredienti_preferiti": "include",
            "ingredienti_rimossi": "exclude",
            "ingredienti_obbligatori_esclusivi": "exclusive",
            "giorno": "day",
            "pasto": "meal",
        }
        for parameter_name, argument_name in mapping.items():
            if parameters.get(parameter_name) != _expected_parameter(
                reference, argument_name
            ):
                errors.append(
                    f"{parameter_name} does not match reference {argument_name}"
                )
        if parameters.get("ha_piano_settimanale") != case.get(
            "tool_context", {}
        ).get("ha_piano_settimanale"):
            errors.append("weekly-plan parameter does not match tool context")
        include = set(reference.get("arguments", {}).get("include", []))
        exclude = set(reference.get("arguments", {}).get("exclude", []))
        exclusive = set(reference.get("arguments", {}).get("exclusive", []))
        if include & exclude or exclusive & exclude:
            errors.append("substitution constraints contradict one another")
    else:
        errors.append(f"unknown reference kind: {kind}")
    expected_evidence = reference.get("expected_evidence")
    if not isinstance(expected_evidence, list):
        errors.append("expected_evidence must be a list")
    elif bool(case.get("expect_abstention")) != (len(expected_evidence) == 0):
        errors.append("expect_abstention must equal whether gold evidence is empty")

    targets = set(case.get("answer_targets", []))
    if kind == "dish_properties":
        expected_targets = set(reference.get("arguments", {}).get("fields", []))
    else:
        expected_targets = EXPECTED_TARGETS.get(str(kind), set())
    if case.get("expect_abstention"):
        expected_targets = set()
    if targets != expected_targets:
        errors.append(
            f"answer_targets {sorted(targets)} do not match task targets "
            f"{sorted(expected_targets)}"
        )
    for row in expected_evidence if isinstance(expected_evidence, list) else []:
        if not isinstance(row, dict):
            errors.append("every evidence row must be an object")
            continue
        missing_targets = targets - set(row)
        if missing_targets:
            errors.append(
                f"evidence row is missing answer targets {sorted(missing_targets)}"
            )
    return errors


def validate_dataset(
    dataset: dict[str, Any], database: ReferenceDatabase
) -> dict[str, Any]:
    """Fail closed unless every frozen reference matches the exact database."""
    errors: list[str] = []
    cases = dataset.get("cases", [])
    case_ids = [case.get("id") for case in cases]
    if len(case_ids) != len(set(case_ids)):
        errors.append("case ids are not unique")
    questions = [case.get("user_input") for case in cases]
    if len(questions) != len(set(questions)):
        errors.append("user inputs are not unique")

    actual_fingerprint = database.fingerprint()
    expected_fingerprint = dataset.get("database", {}).get("fingerprint_sha256")
    if actual_fingerprint != expected_fingerprint:
        errors.append(
            "database fingerprint mismatch: "
            f"expected {expected_fingerprint}, got {actual_fingerprint}"
        )

    category_counts: dict[str, int] = {}
    evidence_rows = 0
    empty_evidence_cases = 0
    for case in cases:
        case_id = str(case.get("id"))
        category = str(case.get("category"))
        category_counts[category] = category_counts.get(category, 0) + 1
        for error in validate_case_alignment(case):
            errors.append(f"{case_id}: {error}")
        try:
            actual = database.execute_reference(case["reference"])
        except Exception as exc:  # pragma: no cover - integration diagnostic.
            errors.append(f"{case_id}: reference execution failed: {exc}")
            continue
        expected = canonicalize(case["reference"].get("expected_evidence", []))
        if actual != expected:
            errors.append(
                f"{case_id}: evidence mismatch; expected {_json_key(expected)}, "
                f"got {_json_key(actual)}"
            )
        evidence_rows += len(actual)
        empty_evidence_cases += int(not actual)

    report = {
        "dataset": dataset.get("version"),
        "cases": len(cases),
        "category_counts": dict(sorted(category_counts.items())),
        "database_fingerprint_sha256": actual_fingerprint,
        "reference_evidence_rows": evidence_rows,
        "empty_evidence_cases": empty_evidence_cases,
        "valid": not errors,
        "errors": errors,
    }
    if errors:
        raise ValueError(json.dumps(report, ensure_ascii=False, indent=2))
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, default=DEFAULT_DATASET)
    parser.add_argument("--neo4j-uri", default="bolt://localhost:19687")
    parser.add_argument("--neo4j-username", default="neo4j")
    parser.add_argument("--neo4j-password", default="password")
    parser.add_argument("--print-fingerprint", action="store_true")
    parser.add_argument(
        "--skip-prompt-overlap-audit",
        action="store_true",
        help="skip checking held-out prompts against existing few-shot/test JSON",
    )
    args = parser.parse_args()
    database = ReferenceDatabase(
        args.neo4j_uri, args.neo4j_username, args.neo4j_password
    )
    try:
        if args.print_fingerprint:
            print(database.fingerprint())
            return 0
        dataset = load_dataset(args.dataset)
        report = validate_dataset(dataset, database)
        if not args.skip_prompt_overlap_audit:
            overlap = audit_prompt_overlap(dataset, prior_prompt_paths(args.dataset))
            report["prompt_overlap_audit"] = overlap
            if not overlap["valid"]:
                raise ValueError(json.dumps(report, ensure_ascii=False, indent=2))
    finally:
        database.close()
    report["dataset_sha256"] = hashlib.sha256(args.dataset.read_bytes()).hexdigest()
    print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
