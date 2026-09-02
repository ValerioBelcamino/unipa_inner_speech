"""Reproduce the manuscript's isolated ADVISOR module benchmarks.

This runner avoids ROS 2 and LangSmith while preserving the submitted prompts,
native tool interfaces, datasets, and quantitative metrics. Every model response,
API latency, wall latency, retry count, and token count is stored locally.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import re
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

from dotenv import load_dotenv

from scenario_customization.scenario_customization.ADVISOR.AddToDatabase.intent import (
    AddToDatabase,
)
from scenario_customization.scenario_customization.ADVISOR.DishInfo.intent import (
    DishInfo,
)
from scenario_customization.scenario_customization.ADVISOR.SubstituteDish.intent import (
    SubstituteDish,
)
from scope_detection.scope_detection.domain_examples.ita import domain_descriptions

from .llm_client import CompletionTrace, JsonLLMClient, ProviderRateLimitError
from .module_metrics import write_module_summary
from .task_spec import OUT_OF_SCOPE, normalize_action, normalize_parameters


REPO_ROOT = Path(__file__).resolve().parents[1]
MODULES = ("scope", "intent", "inner", "query", "outer")
DATASETS = {
    "scope": REPO_ROOT / "scope_detection" / "test" / "examples_ADVISOR.json",
    "intent": REPO_ROOT / "intent_recognition" / "test" / "examples_ADVISOR.json",
    "inner": REPO_ROOT / "inner_speech" / "test" / "examples_ADVISOR.json",
    "query": REPO_ROOT / "query_generation" / "test" / "examples_ADVISOR.json",
    "outer": REPO_ROOT / "explainability" / "test" / "examples_ADVISOR.json",
}
SCOPE_CONFIG_PATH = (
    REPO_ROOT
    / "scope_detection"
    / "scope_detection"
    / "selected_domain_combinations.json"
)
SCENARIO_ROOT = (
    REPO_ROOT / "scenario_customization" / "scenario_customization" / "ADVISOR"
)
SCENARIO_DESCRIPTION = (
    (SCENARIO_ROOT / "scenario_description.txt").read_text(encoding="utf-8").strip()
)
SCOPE_CODES = {
    "exp_0-6-7": "MHS",
    "exp_6-7-9": "HSR",
    "exp_1-2-4": "ATT",
    "exp_4-5-10": "TEF",
    "exp_2-8-9": "TER",
    "exp_full": "ALL",
}
PAPER_TEMPERATURES = {
    "scope": 0.0,
    "intent": 0.0,
    "inner": 0.2,
    "query": 0.0,
    "outer": 0.4,
}

SUBMITTED_SCOPE_SYSTEM = """You are the Scope Detection module of the architecture and you have to select the most appropriate scenario given a user request.
You have at your disposal:
-The inner speech reasoning provided by the previous module
-The list of all the available scenarios together with their supported tasks
You have to select one of these scenario tools.
If the user prompt is not related to the tasks supported by any scenario you should not answer."""

SUBMITTED_INTENT_SYSTEM = """You are tasked with identifying the correct intent from a set of available tools and extracting only the parameters explicitly provided by the user.
You must not use external knowledge, assumptions, or inference to guess or complete missing information.
You will also receive a short term memory with additional information on past interactions.
If the user input is not relevant to any of the available tools, do not respond or assign an intent.
Only fill tool parameters when the necessary information is clearly and explicitly included in the user input.
Do not hallucinate.
Do not fill gaps, or rephrase missing data.
If a question seems to be correlated to the current topic, but it is too vague and doesn't directly refer to a tool, don't answer!
If a parameter is missing, ambiguous, or incomplete, leave it blank and do not attempt to infer or complete it.
Follow these constraints strictly to ensure reliability and factual accuracy in tool usage."""

INNER_SCHEMA = {
    "type": "object",
    "properties": {
        "inner_speech": {"type": "string", "description": "Il tuo ragionamento"},
        "can_proceed": {
            "type": "boolean",
            "description": "Se la richiesta dell'utente può essere accolta",
        },
    },
    "required": ["inner_speech", "can_proceed"],
}
INNER_TOOL_DESCRIPTION = (
    "Dato un prompt di un utente, l'azione ed i parametri estratti dal "
    "riconoscimento dell'intento devi elaborare un discorso interiore che "
    "spieghi se l'azione può essere portata a termine oppure no."
)
QUERY_TOOLS = {
    "AddToDatabase": {
        "name": "UserInsertionTool",
        "description": (
            "Inserts user details (calories, macros, allergies) into the knowledge "
            "graph. Additionally, you must create the relations to allergens if provided."
        ),
        "schema": {
            "type": "object",
            "properties": {
                "query": {
                    "type": "string",
                    "description": "Cypher query to insert a user.",
                }
            },
            "required": ["query"],
        },
    },
    "DishInfo": {
        "name": "DishInfoTool",
        "description": (
            "Returns a query to fetch dish info, and optionally evaluate user compatibility."
        ),
        "schema": {
            "type": "object",
            "properties": {
                "query": {
                    "type": "string",
                    "description": (
                        "Cypher query to fetch dish information and allergy compatibility."
                    ),
                }
            },
            "required": ["query"],
        },
    },
    "SubstituteDish": {
        "name": "MealPreparationTool",
        "description": (
            "Generates 2 queries: first one to check user's allergies, second one "
            "return the dishes compatible with the user's allergies."
        ),
        "schema": {
            "type": "object",
            "properties": {
                "query": {
                    "type": "array",
                    "items": {"type": "string"},
                    "description": "List of Cypher queries for meal planning and preparation.",
                }
            },
            "required": ["query"],
        },
    },
}
MANUSCRIPT_INTENT_MODELS = {
    "AddToDatabase": AddToDatabase,
    "DishInfo": DishInfo,
    "SubstituteDish": SubstituteDish,
}
ACTION_DESCRIPTIONS = {
    name: model.__doc__ or "" for name, model in MANUSCRIPT_INTENT_MODELS.items()
}
ACTION_DESCRIPTIONS[OUT_OF_SCOPE] = (
    "L'azione non è rilevante per il sistema, quindi il sistema non è in grado "
    "di fornire una risposta all'utente."
)
NEO4J_PROMPT = """You are an expert Neo4j Cypher translator who understands questions in Italian 
        and converts them to Cypher strictly following the instructions below:

        1. Generate a Cypher query compatible ONLY with Neo4j Version 5.
        2. Do not use the same variable names for different nodes and relationships.
        3. Use only the nodes and relationships mentioned in the schema.
        4. Always use the AS keyword to assign aliases to the returned nodes and relationships.
        5. Always use aliases to refer to nodes throughout the query.
        6. Do not use the word 'Answer' in the query (it is not a Cypher keyword).
        7. You may generate multiple queries if required.

        Schema:
        {schema}"""


def _remove_schema_titles(value: Any) -> Any:
    """Match LangChain's Pydantic-to-function schema conversion."""
    if isinstance(value, dict):
        return {
            key: _remove_schema_titles(item)
            for key, item in value.items()
            if key != "title"
        }
    if isinstance(value, list):
        return [_remove_schema_titles(item) for item in value]
    return value


def manuscript_intent_tools() -> list[dict[str, Any]]:
    """Build the Pydantic tools bound by the submitted runtime."""
    tools = []
    for name, model in MANUSCRIPT_INTENT_MODELS.items():
        schema = model.model_json_schema()
        title = str(schema.pop("title", name))
        description = str(schema.pop("description", model.__doc__ or ""))
        tools.append(
            {
                "type": "function",
                "function": {
                    "name": title,
                    "description": description,
                    "parameters": _remove_schema_titles(schema),
                },
            }
        )
    return tools


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--module",
        action="append",
        choices=(*MODULES, "all"),
        default=[],
        help="module to run; repeat the option or use all (default: all)",
    )
    parser.add_argument(
        "--scope-configuration",
        action="append",
        choices=(*SCOPE_CODES.values(), "all"),
        default=[],
        help="scope configuration to run (default: all six manuscript configurations)",
    )
    parser.add_argument("--provider", choices=("groq", "custom"), default="groq")
    parser.add_argument("--model", default="qwen/qwen3.8-27b")
    parser.add_argument("--base-url")
    parser.add_argument("--api-key-env", default="GROQ_API_KEY")
    parser.add_argument("--reasoning-effort", choices=("none",), default="none")
    parser.add_argument("--max-cases", type=int)
    parser.add_argument("--case-id", action="append", default=[])
    parser.add_argument("--repeats", type=int, default=1)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--request-delay", type=float, default=0.0)
    parser.add_argument("--max-retry-wait", type=float, default=60.0)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--max-attempts", type=int, default=2)
    parser.add_argument("--max-completion-tokens", type=int, default=1024)
    parser.add_argument("--scope-temperature", type=float, default=0.0)
    parser.add_argument("--intent-temperature", type=float, default=0.0)
    parser.add_argument("--inner-temperature", type=float, default=0.2)
    parser.add_argument("--query-temperature", type=float, default=0.0)
    parser.add_argument("--outer-temperature", type=float, default=0.4)
    parser.add_argument(
        "--query-protocol",
        choices=("legacy",),
        default="legacy",
        help="legacy reproduces the submitted in-sample functional test",
    )
    parser.add_argument("--neo4j-uri", default="bolt://localhost:7687")
    parser.add_argument("--neo4j-user", default="neo4j")
    parser.add_argument("--neo4j-password-env", default="NEO4J_PASSWORD")
    parser.add_argument("--validate-only", action="store_true")
    return parser


def _load_json(path: Path) -> list[dict[str, Any]]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, list):
        raise ValueError(f"expected a JSON array: {path}")
    return data


def _selected_modules(values: list[str]) -> list[str]:
    if not values or "all" in values:
        return list(MODULES)
    selected = set(values)
    return [module for module in MODULES if module in selected]


def _selected_scope_configs(values: list[str]) -> list[str]:
    available = list(SCOPE_CODES.values())
    if not values or "all" in values:
        return available
    selected = set(values)
    return [configuration for configuration in available if configuration in selected]


def _scope_configurations() -> dict[str, list[str]]:
    configs = _load_json(SCOPE_CONFIG_PATH)
    return {SCOPE_CODES[entry["name"]]: list(entry["labels"]) for entry in configs}


def load_module_cases(
    modules: list[str],
    scope_configs: list[str],
    *,
    max_cases: int | None = None,
    case_ids: Iterable[str] = (),
) -> list[dict[str, Any]]:
    """Load stable case identifiers and manuscript grouping information."""
    selected_ids = set(case_ids)
    cases: list[dict[str, Any]] = []
    if "scope" in modules:
        source = _load_json(DATASETS["scope"])
        for configuration in scope_configs:
            labels = _scope_configurations()[configuration]
            # The paper reports N=66 for ALL: its six OutOfScope examples are
            # excluded there, while each 3-domain condition includes them (N=24).
            allowed = set(labels)
            selected = [
                (index, entry)
                for index, entry in enumerate(source)
                if entry["scenario"] in allowed
                or (configuration != "ALL" and entry["scenario"] == OUT_OF_SCOPE)
            ]
            if max_cases is not None:
                selected = selected[: max(0, max_cases)]
            for index, entry in selected:
                cases.append(
                    {
                        "module": "scope",
                        "configuration": configuration,
                        "case_id": f"scope-{configuration}-{index:03d}",
                        "payload": entry,
                        "scope_labels": labels,
                    }
                )
    for module in ("intent", "inner", "query", "outer"):
        if module not in modules:
            continue
        selected = list(enumerate(_load_json(DATASETS[module])))
        if max_cases is not None:
            selected = selected[: max(0, max_cases)]
        configuration = "ADVISOR-legacy" if module == "query" else "ADVISOR"
        for index, entry in selected:
            cases.append(
                {
                    "module": module,
                    "configuration": configuration,
                    "case_id": f"{module}-{index:03d}",
                    "payload": entry,
                }
            )
    if selected_ids:
        cases = [case for case in cases if case["case_id"] in selected_ids]
    if not cases:
        raise SystemExit("No module cases selected")
    return cases


def _scope_tools(labels: list[str]) -> list[dict[str, Any]]:
    selected = set(labels) | {OUT_OF_SCOPE}
    tools = []
    for entry in domain_descriptions:
        name, description = entry.split(":", maxsplit=1)
        name = name.strip()
        if name not in selected:
            continue
        tools.append(
            {
                "type": "function",
                "function": {
                    "name": name,
                    "description": description.strip(),
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "reason": {
                                "type": "string",
                                "description": (
                                    "Il tuo ragionamento. Devi spiegare perché questo tool "
                                    "è adeguato alla domanda dell'utente."
                                ),
                            }
                        },
                        "required": ["reason"],
                    },
                },
            }
        )
    return tools


def _runtime_intent_parameters(action: str, raw: Any) -> dict[str, Any]:
    parameters = normalize_parameters(action, raw)
    model = MANUSCRIPT_INTENT_MODELS.get(action)
    if model is None:
        return {}
    schema = model.model_json_schema()
    for name, field in schema["properties"].items():
        if name in parameters:
            continue
        field_type = field.get("type")
        if field_type is None:
            variants = field.get("anyOf", [])
            field_type = next(
                (
                    variant.get("type")
                    for variant in variants
                    if variant.get("type") != "null"
                ),
                None,
            )
        parameters[name] = {
            "array": [],
            "boolean": False,
            "integer": 0,
            "number": 0.0,
            "string": "",
        }.get(field_type)
    return parameters


def _legacy_parameter_f1(
    expected: dict[str, Any], predicted: dict[str, Any]
) -> float | None:
    if not expected:
        return None
    true_positive = sum(
        1
        for key, expected_value in expected.items()
        if predicted.get(key) == expected_value
    )
    precision = true_positive / len(predicted) if predicted else 0.0
    recall = true_positive / len(expected)
    return 2 * precision * recall / (precision + recall) if precision + recall else 0.0


def _record(
    case: dict[str, Any],
    repeat: int,
    trace: CompletionTrace,
    wall_latency: float,
    *,
    expected: dict[str, Any],
    prediction: dict[str, Any],
    scores: dict[str, Any],
) -> dict[str, Any]:
    return {
        "module": case["module"],
        "configuration": case["configuration"],
        "case_id": case["case_id"],
        "repeat": repeat,
        "input": case["payload"],
        "expected": expected,
        "prediction": prediction,
        "scores": scores,
        "api_latency_seconds": trace.latency_seconds,
        "wall_latency_seconds": wall_latency,
        "throttle_seconds": max(0.0, wall_latency - trace.latency_seconds),
        "prompt_tokens": trace.prompt_tokens,
        "completion_tokens": trace.completion_tokens,
        "total_tokens": trace.total_tokens,
        "attempts": trace.attempts,
        "failure": trace.parsed is None,
        "error": trace.error,
        "trace": trace.to_dict(),
    }


def _run_scope(
    client: JsonLLMClient, case: dict[str, Any], repeat: int, temperature: float
) -> dict[str, Any]:
    payload = case["payload"]
    user = (
        f"User Question is: {payload['question']}.\n"
        f"The inner speech is: {payload.get('inner_speech')}."
    )
    started = time.perf_counter()
    trace = client.complete_tool_call(
        system=SUBMITTED_SCOPE_SYSTEM,
        user=user,
        tools=_scope_tools(case["scope_labels"]),
        temperature=temperature,
    )
    wall = time.perf_counter() - started
    parsed = trace.parsed or {}
    predicted = str(parsed.get("name", OUT_OF_SCOPE))
    expected = str(payload["scenario"])
    return _record(
        case,
        repeat,
        trace,
        wall,
        expected={"scenario": expected, "reason": payload.get("reason", "")},
        prediction={
            "scenario": predicted,
            "reason": str(parsed.get("arguments", {}).get("reason", "")),
        },
        scores={"domain_correct": trace.parsed is not None and predicted == expected},
    )


def _run_intent(
    client: JsonLLMClient, case: dict[str, Any], repeat: int, temperature: float
) -> dict[str, Any]:
    payload = case["payload"]
    user = f"Memory: \nUser Input: {payload['question']}"
    started = time.perf_counter()
    trace = client.complete_tool_call(
        system=SUBMITTED_INTENT_SYSTEM,
        user=user,
        tools=manuscript_intent_tools(),
        temperature=temperature,
    )
    wall = time.perf_counter() - started
    parsed = trace.parsed or {}
    predicted_action = normalize_action(parsed.get("name"))
    expected_action = normalize_action(payload["action_name"])
    predicted_parameters = _runtime_intent_parameters(
        predicted_action, parsed.get("arguments", {})
    )
    expected_parameters = _runtime_intent_parameters(
        expected_action, payload.get("parameters", {})
    )
    return _record(
        case,
        repeat,
        trace,
        wall,
        expected={"action": expected_action, "parameters": expected_parameters},
        prediction={"action": predicted_action, "parameters": predicted_parameters},
        scores={
            "task_correct": trace.parsed is not None
            and predicted_action == expected_action,
            "parameters_exact": (
                trace.parsed is not None and predicted_parameters == expected_parameters
            ),
            "parameter_f1": _legacy_parameter_f1(
                expected_parameters, predicted_parameters
            ),
        },
    )


def _run_inner(
    client: JsonLLMClient, case: dict[str, Any], repeat: int, temperature: float
) -> dict[str, Any]:
    payload = case["payload"]
    action = str(payload["action_name"])
    system = f"""{SCENARIO_DESCRIPTION}.
Hai a disposizione anche una memoria a breve termine con interazioni passate.
Devi impedire l'esecuzione di domande non pertinenti al tuo scopo
Devi impedire l'esecuzione di domande con parametri obbligatori mancanti.
Devi filtrare domande relative al tuo argomento ma troppo vaghe."""
    user = f"""Memoria: None
La domanda dell'utente è: {payload["question"]}.
Il riconoscimento dell'intento ha assegnato la seguente funzione: {action}.
Action description:{ACTION_DESCRIPTIONS[action]}
Con i seguenti parametri: {payload["parameters"]}.
Parametri obbligatori mancanti: {payload["missing_parameters"]}"""
    started = time.perf_counter()
    trace = client.complete_structured_tool_call(
        system=system,
        user=user,
        tool_name="InnerSeechOutputFormat",
        tool_description=INNER_TOOL_DESCRIPTION,
        output_schema=INNER_SCHEMA,
        temperature=temperature,
    )
    wall = time.perf_counter() - started
    parsed = trace.parsed or {}
    predicted_can_proceed = bool(parsed.get("can_proceed", False))
    expected_can_proceed = bool(payload["can_proceed"])
    return _record(
        case,
        repeat,
        trace,
        wall,
        expected={
            "can_proceed": expected_can_proceed,
            "inner_speech": payload["inner_speech"],
        },
        prediction={
            "can_proceed": predicted_can_proceed,
            "inner_speech": str(parsed.get("inner_speech", "")),
        },
        scores={
            "readiness_correct": (
                trace.parsed is not None
                and predicted_can_proceed == expected_can_proceed
            )
        },
    )


def _query_examples() -> dict[str, list[dict[str, Any]]]:
    grouped: dict[str, list[dict[str, Any]]] = {}
    for entry in _load_json(DATASETS["query"]):
        grouped.setdefault(str(entry["action_name"]), []).append(entry)
    return grouped


def _format_query_prompt(payload: dict[str, Any], database_schema: str) -> str:
    prefix = NEO4J_PROMPT.format(schema=database_schema)
    blocks = [prefix]
    for example in _query_examples()[str(payload["action_name"])]:
        blocks.append(
            "User asks: {question}\nParameters: {parameters}\nQueries: {queries}".format(
                question=example["question"],
                parameters=json.dumps(example["parameters"], ensure_ascii=False),
                queries=example["queries"],
            )
        )
    blocks.append(
        "User asks: {question}\nParameters: {parameters}\nQuery: ".format(
            question=payload["question"],
            parameters=json.dumps(payload["parameters"], ensure_ascii=False),
        )
    )
    return "\n\n".join(blocks)


def _normalize_value(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): _normalize_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        normalized = [_normalize_value(item) for item in value]
        try:
            return sorted(
                normalized,
                key=lambda item: json.dumps(item, ensure_ascii=False, sort_keys=True),
            )
        except TypeError:
            return sorted(normalized, key=repr)
    if hasattr(value, "items"):
        return {str(key): _normalize_value(item) for key, item in value.items()}
    return value


class Neo4jExecutor:
    """Execute generated Cypher against the dedicated local test graph."""

    def __init__(self, uri: str, username: str, password: str) -> None:
        try:
            from neo4j import GraphDatabase
        except ImportError as exc:  # pragma: no cover - environment-specific failure.
            raise RuntimeError(
                "install the neo4j Python package for query tests"
            ) from exc
        self.uri = uri
        self.username = username
        self.password = password
        self.driver = GraphDatabase.driver(uri, auth=(username, password))
        self.driver.verify_connectivity()

    def close(self) -> None:
        self.driver.close()

    def schema(self) -> str:
        """Return the concise schema string produced by ``Neo4jGraph.schema``.

        The submitted runtime obtains the same information through APOC. The
        built-in procedures below avoid making APOC a benchmark prerequisite.
        """
        with self.driver.session() as session:
            node_rows = [
                record.data()
                for record in session.run("CALL db.schema.nodeTypeProperties()")
            ]
            relationship_rows = [
                record.data()
                for record in session.run("CALL db.schema.relTypeProperties()")
            ]
            patterns = [
                record.data()
                for record in session.run(
                    "MATCH (a)-[r]->(b) "
                    "RETURN DISTINCT labels(a)[0] AS start, type(r) AS type, "
                    "labels(b)[0] AS end ORDER BY type, start, end"
                )
            ]

        type_names = {
            "Boolean": "BOOLEAN",
            "Double": "FLOAT",
            "Long": "INTEGER",
            "String": "STRING",
        }
        node_properties: dict[str, list[str]] = {}
        for row in node_rows:
            label = str(row["nodeLabels"][0])
            property_types = row.get("propertyTypes") or ["ANY"]
            property_type = type_names.get(
                str(property_types[0]), str(property_types[0]).upper()
            )
            node_properties.setdefault(label, []).append(
                f"{row['propertyName']}: {property_type}"
            )
        relationship_properties: dict[str, list[str]] = {}
        for row in relationship_rows:
            if row.get("propertyName") is None:
                continue
            relationship = str(row["relType"]).strip(":`")
            property_types = row.get("propertyTypes") or ["ANY"]
            property_type = type_names.get(
                str(property_types[0]), str(property_types[0]).upper()
            )
            relationship_properties.setdefault(relationship, []).append(
                f"{row['propertyName']}: {property_type}"
            )

        lines = ["Node properties:"]
        lines.extend(
            f"{label} {{{', '.join(properties)}}}"
            for label, properties in node_properties.items()
        )
        lines.append("Relationship properties:")
        lines.extend(
            f"{relationship} {{{', '.join(properties)}}}"
            for relationship, properties in relationship_properties.items()
        )
        lines.append("The relationships:")
        lines.extend(
            f"(:{row['start']})-[:{row['type']}]->(:{row['end']})" for row in patterns
        )
        return "\n".join(lines)

    @staticmethod
    def _is_safe(query: str, action: str) -> bool:
        forbidden = re.compile(r"\b(DELETE|DETACH|DROP|REMOVE|LOAD\s+CSV)\b", re.I)
        if forbidden.search(query):
            return False
        if action != "AddToDatabase" and re.search(
            r"\b(CREATE|MERGE|SET)\b", query, re.I
        ):
            return False
        return True

    def execute(self, queries: list[str], action: str) -> list[Any]:
        if any(not self._is_safe(query, action) for query in queries):
            return ["ERROR::unsafe generated query"]
        outputs: list[Any] = []
        with self.driver.session() as session:
            transaction = session.begin_transaction()
            try:
                for query in queries:
                    result = transaction.run(query)
                    outputs.append(
                        [
                            {
                                str(key): _normalize_value(value)
                                for key, value in record.items()
                            }
                            for record in result
                        ]
                    )
            except Exception as exc:
                outputs.append(f"ERROR::{type(exc).__name__}: {exc}")
            finally:
                transaction.rollback()
        return outputs


def _flatten_results(results: list[Any]) -> list[Any]:
    flattened: list[Any] = []

    def visit(item: Any) -> None:
        if isinstance(item, list):
            if not item:
                flattened.append("EMPTY_RESULT")
            else:
                for child in item:
                    visit(child)
        elif isinstance(item, dict):
            flattened.append(_normalize_value(item))
        else:
            flattened.append(item)

    for result in results:
        visit(result)
    return flattened


def query_result_overlap(reference: list[Any], generated: list[Any]) -> float:
    """Reproduce the legacy subset-aware result overlap metric."""
    reference_flat = _flatten_results(reference)
    generated_flat = _flatten_results(generated)
    if any(
        isinstance(item, str) and item.startswith("ERROR::") for item in generated_flat
    ):
        return 0.0
    reference_dicts = [item for item in reference_flat if isinstance(item, dict)]
    generated_dicts = [item for item in generated_flat if isinstance(item, dict)]
    reference_atoms = {
        json.dumps(item, ensure_ascii=False, sort_keys=True)
        for item in reference_flat
        if not isinstance(item, dict)
    }
    generated_atoms = {
        json.dumps(item, ensure_ascii=False, sort_keys=True)
        for item in generated_flat
        if not isinstance(item, dict)
    }

    def is_subset(left: dict[str, Any], right: dict[str, Any]) -> bool:
        return all(key in right and right[key] == value for key, value in left.items())

    matches = 0
    used_reference = [False] * len(reference_dicts)
    for generated_item in generated_dicts:
        for index, reference_item in enumerate(reference_dicts):
            if used_reference[index]:
                continue
            if is_subset(generated_item, reference_item) or is_subset(
                reference_item, generated_item
            ):
                matches += 1
                used_reference[index] = True
                break
    atom_intersection = len(reference_atoms & generated_atoms)
    atom_union = len(reference_atoms | generated_atoms)
    dict_union = len(reference_dicts) + len(generated_dicts) - matches
    denominator = atom_union + dict_union
    return (atom_intersection + matches) / denominator if denominator else 1.0


def _run_query(
    client: JsonLLMClient,
    database: Neo4jExecutor,
    database_schema: str,
    case: dict[str, Any],
    repeat: int,
    temperature: float,
) -> dict[str, Any]:
    payload = case["payload"]
    action = str(payload["action_name"])
    tool = QUERY_TOOLS[action]
    prompt = _format_query_prompt(payload, database_schema)
    started = time.perf_counter()
    trace = client.complete_structured_tool_call(
        system=None,
        user=prompt,
        tool_name=str(tool["name"]),
        tool_description=str(tool["description"]),
        output_schema=tool["schema"],
        temperature=temperature,
    )
    wall = time.perf_counter() - started
    parsed = trace.parsed or {}
    generated_queries = parsed.get("query", [])
    if isinstance(generated_queries, str):
        generated_queries = [generated_queries]
    if not isinstance(generated_queries, list):
        generated_queries = []
    generated_queries = [str(query) for query in generated_queries]
    reference_queries = [str(query) for query in payload["queries"]]
    reference_results = database.execute(reference_queries, action)
    generated_results = database.execute(generated_queries, action)
    validity = [
        not (isinstance(result, str) and result.startswith("ERROR::"))
        for result in generated_results
    ]
    overlap = query_result_overlap(reference_results, generated_results)
    return _record(
        case,
        repeat,
        trace,
        wall,
        expected={"queries": reference_queries, "results": reference_results},
        prediction={"queries": generated_queries, "results": generated_results},
        scores={
            "valid_queries": sum(validity),
            "generated_query_count": len(generated_queries),
            "all_queries_valid": bool(generated_queries) and all(validity),
            "result_overlap": overlap,
        },
    )


def _explainability_examples(action: str) -> list[dict[str, Any]]:
    path = SCENARIO_ROOT / action / "explainability_examples" / f"{action}.json"
    return _load_json(path)


def _format_outer_prompt(payload: dict[str, Any]) -> str:
    instructions = (
        f"{SCENARIO_DESCRIPTION}. Data una richiesta e la sua traduzione in query "
        "con i relativi risultati, devi spiegare all'utente il processo decisionale "
        "ed il risulato."
    )
    blocks = [instructions]
    for example in _explainability_examples(str(payload["action_name"])):
        blocks.append(
            "User Input: {user_input}\nQueries: {queries}\nQuery Results: {results}"
            "\nExplanation: {explanation}".format(**example)
        )
    blocks.append(
        "Rispondini in linguaggio naturale in lingua Italiana in modo sintetico.\n"
        "User Input: {question}\nQueries: {queries}\nQuery Results: {results}\n"
        "Explanation: ".format(**payload)
    )
    return "\n\n".join(blocks)


def _run_outer(
    client: JsonLLMClient, case: dict[str, Any], repeat: int, temperature: float
) -> dict[str, Any]:
    payload = case["payload"]
    started = time.perf_counter()
    trace = client.complete_text(
        system=None,
        user=_format_outer_prompt(payload),
        temperature=temperature,
    )
    wall = time.perf_counter() - started
    text = trace.raw_text
    return _record(
        case,
        repeat,
        trace,
        wall,
        expected={"explanation": payload["explanation"]},
        prediction={"explanation": text},
        scores={},
    )


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _git_sha() -> str:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=REPO_ROOT, text=True
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def _git_dirty() -> bool:
    try:
        output = subprocess.check_output(
            ["git", "status", "--porcelain"], cwd=REPO_ROOT, text=True
        )
        return bool(output.strip())
    except (OSError, subprocess.CalledProcessError):
        return True


def _hardware() -> dict[str, Any]:
    cpu = platform.processor()
    cpuinfo = Path("/proc/cpuinfo")
    if cpuinfo.exists():
        for line in cpuinfo.read_text(encoding="utf-8", errors="replace").splitlines():
            if line.lower().startswith("model name"):
                cpu = line.split(":", maxsplit=1)[-1].strip()
                break
    return {
        "platform": platform.platform(),
        "cpu_model": cpu,
        "logical_cpu_count": os.cpu_count(),
        "python": sys.version,
    }


def _provider(args: argparse.Namespace) -> tuple[str, str, str]:
    if args.provider == "groq":
        base_url = args.base_url or "https://api.groq.com/openai/v1"
    else:
        if not args.base_url:
            raise SystemExit("--provider custom requires --base-url")
        base_url = args.base_url
    api_key = os.getenv(args.api_key_env, "")
    if not api_key:
        raise SystemExit(f"Missing API key in {args.api_key_env}")
    return args.model, base_url, api_key


def _read_records(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    records = []
    for line_number, line in enumerate(
        path.read_text(encoding="utf-8").splitlines(), 1
    ):
        if not line.strip():
            continue
        try:
            records.append(json.loads(line))
        except json.JSONDecodeError as exc:
            raise ValueError(f"invalid JSONL at {path}:{line_number}") from exc
    return records


def _print_summary(rows: list[dict[str, Any]]) -> None:
    print("\nModule summary")
    for row in rows:
        metric = ""
        for name in (
            "domain_selection_accuracy",
            "task_selection_accuracy",
            "execution_readiness_accuracy",
            "query_validity_rate",
        ):
            if name in row:
                metric = f" {name}={row[name]:.3f}"
                break
        print(
            f"{row['module']}/{row['configuration']}: n={row['n']}{metric} "
            f"api_p50={row['api_latency_p50_seconds']:.3f}s "
            f"tokens_p50={row['total_tokens_p50']:.1f} "
            f"failures={row['failure_rate']:.3f}"
        )


def main(argv: list[str] | None = None) -> int:
    load_dotenv(REPO_ROOT / ".env", override=True)
    args = _parser().parse_args(argv)
    modules = _selected_modules(args.module)
    scope_configs = _selected_scope_configs(args.scope_configuration)
    cases = load_module_cases(
        modules,
        scope_configs,
        max_cases=args.max_cases,
        case_ids=args.case_id,
    )
    grouped_counts: dict[tuple[str, str], int] = {}
    for case in cases:
        key = (case["module"], case["configuration"])
        grouped_counts[key] = grouped_counts.get(key, 0) + 1
    if args.validate_only:
        for (module, configuration), count in grouped_counts.items():
            print(f"{module}/{configuration}: {count} cases")
        return 0

    model, base_url, api_key = _provider(args)
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output_dir = args.output_dir or (
        REPO_ROOT / "evaluation" / "results" / "modules" / timestamp
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    raw_path = output_dir / "raw.jsonl"
    records = _read_records(raw_path)
    completed = {
        (record["module"], record["configuration"], record["case_id"], record["repeat"])
        for record in records
    }
    temperatures = {
        "scope": args.scope_temperature,
        "intent": args.intent_temperature,
        "inner": args.inner_temperature,
        "query": args.query_temperature,
        "outer": args.outer_temperature,
    }
    metadata = {
        "created_at_utc": timestamp,
        "git_sha": _git_sha(),
        "git_dirty": _git_dirty(),
        "git_branch": subprocess.check_output(
            ["git", "branch", "--show-current"], cwd=REPO_ROOT, text=True
        ).strip(),
        "provider": args.provider,
        "model": model,
        "base_url": base_url,
        "reasoning_effort": args.reasoning_effort,
        "prompt_profile": "submitted_manuscript",
        "temperatures": temperatures,
        "paper_temperatures": PAPER_TEMPERATURES,
        "modules": modules,
        "scope_configurations": scope_configs,
        "scope_all_case_policy": "66 in-domain cases; OutOfScope remains a candidate tool",
        "query_protocol": args.query_protocol,
        "query_protocol_limitation": (
            "legacy protocol injects the evaluated examples as few-shot demonstrations; "
            "treat as an in-sample functional regression, not held-out generalization"
        ),
        "max_cases_per_group": args.max_cases,
        "case_ids": [case["case_id"] for case in cases],
        "repeats": args.repeats,
        "request_delay": args.request_delay,
        "max_retry_wait": args.max_retry_wait,
        "max_attempts": args.max_attempts,
        "max_completion_tokens": args.max_completion_tokens,
        "dataset_sha256": {module: _sha256(DATASETS[module]) for module in modules},
        "scope_config_sha256": _sha256(SCOPE_CONFIG_PATH),
        "hardware": _hardware(),
    }
    metadata_path = output_dir / "metadata.json"
    if records and metadata_path.exists():
        previous = json.loads(metadata_path.read_text(encoding="utf-8"))
        immutable = (
            "provider",
            "model",
            "base_url",
            "reasoning_effort",
            "prompt_profile",
            "temperatures",
            "modules",
            "scope_configurations",
            "query_protocol",
            "max_cases_per_group",
            "case_ids",
            "repeats",
            "request_delay",
            "max_retry_wait",
            "max_attempts",
            "max_completion_tokens",
            "dataset_sha256",
        )
        changed = [
            field for field in immutable if previous.get(field) != metadata.get(field)
        ]
        if changed:
            raise SystemExit(
                "Cannot resume with changed settings: " + ", ".join(changed)
            )
        metadata = previous
        metadata.setdefault("resume_events", []).append(
            {
                "resumed_at_utc": timestamp,
                "git_sha": _git_sha(),
                "git_dirty": _git_dirty(),
                "completed_records_before_resume": len(records),
            }
        )
    metadata_path.write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    client = JsonLLMClient(
        model=model,
        api_key=api_key,
        base_url=base_url,
        timeout=args.timeout,
        max_attempts=args.max_attempts,
        max_completion_tokens=args.max_completion_tokens,
        request_delay=args.request_delay,
        max_retry_wait=args.max_retry_wait,
        reasoning_effort=args.reasoning_effort,
    )
    database: Neo4jExecutor | None = None
    database_schema = ""
    if "query" in modules:
        password = os.getenv(args.neo4j_password_env, "password")
        database = Neo4jExecutor(args.neo4j_uri, args.neo4j_user, password)
        database_schema = database.schema()

    run_started = time.perf_counter()
    try:
        with raw_path.open("a", encoding="utf-8") as raw_handle:
            for repeat in range(args.repeats):
                for position, case in enumerate(cases, 1):
                    key = (
                        case["module"],
                        case["configuration"],
                        case["case_id"],
                        repeat,
                    )
                    if key in completed:
                        continue
                    print(
                        f"[{repeat + 1}/{args.repeats} {position}/{len(cases)}] "
                        f"{case['case_id']}",
                        flush=True,
                    )
                    if case["module"] == "scope":
                        record = _run_scope(client, case, repeat, temperatures["scope"])
                    elif case["module"] == "intent":
                        record = _run_intent(
                            client, case, repeat, temperatures["intent"]
                        )
                    elif case["module"] == "inner":
                        record = _run_inner(client, case, repeat, temperatures["inner"])
                    elif case["module"] == "query":
                        assert database is not None
                        record = _run_query(
                            client,
                            database,
                            database_schema,
                            case,
                            repeat,
                            temperatures["query"],
                        )
                    else:
                        record = _run_outer(client, case, repeat, temperatures["outer"])
                    raw_handle.write(
                        json.dumps(record, ensure_ascii=False, sort_keys=True) + "\n"
                    )
                    raw_handle.flush()
                    records.append(record)
                    completed.add(key)
                    print(
                        f"  api={record['api_latency_seconds']:.3f}s "
                        f"tokens={record['total_tokens']} failure={record['failure']}",
                        flush=True,
                    )
    finally:
        if database is not None:
            database.close()

    rows = write_module_summary(records, output_dir)
    metadata["last_completed_at_utc"] = datetime.now(timezone.utc).strftime(
        "%Y%m%dT%H%M%SZ"
    )
    metadata["last_run_wall_seconds"] = time.perf_counter() - run_started
    metadata["completed_records"] = len(records)
    metadata_path.write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    _print_summary(rows)
    print(f"Results: {output_dir}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except ProviderRateLimitError as exc:
        retry = (
            f" Suggested retry delay: {exc.retry_after_seconds:.1f}s."
            if exc.retry_after_seconds is not None
            else ""
        )
        print(
            "Provider rate limit paused the module run before recording the current case."
            + retry,
            file=sys.stderr,
        )
        raise SystemExit(75) from None
