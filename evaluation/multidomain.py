"""Three-domain controller comparison using the repository's implemented tasks."""

from __future__ import annotations

import json
import time
from typing import Any

from .controllers import DIRECT_SCHEMA, GATE_SCHEMA, _base_result, _complete_structured
from .llm_client import JsonLLMClient
from .task_spec import (
    NATIVE_INTENT_SCHEMAS,
    OUT_OF_SCOPE,
    TASK_SPECS,
    missing_parameters,
    normalize_action,
    normalize_decision,
    normalize_parameters,
)


DOMAIN_DESCRIPTIONS = {
    "ADVISOR": "Dietary regimen assistance and food knowledge.",
    "MOVIES": "Movie information and cinema screening schedules.",
    "I-TROPHYTS": "Rehabilitation exercises and rehabilitation sensor data.",
}

MOVIE_TASK_SPECS: dict[str, dict[str, Any]] = {
    "MovieInfo": {
        "description": "Retrieve information about a movie identified by supplied facts.",
        "parameters": {
            "title": "movie title (string)",
            "director": "director name (string)",
            "genres": "movie genres (list)",
            "year": "release year (integer)",
            "actors": "actor names (list)",
            "descriptive_movie_facts": "other identifying movie facts (list)",
        },
        "required": [],
        "defaults": {
            "title": "",
            "director": "",
            "genres": [],
            "year": 0,
            "actors": [],
            "descriptive_movie_facts": [],
        },
    },
    "TimetableInfo": {
        "description": "Retrieve cinema screening times for one specific movie.",
        "parameters": {
            "title": "movie title (string)",
            "cinema": "cinema name (string)",
            "language": "ISO 639-1 language code (string)",
            "dates": "screening dates in YYYY-MM-DD format (list)",
            "time": "screening times in HH:MM format (list)",
        },
        "required": [],
        "defaults": {"cinema": "", "language": "", "dates": [], "time": []},
    },
}

REHAB_TASK_SPECS: dict[str, dict[str, Any]] = {
    "ExerciseInformation": {
        "description": (
            "Retrieve rehabilitation exercise information, or patient parameters when "
            "a patient name is supplied."
        ),
        "parameters": {
            "exercise_name": "rehabilitation exercise name (string)",
            "user_name": "patient name (string)",
            "step_number": "exercise step index (integer)",
            "giorno": "Italian weekday (string)",
        },
        "required": [],
        "defaults": {"exercise_name": "", "user_name": "", "step_number": 1, "giorno": ""},
    },
    "SensorReadings": {
        "description": "Store structured rehabilitation angle or heart-rate sensor readings.",
        "parameters": {
            "angles_readings": "structured angle sample (object)",
            "heart_rate_readings": "structured heart-rate sample (object)",
        },
        "required": [],
        "defaults": {"angles_readings": None, "heart_rate_readings": None},
    },
}

DOMAIN_TASK_SPECS = {
    "ADVISOR": TASK_SPECS,
    "MOVIES": MOVIE_TASK_SPECS,
    "I-TROPHYTS": REHAB_TASK_SPECS,
}
ALL_TASK_SPECS = {
    name: spec for domain_specs in DOMAIN_TASK_SPECS.values() for name, spec in domain_specs.items()
}
TASK_TO_DOMAIN = {
    name: domain
    for domain, domain_specs in DOMAIN_TASK_SPECS.items()
    for name in domain_specs
}

MOVIE_NATIVE_SCHEMAS = {
    "MovieInfo": {
        "description": MOVIE_TASK_SPECS["MovieInfo"]["description"],
        "properties": {
            "title": {"type": "string", "default": ""},
            "director": {"type": "string", "default": ""},
            "genres": {"type": "array", "items": {"type": "string"}, "default": []},
            "year": {"type": "integer", "default": 0},
            "actors": {"type": "array", "items": {"type": "string"}, "default": []},
            "descriptive_movie_facts": {
                "type": "array",
                "items": {"type": "string"},
                "default": [],
            },
        },
        "required": [],
    },
    "TimetableInfo": {
        "description": MOVIE_TASK_SPECS["TimetableInfo"]["description"],
        "properties": {
            "title": {"type": "string"},
            "cinema": {"type": "string", "default": ""},
            "language": {"type": "string", "default": ""},
            "dates": {"type": "array", "items": {"type": "string"}, "default": []},
            "time": {"type": "array", "items": {"type": "string"}, "default": []},
        },
        "required": [],
    },
}

REHAB_NATIVE_SCHEMAS = {
    "ExerciseInformation": {
        "description": REHAB_TASK_SPECS["ExerciseInformation"]["description"],
        "properties": {
            "exercise_name": {"type": "string", "default": ""},
            "user_name": {"type": "string", "default": ""},
            "step_number": {"type": "integer", "default": 1},
            "giorno": {"type": "string", "default": ""},
        },
        "required": [],
    },
    "SensorReadings": {
        "description": REHAB_TASK_SPECS["SensorReadings"]["description"],
        "properties": {
            "angles_readings": {"type": ["object", "null"], "default": None},
            "heart_rate_readings": {"type": ["object", "null"], "default": None},
        },
        "required": [],
    },
}

DOMAIN_NATIVE_SCHEMAS = {
    "ADVISOR": NATIVE_INTENT_SCHEMAS,
    "MOVIES": MOVIE_NATIVE_SCHEMAS,
    "I-TROPHYTS": REHAB_NATIVE_SCHEMAS,
}


def _tools(schemas: dict[str, dict[str, Any]]) -> list[dict[str, Any]]:
    return [
        {
            "type": "function",
            "function": {
                "name": name,
                "description": spec["description"],
                "parameters": {
                    "type": "object",
                    "properties": spec["properties"],
                    "required": spec["required"],
                },
            },
        }
        for name, spec in schemas.items()
    ]


def _scope_tools() -> list[dict[str, Any]]:
    tools = []
    for domain, description in DOMAIN_DESCRIPTIONS.items():
        tasks = DOMAIN_TASK_SPECS[domain]
        task_text = "; ".join(f"{name}: {spec['description']}" for name, spec in tasks.items())
        tools.append(
            {
                "type": "function",
                "function": {
                    "name": domain,
                    "description": f"{description} Supported tasks: {task_text}",
                    "parameters": {
                        "type": "object",
                        "properties": {"reason": {"type": "string"}},
                        "required": [],
                    },
                },
            }
        )
    return tools


def _domain(value: Any) -> str:
    if not isinstance(value, str):
        return OUT_OF_SCOPE
    lookup = {name.lower(): name for name in (*DOMAIN_DESCRIPTIONS, OUT_OF_SCOPE)}
    return lookup.get(value.strip().lower(), OUT_OF_SCOPE)


def _context(case: dict[str, Any]) -> str:
    return json.dumps(
        {
            "user_input": case["user_input"],
            "memory": case.get("memory", []),
            "tool_context": case.get("tool_context", {}),
        },
        ensure_ascii=False,
        sort_keys=True,
    )


def _intent_context(case: dict[str, Any]) -> str:
    """Match JANUS Intent input; DB-derived tool context is applied afterwards."""
    return json.dumps(
        {"user_input": case["user_input"], "memory": case.get("memory", [])},
        ensure_ascii=False,
        sort_keys=True,
    )


def _semantic_missing(action: str, parameters: dict[str, Any]) -> list[str]:
    missing = missing_parameters(action, parameters, ALL_TASK_SPECS)
    if action == "MovieInfo" and not any(
        parameters.get(name) not in (None, "", [], 0)
        for name in MOVIE_TASK_SPECS["MovieInfo"]["parameters"]
    ):
        missing.append("movie_identifier")
    if action == "ExerciseInformation" and not (
        parameters.get("exercise_name") or parameters.get("user_name")
    ):
        missing.append("exercise_name_or_user_name")
    if action == "TimetableInfo" and not any(
        parameters.get(name) not in (None, "", [], 0)
        for name in MOVIE_TASK_SPECS["TimetableInfo"]["parameters"]
    ):
        missing.append("screening_filter")
    if action == "SensorReadings" and not (
        parameters.get("angles_readings") or parameters.get("heart_rate_readings")
    ):
        missing.append("sensor_sample")
    return missing


def _record(
    *,
    architecture: str,
    case: dict[str, Any],
    repeat: int,
    domain: str,
    decision: str,
    action: str,
    parameters: dict[str, Any],
    missing: list[str],
    rationale: str,
    traces: list,
    wall: float,
) -> dict[str, Any]:
    record = _base_result(
        architecture=architecture,
        case=case,
        repeat=repeat,
        decision=decision,
        action=action,
        parameters=parameters,
        missing=missing,
        rationale=rationale,
        traces=traces,
        wall_latency_seconds=wall,
    )
    record["prediction"]["domain"] = domain
    return record


def run_multidomain_factored_and_rule(
    client: JsonLLMClient,
    case: dict[str, Any],
    *,
    repeat: int,
    scope_temperature: float,
    intent_temperature: float,
    gate_temperature: float,
    include_factored: bool,
    include_rule: bool,
    structured_interface: str,
) -> list[dict[str, Any]]:
    """Run Scope -> domain-specific Intent -> gate, sharing upstream state."""
    started = time.perf_counter()
    scope_trace = client.complete_tool_call(
        system=(
            "Select the single most appropriate implemented scenario for the current "
            "user request. Use conversation memory only to resolve an otherwise clear "
            "reference. Do not select a scenario when no supported task applies."
        ),
        user=_context(case),
        tools=_scope_tools(),
        temperature=scope_temperature,
    )
    scope_data = scope_trace.parsed or {}
    domain = _domain(scope_data.get("name"))
    traces = [scope_trace]
    action = OUT_OF_SCOPE
    parameters: dict[str, Any] = {}

    if domain != OUT_OF_SCOPE:
        intent_trace = client.complete_tool_call(
            system=(
                "Select the relevant task in the active scenario and extract only values "
                "explicitly stated by the user or unambiguously available in memory. "
                "Never guess an ambiguous value. Do not call a task when none applies."
            ),
            user=_intent_context(case),
            tools=_tools(DOMAIN_NATIVE_SCHEMAS[domain]),
            temperature=intent_temperature,
        )
        traces.append(intent_trace)
        intent_data = intent_trace.parsed or {}
        action = normalize_action(intent_data.get("name"), DOMAIN_TASK_SPECS[domain])
        parameters = normalize_parameters(
            action, intent_data.get("arguments", {}), ALL_TASK_SPECS
        )
        if action == "SubstituteDish" and "ha_piano_settimanale" in case.get(
            "tool_context", {}
        ):
            parameters["ha_piano_settimanale"] = bool(
                case["tool_context"]["ha_piano_settimanale"]
            )

    missing = _semantic_missing(action, parameters)
    upstream_wall = time.perf_counter() - started
    outputs = []
    if include_rule:
        if domain == OUT_OF_SCOPE or action == OUT_OF_SCOPE:
            rule_decision = "reject"
        elif missing:
            rule_decision = "clarify"
        else:
            rule_decision = "execute"
        outputs.append(
            _record(
                architecture="janus_rule_gate",
                case=case,
                repeat=repeat,
                domain=domain,
                decision=rule_decision,
                action=action,
                parameters=parameters,
                missing=missing,
                rationale="domain routing plus deterministic required-slot gate",
                traces=traces,
                wall=upstream_wall,
            )
        )

    if include_factored:
        if domain == OUT_OF_SCOPE or action == OUT_OF_SCOPE:
            decision = "reject"
            rationale = "No implemented domain/task selected."
            gate_traces = traces
            wall = upstream_wall
        else:
            gate_system = (
                "You are JANUS Inner Speech. Audit whether the selected domain, task, "
                "and arguments faithfully match the original request and memory. Block "
                "missing, invented, ambiguous, contradictory, or impossible values and "
                "wrong domain/task selections. Proceed only when execution needs no guess. "
                "A known false boolean is not missing. MovieInfo needs at least one clear "
                "movie identifier; ExerciseInformation needs an exercise or patient name; "
                "SensorReadings needs at least one actual sample."
            )
            gate_user = json.dumps(
                {
                    "request_context": json.loads(_context(case)),
                    "selected_domain": domain,
                    "selected_action": action,
                    "parameters": parameters,
                    "missing_parameters": missing,
                    "action_contract": ALL_TASK_SPECS.get(action, {}),
                },
                ensure_ascii=False,
                sort_keys=True,
            )
            gate_started = time.perf_counter()
            gate_trace = _complete_structured(
                client,
                system=gate_system,
                user=gate_user,
                output_schema=GATE_SCHEMA,
                temperature=gate_temperature,
                interface=structured_interface,
                tool_name="InnerSpeechDecision",
                tool_description="Execution-readiness decision and internal diagnostic.",
            )
            gate_data = gate_trace.parsed or {}
            decision = "clarify" if missing or not gate_data.get("can_proceed", False) else "execute"
            rationale = str(gate_data.get("reason", ""))
            gate_traces = [*traces, gate_trace]
            wall = upstream_wall + (time.perf_counter() - gate_started)
        outputs.append(
            _record(
                architecture="janus_factored",
                case=case,
                repeat=repeat,
                domain=domain,
                decision=decision,
                action=action,
                parameters=parameters,
                missing=missing,
                rationale=rationale,
                traces=gate_traces,
                wall=wall,
            )
        )
    return outputs


MULTIDOMAIN_DIRECT_SCHEMA = {
    **DIRECT_SCHEMA,
    "properties": {
        **DIRECT_SCHEMA["properties"],
        "domain": {"type": "string", "enum": [*DOMAIN_DESCRIPTIONS, OUT_OF_SCOPE]},
        "action": {"type": "string", "enum": [*ALL_TASK_SPECS, OUT_OF_SCOPE]},
    },
    "required": [*DIRECT_SCHEMA["required"], "domain"],
}


def run_multidomain_direct(
    client: JsonLLMClient,
    case: dict[str, Any],
    *,
    repeat: int,
    temperature: float,
    structured_interface: str,
) -> dict[str, Any]:
    """Run one strong decision over every implemented domain and task."""
    contracts = json.dumps(DOMAIN_TASK_SPECS, ensure_ascii=False, sort_keys=True)
    system = f"""You are a direct multi-domain controller. In one step select the domain,
task, decision (execute, clarify, reject), and parameters. You receive the same request,
memory, tool context, and contracts as JANUS. Execute only if all information is clear
and consistent; clarify relevant ambiguity or missing information; reject only if no
implemented task applies. Never invent constraints or parameter values. A known false
boolean is not missing. Preserve known partial parameters when clarifying.

Implemented domain and task contracts: {contracts}"""
    started = time.perf_counter()
    trace = _complete_structured(
        client,
        system=system,
        user=_context(case),
        output_schema=MULTIDOMAIN_DIRECT_SCHEMA,
        temperature=temperature,
        interface=structured_interface,
        tool_name="DirectMultiDomainDecision",
        tool_description="Select one domain, task, control decision, and arguments.",
    )
    wall = time.perf_counter() - started
    data = trace.parsed or {}
    domain = _domain(data.get("domain"))
    action = normalize_action(data.get("action"), ALL_TASK_SPECS)
    parameters = normalize_parameters(action, data.get("parameters", {}), ALL_TASK_SPECS)
    decision = normalize_decision(data.get("decision"))
    if trace.parsed is None:
        domain, action, decision, parameters = OUT_OF_SCOPE, OUT_OF_SCOPE, "reject", {}
    return _record(
        architecture="direct_llm",
        case=case,
        repeat=repeat,
        domain=domain,
        decision=decision,
        action=action,
        parameters=parameters,
        missing=_semantic_missing(action, parameters),
        rationale=str(data.get("reason", "")),
        traces=[trace],
        wall=wall,
    )
