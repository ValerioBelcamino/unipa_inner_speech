"""Controller conditions for the paired reviewer benchmark."""

from __future__ import annotations

import json
import time
from dataclasses import asdict, dataclass
from typing import Any

from .llm_client import CompletionTrace, JsonLLMClient
from .task_spec import (
    OUT_OF_SCOPE,
    TASK_SPECS,
    missing_parameters,
    normalize_action,
    normalize_decision,
    normalize_parameters,
    serialized_task_specs,
)


INTENT_SCHEMA = {
    "type": "object",
    "properties": {
        "action": {"type": "string", "enum": [*TASK_SPECS.keys(), OUT_OF_SCOPE]},
        "parameters": {"type": "object"},
    },
    "required": ["action", "parameters"],
    "additionalProperties": False,
}

GATE_SCHEMA = {
    "type": "object",
    "properties": {
        "can_proceed": {"type": "boolean"},
        "reason": {"type": "string"},
    },
    "required": ["can_proceed", "reason"],
    "additionalProperties": False,
}

DIRECT_SCHEMA = {
    "type": "object",
    "properties": {
        "decision": {"type": "string", "enum": ["execute", "clarify", "reject"]},
        "action": {"type": "string", "enum": [*TASK_SPECS.keys(), OUT_OF_SCOPE]},
        "parameters": {"type": "object"},
        "reason": {"type": "string"},
        "clarification": {"type": "string"},
    },
    "required": ["decision", "action", "parameters", "reason", "clarification"],
    "additionalProperties": False,
}


@dataclass
class IntentPrediction:
    action: str
    parameters: dict[str, Any]
    missing: list[str]
    trace: CompletionTrace


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


def infer_intent(
    client: JsonLLMClient,
    case: dict[str, Any],
    *,
    temperature: float,
) -> IntentPrediction:
    """Run the factorized semantic-parsing stage."""
    system = f"""You are the Intent Recognition stage of an assistive dietary controller.
Select exactly one task and extract typed parameters from the user request, memory, and
explicit tool context. Never invent a value. Resolve a pronoun from memory only when its
referent is unambiguous. Values marked as coming from tool_context may be copied. If no
task applies, choose OutOfScope. If a relevant request is incomplete or ambiguous, still
choose its task but leave the unresolved required value absent or empty.

Available task contracts:
{serialized_task_specs()}"""
    trace = client.complete_json(
        system=system,
        user=_context(case),
        output_schema=INTENT_SCHEMA,
        temperature=temperature,
    )
    parsed = trace.parsed or {}
    action = normalize_action(parsed.get("action"))
    parameters = normalize_parameters(action, parsed.get("parameters", {}))
    return IntentPrediction(
        action=action,
        parameters=parameters,
        missing=missing_parameters(action, parameters),
        trace=trace,
    )


def _base_result(
    *,
    architecture: str,
    case: dict[str, Any],
    repeat: int,
    decision: str,
    action: str,
    parameters: dict[str, Any],
    missing: list[str],
    rationale: str,
    traces: list[CompletionTrace],
    wall_latency_seconds: float,
) -> dict[str, Any]:
    return {
        "architecture": architecture,
        "case_id": case["id"],
        "category": case["category"],
        "tags": case.get("tags", []),
        "repeat": repeat,
        "expected": case["expected"],
        "prediction": {
            "decision": decision,
            "action": action,
            "parameters": parameters,
            "missing_parameters": missing,
            "reason": rationale,
        },
        "wall_latency_seconds": wall_latency_seconds,
        "llm_latency_seconds": sum(trace.latency_seconds for trace in traces),
        "llm_calls": sum(trace.attempts for trace in traces),
        "logical_llm_stages": len(traces),
        "prompt_tokens": sum(trace.prompt_tokens for trace in traces),
        "completion_tokens": sum(trace.completion_tokens for trace in traces),
        "total_tokens": sum(trace.total_tokens for trace in traces),
        "structured_output_failure": any(trace.parsed is None for trace in traces),
        "traces": [trace.to_dict() for trace in traces],
    }


def run_factored_and_rule(
    client: JsonLLMClient,
    case: dict[str, Any],
    *,
    repeat: int,
    intent_temperature: float,
    gate_temperature: float,
    include_factored: bool,
    include_rule: bool,
) -> list[dict[str, Any]]:
    """Run both ablations with a shared Intent result for a strictly paired test."""
    started = time.perf_counter()
    intent = infer_intent(client, case, temperature=intent_temperature)
    intent_wall = time.perf_counter() - started
    outputs: list[dict[str, Any]] = []

    if include_rule:
        rule_started = time.perf_counter()
        if intent.action == OUT_OF_SCOPE:
            rule_decision = "reject"
        elif intent.missing:
            rule_decision = "clarify"
        else:
            rule_decision = "execute"
        rule_wall = time.perf_counter() - rule_started
        outputs.append(
            _base_result(
                architecture="janus_rule_gate",
                case=case,
                repeat=repeat,
                decision=rule_decision,
                action=intent.action,
                parameters=intent.parameters,
                missing=intent.missing,
                rationale="deterministic required-slot gate",
                traces=[intent.trace],
                wall_latency_seconds=intent_wall + rule_wall,
            )
        )

    if include_factored:
        gate_system = """You are the Inner Speech execution-readiness gate of an assistive
dietary controller. Decide whether execution is safe and semantically well specified.
Return can_proceed=false if the request is out of scope; a mandatory slot is missing;
the request, memory, selected task, or extracted parameters conflict; a value is
ambiguous or impossible; a numeric daily target extracted for AddToDatabase is
non-positive;
the selected task does not match the request; or parameters were invented upstream.
Return true only when the selected tool can be executed without guessing. The reason is
an internal diagnostic, not a response to the user. A nutrient or allergen name in
DishInfo.controllo_ingredienti is a property to retrieve, not a numeric target; do not
require its value before executing the retrieval tool."""
        gate_system += """
Apply these task-specific checks:
- DishInfo: a concrete dish name is enough to retrieve general information. Names such
  as calorie, proteine, grassi, carboidrati, glutine, or lattosio in
  controllo_ingredienti are requested database properties. Their values are intentionally
  unknown until the tool runs and must never cause a block.
- AddToDatabase: all daily numeric targets must be present and strictly positive.
- SubstituteDish: user, day, meal, and weekly-plan status must be known, and included,
  excluded, and exclusive-only ingredient constraints must not contradict one another.
"""
        gate_user = json.dumps(
            {
                "request_context": json.loads(_context(case)),
                "selected_action": intent.action,
                "parameters": intent.parameters,
                "missing_parameters": intent.missing,
                "action_contract": TASK_SPECS.get(intent.action, {}),
            },
            ensure_ascii=False,
            sort_keys=True,
        )
        gate_started = time.perf_counter()
        gate_trace = client.complete_json(
            system=gate_system,
            user=gate_user,
            output_schema=GATE_SCHEMA,
            temperature=gate_temperature,
        )
        gate_wall = time.perf_counter() - gate_started
        gate_data = gate_trace.parsed or {}
        can_proceed = bool(gate_data.get("can_proceed", False))
        if intent.action == OUT_OF_SCOPE:
            decision = "reject"
        elif intent.missing or not can_proceed:
            decision = "clarify"
        else:
            decision = "execute"
        outputs.append(
            _base_result(
                architecture="janus_factored",
                case=case,
                repeat=repeat,
                decision=decision,
                action=intent.action,
                parameters=intent.parameters,
                missing=intent.missing,
                rationale=str(gate_data.get("reason", "")),
                traces=[intent.trace, gate_trace],
                wall_latency_seconds=intent_wall + gate_wall,
            )
        )
    return outputs


def run_direct(
    client: JsonLLMClient,
    case: dict[str, Any],
    *,
    repeat: int,
    temperature: float,
) -> dict[str, Any]:
    """Run a strong single-LLM controller with the same information and contracts."""
    system = f"""You are a direct LLM tool-use controller for dietary assistance. In one
step, choose exactly one decision: execute a task, ask a clarification, or reject an
out-of-scope request. You receive the same user request, memory, tool context, and task
contracts as the factored controller. Never invent parameters. Resolve a pronoun from
memory only when unambiguous. Use execute only if every required value is present and
the request and extracted values are mutually consistent, unambiguous, and safe to pass
to the tool. Use clarify for a relevant but incomplete, ambiguous, contradictory, or
invalid request. Use reject only when no task applies. If clarifying, state one concise
question in the clarification field; otherwise make it empty. Always preserve every
parameter explicitly provided by the user or unambiguously resolved from memory or tool
context in the parameters object, including when the decision is clarify; omit only
unknown values.

Available task contracts:
{serialized_task_specs()}"""
    started = time.perf_counter()
    trace = client.complete_json(
        system=system,
        user=_context(case),
        output_schema=DIRECT_SCHEMA,
        temperature=temperature,
    )
    wall = time.perf_counter() - started
    parsed = trace.parsed or {}
    action = normalize_action(parsed.get("action"))
    parameters = normalize_parameters(action, parsed.get("parameters", {}))
    decision = normalize_decision(parsed.get("decision"))
    if trace.parsed is None:
        decision = "reject"
    return _base_result(
        architecture="direct_llm",
        case=case,
        repeat=repeat,
        decision=decision,
        action=action,
        parameters=parameters,
        missing=missing_parameters(action, parameters),
        rationale=str(parsed.get("reason", "")),
        traces=[trace],
        wall_latency_seconds=wall,
    )


def run_readiness_gate(
    client: JsonLLMClient,
    case: dict[str, Any],
    *,
    repeat: int,
    temperature: float,
    include_inner: bool = True,
    include_rule: bool = True,
) -> list[dict[str, Any]]:
    """Compare Inner Speech to a rule gate on identical, frozen upstream state."""
    action = normalize_action(case["action"])
    parameters = normalize_parameters(action, case.get("parameters", {}))
    missing = list(case.get("missing_parameters", []))
    expected = {"can_proceed": bool(case["expected_can_proceed"])}

    rule_started = time.perf_counter()
    rule_prediction = action != OUT_OF_SCOPE and not missing
    rule_wall = time.perf_counter() - rule_started
    rule_result = {
        "architecture": "rule_gate",
        "case_id": case["id"],
        "category": case["category"],
        "tags": case.get("tags", []),
        "repeat": repeat,
        "expected": expected,
        "prediction": {"can_proceed": rule_prediction, "reason": "required-slot rule"},
        "wall_latency_seconds": rule_wall,
        "llm_latency_seconds": 0.0,
        "llm_calls": 0,
        "logical_llm_stages": 0,
        "prompt_tokens": 0,
        "completion_tokens": 0,
        "total_tokens": 0,
        "structured_output_failure": False,
        "traces": [],
    }

    outputs = [rule_result] if include_rule else []
    if not include_inner:
        return outputs

    gate_system = """You are the Inner Speech execution-readiness gate of an assistive
dietary controller. Audit the frozen upstream state against the original request and
memory. Return false for missing required information, out-of-scope or mismatched task,
ambiguity, contradiction, impossible values, non-positive numeric daily targets in
AddToDatabase,
unsupported extracted parameters, or conflicting include/exclude constraints. Return
true only if the selected action can be executed without guessing. A nutrient name in
DishInfo.controllo_ingredienti is the requested property, not a missing numeric value."""
    gate_system += """
Task-specific checks: DishInfo needs a concrete dish, but requested properties such as
calorie, proteine, grassi, carboidrati, glutine, and lattosio are meant to be unknown
before retrieval and never justify blocking. AddToDatabase numeric daily targets must be
strictly positive. SubstituteDish constraints must not contradict one another.
"""
    gate_user = json.dumps(
        {
            "user_input": case["user_input"],
            "memory": case.get("memory", []),
            "tool_context": case.get("tool_context", {}),
            "selected_action": action,
            "parameters": parameters,
            "missing_parameters": missing,
            "action_contract": TASK_SPECS.get(action, {}),
        },
        ensure_ascii=False,
        sort_keys=True,
    )
    started = time.perf_counter()
    trace = client.complete_json(
        system=gate_system,
        user=gate_user,
        output_schema=GATE_SCHEMA,
        temperature=temperature,
    )
    wall = time.perf_counter() - started
    data = trace.parsed or {}
    llm_result = {
        "architecture": "inner_speech_gate",
        "case_id": case["id"],
        "category": case["category"],
        "tags": case.get("tags", []),
        "repeat": repeat,
        "expected": expected,
        "prediction": {
            "can_proceed": bool(data.get("can_proceed", False)),
            "reason": str(data.get("reason", "")),
        },
        "wall_latency_seconds": wall,
        "llm_latency_seconds": trace.latency_seconds,
        "llm_calls": trace.attempts,
        "logical_llm_stages": 1,
        "prompt_tokens": trace.prompt_tokens,
        "completion_tokens": trace.completion_tokens,
        "total_tokens": trace.total_tokens,
        "structured_output_failure": trace.parsed is None,
        "traces": [trace.to_dict()],
    }
    outputs.append(llm_result)
    return outputs
