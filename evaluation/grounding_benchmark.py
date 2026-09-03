"""Run the held-out JANUS versus Direct-LLM evidence-grounding benchmark."""

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
from typing import Any

from dotenv import load_dotenv

from .controllers import run_factored_and_rule
from .grounding import (
    DEFAULT_DATASET,
    REPO_ROOT,
    ReferenceDatabase,
    audit_prompt_overlap,
    load_dataset,
    prior_prompt_paths,
    validate_dataset,
)
from .grounding_metrics import score_grounding_record, write_grounding_summary
from .llm_client import CompletionTrace, JsonLLMClient, ProviderRateLimitError
from .module_benchmark import NEO4J_PROMPT, QUERY_TOOLS, Neo4jExecutor
from .task_spec import (
    OUT_OF_SCOPE,
    TASK_SPECS,
    missing_parameters,
    normalize_action,
    normalize_decision,
    normalize_parameters,
    serialized_task_specs,
)


SCENARIO_ROOT = (
    REPO_ROOT / "scenario_customization" / "scenario_customization" / "ADVISOR"
)
EVALUATED_ACTIONS = ("DishInfo", "SubstituteDish")
ANSWER_SCHEMA = {
    "type": "object",
    "properties": {
        "abstain": {"type": "boolean"},
        "claims": {
            "type": "array",
            "items": {
                "type": "object",
                "properties": {
                    "entity": {"type": "string"},
                    "field": {"type": "string"},
                    "value_json": {"type": "string"},
                },
                "required": ["entity", "field", "value_json"],
                "additionalProperties": False,
            },
        },
        "answer": {"type": "string"},
    },
    "required": ["abstain", "claims", "answer"],
    "additionalProperties": False,
}
DIRECT_PLAN_SCHEMA = {
    "type": "object",
    "properties": {
        "decision": {"type": "string", "enum": ["execute", "clarify", "reject"]},
        "action": {"type": "string", "enum": [*TASK_SPECS.keys(), OUT_OF_SCOPE]},
        "parameters": {"type": "object"},
        "queries": {"type": "array", "items": {"type": "string"}},
        "reason": {"type": "string"},
    },
    "required": ["decision", "action", "parameters", "queries", "reason"],
    "additionalProperties": False,
}


def _runtime_query_examples(action: str) -> list[dict[str, Any]]:
    path = SCENARIO_ROOT / action / "query_examples" / f"{action}.json"
    if not path.exists():
        return []
    examples = json.loads(path.read_text(encoding="utf-8"))
    prepared: list[dict[str, Any]] = []
    for example in examples:
        queries = [
            value
            for key, value in example.items()
            if "query" in key.casefold() and isinstance(value, str)
        ]
        prepared.append(
            {
                "question": example["question"],
                "parameters": example["parameters"],
                "queries": queries,
            }
        )
    return prepared


def format_janus_query_prompt(
    *, action: str, question: str, parameters: dict[str, Any], database_schema: str
) -> str:
    blocks = [NEO4J_PROMPT.format(schema=database_schema)]
    for example in _runtime_query_examples(action):
        blocks.append(
            "User asks: {question}\nParameters: {parameters}\nQueries: {queries}".format(
                question=example["question"],
                parameters=example["parameters"],
                queries=json.dumps(example["queries"], ensure_ascii=False),
            )
        )
    blocks.append(
        "User asks: {question}\nParameters: {parameters}\nQuery: ".format(
            question=question,
            parameters=json.dumps(parameters, ensure_ascii=False, sort_keys=True),
        )
    )
    return "\n\n".join(blocks)


def format_direct_plan_prompt(case: dict[str, Any], database_schema: str) -> tuple[str, str]:
    demonstrations: list[str] = []
    # The held-out suite contains only these two read-task families. Supplying
    # unrelated AddToDatabase write demonstrations would inflate and weaken the
    # Direct baseline without giving JANUS information relevant to these cases.
    for action in EVALUATED_ACTIONS:
        for example in _runtime_query_examples(action):
            demonstrations.append(
                "Action: {action}\nUser asks: {question}\nParameters: {parameters}"
                "\nQueries: {queries}".format(
                    action=action,
                    question=example["question"],
                    parameters=example["parameters"],
                    queries=json.dumps(example["queries"], ensure_ascii=False),
                )
            )
    system = f"""You are a strong direct LLM tool-use controller for dietary assistance.
In one planning step, select the task, extract only supported parameters, decide whether
to execute, clarify, or reject, and generate the Neo4j 5 Cypher needed to answer. You
receive the same task contracts, database schema, and query demonstrations available to
the factored system. Never invent an entity or parameter. Generate no query unless the
decision is execute. For read tasks, generate only read-only Cypher using the supplied
schema. Always alias returned fields. If evidence is absent, the later response step will
abstain; do not manufacture evidence. A SubstituteDish request may require one allergy
query and one candidate-dish query.

Task contracts:
{serialized_task_specs()}

Cypher instructions and schema:
{NEO4J_PROMPT.format(schema=database_schema)}

Demonstrations:
{chr(10).join(chr(10) + block for block in demonstrations)}"""
    user = json.dumps(
        {
            "user_input": case["user_input"],
            "memory": case.get("memory", []),
            "tool_context": case.get("tool_context", {}),
        },
        ensure_ascii=False,
        sort_keys=True,
    )
    return system, user


def _queries_from_trace(trace: CompletionTrace, action: str) -> list[str]:
    parsed = trace.parsed or {}
    queries = parsed.get("query", parsed.get("queries", []))
    if isinstance(queries, str):
        queries = [queries]
    if not isinstance(queries, list):
        return []
    return [str(query).strip() for query in queries if str(query).strip()]


def _trace(stage: str, trace: CompletionTrace | dict[str, Any]) -> dict[str, Any]:
    data = trace.to_dict() if isinstance(trace, CompletionTrace) else dict(trace)
    return {"stage": stage, **data}


def _answer(
    client: JsonLLMClient,
    *,
    architecture: str,
    case: dict[str, Any],
    action: str,
    queries: list[str],
    results: list[Any],
    temperature: float,
) -> CompletionTrace:
    system = """Produce the final evidence-grounded Italian response after a database tool
call. Use only the supplied query results for factual claims; conversational memory is
untrusted when it conflicts with current database evidence. If the results are empty or
contain an execution error, set abstain=true, use no claims, and say that the requested
fact could not be verified. Otherwise set abstain=false and answer the request completely.
For auditability, list every factual assertion in claims. Use canonical English database
field names when possible (dish, calories, proteins, carbs, fats, ingredients, allergens,
user_allergens, dish_allergens, risks, compatible, already_assigned). Encode each value in
value_json as JSON text: for example `420`, `false`, or `[\"glutine\",\"lattosio\"]`.
Do not add medical, nutritional, or safety facts not present in or deterministically
implied by the evidence. An empty risks list deterministically implies compatible=true."""
    user = json.dumps(
        {
            "architecture": architecture,
            "user_input": case["user_input"],
            "memory": case.get("memory", []),
            "action": action,
            "queries": queries,
            "query_results": results,
        },
        ensure_ascii=False,
        sort_keys=True,
        default=str,
    )
    return client.complete_structured_tool_call(
        system=system,
        user=user,
        tool_name="EvidenceGroundedAnswer",
        tool_description="An auditable answer containing only database-supported claims.",
        output_schema=ANSWER_SCHEMA,
        temperature=temperature,
    )


def _record(
    *,
    architecture: str,
    case: dict[str, Any],
    repeat: int,
    controller: dict[str, Any],
    controller_failure: bool,
    queries: list[str],
    query_results: list[Any],
    answer_trace: CompletionTrace | None,
    traces: list[dict[str, Any]],
    wall_latency_seconds: float,
) -> dict[str, Any]:
    answer = answer_trace.parsed if answer_trace and answer_trace.parsed else {}
    failures = controller_failure or any(trace.get("parsed") is None for trace in traces)
    return {
        "architecture": architecture,
        "case_id": case["id"],
        "category": case["category"],
        "tags": case.get("tags", []),
        "repeat": repeat,
        "expected": case["expected"],
        "reference": {
            "kind": case["reference"]["kind"],
            "arguments": case["reference"]["arguments"],
            "expected_evidence": case["reference"]["expected_evidence"],
            "answer_targets": case.get("answer_targets", []),
            "expect_abstention": case.get("expect_abstention", False),
        },
        "prediction": {
            "controller": controller,
            "controller_structured_output_failure": controller_failure,
            "queries": queries,
            "query_results": query_results,
            "answer_attempted": answer_trace is not None,
            "answer": answer,
        },
        "wall_latency_seconds": wall_latency_seconds,
        "llm_latency_seconds": sum(float(trace.get("latency_seconds", 0)) for trace in traces),
        "llm_calls": sum(int(trace.get("attempts", 0)) for trace in traces),
        "logical_llm_stages": len(traces),
        "prompt_tokens": sum(int(trace.get("prompt_tokens", 0)) for trace in traces),
        "completion_tokens": sum(int(trace.get("completion_tokens", 0)) for trace in traces),
        "total_tokens": sum(int(trace.get("total_tokens", 0)) for trace in traces),
        "structured_output_failure": failures,
        "traces": traces,
    }


def run_janus_grounding(
    client: JsonLLMClient,
    database: Neo4jExecutor,
    database_schema: str,
    case: dict[str, Any],
    *,
    repeat: int,
    intent_temperature: float,
    gate_temperature: float,
    query_temperature: float,
    answer_temperature: float,
    include_answer: bool,
) -> dict[str, Any]:
    started = time.perf_counter()
    control = run_factored_and_rule(
        client,
        case,
        repeat=repeat,
        intent_temperature=intent_temperature,
        gate_temperature=gate_temperature,
        include_factored=True,
        include_rule=False,
        intent_interface="native_tools",
        structured_interface="native_tools",
    )[0]
    controller = control["prediction"]
    traces = [
        _trace(stage, trace)
        for stage, trace in zip(("intent", "inner_speech"), control["traces"])
    ]
    queries: list[str] = []
    query_results: list[Any] = []
    answer_trace: CompletionTrace | None = None
    action = normalize_action(controller.get("action"))
    if controller.get("decision") == "execute" and action in QUERY_TOOLS:
        tool = QUERY_TOOLS[action]
        query_trace = client.complete_structured_tool_call(
            system=None,
            user=format_janus_query_prompt(
                action=action,
                question=case["user_input"],
                parameters=controller.get("parameters", {}),
                database_schema=database_schema,
            ),
            tool_name=str(tool["name"]),
            tool_description=str(tool["description"]),
            output_schema=tool["schema"],
            temperature=query_temperature,
        )
        queries = _queries_from_trace(query_trace, action)
        query_results = database.execute(queries, action)
        traces.append(_trace("query_generation", query_trace))
        if include_answer:
            answer_trace = _answer(
                client,
                architecture="janus_factored",
                case=case,
                action=action,
                queries=queries,
                results=query_results,
                temperature=answer_temperature,
            )
            traces.append(_trace("outer_speech", answer_trace))
    return _record(
        architecture="janus_factored",
        case=case,
        repeat=repeat,
        controller=controller,
        controller_failure=bool(control.get("structured_output_failure", False)),
        queries=queries,
        query_results=query_results,
        answer_trace=answer_trace,
        traces=traces,
        wall_latency_seconds=time.perf_counter() - started,
    )


def run_direct_grounding(
    client: JsonLLMClient,
    database: Neo4jExecutor,
    database_schema: str,
    case: dict[str, Any],
    *,
    repeat: int,
    plan_temperature: float,
    answer_temperature: float,
    include_answer: bool,
) -> dict[str, Any]:
    started = time.perf_counter()
    system, user = format_direct_plan_prompt(case, database_schema)
    plan_trace = client.complete_structured_tool_call(
        system=system,
        user=user,
        tool_name="DirectToolUsePlan",
        tool_description="A direct control decision with task arguments and Cypher queries.",
        output_schema=DIRECT_PLAN_SCHEMA,
        temperature=plan_temperature,
    )
    parsed = plan_trace.parsed or {}
    action = normalize_action(parsed.get("action"))
    parameters = normalize_parameters(action, parsed.get("parameters", {}))
    if action == "SubstituteDish" and "ha_piano_settimanale" in case.get(
        "tool_context", {}
    ):
        parameters["ha_piano_settimanale"] = bool(
            case["tool_context"]["ha_piano_settimanale"]
        )
    decision = normalize_decision(parsed.get("decision"))
    if plan_trace.parsed is None:
        decision = "reject"
    controller = {
        "decision": decision,
        "action": action,
        "parameters": parameters,
        "missing_parameters": missing_parameters(action, parameters),
        "reason": str(parsed.get("reason", "")),
    }
    queries = _queries_from_trace(plan_trace, action) if decision == "execute" else []
    query_results = database.execute(queries, action) if queries else []
    traces = [_trace("direct_plan_and_query", plan_trace)]
    answer_trace: CompletionTrace | None = None
    if decision == "execute" and include_answer:
        answer_trace = _answer(
            client,
            architecture="direct_llm",
            case=case,
            action=action,
            queries=queries,
            results=query_results,
            temperature=answer_temperature,
        )
        traces.append(_trace("direct_post_tool_answer", answer_trace))
    return _record(
        architecture="direct_llm",
        case=case,
        repeat=repeat,
        controller=controller,
        controller_failure=plan_trace.parsed is None,
        queries=queries,
        query_results=query_results,
        answer_trace=answer_trace,
        traces=traces,
        wall_latency_seconds=time.perf_counter() - started,
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, default=DEFAULT_DATASET)
    parser.add_argument("--provider", choices=("groq", "custom"), default="groq")
    parser.add_argument("--model", default="qwen/qwen3.8-27b")
    parser.add_argument("--base-url")
    parser.add_argument("--api-key-env", default="GROQ_API_KEY")
    parser.add_argument("--architectures", default="factored,direct")
    parser.add_argument("--neo4j-uri", default="bolt://localhost:19687")
    parser.add_argument("--neo4j-username", default="neo4j")
    parser.add_argument("--neo4j-password", default="password")
    parser.add_argument("--max-cases", type=int)
    parser.add_argument("--case-id", action="append", default=[])
    parser.add_argument("--category", action="append", default=[])
    parser.add_argument("--repeats", type=int, default=1)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--request-delay", type=float, default=7.5)
    parser.add_argument("--max-retry-wait", type=float, default=60.0)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--max-attempts", type=int, default=2)
    parser.add_argument("--max-completion-tokens", type=int, default=1024)
    parser.add_argument("--reasoning-effort", default="none")
    parser.add_argument("--intent-temperature", type=float, default=0.0)
    parser.add_argument("--gate-temperature", type=float, default=0.2)
    parser.add_argument("--query-temperature", type=float, default=0.0)
    parser.add_argument("--direct-temperature", type=float, default=0.0)
    parser.add_argument("--answer-temperature", type=float, default=0.0)
    parser.add_argument("--skip-answer", action="store_true")
    parser.add_argument("--validate-only", action="store_true")
    return parser


def _slug(value: str) -> str:
    return re.sub(r"[^a-zA-Z0-9._-]+", "-", value).strip("-")


def _git_sha() -> str:
    return subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=REPO_ROOT, text=True
    ).strip()


def _git_dirty() -> bool:
    return bool(
        subprocess.check_output(
            ["git", "status", "--porcelain"], cwd=REPO_ROOT, text=True
        ).strip()
    )


def _read_records(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line]


def _selected_architectures(value: str) -> set[str]:
    selected = {part.strip().casefold() for part in value.split(",") if part.strip()}
    unknown = selected - {"factored", "direct"}
    if not selected or unknown:
        raise SystemExit(f"architectures must be factored/direct, got {sorted(selected)}")
    return selected


def _validate_and_open_databases(args: argparse.Namespace) -> tuple[dict[str, Any], Neo4jExecutor]:
    dataset = load_dataset(args.dataset)
    reference = ReferenceDatabase(args.neo4j_uri, args.neo4j_username, args.neo4j_password)
    try:
        report = validate_dataset(dataset, reference)
        overlap = audit_prompt_overlap(dataset, prior_prompt_paths(args.dataset))
        report["prompt_overlap_audit"] = overlap
        if not overlap["valid"]:
            raise ValueError(json.dumps(report, ensure_ascii=False, indent=2))
    finally:
        reference.close()
    return dataset, Neo4jExecutor(args.neo4j_uri, args.neo4j_username, args.neo4j_password)


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    load_dotenv(REPO_ROOT / ".env", override=False)
    dataset, database = _validate_and_open_databases(args)
    try:
        cases = list(dataset["cases"])
        if args.category:
            cases = [case for case in cases if case["category"] in set(args.category)]
        if args.case_id:
            cases = [case for case in cases if case["id"] in set(args.case_id)]
        if args.max_cases is not None:
            cases = cases[: max(0, args.max_cases)]
        if not cases:
            raise SystemExit("No cases selected")
        database_schema = database.schema()
        if args.validate_only:
            print(
                json.dumps(
                    {
                        "valid": True,
                        "selected_cases": len(cases),
                        "dataset_sha256": hashlib.sha256(args.dataset.read_bytes()).hexdigest(),
                        "database_fingerprint_sha256": dataset["database"]["fingerprint_sha256"],
                    },
                    indent=2,
                )
            )
            return 0

        selected = _selected_architectures(args.architectures)
        base_url = args.base_url or "https://api.groq.com/openai/v1"
        if args.provider == "custom" and not args.base_url:
            raise SystemExit("--provider custom requires --base-url")
        api_key = os.getenv(args.api_key_env, "")
        if not api_key:
            raise SystemExit(f"Missing API key in {args.api_key_env}")
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        output_dir = args.output_dir or (
            Path(__file__).parent
            / "results"
            / "grounding"
            / f"{timestamp}_{args.provider}_{_slug(args.model)}"
        )
        output_dir.mkdir(parents=True, exist_ok=True)
        raw_path = output_dir / "raw.jsonl"
        existing = _read_records(raw_path)
        completed = {
            (record["architecture"], record["case_id"], int(record["repeat"]))
            for record in existing
        }
        metadata = {
            "created_at_utc": timestamp,
            "suite": "grounding_v1",
            "dataset": str(args.dataset.resolve().relative_to(REPO_ROOT)),
            "dataset_sha256": hashlib.sha256(args.dataset.read_bytes()).hexdigest(),
            "database_fingerprint_sha256": dataset["database"]["fingerprint_sha256"],
            "git_sha": _git_sha(),
            "git_dirty": _git_dirty(),
            "git_branch": subprocess.check_output(
                ["git", "branch", "--show-current"], cwd=REPO_ROOT, text=True
            ).strip(),
            "provider": args.provider,
            "model": args.model,
            "base_url": base_url,
            "architectures": sorted(selected),
            "repeats": args.repeats,
            "reasoning_effort": args.reasoning_effort,
            "temperatures": {
                "intent": args.intent_temperature,
                "gate": args.gate_temperature,
                "query": args.query_temperature,
                "direct": args.direct_temperature,
                "answer": args.answer_temperature,
            },
            "include_answer": not args.skip_answer,
            "max_attempts": args.max_attempts,
            "max_completion_tokens": args.max_completion_tokens,
            "request_delay": args.request_delay,
            "case_ids": [case["id"] for case in cases],
            "hardware": {
                "platform": platform.platform(),
                "cpu": platform.processor(),
                "logical_cpu_count": os.cpu_count(),
                "python": sys.version,
            },
        }
        metadata_path = output_dir / "metadata.json"
        if existing and metadata_path.exists():
            previous = json.loads(metadata_path.read_text(encoding="utf-8"))
            immutable = (
                "dataset_sha256",
                "database_fingerprint_sha256",
                "provider",
                "model",
                "base_url",
                "architectures",
                "repeats",
                "reasoning_effort",
                "temperatures",
                "include_answer",
                "max_attempts",
                "max_completion_tokens",
                "case_ids",
            )
            changed = [name for name in immutable if previous.get(name) != metadata.get(name)]
            if changed:
                raise SystemExit("Cannot resume changed configuration: " + ", ".join(changed))
            metadata = previous
        metadata_path.write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        client = JsonLLMClient(
            model=args.model,
            api_key=api_key,
            base_url=base_url,
            timeout=args.timeout,
            max_attempts=args.max_attempts,
            max_completion_tokens=args.max_completion_tokens,
            request_delay=args.request_delay,
            max_retry_wait=args.max_retry_wait,
            reasoning_effort=args.reasoning_effort,
        )

        with raw_path.open("a", encoding="utf-8") as raw_handle:
            for repeat in range(args.repeats):
                for position, case in enumerate(cases, 1):
                    print(
                        f"[{repeat + 1}/{args.repeats} {position}/{len(cases)}] {case['id']}",
                        flush=True,
                    )
                    produced: list[dict[str, Any]] = []
                    if (
                        "factored" in selected
                        and ("janus_factored", case["id"], repeat) not in completed
                    ):
                        produced.append(
                            run_janus_grounding(
                                client,
                                database,
                                database_schema,
                                case,
                                repeat=repeat,
                                intent_temperature=args.intent_temperature,
                                gate_temperature=args.gate_temperature,
                                query_temperature=args.query_temperature,
                                answer_temperature=args.answer_temperature,
                                include_answer=not args.skip_answer,
                            )
                        )
                    if (
                        "direct" in selected
                        and ("direct_llm", case["id"], repeat) not in completed
                    ):
                        produced.append(
                            run_direct_grounding(
                                client,
                                database,
                                database_schema,
                                case,
                                repeat=repeat,
                                plan_temperature=args.direct_temperature,
                                answer_temperature=args.answer_temperature,
                                include_answer=not args.skip_answer,
                            )
                        )
                    for record in produced:
                        record["scores"] = score_grounding_record(record)
                        raw_handle.write(
                            json.dumps(record, ensure_ascii=False, sort_keys=True) + "\n"
                        )
                        raw_handle.flush()
                        completed.add((record["architecture"], record["case_id"], repeat))

        records = _read_records(raw_path)
        rows = write_grounding_summary(records, output_dir)
        print(f"Results: {output_dir}")
        for row in rows:
            if row["category"] == "ALL":
                print(
                    f"  {row['architecture']}: n={row['n']} "
                    f"retrieval={row['exact_retrieval_success']:.3f} "
                    f"grounded={row['grounded_task_success']:.3f} "
                    f"tokens={row['total_tokens_median']:.0f} "
                    f"p50={row['wall_latency_p50_seconds']:.3f}s"
                )
        return 0
    finally:
        database.close()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except ProviderRateLimitError as exc:
        retry = (
            f" Suggested retry delay: {exc.retry_after_seconds:.1f}s."
            if exc.retry_after_seconds is not None
            else ""
        )
        print("Provider rate limit paused the run." + retry, file=sys.stderr)
        raise SystemExit(75) from None
