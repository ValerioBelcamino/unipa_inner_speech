"""CLI for the paired JANUS reviewer benchmarks.

Examples:
    python -m evaluation.benchmark --suite controller --provider groq
    python -m evaluation.benchmark --suite readiness --provider ollama --model qwen2.5:3b-instruct
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
import urllib.request
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from dotenv import load_dotenv

from .controllers import run_direct, run_factored_and_rule, run_readiness_gate
from .llm_client import JsonLLMClient
from .metrics import write_summary


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DATASETS = {
    "controller": Path(__file__).parent / "datasets" / "controller_v1.json",
    "readiness": Path(__file__).parent / "datasets" / "readiness_v1.json",
}


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--suite", choices=sorted(DEFAULT_DATASETS), default="controller")
    parser.add_argument("--dataset", type=Path)
    parser.add_argument("--provider", choices=("groq", "ollama", "custom"), default="groq")
    parser.add_argument("--model")
    parser.add_argument("--base-url")
    parser.add_argument("--api-key-env", default=None)
    parser.add_argument("--architectures", default="all")
    parser.add_argument(
        "--intent-interface",
        choices=("native_tools", "json"),
        default="native_tools",
        help="JANUS Intent output interface; native_tools mirrors the runtime bind_tools path",
    )
    parser.add_argument(
        "--structured-interface",
        choices=("native_tools", "json"),
        default="native_tools",
        help=(
            "typed-output interface for Inner Speech and Direct; native_tools "
            "mirrors LangChain Groq with_structured_output"
        ),
    )
    parser.add_argument("--repeats", type=int, default=1)
    parser.add_argument("--max-cases", type=int)
    parser.add_argument("--category", action="append", default=[])
    parser.add_argument("--case-id", action="append", default=[])
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--request-delay", type=float, default=0.0)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--max-attempts", type=int, default=2)
    parser.add_argument("--max-completion-tokens", type=int)
    parser.add_argument("--intent-temperature", type=float, default=0.0)
    parser.add_argument("--gate-temperature", type=float, default=0.2)
    parser.add_argument("--direct-temperature", type=float, default=0.0)
    parser.add_argument("--validate-only", action="store_true")
    return parser


def _provider_config(args: argparse.Namespace) -> tuple[str, str, str]:
    if args.provider == "groq":
        model = args.model or "qwen/qwen3.8-27b"
        base_url = args.base_url or "https://api.groq.com/openai/v1"
        key_env = args.api_key_env or "GROQ_API_KEY"
        api_key = os.getenv(key_env, "")
        if not api_key:
            raise SystemExit(f"Missing API key: set {key_env} in the environment or .env")
    elif args.provider == "ollama":
        model = args.model or "qwen2.5:3b-instruct"
        base_url = args.base_url or "http://localhost:11434/v1"
        api_key = "ollama"
    else:
        if not args.model or not args.base_url:
            raise SystemExit("--provider custom requires --model and --base-url")
        model = args.model
        base_url = args.base_url
        key_env = args.api_key_env or "OPENAI_API_KEY"
        api_key = os.getenv(key_env, "not-needed")
    return model, base_url, api_key


def _load_cases(path: Path, suite: str) -> list[dict[str, Any]]:
    with path.open("r", encoding="utf-8") as handle:
        cases = json.load(handle)
    if not isinstance(cases, list) or not cases:
        raise ValueError("dataset must be a non-empty JSON array")
    ids: set[str] = set()
    for index, case in enumerate(cases):
        for field in ("id", "category", "user_input"):
            if field not in case:
                raise ValueError(f"case {index} is missing {field}")
        if case["id"] in ids:
            raise ValueError(f"duplicate case id: {case['id']}")
        ids.add(case["id"])
        if suite == "controller" and "expected" not in case:
            raise ValueError(f"controller case {case['id']} has no expected object")
        if suite == "readiness":
            for field in ("action", "parameters", "missing_parameters", "expected_can_proceed"):
                if field not in case:
                    raise ValueError(f"readiness case {case['id']} is missing {field}")
    return cases


def _git_sha() -> str:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=REPO_ROOT, text=True
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def _git_dirty() -> bool:
    try:
        status = subprocess.check_output(
            ["git", "status", "--porcelain"], cwd=REPO_ROOT, text=True
        )
        return bool(status.strip())
    except (OSError, subprocess.CalledProcessError):
        return True


def _hardware_metadata() -> dict[str, Any]:
    cpu_model = platform.processor()
    cpuinfo = Path("/proc/cpuinfo")
    if cpuinfo.exists():
        for line in cpuinfo.read_text(encoding="utf-8", errors="replace").splitlines():
            if line.lower().startswith("model name"):
                cpu_model = line.split(":", 1)[-1].strip()
                break
    memory_bytes = None
    meminfo = Path("/proc/meminfo")
    if meminfo.exists():
        for line in meminfo.read_text(encoding="utf-8", errors="replace").splitlines():
            if line.startswith("MemTotal:"):
                memory_bytes = int(line.split()[1]) * 1024
                break
    return {
        "platform": platform.platform(),
        "cpu_model": cpu_model,
        "logical_cpu_count": os.cpu_count(),
        "memory_bytes": memory_bytes,
        "python": sys.version,
    }


def _ollama_metadata(base_url: str, model: str) -> dict[str, Any] | None:
    """Read local model digest/quantization without adding an HTTP dependency."""
    root = base_url.removesuffix("/v1").rstrip("/")
    try:
        request = urllib.request.Request(
            f"{root}/api/show",
            data=json.dumps({"model": model}).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urllib.request.urlopen(request, timeout=10) as response:
            shown = json.load(response)
        with urllib.request.urlopen(f"{root}/api/tags", timeout=10) as response:
            tags = json.load(response)
        selected = next(
            (
                entry
                for entry in tags.get("models", [])
                if entry.get("name") == model or entry.get("model") == model
            ),
            {},
        )
        return {
            "digest": selected.get("digest"),
            "size_bytes": selected.get("size"),
            "details": shown.get("details", selected.get("details")),
            "capabilities": shown.get("capabilities"),
        }
    except Exception as exc:
        return {"metadata_error": f"{type(exc).__name__}: {exc}"}


def _slug(value: str) -> str:
    return re.sub(r"[^a-zA-Z0-9._-]+", "-", value).strip("-")


def _read_records(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    records = []
    with path.open("r", encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, 1):
            if line.strip():
                try:
                    records.append(json.loads(line))
                except json.JSONDecodeError as exc:
                    raise ValueError(f"invalid JSONL at {path}:{line_number}: {exc}") from exc
    return records


def _architectures(args: argparse.Namespace) -> set[str]:
    if args.architectures == "all":
        return {"factored", "rule", "direct"} if args.suite == "controller" else {"inner", "rule"}
    selected = {part.strip().lower() for part in args.architectures.split(",") if part.strip()}
    allowed = {"factored", "rule", "direct"} if args.suite == "controller" else {"inner", "rule"}
    unknown = selected - allowed
    if unknown:
        raise SystemExit(f"Unknown architectures for {args.suite}: {sorted(unknown)}")
    return selected


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    load_dotenv(REPO_ROOT / ".env", override=False)
    dataset_path = (args.dataset or DEFAULT_DATASETS[args.suite]).resolve()
    cases = _load_cases(dataset_path, args.suite)
    if args.category:
        cases = [case for case in cases if case["category"] in set(args.category)]
    if args.case_id:
        cases = [case for case in cases if case["id"] in set(args.case_id)]
    if args.max_cases is not None:
        cases = cases[: max(0, args.max_cases)]
    if not cases:
        raise SystemExit("No cases selected")
    selected = _architectures(args)
    if args.validate_only:
        print(f"Validated {len(cases)} {args.suite} cases from {dataset_path}")
        return 0

    model, base_url, api_key = _provider_config(args)
    if args.max_completion_tokens is None:
        # Groq reasoning models count internal reasoning toward this budget and
        # can exhaust a 256-token cap before emitting their JSON document.
        args.max_completion_tokens = 1024 if args.provider == "groq" else 256
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output_dir = args.output_dir or (
        Path(__file__).parent / "results" / args.suite / f"{timestamp}_{args.provider}_{_slug(model)}"
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    raw_path = output_dir / "raw.jsonl"
    existing = _read_records(raw_path)
    completed = {
        (record["architecture"], record["case_id"], int(record["repeat"])) for record in existing
    }
    dataset_bytes = dataset_path.read_bytes()
    metadata = {
        "created_at_utc": timestamp,
        "suite": args.suite,
        "dataset": str(dataset_path.relative_to(REPO_ROOT)),
        "dataset_sha256": hashlib.sha256(dataset_bytes).hexdigest(),
        "git_sha": _git_sha(),
        "git_dirty": _git_dirty(),
        "git_branch": subprocess.check_output(
            ["git", "branch", "--show-current"], cwd=REPO_ROOT, text=True
        ).strip(),
        "provider": args.provider,
        "model": model,
        "base_url": base_url,
        "architectures": sorted(selected),
        "intent_interface": args.intent_interface,
        "structured_interface": args.structured_interface,
        "repeats": args.repeats,
        "temperatures": {
            "intent": args.intent_temperature,
            "gate": args.gate_temperature,
            "direct": args.direct_temperature,
        },
        "max_attempts": args.max_attempts,
        "max_completion_tokens": args.max_completion_tokens,
        "request_delay": args.request_delay,
        "hardware": _hardware_metadata(),
        "case_ids": [case["id"] for case in cases],
    }
    if args.provider == "ollama":
        metadata["local_model"] = _ollama_metadata(base_url, model)
    (output_dir / "metadata.json").write_text(
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
    )

    total = len(cases) * args.repeats
    with raw_path.open("a", encoding="utf-8") as raw_handle:
        for repeat in range(args.repeats):
            for position, case in enumerate(cases, 1):
                print(f"[{repeat + 1}/{args.repeats} {position}/{len(cases)}] {case['id']}", flush=True)
                produced: list[dict[str, Any]] = []
                if args.suite == "controller":
                    need_factored = (
                        "factored" in selected
                        and ("janus_factored", case["id"], repeat) not in completed
                    )
                    need_rule = (
                        "rule" in selected
                        and ("janus_rule_gate", case["id"], repeat) not in completed
                    )
                    if need_factored or need_rule:
                        produced.extend(
                            run_factored_and_rule(
                                client,
                                case,
                                repeat=repeat,
                                intent_temperature=args.intent_temperature,
                                gate_temperature=args.gate_temperature,
                                include_factored=need_factored,
                                include_rule=need_rule,
                                intent_interface=args.intent_interface,
                                structured_interface=args.structured_interface,
                            )
                        )
                    if (
                        "direct" in selected
                        and ("direct_llm", case["id"], repeat) not in completed
                    ):
                        produced.append(
                            run_direct(
                                client,
                                case,
                                repeat=repeat,
                                temperature=args.direct_temperature,
                                structured_interface=args.structured_interface,
                            )
                        )
                else:
                    need_inner = (
                        "inner" in selected
                        and ("inner_speech_gate", case["id"], repeat) not in completed
                    )
                    need_rule = (
                        "rule" in selected
                        and ("rule_gate", case["id"], repeat) not in completed
                    )
                    if need_inner or need_rule:
                        readiness = run_readiness_gate(
                            client,
                            case,
                            repeat=repeat,
                            temperature=args.gate_temperature,
                            include_inner=need_inner,
                            include_rule=need_rule,
                            structured_interface=args.structured_interface,
                        )
                        produced.extend(readiness)

                for record in produced:
                    raw_handle.write(json.dumps(record, ensure_ascii=False, sort_keys=True) + "\n")
                    raw_handle.flush()
                    completed.add((record["architecture"], record["case_id"], repeat))

    records = _read_records(raw_path)
    rows = write_summary(records, args.suite, output_dir)
    print(f"Completed {total} case/repeat pairs; results: {output_dir}")
    for row in rows:
        if row["category"] == "ALL":
            primary = row.get("operational_task_success", row.get("readiness_accuracy"))
            print(
                f"  {row['architecture']}: n={row['n']} primary={primary:.3f} "
                f"p50={row['latency_p50_seconds']:.3f}s p95={row['latency_p95_seconds']:.3f}s"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
