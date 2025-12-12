import ast
import json
import os
import sys
import time
from pathlib import Path

import pytest
from dotenv import load_dotenv
import importlib
from langsmith import testing as t
from shared_utils.shared_utils.fewshot_helpers import escape_curly_braces


REPO_ROOT = Path(__file__).resolve().parents[2]
SHARED_ROOT = REPO_ROOT / "shared_utils"
DB_ADAPTERS_ROOT = REPO_ROOT / "db_adapters"
SCENARIO_ROOT = REPO_ROOT / "scenario_customization"
extras = [
    str(SHARED_ROOT),
    str(REPO_ROOT),
    str(SCENARIO_ROOT),
    str(DB_ADAPTERS_ROOT),
]
rest = [p for p in sys.path if p not in extras]
sys.path = extras + rest
importlib.invalidate_caches()
try:
    importlib.import_module("shared_utils.customization_helpers")
except ModuleNotFoundError:
    # Fallback: load directly from path to avoid ROS/pytest path mangling
    import importlib.util

    pkg_root = SHARED_ROOT / "shared_utils"
    pkg_init = pkg_root / "__init__.py"
    spec_pkg = importlib.util.spec_from_file_location("shared_utils", pkg_init)
    shared_pkg = importlib.util.module_from_spec(spec_pkg)
    sys.modules["shared_utils"] = shared_pkg
    spec_pkg.loader.exec_module(shared_pkg)

    mod_path = pkg_root / "customization_helpers.py"
    spec_mod = importlib.util.spec_from_file_location("shared_utils.customization_helpers", mod_path)
    ch = importlib.util.module_from_spec(spec_mod)
    sys.modules["shared_utils.customization_helpers"] = ch
    spec_mod.loader.exec_module(ch)

from query_generation.query_generation.query_generation_llm import QueryGeneration_LLM


NODE_NAME = "query_generation"
SCENARIO = "ADVISOR"

# Load environment configuration
load_dotenv(REPO_ROOT / ".env", override=True)
load_dotenv(REPO_ROOT / ".config", override=True)

# LangSmith setup
os.environ["LANGSMITH_TRACING"] = "true"
os.environ["LANGSMITH_ENDPOINT"] = "https://api.smith.langchain.com"
os.environ["LANGSMITH_PROJECT"] = f'{SCENARIO}:{ast.literal_eval(os.getenv("LLM_CONFIG"))[NODE_NAME]["model_name"]}'
LANGSMITH_API_KEY = os.getenv("LANGSMITH_API_KEY")
USE_LANGSMITH = bool(LANGSMITH_API_KEY)
if USE_LANGSMITH:
    os.environ["LANGSMITH_API_KEY"] = LANGSMITH_API_KEY
    os.environ["LANGSMITH_TEST"] = "1"
else:
    # Fall back to no LangSmith logging if key is missing
    os.environ["LANGSMITH_TRACING"] = "false"
os.environ["LANGSMITH_TEST_SUITE"] = "Query Generation ADVISOR"

def parse_parameters(raw: str):
    if isinstance(raw, dict):
        return raw
    cleaned = raw.strip()
    if cleaned.startswith("{{") and cleaned.endswith("}}"):
        cleaned = cleaned[1:-1]
    return ast.literal_eval(cleaned)


def load_query_examples():
    """Load ADVISOR query examples directly from the scenario folder."""
    repo_root = Path(__file__).resolve().parents[2]
    scenario_root = repo_root / "scenario_customization" / "scenario_customization" / SCENARIO
    if not scenario_root.is_dir():
        raise FileNotFoundError(f"Scenario folder not found: {scenario_root}")

    examples = []
    for action_dir in sorted(scenario_root.iterdir()):
        q_examples_dir = action_dir / "query_examples"
        if not q_examples_dir.is_dir():
            continue
        for json_file in sorted(q_examples_dir.glob("*.json")):
            with open(json_file, "r", encoding="utf-8") as f:
                data = json.load(f)
                for entry in data:
                    reference_queries = [v for k, v in entry.items() if "query" in k.lower()]
                    examples.append(
                        (
                            entry["question"],
                            action_dir.name,
                            parse_parameters(entry.get("parameters", "{}")),
                            reference_queries,
                        )
                    )
    if not examples:
        raise ValueError(f"No query examples found for scenario {SCENARIO}")
    return examples


EXAMPLES = load_query_examples()

def normalize_record(record):
    if isinstance(record, dict):
        return json.dumps(record, sort_keys=True)
    return json.dumps(record, sort_keys=True)


def flatten_results(query_results):
    flat = []
    for res in query_results:
        if isinstance(res, list):
            for item in res:
                flat.append(normalize_record(item))
    return flat


def compute_overlap(ref_results, gen_results):
    ref_flat = set(flatten_results(ref_results))
    gen_flat = set(flatten_results(gen_results))
    if not ref_flat or not gen_flat:
        return 0.0
    return len(ref_flat & gen_flat) / len(ref_flat | gen_flat)


def run_queries(db, queries):
    outputs = []
    for q in queries:
        try:
            res = db.execute_query(q)
        except Exception as exc:
            res = f"Query failed: {exc}"
        outputs.append(res)
    return outputs


def valid_query_rate(results):
    if not results:
        return 0.0
    valid = sum(1 for res in results if isinstance(res, list))
    return valid / len(results)


@pytest.fixture(scope="session")
def qg_llm():
    # Keep initialization once and enforce scenario switch if needed.
    llm = QueryGeneration_LLM(NODE_NAME)
    if llm.scenario != SCENARIO:
        llm.change_scenario(SCENARIO)
    # Inject local examples so we are independent from installed scenario assets.
    action_examples = {}
    for q, act, params, refs in EXAMPLES:
        params_str = escape_curly_braces(json.dumps(params, ensure_ascii=False))
        refs_escaped = [escape_curly_braces(r) for r in refs]
        action_examples.setdefault(act, []).append(
            {"question": q, "parameters": params_str, "queries": refs_escaped}
        )
    llm.examples.update(action_examples)
    return llm


@pytest.fixture(scope="session", autouse=True)
def ensure_neo4j_available(qg_llm):
    db = qg_llm.db_dict.get("default")
    try:
        res = db.execute_query("RETURN 1 AS ok")
        if isinstance(res, str):
            pytest.skip(f"Neo4j unavailable: {res}")
    except Exception as exc:
        pytest.skip(f"Neo4j unavailable: {exc}")


@pytest.mark.langsmith
@pytest.mark.parametrize("question,action_name,parameters,reference_queries", EXAMPLES)
def test_query_generation_llm(qg_llm, question, action_name, parameters, reference_queries):
    db = qg_llm.db_dict.get(action_name, qg_llm.db_dict["default"])

    # Reference logging and execution
    reference_results = run_queries(db, reference_queries)
    if USE_LANGSMITH:
        t.log_reference_outputs(
            {
                "queries": reference_queries,
                "results": reference_results,
                "parameters": parameters,
            }
        )

    start_time = time.time()
    llm_response = qg_llm.get_LLM_response(question, action_name, parameters, return_time=False)
    total_time = time.time() - start_time

    generated_queries = getattr(llm_response, "query", [])
    if isinstance(generated_queries, str):
        generated_queries = [generated_queries]

    generated_results = run_queries(db, generated_queries)

    vqr = valid_query_rate(generated_results)
    overlap = compute_overlap(reference_results, generated_results)

    if USE_LANGSMITH:
        t.log_outputs(
            {
                "generated_queries": generated_queries,
                "generated_results": generated_results,
            }
        )
        t.log_feedback(key="total_time", score=round(total_time, 3))
        t.log_feedback(key="valid_query_rate", score=round(vqr, 3))
        t.log_feedback(key="result_overlap", score=round(overlap, 3))

    assert generated_queries, "No queries generated"
    assert vqr > 0, "All generated queries failed to execute"
