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

# Backwards compat: older langsmith.testing may not expose log_question_en
if not hasattr(t, "log_question_en"):
    def _log_question_en_stub(*args, **kwargs):
        return None
    t.log_question_en = _log_question_en_stub


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
    """Load ADVISOR query examples, preferring the local test file for richer metadata."""
    repo_root = Path(__file__).resolve().parents[2]
    local_test_file = repo_root / "query_generation" / "test" / f"examples_{SCENARIO}.json"

    def _load_entries(data, action_name_lookup=None):
        examples = []
        for entry in data:
            reference_queries = entry.get("queries") or [v for k, v in entry.items() if "query" in k.lower()]
            action_name = entry.get("action_name")
            if action_name_lookup is not None and action_name is None:
                action_name = action_name_lookup
            examples.append(
                {
                    "question": entry["question"],
                    "question_en": entry.get("question_en", entry["question"]),
                    "action_name": action_name,
                    "parameters": parse_parameters(entry.get("parameters", "{}")),
                    "reference_queries": reference_queries,
                }
            )
        return examples

    if local_test_file.is_file():
        with open(local_test_file, "r", encoding="utf-8") as f:
            data = json.load(f)
        return _load_entries(data)

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
            examples.extend(_load_entries(data, action_name_lookup=action_dir.name))

    if not examples:
        raise ValueError(f"No query examples found for scenario {SCENARIO}")
    return examples


EXAMPLES = load_query_examples()
EXAMPLE_IDS = [f"{ex['action_name']}::{ex['question']}" for ex in EXAMPLES]

def normalize_record(record):
    if isinstance(record, dict):
        return json.dumps(record, sort_keys=True)
    return json.dumps(record, sort_keys=True)


def flatten_results(query_results):
    """
    Flatten arbitrarily nested Neo4j results while preserving record structure.
    - Dicts are kept as dicts so we can check subset matches.
    - Errors are tagged so they can be penalized.
    - Empty lists are surfaced via a sentinel to avoid disappearing matches.
    """

    def normalize_for_compare(val):
        """Normalize nested structures so ordering differences don't break equality."""
        if isinstance(val, dict):
            return {k: normalize_for_compare(v) for k, v in val.items()}
        if isinstance(val, list):
            normed = [normalize_for_compare(v) for v in val]
            try:
                return sorted(normed, key=lambda x: json.dumps(x, sort_keys=True))
            except TypeError:
                return sorted(normed, key=lambda x: repr(x))
        return val

    def _flatten(item, acc):
        if isinstance(item, str):
            tag = f"ERROR::{item}" if "Query failed" in item or item.startswith("ERROR::") else item
            acc.append(tag)
            return
        if isinstance(item, list):
            if not item:
                acc.append("EMPTY_RESULT")
                return
            for sub in item:
                _flatten(sub, acc)
            return
        if isinstance(item, dict):
            acc.append(normalize_for_compare(item))
            return
        acc.append(normalize_record(item))

    flat = []
    for res in query_results:
        _flatten(res, flat)
    return flat


def compute_overlap(ref_results, gen_results):
    ref_flat = flatten_results(ref_results)
    gen_flat = flatten_results(gen_results)

    ref_dicts = [item for item in ref_flat if isinstance(item, dict)]
    gen_dicts = [item for item in gen_flat if isinstance(item, dict)]
    ref_atoms = {item for item in ref_flat if not isinstance(item, dict)}
    gen_atoms = {item for item in gen_flat if not isinstance(item, dict)}

    # Penalize if any generated query failed outright.
    if any(
        isinstance(item, str) and (item.startswith("ERROR::") or "Query failed" in item)
        for item in gen_atoms
    ):
        return 0.0

    def dict_is_subset(a, b):
        return all(k in b and b[k] == v for k, v in a.items())

    dict_matches = 0
    used_ref = [False] * len(ref_dicts)
    for g in gen_dicts:
        for idx, r in enumerate(ref_dicts):
            if used_ref[idx]:
                continue
            if dict_is_subset(g, r) or dict_is_subset(r, g):
                dict_matches += 1
                used_ref[idx] = True
                break

    atom_intersection = len(ref_atoms & gen_atoms)
    atom_union = len(ref_atoms | gen_atoms)
    dict_union = len(ref_dicts) + len(gen_dicts) - dict_matches

    if dict_union + atom_union == 0:
        return 1.0  # both empty
    total_intersection = dict_matches + atom_intersection
    total_union = dict_union + atom_union
    return total_intersection / total_union


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
    for ex in EXAMPLES:
        params_str = escape_curly_braces(json.dumps(ex["parameters"], ensure_ascii=False))
        refs_escaped = [escape_curly_braces(r) for r in ex["reference_queries"]]
        action_examples.setdefault(ex["action_name"], []).append(
            {"question": ex["question"], "parameters": params_str, "queries": refs_escaped}
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
@pytest.mark.parametrize("example", EXAMPLES, ids=EXAMPLE_IDS)
def test_query_generation_llm(qg_llm, example):
    question = example["question"]
    question_en = example.get("question_en", question)
    action_name = example["action_name"]
    parameters = example["parameters"]
    reference_queries = example["reference_queries"]
    db = qg_llm.db_dict.get(action_name, qg_llm.db_dict["default"])

    # Reference logging and execution
    reference_results = run_queries(db, reference_queries)
    if USE_LANGSMITH:
        t.log_question_en({"question_en": question_en})
        t.log_inputs(
            {
                "question": question,
                "question_en": question_en,
                "action_name": action_name,
                "parameters": parameters,
            }
        )
        t.log_reference_outputs(
            {
                "queries": reference_queries,
                "results": reference_results,
                # "parameters": parameters,
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

    # Throttle between examples to avoid free-tier API rate limits.
    time.sleep(90)
