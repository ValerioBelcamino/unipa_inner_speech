# Reviewer evaluation harness

This directory contains paired experiments for the major revision. The main
architecture comparisons are standalone: ROS 2, Neo4j, and LangSmith are not
required, and every raw response is saved locally before metrics are aggregated.

## What the experiments test

`controller_v1` compares three conditions on the same 44 previously unseen
Italian requests, task contracts, memory, and tool context:

- `janus_factored`: Intent Recognition followed by the LLM Inner Speech gate;
- `janus_rule_gate`: the *same Intent Recognition output* followed by a
  deterministic required-slot gate;
- `direct_llm`: one strong LLM call jointly selects reject/clarify/execute,
  the task, and its arguments.

The paired Intent output makes `janus_factored` versus `janus_rule_gate` an
ablation of the gate rather than a comparison confounded by different upstream
predictions.  `direct_llm` receives the same information and schema, so it is
not a deliberately weak chatbot baseline.

By default, JANUS Intent Recognition uses native function calling with
`tool_choice=auto`, matching the runtime `bind_tools` implementation and its
Pydantic schemas. The legacy single-JSON interface is retained only for audit
with `--intent-interface json`; it should not be used for the primary table.
Inner Speech and the Direct controller both use a single forced function for
their typed decision by default. This mirrors the default function-calling
implementation of `with_structured_output` in the repository's pinned
`langchain_groq==0.3.2` while giving the Direct baseline the same structured
output mechanism. Their legacy JSON mode is available only for interface
ablation with `--structured-interface json`.

`readiness_v1` freezes the upstream action and parameters and compares only the
Inner Speech gate with the rule gate.  In addition to complete and syntactically
incomplete requests, it includes semantic upstream errors: wrong actions,
invented parameters, ambiguous values, contradictory constraints, and invalid
nutritional targets.  These are the cases missing from the original unit suite.

`multidomain_v1` is a separately frozen stress test over the three scenarios
with task implementations in this repository: ADVISOR, MOVIES, and
I-TROPHYTS (seven task tools in total). JANUS performs native Scope Detection,
then exposes only the selected domain's tools to Intent Recognition, followed
by Inner Speech. The one-call Direct baseline receives all seven contracts.
Its 26 cases were committed before any model run and cover valid execution,
missing information, semantic ambiguity, memory, cross-domain context, and
out-of-scope requests. Domain-selection accuracy is reported explicitly.

Primary metrics are operational task success and decision macro-recall
(controller suite), readiness balanced accuracy and separate proceed/block recall (readiness
suite), premature execution/proceed rate, unnecessary clarification or blocking
rate, and structured-output failure rate.  Efficiency metrics are
p50/p95 wall latency, API-reported token use, and number of calls including
retries. Operational task success requires the decision and action to be correct,
and exact parameters when the gold decision is `execute`. Strict state match is
also reported and requires exact parameters even for clarification/rejection
turns, where preserving partial state can still matter for the next turn.
Groq completions are capped at 1024 tokens because reasoning tokens
count toward the budget; local/custom completions default to 256 tokens so a
malformed local generation cannot dominate the latency distribution. Override
the cap only if the same value is used for every compared condition.

## Validate without calling a model

From the repository root:

```bash
python3 -m evaluation.benchmark --suite controller --validate-only
python3 -m evaluation.benchmark --suite multidomain --validate-only
python3 -m evaluation.benchmark --suite readiness --validate-only
python3 -m pytest -q evaluation/test/test_evaluation.py
```

## Groq (main architectural comparison)

Create the ignored `.env` file and set `GROQ_API_KEY`. Use one model for every
condition. The manuscript model, `meta-llama/llama-4-scout-17b-16e-instruct`,
returned `model_not_found` on 2026-09-02. The current default is therefore
`qwen/qwen3.8-27b`, which was frozen for both primary suites after the GPT-OSS
20B rolling daily quota was exhausted during pilot runs. Record this forced
model substitution and the exact model identifier explicitly in the revision.

Start with a five-case smoke test:

```bash
python3 -m evaluation.benchmark \
  --suite controller \
  --provider groq \
  --max-cases 5 \
  --request-delay 7.5
```

Then run both complete suites:

```bash
python3 -m evaluation.benchmark \
  --suite readiness \
  --provider groq \
  --request-delay 7.5

python3 -m evaluation.benchmark \
  --suite controller \
  --provider groq \
  --request-delay 7.5

python3 -m evaluation.benchmark \
  --suite multidomain \
  --provider groq \
  --request-delay 7.5
```

The checked-in `evaluation/run_qwen38_final.sh` launcher runs the corrected
native-tool controller suite first and the exploratory multi-domain suite
second, using separate stable output directories. It uses a 384-token
completion cap selected from the native-tool pilot (observed maximum: 198),
while treating any length-truncated tool call as a failed structured output
rather than `OutOfScope`. If the rolling daily quota pauses the script, invoke
the same launcher later; completed records are skipped.

Use `--output-dir evaluation/results/<name>` to make a run resumable at a
stable location.  Completed `(architecture, case, repeat)` records are skipped.
Provider rate limits requiring more than 60 seconds stop the process before the
current case is written; rerun the same command later to resume. Adjust this cap
with `--max-retry-wait`, but do not count provider-throttled attempts as model
failures. Resume rejects changes to model, dataset, interfaces, temperatures,
token/retry settings, or selected cases. It preserves the initial provenance
and adds a timestamped resume event with the later Git revision and hardware.
For a deterministic primary table, one repetition at temperatures 0/0.2/0 is
the default.  If API budget permits, use `--repeats 3` and report that repeated
generations, rather than unique cases, are the aggregation unit.

## Local CPU profiling with Ollama

The included container is pinned to Ollama 0.11.10.  On the current development
machine there is no NVIDIA GPU; `qwen2.5:3b-instruct` is therefore the quick
CPU pilot.  It should be reported as a deployment/profile condition, not as a
causal architecture comparison against Llama 4.

```bash
docker compose -f evaluation/docker-compose.local.yml up -d
docker compose -f evaluation/docker-compose.local.yml exec ollama \
  ollama pull qwen2.5:3b-instruct

python3 -m evaluation.benchmark \
  --suite readiness \
  --provider ollama \
  --model qwen2.5:3b-instruct \
  --max-cases 5 \
  --max-attempts 1
```

For a stronger but slower CPU condition, pull and select
`qwen2.5:7b-instruct`.  Warm the model before measuring and run the complete
suite only after the smoke test succeeds:

```bash
curl -s http://localhost:11434/api/generate \
  -d '{"model":"qwen2.5:3b-instruct","prompt":"warmup","stream":false}' >/dev/null

python3 -m evaluation.benchmark \
  --suite readiness \
  --provider ollama \
  --model qwen2.5:3b-instruct \
  --max-attempts 1
```

The local run metadata records OS, CPU string, Git revision, dataset hash,
model name, temperatures, and retry policy.  Add the exact Ollama model digest
to the experimental report using:

```bash
docker compose -f evaluation/docker-compose.local.yml exec ollama \
  ollama show qwen2.5:3b-instruct --modelfile
```

## Outputs

Each timestamped directory contains:

- `metadata.json`: provenance and experiment settings;
- `raw.jsonl`: one auditable record per architecture/case/repeat, including raw
  model output, retries, usage, and latency;
- `summary.json` and `summary.csv`: overall and per-category metrics.

The entire `evaluation/results/` directory is ignored so pilot results are not
accidentally committed.  Copy only a frozen, explicitly selected final run into
an archival location if publication artifacts should be versioned.

The selected reviewer runs are versioned under `evaluation/frozen_results/`.
See `REVIEWER_RESULTS.md` for the primary tables, statistical tests, caveats,
and the distinction between API latency and rate-limit throttling.

## Reproduce the manuscript module tests with Qwen 3.8

`module_benchmark.py` reruns the isolated quantitative tests reported for the
submitted manuscript without ROS 2 or LangSmith. It uses the submitted prompts,
native function calling, manuscript temperatures, and the original ADVISOR JSON
datasets. The default is `qwen/qwen3.8-27b` with reasoning explicitly disabled.

Validate the exact case counts without making an API call:

```bash
python3 -m evaluation.module_benchmark --validate-only
python3 -m pytest -q evaluation/test/test_module_benchmark.py
```

Run a one-case-per-module smoke test first:

```bash
docker compose -f evaluation/docker-compose.query.yml up -d
# Run this only for a new or intentionally reset dedicated test graph:
NEO4J_URI=neo4j://localhost:19687 \
  POPULATE_RANDOM_SEED=42 \
  python3 query_generation/query_generation/populate_database.py

python3 -m evaluation.module_benchmark \
  --max-cases 1 \
  --request-delay 8 \
  --max-completion-tokens 512 \
  --neo4j-uri bolt://localhost:19687 \
  --output-dir evaluation/results/module_smoke_qwen38
```

Then run the complete isolated experiment at the same settings. The stable
output directory makes the command resumable after a provider limit:

```bash
# Intent uses a dedicated graph because its reference set includes users both
# with and without a complete weekly plan. The Query population script instead
# gives every seeded user a seven-day plan.
docker compose -f evaluation/docker-compose.intent.yml up -d
python3 -m evaluation.seed_intent_database

python3 -m evaluation.module_benchmark \
  --module intent \
  --request-delay 11 \
  --max-completion-tokens 512 \
  --output-dir evaluation/results/modules_qwen38_intent_final

python3 -m evaluation.module_benchmark \
  --module inner \
  --module outer \
  --request-delay 5.5 \
  --max-completion-tokens 512 \
  --output-dir evaluation/results/modules_qwen38_inner_outer_final
```

Scope Detection and Query Generation should be run separately because their
prompt sizes require different rate-limit pacing. Query Generation uses
`--neo4j-uri` (the dedicated compose file exposes port 19687); Intent uses
`--intent-neo4j-uri` (default port 18687). Query's container includes APOC,
which generated Cypher may legitimately use. To repair a raw Intent run
produced before DB post-processing was enabled, rescore it without calling the
model:

```bash
python3 -m evaluation.module_benchmark \
  --rescore-intent-from evaluation/results/modules_qwen38_intent_final \
  --output-dir evaluation/results/modules_qwen38_intent_final_postprocessed
```

The runner stores client-observed API latency, Groq's server-side total/queue
time, and wall-clock latency (mean, p50, and p95), plus prompt, completion, and
total tokens, retries, raw model output, and module-specific accuracy metrics.
The submitted Inner and Explainability tests also used BERTScore. Compute it
offline after installing the optional metric dependencies (no Groq calls):

```bash
python3 -m pip install -r evaluation/requirements-metrics.txt
python3 -m evaluation.score_module_text \
  --source evaluation/results/modules_qwen38_inner_final
python3 -m evaluation.score_module_text \
  --source evaluation/results/modules_qwen38_outer_final
```

The scorer records the exact checkpoint, layer, and metric hash. It preserves
the submitted Explainability configuration (`lang="en"`, hence
`roberta-large`) even though the reference and prediction strings are Italian.
Query Generation additionally requires the dedicated Neo4j test graph. Its
`legacy` protocol deliberately reproduces the original in-sample functional
test: evaluated examples are also present among its few-shot demonstrations,
so its scores must not be described as held-out generalization.

## Interpretation boundaries

Call this a comparison between a **structured factored controller** and a
**direct LLM tool-use controller**, not an optimal POMDP policy comparison.
JANUS uses the POMDP as a formal scaffold and does not solve a reward-optimized
POMDP policy.  The controller suite is end-to-end across control decisions, but
it is not an embodied robot/user study and does not measure actuation safety.
