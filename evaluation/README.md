# Reviewer evaluation harness

This directory contains two paired experiments for the major revision.  It is
standalone: ROS 2, Neo4j, and LangSmith are not required, and every raw response
is saved locally before metrics are aggregated.

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
```

Use `--output-dir evaluation/results/<name>` to make a run resumable at a
stable location.  Completed `(architecture, case, repeat)` records are skipped.
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

## Interpretation boundaries

Call this a comparison between a **structured factored controller** and a
**direct LLM tool-use controller**, not an optimal POMDP policy comparison.
JANUS uses the POMDP as a formal scaffold and does not solve a reward-optimized
POMDP policy.  The controller suite is end-to-end across control decisions, but
it is not an embodied robot/user study and does not measure actuation safety.
