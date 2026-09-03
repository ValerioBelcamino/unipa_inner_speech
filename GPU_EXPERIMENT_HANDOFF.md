# GPU experiment handoff for the ROBOT major revision

This file is the operational and scientific handoff for the Codex session that
will run the local-model experiments. It is intentionally self-contained. Read
it before changing prompts, datasets, metrics, or runner interfaces.

## 1. Mission and division of work

The GPU machine owns these tasks:

1. serve the official `Qwen/Qwen3.5-4B` and `Qwen/Qwen3.5-9B` checkpoints
   locally through an OpenAI-compatible vLLM server;
2. reproduce the isolated manuscript module tests for each local model;
3. run the held-out readiness ablation, end-to-end controller comparison, and
   evidence-grounding comparison for each local model;
4. compute the pending BERTScore metrics for the already-frozen Groq Qwen 3.8
   27B outputs and for the local models' Inner/Outer Speech outputs;
5. preserve raw results and complete provenance, perform the integrity checks
   in this document, and push accepted frozen results on a separate GPU branch.

The original machine continues the Qwen 3.8 27B grounding experiment through
Groq. Do not spend Groq quota on the GPU machine and do not combine local-model
records with the ignored partial 27B API directories.

Immediately after pulling the handoff branch, create a separate result branch
so the two machines do not race on the same branch:

```bash
git fetch origin
git switch reviewer-evaluation
git pull --ff-only origin reviewer-evaluation
git switch -c gpu-local-evaluation
```

If `gpu-local-evaluation` already exists remotely, use that branch and pull it
with `--ff-only`. Do not force-push either branch.

## 2. Manuscript and review context

- Manuscript number: `ROBOT-D-26-00275`
- Title: **Factored Reasoning with Inner Speech and Persistent Memory for
  Evidence-Grounded Human-Robot Interaction**
- Journal: *Robotics and Autonomous Systems*
- Decision: major revision
- Revision deadline: **2026-09-30**
- Paper: <https://arxiv.org/pdf/2602.00675>
- Original repository branch:
  <https://github.com/ValerioBelcamino/unipa_inner_speech/tree/memory-agent>
- Evaluation work branch: `reviewer-evaluation`

The special-issue editor asks for a stronger experiment that comparatively
evaluates the proposed Inner Speech and POMDP-inspired structure.

Reviewer 1 considers the integration promising, especially the separation of
inner and outer monologue and persistent memory, but says the current module
tests show functionality rather than measurable architectural benefit. The
requested extension is a comparison against:

- internal-reasoning/Inner-Monologue-style approaches; and
- direct LLM planning or tool-use approaches such as the COPAL family.

The reviewer also questions the benefit of graph memory over document memory,
but explicitly treats it as secondary to the controller/Inner Speech
comparison. We are therefore not adding a Neo4j-versus-document experiment in
this time-critical revision.

Reviewer 2 asks the paper to state clearly that it demonstrates technical
feasibility and has not yet evaluated human-robot interaction on a real robot.
The review also notes high latency in some modules, absence of an embodied
controller/user study, and lack of coverage of embodiment-specific safety and
latency concerns. The positive assessment is that JANUS is a promising,
extensible, multi-domain architecture for controlled and grounded actions.

The revised paper must call the main comparison a **structured factored
controller versus a direct LLM tool-use controller**, not a solved POMDP policy
versus an LLM planner. JANUS uses the POMDP as a formal scaffold; this code does
not optimize an explicit POMDP reward or solve for an optimal policy.

## 3. Scientific questions and claims

The experiments are designed to answer four separate questions.

### Q1: Does Inner Speech add value over a procedural completeness rule?

Compare `janus_factored`/`inner_speech_gate` with the paired RuleGate ablation.
The same upstream Intent output is reused, so this is a clean gate ablation.
The readiness set includes semantic errors that cannot be detected by checking
only whether required slots are non-empty.

Primary evidence:

- readiness balanced accuracy, proceed recall, and block recall;
- premature proceed/execution and unnecessary block/clarification rates;
- paired operational task success in the controller suite.

This claim is already supported with Qwen 3.8 27B and GPT-OSS 20B. The local
models test whether the effect survives at lower capacity.

### Q2: Does factorization help a smaller model relative to one direct call?

Compare `janus_factored` and `direct_llm` using the same local checkpoint,
temperature policy, task contracts, memory, native structured-output
mechanism, and test cases. The Direct baseline is deliberately strong; it is
not a chatbot denied access to tools or memory.

The full 44-case controller suite is the primary analysis. Its full-dataset
operational success is more important than finding a favorable category after
the run. Per-category results are secondary, predeclared error analysis. Do not
select or invent a post-hoc subset merely because JANUS wins it.

A publishable positive claim would be that factorization degrades more
gracefully as model capacity falls, but only make it if the complete local runs
support it. The already-frozen 27B result does **not** show global superiority:
Direct is 43/44 and JANUS is 39/44, with a non-significant paired difference.

### Q3: Does the structured pipeline improve evidence grounding?

The 30-case grounding benchmark gives both JANUS and Direct access to the same
Neo4j snapshot, schema, read tools, and post-tool answer interface. JANUS uses
Intent, Inner Speech, action-specific Query Generation, and Outer Speech.
Direct generates its decision, action, parameters, and Cypher jointly, then
receives the same query result and produces a structured answer.

The deterministic primary metrics are:

- exact retrieval success;
- evidence micro precision, recall, and F1;
- entity precision and recall;
- correct abstention when the database returns no evidence;
- support of explicitly declared answer claims;
- grounded task success.

No LLM judge is used. Field aliases and collection-valued evidence are
canonicalized. Broad queries are penalized. The primary analysis is all 30
cases; the six frozen categories are secondary analyses.

### Q4: Is local accelerated inference a feasible latency condition?

Report local p50/p95 client-observed latency, token use, and logical LLM calls
for every condition. This is a deployment/profile experiment, not evidence
that architecture alone changes latency. Do not compare Groq server time and
local wall time as if they were the same quantity. Within one local model and
one server configuration, JANUS-versus-Direct latency is a valid architectural
trade-off.

Keep Qwen thinking disabled, as agreed, to prioritize interactive latency.
The runner sends Qwen's official
`chat_template_kwargs.enable_thinking=false` request extension. It does not
send Groq's `reasoning_effort` extension to the local server.

## 4. Work already completed

### 4.1 Frozen controller comparison with Qwen 3.8 27B on Groq

The final 44-case native-tool result is under
`evaluation/frozen_results/qwen38_controller_native_final/`.

| Condition | Operational success | Strict state | Parameter micro-F1 | Premature execution | API p50/p95 | Median tokens |
|---|---:|---:|---:|---:|---:|---:|
| Direct LLM | 43/44 (97.7%) | 26/44 (59.1%) | 93.0% | 0/26 | 0.930/1.320 s | 1398.5 |
| JANUS factored | 39/44 (88.6%) | 25/44 (56.8%) | 88.3% | 2/26 | 1.434/1.903 s | 2049 |
| JANUS RuleGate | 31/44 (70.5%) | 23/44 (52.3%) | 88.3% | 10/26 | 0.691/0.928 s | 1081.5 |

JANUS beat RuleGate on eight discordant pairs and lost none (exact two-sided
McNemar `p=0.0078125`), reducing premature execution by 80%. Direct versus
JANUS had five discordant pairs favoring Direct and one favoring JANUS
(`p=0.21875`). This supports the Inner Speech ablation, not global superiority
over Direct.

### 4.2 Frozen readiness ablation

The Qwen result is under `evaluation/frozen_results/qwen38_readiness/`; an
independent GPT-OSS result is also frozen.

| Model/gate | Accuracy | Balanced accuracy | Proceed recall | Block recall | Premature proceed |
|---|---:|---:|---:|---:|---:|
| Qwen 3.8 Inner Speech | 25/29 | 91.3% | 6/6 | 19/23 | 4/23 |
| RuleGate | 15/29 | 69.6% | 6/6 | 9/23 | 14/23 |
| GPT-OSS Inner Speech | 23/29 | 87.0% | 6/6 | 17/23 | 6/23 |

Qwen Inner Speech versus RuleGate has 11 versus 1 discordant correct pairs
(`p=0.00635`). GPT-OSS independently has 8 versus 0 (`p=0.00781`).

### 4.3 Frozen reproduction of the submitted module tests

All 338 Qwen 3.8 27B generations completed without API or structured-output
failures. Raw data are under `evaluation/frozen_results/qwen38_modules/` and
the full report is `evaluation/MODULE_RESULTS_QWEN38.md`.

| Module/configuration | Cases | Submitted Llama 4 Scout | Qwen 3.8 27B |
|---|---:|---:|---:|
| Scope MHS | 24 | 100% | 100% |
| Scope HSR | 24 | 95.8% | 100% |
| Scope ATT | 24 | 100% | 100% |
| Scope TEF | 24 | 100% | 100% |
| Scope TER | 24 | 100% | 100% |
| Scope ALL | 66 | 98.5% | 100% |
| Intent task selection | 51 | 100% | 100% |
| Intent parameter micro-F1 | 51 | 100% | 94.7% |
| Inner Speech readiness | 40 | 100% | 95.0% |
| Query valid-query rate | 30 cases/40 queries | 100% | 100% |
| Query legacy result overlap | 30 | 96.7% | 93.3% |
| Outer Speech BERTScore F1 | 31 | 0.87 (SD 0.03) | pending GPU run |

The two Query result-overlap failures are top-level alias differences; both
queries returned the correct semantic entities and values. The secondary
alias-invariant audit is 30/30. The two Inner Speech unit disagreements are
upstream `OutOfScope` cases that the deterministic controller would still
block; the effective dispatch audit is 40/40. Keep the strict submitted-style
scores in the main reproduction table and label the alternative views as error
analysis unless the old Llama outputs can be rescored identically.

The Groq provider latency/token comparison with the submitted Llama values is
already documented in `evaluation/MODULE_RESULTS_QWEN38.md`. Qwen 3.8 used
fewer median tokens in four of five modules, but it was not uniformly faster.

### 4.4 Frozen and audited grounding benchmark

`evaluation/datasets/grounding_v1.json` was committed before model runs.

- cases: 30;
- fixed reference evidence rows: 29;
- intentional empty-evidence cases: 2;
- reference mismatches: 0;
- dataset SHA-256:
  `bd35e2c47044fa9c0c7491bacb3591430428996d8c15ed1e3a2d68cf0151fd5c`;
- Neo4j snapshot SHA-256:
  `e6945f94f060aef7e04ce17b0bafe0df372abb60ae8c79c284ddd202251b8436`;
- normalized exact overlap with 657 existing prompt strings: 0.

The six categories are dish properties, dish composition, personalized
compatibility, substitution, missing evidence/abstention, and stale-memory
conflict. The full audit is in `evaluation/GROUNDING_V1_AUDIT.md`.

Small API pilots showed exact retrieval for both JANUS and Direct on five
representative cases. An early answer protocol undercounted logically
query-supported ingredient claims that were not returned explicitly; the
current committed answer contract fixes that ambiguity by requiring auditable
declared claims. One post-fix substitution smoke case scored retrieval and
grounding success for both systems.

The 27B API run is currently incomplete because all available Groq
organizations reached the rolling Qwen token quota. The ignored directory
`evaluation/results/grounding_qwen38_factored_final` contains only the first
three JANUS cases, all successful; the corresponding Direct directory has no
completed case. The original machine will resume these. Do not copy, delete,
freeze, or mix them into local results.

### 4.5 Other completed audit findings

- The submitted Llama 4 Scout Groq identifier returned `model_not_found` on
  2026-09-02, so the isolated tests were rerun with Qwen 3.8 27B.
- Current runtime Intent Recognition and Inner Speech are zero-shot. Their old
  few-shot JSON files are orphaned, schema-incompatible, and partly
  inconsistent. Do not re-enable them. Query Generation and Outer Speech do
  use scenario few-shot examples. See `evaluation/FEWSHOT_AUDIT.md`.
- The primary Intent interface is native auto tool calling, matching runtime
  `bind_tools`. The primary Inner/Direct interface is one forced native
  function, matching `with_structured_output`. The earlier legacy-JSON
  controller result is diagnostic only.
- The Query module reproduction is in-sample because the evaluated examples
  are also demonstrations. Treat it as a functional regression, not held-out
  generalization.
- The submitted Outer Speech example assets include known inconsistencies and
  copy/paste errors, including mismatched dish/result pairs. They were not
  silently repaired because the frozen 27B reproduction used the submitted
  assets. Keep them unchanged for the primary local model-size reproduction.
  A corrected-prompt study would be a separate, clearly versioned experiment
  requiring every affected condition to be rerun.
- A CPU-only Qwen 2.5 3B pilot was too slow and blocked all valid readiness
  cases. It cannot support the latency response; accelerated inference is why
  this GPU experiment exists.
- `janus_selective` is not part of the requested experiment and must not be
  added. The controller conditions are full JANUS, RuleGate, and Direct. The
  grounding conditions are full JANUS and Direct only.
- No traces, human ratings, real-robot study, or graph-versus-document memory
  benchmark will be rerun in this time window.

## 5. Frozen experiment matrix

Run every primary suite once for each of these exact model IDs:

1. `Qwen/Qwen3.5-4B`
2. `Qwen/Qwen3.5-9B`

Use the same checkpoint within every architectural comparison. Prefer the
unquantized official checkpoints and one GPU for both models if they fit; this
keeps the latency comparison interpretable. If 9B cannot fit, first use tensor
parallelism with the same dtype. Quantization is a last resort and creates a
separate explicitly named condition: record the exact quantized checkpoint,
revision, method, dtype, and engine settings. Never label quantized results as
the unqualified official 9B result.

| Experiment | Frozen cases | Raw records | Approx. model calls | Conditions |
|---|---:|---:|---:|---|
| Module reproduction | 338 | 338 | 338 | Scope, Intent, Inner, Query, Outer |
| Readiness ablation | 29 | 58 | 29 | Inner Speech, RuleGate |
| Controller comparison | 44 | 132 | 132 | JANUS, RuleGate, Direct |
| Grounding comparison | 30 | 60 | 180 | JANUS, Direct |
| Total per model | - | 588 | about 679 | - |

RuleGate is deterministic after reusing the paired Intent output, so it adds a
raw record but no inference. Grounding assumes four calls for successful JANUS
cases and two for successful Direct cases. Failures can reduce those counts.
BERTScore adds no LLM/API calls.

The optional `multidomain_v1` suite is not in the primary local matrix. Run it
only after both models, all BERTScore jobs, and quality checks are complete.

## 6. Software and serving setup

The client-side benchmark requirements are lightweight:

```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install -r evaluation/requirements.txt
python3 -m pip install pytest
```

For BERTScore, install a CUDA-enabled PyTorch build appropriate for the GPU
driver first, then install the metric. Do not accidentally replace a working
CUDA PyTorch build with a CPU-only wheel:

```bash
python3 -c 'import torch; print(torch.__version__, torch.version.cuda, torch.cuda.is_available())'
python3 -m pip install bert-score==0.3.13
python3 -c 'import torch; print(torch.cuda.get_device_name(0))'
```

`evaluation/requirements-metrics.txt` documents the optional requirements. The
first BERTScore run downloads `bert-base-multilingual-cased` and
`roberta-large` (the latter is roughly 1.4 GB), so allow time and disk space.

The supplied vLLM compose file follows the Qwen 3.5 model-card serving setup:
native tool calling uses the `qwen3_coder` tool parser and the `qwen3`
reasoning parser. Qwen 3.5 requires a current vLLM main/nightly build. The
nightly tag is mutable, so pull it once, record its content digest, and use the
same image for both model runs.

```bash
cp evaluation/gpu.env.example evaluation/gpu.env
docker pull vllm/vllm-openai:nightly
docker image inspect vllm/vllm-openai:nightly \
  --format '{{json .RepoDigests}}'
docker compose --env-file evaluation/gpu.env \
  -f evaluation/docker-compose.gpu.yml config
```

Edit the ignored `evaluation/gpu.env` for the actual machine. The default is:

```dotenv
LOCAL_MODEL=Qwen/Qwen3.5-4B
VLLM_IMAGE=vllm/vllm-openai:nightly
VLLM_PORT=8000
TENSOR_PARALLEL_SIZE=1
MAX_MODEL_LEN=16384
GPU_MEMORY_UTILIZATION=0.90
VLLM_DTYPE=auto
CUDA_VISIBLE_DEVICES=0
HF_CACHE_DIR=./.cache/huggingface
```

The 16,384-token serving context is deliberate: these non-thinking benchmark
prompts fit comfortably and the shorter context reduces KV-cache pressure. Do
not change it between 4B and 9B. Keep tensor parallelism and visible hardware
the same if both fit. If they cannot be the same, report the difference
prominently and do not interpret cross-size latency as a pure model-size effect.

Start and wait for the server:

```bash
docker compose --env-file evaluation/gpu.env \
  -f evaluation/docker-compose.gpu.yml up -d --wait
curl -fsS http://localhost:8000/v1/models | python3 -m json.tool
docker compose -f evaluation/docker-compose.gpu.yml logs --tail=100 vllm
```

The benchmark uses sequential requests (`concurrency=1`) and no artificial
delay. The smoke run warms the loaded checkpoint before the full latency run.
The result metadata automatically records the host's NVIDIA GPU name, memory,
and driver. The launcher additionally saves `/v1/models`, `nvidia-smi`, Docker
information, and the selected image inspection under the ignored result root.
Do not put API or Hugging Face tokens in committed files.

## 7. Prepare and validate deterministic databases

Docker and its Compose plugin must be available. The preparation script
recreates only the two dedicated, tmpfs-backed benchmark graphs, seeds them,
then validates every grounding reference and all case files:

```bash
./evaluation/prepare_gpu_databases.sh
```

Expected grounding validator facts are the hashes and counts in section 4.4.
If they differ, stop. Do not run a model against a mismatched graph. Common
causes are an old container on ports 19687/18687, a changed population seed, or
a different repository commit.

The preparation script intentionally destroys only the dedicated benchmark
containers `janus-query-benchmark` and `janus-intent-benchmark`. It does not
touch arbitrary Neo4j instances or persistent volumes.

## 8. Exact runbook

### 8.1 Run automated tests before inference

```bash
python3 -m pytest -q evaluation/test
python3 -m evaluation.module_benchmark --validate-only --module all
python3 -m evaluation.benchmark --suite readiness --validate-only
python3 -m evaluation.benchmark --suite controller --validate-only
```

### 8.2 Qwen 3.5 4B

Set `LOCAL_MODEL=Qwen/Qwen3.5-4B` in `evaluation/gpu.env`, start vLLM, prepare
the databases, and run the smoke matrix:

```bash
time ./evaluation/run_local_gpu_model.sh \
  Qwen/Qwen3.5-4B qwen35_4b smoke
```

The smoke makes about 25 model calls across every important interface. Inspect
all three summaries and every error before proceeding. If it is valid:

```bash
time ./evaluation/run_local_gpu_model.sh \
  Qwen/Qwen3.5-4B qwen35_4b full
```

The full command is resumable: invoke the exact same command and output tag
after an interruption. Completed records are skipped. It runs each module in a
separate directory, then readiness, controller, grounding, and local
BERTScore.

### 8.3 Pending Qwen 3.8 27B BERTScore

This uses the already-frozen Groq outputs and does not contact Groq:

```bash
time ./evaluation/run_qwen38_bertscore.sh
```

Run this once on the GPU machine. It computes both Inner Speech and Outer
Speech BERTScore. Outer Speech is the pending value needed for the submitted
module table; Inner is retained for completeness and audit.

### 8.4 Qwen 3.5 9B

Stop the 4B server, set `LOCAL_MODEL=Qwen/Qwen3.5-9B` in the ignored env file,
and recreate the service using the same vLLM image/dtype/context/GPU topology:

```bash
docker compose --env-file evaluation/gpu.env \
  -f evaluation/docker-compose.gpu.yml down
docker compose --env-file evaluation/gpu.env \
  -f evaluation/docker-compose.gpu.yml up -d --wait
curl -fsS http://localhost:8000/v1/models | python3 -m json.tool

time ./evaluation/run_local_gpu_model.sh \
  Qwen/Qwen3.5-9B qwen35_9b smoke
```

If the smoke is valid:

```bash
time ./evaluation/run_local_gpu_model.sh \
  Qwen/Qwen3.5-9B qwen35_9b full
```

Do not reuse the 4B tag for the 9B run. The runner rejects most changed resume
settings, but separate tags also make accidental mixing obvious.

### 8.5 If the full run finishes but BERTScore was not installed

Install/repair the metric environment, then run only the final phase:

```bash
./evaluation/run_local_gpu_model.sh \
  Qwen/Qwen3.5-4B qwen35_4b bertscore
./evaluation/run_local_gpu_model.sh \
  Qwen/Qwen3.5-9B qwen35_9b bertscore
```

## 9. Quality-control checklist

Do not freeze a run until all checks below pass.

### 9.1 Expected raw record counts per local model

```bash
wc -l evaluation/results/local_qwen35_4b/modules_scope/raw.jsonl
wc -l evaluation/results/local_qwen35_4b/modules_intent/raw.jsonl
wc -l evaluation/results/local_qwen35_4b/modules_inner/raw.jsonl
wc -l evaluation/results/local_qwen35_4b/modules_query/raw.jsonl
wc -l evaluation/results/local_qwen35_4b/modules_outer/raw.jsonl
wc -l evaluation/results/local_qwen35_4b/readiness/raw.jsonl
wc -l evaluation/results/local_qwen35_4b/controller/raw.jsonl
wc -l evaluation/results/local_qwen35_4b/grounding/raw.jsonl
```

Expected values are respectively `186`, `51`, `40`, `30`, `31`, `58`, `132`,
and `60`. Repeat with the 9B root. BERTScore output must have 40 Inner records
and 31 Outer records.

### 9.2 Provenance

For each model verify:

- every metadata file names the exact model and `provider=custom`;
- `base_url` is the intended local endpoint;
- `qwen_disable_thinking=true` and `reasoning_effort=null`;
- `git_dirty=false` for the initial full run;
- the dataset hashes match across conditions;
- GPU model, VRAM, and driver are present in metadata;
- `server_models.json` identifies the served model;
- `local_provenance.txt` records the vLLM image inspection and hardware;
- both model runs used the same completion cap (default 512), context length,
  dtype, and serving topology unless an unavoidable exception is documented.

### 9.3 Failures versus infrastructure errors

Structured-output errors produced by a healthy server are model outcomes and
must remain in the results. HTTP 500 errors, CUDA OOM, crashed workers, truncated
downloads, or connection failures are infrastructure failures and must not be
quietly scored as conservative model decisions. If infrastructure fails:

1. preserve/rename the invalid output directory for diagnosis;
2. fix the server configuration;
3. start a new clearly named result directory;
4. rerun the complete affected condition; and
5. document why the earlier run was excluded.

Do not delete individual unfavorable model records and do not splice runs with
different prompts, checkpoints, quantization, or serving settings.

Inspect retries and structured-output failures in every `raw.jsonl`, not just
aggregate accuracy. The frozen 27B module run had zero of both; smaller models
may legitimately have more.

### 9.4 Database and grounding sanity

Rerun the deterministic grounding validator after any database restart. In the
full grounding results, verify that both conditions use the same expected
evidence and that no write query ran. Check exact retrieval, evidence precision
and recall, abstention, declared-claim support, and grounded task success. Do
not report natural-language fluency as grounding correctness.

### 9.5 Paired statistics

Run the existing exact paired tests on each accepted local raw file:

```bash
python3 -m evaluation.paired_stats \
  --suite readiness \
  --raw evaluation/results/local_qwen35_4b/readiness/raw.jsonl \
  --left inner_speech_gate \
  --right rule_gate

python3 -m evaluation.paired_stats \
  --suite controller \
  --raw evaluation/results/local_qwen35_4b/controller/raw.jsonl \
  --left janus_factored \
  --right direct_llm
```

Repeat for 9B and also compare JANUS with `janus_rule_gate`. Report point
estimates and intervals even if significance is not reached. One deterministic
generation per unique case is the analysis unit.

## 10. Interpreting latency and token results

- `api_latency_*` is client-observed request latency and is the primary local
  latency field.
- Groq-only provider timing may be absent from vLLM responses; do not replace
  missing provider timing with zero.
- `wall_latency` in the runner includes all orchestration overhead. There is no
  artificial local request delay, but database execution is part of the
  grounding interaction and should be described accurately.
- Report p50 and p95, not only a mean. A few warmup/compilation outliers can be
  visible at p95.
- The smoke is the warmup and is excluded from full-run summaries because it
  uses a separate result directory.
- Compare JANUS and Direct within the same model/server condition. Cross-provider
  Groq-versus-local latency is descriptive deployment profiling only.
- JANUS is expected to make more calls. A defensible result can be higher
  latency but lower premature execution or grounding failure; do not imply
  that JANUS is faster if it is not.

Use the smoke elapsed time to replace broad runtime guesses with a machine-
specific estimate. There are about 679 sequential model calls per checkpoint.
On a modern datacenter GPU, the full matrix may take roughly 1-4 hours per
model depending on prompt-prefill throughput, GPU topology, dtype, and tool-call
generation. Downloads/server initialization can add 10-60 minutes; BERTScore
itself should usually take minutes after checkpoints are cached. These are
planning estimates, not promised timings.

## 11. Freezing and returning the results

`evaluation/results/` is ignored by Git so exploratory and interrupted runs are
not committed accidentally. After both runs pass quality control, copy only
accepted complete artifacts into a new archival layout, for example:

```text
evaluation/frozen_results/local_qwen35_4b/
evaluation/frozen_results/local_qwen35_9b/
evaluation/frozen_results/qwen38_modules_bertscore/
```

Preserve each subdirectory's `raw.jsonl`, `summary.json`, `summary.csv`, and
`metadata.json`, plus `server_models.json` and `local_provenance.txt`. Add a
short `evaluation/GPU_RUN_REPORT.md` containing:

- exact Git commit;
- GPU(s), VRAM, driver, CUDA, PyTorch, vLLM image digest/version;
- model IDs, resolved Hugging Face snapshot revisions, dtype/quantization,
  tensor parallelism, context length, and completion cap;
- exact commands and start/end timestamps;
- raw-record counts and failure/retry audit;
- full primary summary tables and paired statistics;
- excluded runs and reasons;
- total wall time per model.

One explicit archival procedure is:

```bash
mkdir -p evaluation/frozen_results/local_qwen35_4b
mkdir -p evaluation/frozen_results/local_qwen35_9b
mkdir -p evaluation/frozen_results/qwen38_modules_bertscore
cp -a evaluation/results/local_qwen35_4b/. \
  evaluation/frozen_results/local_qwen35_4b/
cp -a evaluation/results/local_qwen35_9b/. \
  evaluation/frozen_results/local_qwen35_9b/
cp -a evaluation/results/qwen38_modules_inner_bertscore \
  evaluation/results/qwen38_modules_outer_bertscore \
  evaluation/frozen_results/qwen38_modules_bertscore/
```

Remove smoke and excluded diagnostic directories from the archival copies; do
not remove them from the original ignored result roots until the report is
finished. Ensure every retained directory is a complete accepted run.

Then commit and push only on the GPU result branch:

```bash
git add evaluation/frozen_results evaluation/GPU_RUN_REPORT.md
git commit -m "freeze local Qwen GPU reviewer experiments"
git push -u origin gpu-local-evaluation
```

Before pushing, run `git status --short`, inspect `git diff --cached --stat`, and
verify that no `.env`, token, API key, Hugging Face credential, cache, model
weight, or database file is staged. Inform the original-machine Codex of the
branch and commit hash; it can cherry-pick the result commit after finishing
the 27B API run.

## 12. What remains after the GPU handoff

The combined project is complete only after all of the following are done:

- finish the Qwen 3.8 27B JANUS-versus-Direct grounding run on Groq;
- finish and freeze Qwen 3.8 Inner/Outer BERTScore on the GPU;
- finish both local 4B and 9B complete matrices;
- audit every local infrastructure/structured-output failure;
- produce cross-model module, controller, grounding, latency, token, and
  BERTScore tables from frozen artifacts;
- select claims from the full predeclared metrics rather than favorable
  post-hoc subsets;
- update the manuscript's model/provider/hardware descriptions and limitations;
- answer the reviewers explicitly, including the lack of real-robot/user
  evaluation and the formal-scaffold status of the POMDP;
- decide whether the inconsistent Outer Speech few-shots are merely disclosed
  as a limitation or corrected in a separately rerun experiment.

The most realistic desired empirical narrative is:

> The semantic Inner Speech gate substantially reduces premature actions
> relative to a required-slot rule. Against a strong Direct LLM controller,
> factorization trades additional calls and latency for auditable intermediate
> decisions and may improve robustness or evidence grounding as model capacity
> decreases.

The second sentence is a hypothesis until the local and 27B grounding tables
support it. If Direct remains better, report that honestly and center the
significant Inner-Speech-versus-RuleGate result plus the measured
accuracy/latency trade-off.

## 13. Key files

- `evaluation/README.md`: harness overview and existing commands
- `evaluation/REVIEWER_RESULTS.md`: frozen controller/readiness tables
- `evaluation/MODULE_RESULTS_QWEN38.md`: module reproduction and error analysis
- `evaluation/FEWSHOT_AUDIT.md`: actual prompting paths and Git history
- `evaluation/GROUNDING_V1_AUDIT.md`: pre-run grounding integrity audit
- `evaluation/datasets/`: frozen held-out/reference datasets
- `evaluation/docker-compose.gpu.yml`: local Qwen vLLM server
- `evaluation/gpu.env.example`: non-secret server configuration template
- `evaluation/prepare_gpu_databases.sh`: deterministic database reset/seed/audit
- `evaluation/run_local_gpu_model.sh`: smoke/full/BERTScore model launcher
- `evaluation/run_qwen38_bertscore.sh`: pending frozen-27B semantic metrics
- `evaluation/benchmark.py`: readiness/controller comparison
- `evaluation/module_benchmark.py`: isolated manuscript module reproduction
- `evaluation/grounding_benchmark.py`: paired DB-grounding comparison
- `evaluation/score_module_text.py`: offline BERTScore with device provenance
