# Qwen 3.8 reproduction of the submitted module evaluation

Run dates: 2026-09-02--03. Model: `qwen/qwen3.8-27b` through the Groq
OpenAI-compatible API. These runs reproduce the isolated quantitative module
tests from the submitted manuscript; they are separate from the held-out
controller and readiness comparisons documented in `REVIEWER_RESULTS.md`.

All 338 requested generations completed with one successful attempt and no API,
tool-call, or structured-output failure. The runs used the submitted prompts,
few-shot examples, datasets, and temperatures, with reasoning effort set to
`none` and a 512-token completion cap. Raw outputs and provenance are frozen in
`evaluation/frozen_results/qwen38_modules/`.

## Task metrics

| Module / configuration | Cases | Submitted Llama 4 Scout | Qwen 3.8 27B |
|---|---:|---:|---:|
| Scope MHS accuracy | 24 | 100% | 100% |
| Scope HSR accuracy | 24 | 95.8% | 100% |
| Scope ATT accuracy | 24 | 100% | 100% |
| Scope TEF accuracy | 24 | 100% | 100% |
| Scope TER accuracy | 24 | 100% | 100% |
| Scope ALL accuracy | 66 | 98.5% | 100% |
| Intent task-selection accuracy | 51 | 100% | 100% |
| Intent parameter micro-F1 | 51 | 100% | 94.7% |
| Inner Speech readiness accuracy | 40 | 100% | 95.0% |
| Query valid-query rate | 30 cases / 40 queries | 100% | 100% |
| Query result overlap | 30 | 96.7% | 93.3% |
| Outer Speech BERTScore F1 | 31 | 0.87 (SD 0.03) | Pending |

The Intent exact parameter-set match was 86.3%. Its seven mismatches were
omissions rather than routing errors: six omitted an explicit user name for a
`DishInfo` request, and one omitted a carbohydrate constraint. Inner Speech's
two errors were false-proceed decisions. This unit-suite result should not be
substituted for the separate semantic-readiness ablation, which contains the
hard ambiguity and contradiction cases used to test the gate's contribution.

All 40 generated Cypher queries executed successfully. The legacy result-overlap
score is retained unchanged for comparability. Its two zero-score cases returned
the correct entities and values but used descriptive result aliases
(`person`/`allergen`) instead of the reference's `p`/`a`; the prompt explicitly
asks the model to use aliases. A third result differed only by returning the
additional `ingredients` field and received full credit under the submitted
subset-aware scorer. These observations are an error analysis, not a
post-hoc replacement of the primary metric.

## Candidate strict and operational reporting views

Keep the submitted metric in the primary reproducibility table unless the old
Llama outputs can be re-scored with the same new rule. The following secondary
views are nevertheless useful for explaining what the nominal errors mean:

| Component | Strict submitted-style score | Secondary audited score |
|---|---:|---:|
| Query results | 28/30 (93.3%) legacy overlap | 30/30 (100%) expected semantic content, ignoring top-level Cypher aliases |
| Inner Speech | 38/40 (95.0%) `can_proceed` label agreement | 40/40 (100%) effective dispatch decision |
| Intent | 44/51 (86.3%) exact parameter dictionaries | 227/234 (97.0%) individual parameter values |

For Query Generation, the only two legacy failures are alias-only and can be
counted as semantically correct under an alias-invariant evaluator. For Inner
Speech, both disagreements have an upstream `OutOfScope` action: Qwen considers
the nutrition-related utterance answerable, but JANUS's deterministic controller
still blocks dispatch whenever `action_name == OutOfScope`. Thus neither error
causes an unsupported action. For Intent, six of the seven wrong values are an
empty optional `DishInfo.nome_utente` despite an explicit name in the utterance;
the remaining error omits the requested `carboidrati` property. Task selection
is still 51/51. These secondary views should be labelled as error analysis or
operational metrics rather than silently replacing the stricter values.

## Provider latency and token use

`provider_total_time` is the Groq-reported inference time and is the field
comparable to the manuscript latency table. `api_latency` also includes network
and client overhead. Stored wall time includes the deliberate inter-request
throttle and must not be reported as inference latency.

| Module | Submitted p50 | Qwen p50 | Change | Submitted median tokens | Qwen median tokens | Change |
|---|---:|---:|---:|---:|---:|---:|
| Scope, pooled | 0.370 s | 0.443 s | +19.6% | 1,477 | 1,154.5 | -21.8% |
| Intent | 0.260 s | 0.267 s | +2.5% | 1,705 | 1,442 | -15.4% |
| Inner Speech | 0.810 s | 0.710 s | -12.3% | 572 | 741.5 | +29.6% |
| Query Generation | 0.740 s | 0.944 s | +27.5% | 3,986 | 3,521.5 | -11.7% |
| Outer Speech | 0.270 s | 0.158 s | -41.6% | 772 | 751 | -2.7% |

Qwen therefore does not uniformly improve latency: it is faster for Inner and
Outer Speech, nearly unchanged for Intent, and slower for Scope and Query
Generation. It uses fewer median tokens in four of the five modules, while
Inner Speech uses about 30% more. The model replacement should be described as
a new reproducibility run, not as evidence that the architecture itself became
faster.

## Protocol and interpretation limits

- Query Generation uses the submitted legacy protocol, in which evaluated
  examples also occur among the few-shot demonstrations. It is an in-sample
  functional regression and must not be presented as held-out generalization.
- Intent post-processing uses a dedicated Neo4j fixture matching the submitted
  database naming convention. This isolates an inconsistency between the
  repository's current example graph and the original Intent labels.
- Scope results pool six prompt configurations of different sizes. Individual
  configuration token counts remain available in the frozen summaries.
- No traces or human evaluations were rerun. Outer Speech's reference-based
  BERTScore remains pending; zero output failures alone is not a quality score.
- The Groq request delay was used only to respect rate limits. Every summary
  includes API and provider latency separately so the throttle cannot
  accidentally inflate the reported model latency.

## Frozen artifact layout

The archive contains `raw.jsonl`, `summary.json`, `summary.csv`, and
`metadata.json` for each group:

- `scope_mhs_att_tef`, `scope_hsr`, `scope_ter`, and `scope_all`;
- `intent`, `inner`, `query`, and `outer`.

The metadata records the model identifier, prompt profile, dataset hashes, Git
revision, temperatures, reasoning setting, database endpoints, retry policy,
and any resume event. No API keys are stored in the archive.
