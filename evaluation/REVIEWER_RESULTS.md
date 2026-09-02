# Frozen reviewer-evaluation results

Date: 2026-09-02. Primary model: `qwen/qwen3.8-27b` through the Groq
OpenAI-compatible API. Benchmark commit: `4284d90cbc3d86527c72b43bfe311761351a0ac1`.
All primary runs used the same model within every condition, one generation per
case, temperatures 0.0 (Intent), 0.2 (Inner Speech), and 0.0 (Direct), a
1024-token completion cap, and up to three attempts. Both frozen Qwen runs had
zero retries and zero structured-output failures.

Llama 4 Scout, the model named in the submitted manuscript, returned
`model_not_found` on 2026-09-02. GPT-OSS 20B was initially selected as a
similar-scale replacement, but its 200k-token rolling daily quota was exhausted
before the strengthened controller rerun. Qwen 3.8 27B was therefore frozen as
the primary model. A complete GPT-OSS readiness run is retained as a secondary
cross-model check.

## Controller comparison (44 held-out cases)

| Condition | Operational task success | Strict state match | Decision accuracy | Parameter micro-F1 | Premature execution | API p50 / p95 | Median tokens | Calls |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Direct LLM | 42/44 (95.5%) | 25/44 (56.8%) | 95.5% | 92.2% | 1/26 (3.8%) | 0.647 / 1.013 s | 1174 | 1.0 |
| JANUS factored | 36/44 (81.8%) | 31/44 (70.5%) | 81.8% | 87.6% | 1/26 (3.8%) | 0.992 / 1.405 s | 1623.5 | 2.0 |
| JANUS RuleGate | 34/44 (77.3%) | 29/44 (65.9%) | 77.3% | 87.6% | 3/26 (11.5%) | 0.509 / 0.714 s | 944.5 | 1.0 |

Operational task success requires the correct decision and action, plus exact
parameters when the gold decision is `execute`. Strict state match additionally
requires exact partial state on clarify/reject turns. The Direct prompt was
strengthened before freezing to require preservation of all known parameters,
so this is not a deliberately weak chatbot baseline.

Direct outperformed JANUS on operational success. The paired difference between
JANUS factored and Direct was not significant at this sample size (discordant
pairs 2 vs 8; two-sided exact McNemar p=0.109). JANUS had higher strict state
match, also without reaching 0.05 (7 vs 1; p=0.070). JANUS factored versus
RuleGate operational success differed on only two cases (2 vs 0; p=0.5).

The main JANUS failure is upstream: Qwen Intent Recognition mapped seven
in-domain but incomplete/ambiguous requests to `OutOfScope`; the downstream
Inner Speech gate cannot recover the discarded action. This behavior is
consistent with the repository's actual Intent prompt, which says that vague
requests not directly referring to a tool should receive no intent. It should
be reported as a current limitation, not hidden by aggregate parameter metrics.

## Frozen readiness ablation (29 cases)

| Model / gate | Accuracy | Balanced accuracy | Proceed recall | Block recall | Premature proceed | Unnecessary block | API p50 / p95 |
|---|---:|---:|---:|---:|---:|---:|---:|
| Qwen Inner Speech | 25/29 (86.2%) | 91.3% | 6/6 (100%) | 19/23 (82.6%) | 4/23 (17.4%) | 0/6 | 0.436 / 1.198 s |
| Qwen RuleGate | 15/29 (51.7%) | 69.6% | 6/6 (100%) | 9/23 (39.1%) | 14/23 (60.9%) | 0/6 | <0.001 / <0.001 s |
| GPT-OSS Inner Speech | 23/29 (79.3%) | 87.0% | 6/6 (100%) | 17/23 (73.9%) | 6/23 (26.1%) | 0/6 | 0.685 / 1.149 s |
| GPT-OSS RuleGate | 15/29 (51.7%) | 69.6% | 6/6 (100%) | 9/23 (39.1%) | 14/23 (60.9%) | 0/6 | <0.001 / <0.001 s |

The frozen upstream state includes six valid requests, six syntactically
incomplete requests, fourteen semantic upstream errors, and three out-of-scope
requests. Qwen Inner Speech versus RuleGate has 11 vs 1 discordant correct pairs
(two-sided exact McNemar p=0.00635). GPT-OSS independently shows the same
direction, with 8 vs 0 pairs (p=0.00781). This is the strongest evidence for
Inner Speech: it improves semantic readiness auditing over a required-slot rule
without rejecting any of the six valid cases.

## Latency interpretation

The API latency fields exclude the deliberate 7.5-second inter-request throttle;
the stored wall time includes it and must not be presented as model latency.
Relative to Direct, JANUS factored added one logical LLM call, increased median
API latency from 0.647 to 0.992 seconds (about 53%), and increased median token
use from 1174 to 1623.5 (about 38%). This is a measurable cost, not a latency
improvement.

The local CPU pilot is also negative: Qwen 2.5 3B Q4_K_M had a 13.79-second
median in the earlier complete run and blocked all 6/6 valid cases. A clean
rerun later hit a 180-second timeout without returning tokens. Do not claim
local CPU inference solves latency. A publishable local deployment experiment
requires accelerated hardware and a stronger model that retains non-zero
proceed recall.

## Claims supported by these experiments

- Supported: Inner Speech catches more semantic readiness failures than a
  required-slot rule, across two model families, at added latency/token cost.
- Not supported: the current factored controller globally outperforms a strong
  Direct LLM controller on these 44 cases.
- Not tested: embodied HRI quality, actuation safety, real-user perception,
  optimal POMDP policy, or graph-memory superiority over document memory.

Use the phrase **structured factored controller versus direct LLM tool-use
controller**, not “POMDP planner versus LLM planner.” The POMDP in JANUS remains
a formal scaffold rather than a reward-optimized solved policy.

## Reproduction and statistical checks

The primary dataset hashes are recorded in each frozen `metadata.json`. Run:

```bash
python3 -m evaluation.paired_stats \
  --suite readiness \
  --raw evaluation/frozen_results/qwen38_readiness/raw.jsonl \
  --left inner_speech_gate \
  --right rule_gate

python3 -m evaluation.paired_stats \
  --suite controller \
  --raw evaluation/frozen_results/qwen38_controller/raw.jsonl \
  --left janus_factored \
  --right direct_llm
```

Raw model outputs contain synthetic benchmark requests only and no API keys.
