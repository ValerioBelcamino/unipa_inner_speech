# Frozen reviewer-evaluation results

Date: 2026-09-02. Primary model: `qwen/qwen3.8-27b` through the Groq
OpenAI-compatible API. The final native-tool controller run used benchmark
commit `28ba2a095d5737fe9260399fc7a22179157aa49f`; the earlier readiness
ablation used `4284d90cbc3d86527c72b43bfe311761351a0ac1`. All primary runs
used the same model within every paired condition, one generation per case,
temperatures 0.0 (Intent), 0.2 (Inner Speech), and 0.0 (Direct). The controller
used a 384-token completion cap after native-tool pilots observed a maximum of
198; the readiness ablation used 1024. Both frozen Qwen runs had zero retries
and zero structured-output failures.

Llama 4 Scout, the model named in the submitted manuscript, returned
`model_not_found` on 2026-09-02. GPT-OSS 20B was initially selected as a
similar-scale replacement, but its rolling daily quota was exhausted before
the strengthened controller rerun. Qwen 3.8 27B was therefore frozen as the
primary model. A complete GPT-OSS readiness run is retained as a secondary
cross-model check.

## Native-tool controller comparison (44 held-out cases)

| Condition | Operational task success | Strict state match | Decision accuracy | Parameter micro-F1 | Premature execution | API p50 / p95 | Median tokens | Calls |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Direct LLM | 43/44 (97.7%) | 26/44 (59.1%) | 97.7% | 93.0% | 0/26 (0%) | 0.930 / 1.320 s | 1398.5 | 1.0 |
| JANUS factored | 39/44 (88.6%) | 25/44 (56.8%) | 95.5% | 88.3% | 2/26 (7.7%) | 1.434 / 1.903 s | 2049 | 2.0 |
| JANUS RuleGate | 31/44 (70.5%) | 23/44 (52.3%) | 77.3% | 88.3% | 10/26 (38.5%) | 0.691 / 0.928 s | 1081.5 | 1.0 |

Operational task success requires the correct decision and action, plus exact
parameters when the gold decision is `execute`. Strict state match additionally
requires exact partial state on clarify/reject turns.

The Direct baseline used one forced typed function, received the same request,
memory, tool context, and task contracts, and was explicitly instructed to
preserve partial state. It is therefore a strong baseline rather than a weak
chatbot. JANUS Intent used native auto tool calling, as in the runtime; JANUS
Inner Speech and Direct used the same forced-function structured-output method.

JANUS factored significantly outperformed its paired RuleGate ablation on
operational success: eight discordant cases favored JANUS and zero favored the
RuleGate (two-sided exact McNemar p=0.0078125). It reduced premature execution
from 10/26 to 2/26, an 80% relative reduction, while neither condition produced
an unnecessary clarification on the 18 execution-ready cases. In the ten
ambiguous/inconsistent cases, JANUS succeeded on 7/10 while RuleGate succeeded
on 0/10 and prematurely executed all 9 cases whose gold decision was not
`execute`.

Direct achieved higher operational success than JANUS, with five discordant
cases favoring Direct and one favoring JANUS; the paired difference was not
significant at this sample size (two-sided exact McNemar p=0.21875). Direct made
no premature executions but unnecessarily clarified one valid memory case.
JANUS's two unsafe decisions came from accepting an upstream resolution of
contradictory calories and an ambiguous memory reference. Its other three
operational failures were exact-parameter extraction omissions despite correct
control decisions. This supports the Inner Speech ablation claim, not global
superiority over Direct.

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
direction, with 8 vs 0 pairs (p=0.00781). This is the strongest isolated evidence
for Inner Speech: it improves semantic readiness auditing over a required-slot
rule without rejecting any of the six valid cases.

## Latency interpretation

The API latency fields exclude the deliberate 7.5-second inter-request throttle;
the stored wall time includes it and must not be presented as model latency.
Relative to Direct, JANUS factored added one logical LLM call, increased median
API latency from 0.930 to 1.434 seconds (about 54%), and increased median token
use from 1398.5 to 2049 (about 47%). This is a measurable cost, not a latency
improvement. The RuleGate median was 0.691 seconds because it reused Intent and
made no second inference.

The local CPU pilot is also negative: Qwen 2.5 3B Q4_K_M had a 13.79-second
median in the earlier complete run and blocked all 6/6 valid cases. A clean
rerun later hit a 180-second timeout without returning tokens. Do not claim
local CPU inference solves latency. A publishable local deployment experiment
requires accelerated hardware and a stronger model that retains non-zero
proceed recall.

## Claims supported by these experiments

- Supported: Inner Speech catches more semantic readiness failures than a
  required-slot rule, across two model families, at added latency/token cost.
  In the end-to-end controller this produced an 80% relative reduction in
  premature executions and a significant paired operational improvement.
- Not supported: the current factored controller globally outperforms a strong
  Direct LLM controller on these 44 cases. Direct had higher point estimates,
  although the paired operational difference was not significant.
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
  --raw evaluation/frozen_results/qwen38_controller_native_final/raw.jsonl \
  --left janus_factored \
  --right direct_llm
```

Raw model outputs contain synthetic benchmark requests only and no API keys.
The earlier legacy-JSON controller run remains under
`evaluation/frozen_results/qwen38_controller` solely for audit. It predates the
routing/readiness contract correction and must not be reported as the final
architectural comparison.
