# Local readiness pilot (not a publication result)

Date: 2026-09-01.  Suite: `readiness_v1`, 29 cases, one run per case.

This pilot verifies the local runner and establishes whether a small CPU model
is a plausible latency mitigation.  It was executed from a dirty development
tree before the benchmark commit, so it must be rerun from a frozen revision
before any number is copied into the manuscript.

## Configuration

- Ollama 0.11.10 in Docker under WSL2, CPU inference only;
- Intel Core Ultra 7 258V, 8 logical CPUs, 16 GB RAM;
- `qwen2.5:3b-instruct`, 3.1B, GGUF Q4_K_M;
- model digest
  `357c53fb659c5076de1d65ccb0b397446227b71a42be9d1603d46168015c9e4b`;
- temperature 0.2, one attempt, 128-token completion cap, warm model;
- zero structured-output failures.

## Results

| Condition | Accuracy | Balanced accuracy | Proceed recall | Block recall | Premature proceed | Unnecessary block | p50 | p95 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Inner Speech gate | 79.3% | 50.0% | 0.0% | 100.0% | 0/23 | 6/6 | 13.79 s | 25.33 s |
| Required-slot RuleGate | 51.7% | 69.6% | 100.0% | 39.1% | 14/23 | 0/6 | <0.001 s | <0.001 s |

On the 14 semantic upstream-error cases alone, Inner Speech blocked 14/14 and
RuleGate blocked 0/14.  However, the local Inner Speech model also blocked all
6/6 valid cases.  Its apparently high raw accuracy is caused by the negative
class prevalence; balanced accuracy exposes an always-block-like policy.

## Interpretation

This local configuration is not a latency solution and is not reliable enough
to support the paper's architectural claim.  It is evidence that small local
models can detect semantic anomalies, but also that deployment-model selection
is part of the safety/performance trade-off. The main paired comparison should
use the same Groq model in all conditions. Llama 4 Scout was no longer exposed
to this account on 2026-09-02, so the primary frozen run uses
`qwen/qwen3.8-27b` and documents that substitution. A local appendix is worth
including only after rerunning on accelerated hardware or with a stronger
task-specialized model and showing non-zero proceed recall.

## Clean rerun attempt

On 2026-09-02, a clean rerun from commit `4284d90` used the same retained
model, a warm Ollama container, one attempt, and a 128-token cap. The first
readiness request returned no tokens and hit the 180-second client timeout;
the second request was manually stopped. Ollama reported full CPU utilization
and a cancelled generation. This failed attempt reinforces the conclusion that
this CPU-only setup is not a defensible latency mitigation. It is not included
in any accuracy or latency table.
