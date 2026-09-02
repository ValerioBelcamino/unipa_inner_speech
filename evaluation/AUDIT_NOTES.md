# Evaluation audit notes

These notes document issues observed on the `memory-agent` branch before adding
the reviewer benchmark.  They are not changes to the previously reported data.

1. The published module tests stream metrics to LangSmith.  No raw result export
   or frozen aggregate is tracked in this repository, so the paper tables cannot
   be independently reconstructed from repository artifacts alone.
2. `query_generation/test/test_query_generation_LLM.py` injects all entries from
   `examples_ADVISOR.json` into `qg_llm.examples` and evaluates those same 30
   entries.  The reported query result overlap is consequently not a held-out
   generalization estimate.  New reviewer comparisons must not reuse those
   examples as test inputs.
3. The manuscript states that all unit tests use
   `meta-llama/llama-4-scout-17b-16e-instruct`, with module temperatures 0, 0.2,
   and 0.4 according to role.  The current `.config` instead selects Llama 4
   Maverick for Query Generation and sets every module temperature to 0.  Every
   new run must freeze these values in metadata and the revised manuscript must
   report the configuration actually run.
4. The old tests enable LangSmith tracing at import time and several require
   keys or online translation/model downloads.  Query Generation also sleeps 90
   seconds after every item.  They are not suitable as the revision's primary
   reproducible runner.
5. The ROS Inner Speech node treats values equal to numeric zero as missing.
   Because Python considers `False == 0`, a known
   `ha_piano_settimanale=False` flag is classified as missing.  The new benchmark
   makes the distinction explicit; the production semantics should be clarified
   before drawing claims from no-plan cases.
6. The paper's laptop CPU/GPU description is not the LLM inference hardware for
   Groq API runs.  Report Groq latency as remote API end-to-end latency.  Report
   local inference separately with exact local hardware, model quantization or
   Ollama digest, warm/cold policy, and p50/p95.
7. Readiness sets naturally contain more blocking than proceed cases when they
   stress upstream errors.  Accuracy alone rewards an always-block controller.
   The revision benchmark therefore reports proceed recall, block recall, and
   balanced accuracy together with the two directional error rates.
8. Intent Recognition and Inner Speech do not load few-shot demonstrations in
   the current runtime. The similarly named `examples_ADVISOR.json` test files
   are evaluation cases, not prompt examples. Both modules used few-shot text
   prompts in the initial prototype (`732af43`), but Intent switched to native
   Pydantic tool calling in `651d108`, and Inner Speech was explicitly changed
   to zero-shot in `72b05ca`. The retained legacy few-shot JSON files use old
   schemas and contain inconsistent labels; do not re-enable them unchanged.
   See `FEWSHOT_AUDIT.md` for the full trace.
9. During the native multi-domain integration pilot, Scope Detection treated an
   in-domain request with two candidate movies as `OutOfScope`. The runtime
   Intent prompt also previously instructed the model not to select a tool for
   vague in-domain requests. This conflated routing/semantic parsing with the
   downstream readiness decision. The revision branch now makes the factorized
   contract explicit: Scope routes identifiable domains despite incomplete or
   ambiguous values; Intent selects an identifiable task without guessing; and
   Inner Speech alone gates execution or clarification. The same prompt change
   is applied to production modules and the evaluation harness.
10. Eight completed cases in the ignored development directory
    `evaluation/results/qwen38_native_controller_full` predate the contract
    clarification above (Git `f5a9c30`). They are a partial diagnostic only and
    must not be resumed into or combined with a post-change run. The final
    controller experiment requires a new output directory and one Git revision.
11. Post-fix diagnostic probes are not final results. On one frozen ambiguous
    controller case (`ai-dish-01`), Qwen 3.8 selected `DishInfo` with one of two
    candidate dishes but Inner Speech correctly blocked execution because that
    selection required guessing (2 calls, 1958 tokens, 1.244 s API latency).
    This verifies the intended failure-recovery path, but `n=1` is not evidence
    for an aggregate claim. A seven-case Qwen 3.6 pilot was unsuitable as a
    substitute: its auto-tool Intent often emitted no task for relevant
    incomplete inputs and several forced function calls failed. A GPT-OSS 120B
    multi-domain pilot was invalidated by its 8000-token-per-minute provider
    limit and must not be scored as model behavior.
12. The runner now pauses before writing the current case when a provider rate
    limit cannot be retried within the configured wait cap. This prevents 429
    responses from being recorded as conservative `reject`/`clarify` model
    predictions. The Qwen 3.8 final launcher uses new output directories and is
    resumable across its rolling 200000-token daily quota.
