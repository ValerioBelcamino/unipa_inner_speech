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
