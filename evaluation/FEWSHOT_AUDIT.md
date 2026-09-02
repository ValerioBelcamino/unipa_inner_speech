# Few-shot and Intent-interface audit

Date: 2026-09-02. This audit explains which JANUS modules actually receive
few-shot demonstrations and why the first reviewer controller run must not be
used as an architectural comparison.

## Current runtime

| Module | Current prompting path | Few-shot demonstrations in the prompt? |
|---|---|---:|
| Intent Recognition | Pydantic schemas through LangChain `bind_tools` | No |
| Inner Speech | Pydantic `with_structured_output` plus system/user messages | No |
| Query Generation | `prepare_few_shot_prompt` with scenario examples | Yes |
| Explainability / Outer Speech | `prepare_few_shot_prompt` with scenario examples | Yes |

The files below exist in the repository but no current Python runtime imports,
opens, or loads them:

- `intent_recognition/intent_recognition/few_shot_examples/FewShot_intent.json`
- `inner_speech/inner_speech/few_shot_examples/FewShot_intent_IS.json`

The `examples_ADVISOR.json` files under `intent_recognition/test` and
`inner_speech/test` are parametrized **evaluation inputs and references**. They
are not added to the messages sent to the model.

## Git history

- Commit `732af43` (2025-02-06) loaded few-shot files in the original textual
  Intent Recognition and Inner Speech prototypes.
- Commit `651d108` (2025-04-09) replaced textual Intent Recognition with
  Pydantic native tool calling. The few-shot prompt was removed in that
  migration.
- Commit `72b05ca` (2025-04-09), whose subject is
  `fix intent_recognition, 0-shot inner_speech`, removed the Inner Speech
  few-shot prompt. At that stage readiness was computed procedurally and the
  model verbalized the result zero-shot.
- Commit `c90ca5e` (2025-05-21) introduced the current reusable LLM classes.
  Intent Recognition continued to use native tool calling and Inner Speech
  structured output, both without few-shot messages.

The old files remained after these migrations. Their schemas are no longer
compatible with the runtime: they use numeric `action_id` values and obsolete
fields such as `completed`, `response`, `allergeni`, and parameter names marked
with `!`. Some examples also contain inconsistent labels. They must not be
silently re-enabled for the reviewer experiment.

## Manuscript interpretation

The submitted manuscript explicitly describes runtime prompt assets as, for
example, Query Generation and Outer Speech examples. Its later general remark
that long instructions and exemplars are used does not establish that every
module is few-shot. This is consistent with the runtime map above. The revision
should state the prompting regime per module to remove the ambiguity.

## Consequence for the reviewer benchmark

The first frozen Qwen controller run at commit `4284d90` used a single JSON
schema for the factorized Intent stage. That is not JANUS's runtime interface.
It induced seven apparent `OutOfScope` errors on incomplete or ambiguous
in-domain requests. A focused pilot using the native tool interface selected an
in-domain task in six of those seven cases, without adding few-shot examples.

Therefore:

1. the old JSON-interface controller table is a diagnostic pilot, not a result
   for the paper;
2. the primary rerun must use `--intent-interface native_tools` (now the
   harness default);
3. the held-out readiness ablation remains valid because it freezes the
   upstream Intent state and does not depend on the Intent output interface;
4. if new few-shot prompts are tested, they need a clean development set,
   current schemas, and comparable demonstrations for the direct baseline.
   They must not be copied from the held-out evaluation cases.
