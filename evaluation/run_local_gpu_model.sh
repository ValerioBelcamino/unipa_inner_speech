#!/usr/bin/env bash
set -eu

if [ "$#" -lt 2 ] || [ "$#" -gt 3 ]; then
  echo "usage: $0 MODEL_ID RUN_TAG [smoke|full|bertscore]" >&2
  exit 2
fi

model_id=$1
run_tag=$2
mode=${3:-smoke}
result_root="evaluation/results/local_${run_tag}"
max_tokens=${LOCAL_MAX_COMPLETION_TOKENS:-512}

case "$mode" in
  smoke|full|bertscore) ;;
  *)
    echo "mode must be smoke, full, or bertscore" >&2
    exit 2
    ;;
esac

mkdir -p "$result_root"

if [ "$mode" = "bertscore" ]; then
  python3 -m evaluation.score_module_text \
    --source "$result_root/modules_inner" \
    --output-dir "$result_root/modules_inner_bertscore" \
    --batch-size "${BERTSCORE_BATCH_SIZE:-8}" \
    --device "${BERTSCORE_DEVICE:-cuda}"
  python3 -m evaluation.score_module_text \
    --source "$result_root/modules_outer" \
    --output-dir "$result_root/modules_outer_bertscore" \
    --batch-size "${BERTSCORE_BATCH_SIZE:-8}" \
    --device "${BERTSCORE_DEVICE:-cuda}"
  exit 0
fi

if [ -f evaluation/gpu.env ]; then
  set -a
  # shellcheck disable=SC1091
  . evaluation/gpu.env
  set +a
fi
if [ -n "${LOCAL_MODEL:-}" ] && [ "$LOCAL_MODEL" != "$model_id" ]; then
  echo "gpu.env serves $LOCAL_MODEL but the runner was given $model_id" >&2
  exit 2
fi
base_url=${LOCAL_BASE_URL:-http://localhost:${VLLM_PORT:-8000}/v1}

curl -fsS "$base_url/models" >"$result_root/server_models.json"
{
  echo "captured_at_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "git_sha=$(git rev-parse HEAD)"
  echo "git_branch=$(git branch --show-current)"
  echo "git_dirty=$(test -n "$(git status --porcelain)" && echo true || echo false)"
  echo "model_id=$model_id"
  echo "base_url=$base_url"
  echo "mode=$mode"
  echo "max_completion_tokens=$max_tokens"
  command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi || true
  command -v docker >/dev/null 2>&1 && docker version || true
  command -v docker >/dev/null 2>&1 && \
    docker image inspect "${VLLM_IMAGE:-vllm/vllm-openai:nightly}" || true
  if command -v docker >/dev/null 2>&1 && [ -f evaluation/gpu.env ]; then
    docker compose --env-file evaluation/gpu.env \
      -f evaluation/docker-compose.gpu.yml exec -T vllm \
      python3 -c \
      'import torch, transformers, vllm; print("torch", torch.__version__); print("transformers", transformers.__version__); print("vllm", vllm.__version__)' || true
    docker compose --env-file evaluation/gpu.env \
      -f evaluation/docker-compose.gpu.yml exec -T vllm \
      sh -c \
      'find /root/.cache/huggingface/hub -path "*/refs/main" -type f -print -exec sed -n "1p" {} \;' || true
  fi
} >"$result_root/local_provenance.txt" 2>&1

common=(
  --provider custom
  --model "$model_id"
  --base-url "$base_url"
  --qwen-disable-thinking
  --request-delay 0
  --max-retry-wait 60
  --max-attempts 2
  --max-completion-tokens "$max_tokens"
)

if [ "$mode" = "smoke" ]; then
  python3 -m evaluation.module_benchmark \
    --module all \
    --max-cases 1 \
    "${common[@]}" \
    --output-dir "$result_root/modules_smoke"

  python3 -m evaluation.benchmark \
    --suite controller \
    --architectures all \
    --max-cases 3 \
    --intent-interface native_tools \
    --structured-interface native_tools \
    "${common[@]}" \
    --output-dir "$result_root/controller_smoke"

  python3 -m evaluation.grounding_benchmark \
    --case-id g-pc-06 \
    --architectures factored,direct \
    "${common[@]}" \
    --output-dir "$result_root/grounding_smoke"
  exit 0
fi

# The full module suites are deliberately separated. This lets BERTScore
# consume the Inner and Outer raw outputs directly and makes interrupted runs
# cheap to resume.
for module_name in scope intent inner query outer; do
  python3 -m evaluation.module_benchmark \
    --module "$module_name" \
    "${common[@]}" \
    --output-dir "$result_root/modules_${module_name}"
done

python3 -m evaluation.benchmark \
  --suite readiness \
  --architectures all \
  --structured-interface native_tools \
  "${common[@]}" \
  --output-dir "$result_root/readiness"

python3 -m evaluation.benchmark \
  --suite controller \
  --architectures all \
  --intent-interface native_tools \
  --structured-interface native_tools \
  "${common[@]}" \
  --output-dir "$result_root/controller"

python3 -m evaluation.grounding_benchmark \
  --architectures factored,direct \
  "${common[@]}" \
  --output-dir "$result_root/grounding"

"$0" "$model_id" "$run_tag" bertscore
