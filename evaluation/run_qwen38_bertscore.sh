#!/usr/bin/env bash
set -eu

batch_size=${BERTSCORE_BATCH_SIZE:-8}
device=${BERTSCORE_DEVICE:-cuda}

python3 -m evaluation.score_module_text \
  --source evaluation/frozen_results/qwen38_modules/inner \
  --output-dir evaluation/results/qwen38_modules_inner_bertscore \
  --batch-size "$batch_size" \
  --device "$device"

python3 -m evaluation.score_module_text \
  --source evaluation/frozen_results/qwen38_modules/outer \
  --output-dir evaluation/results/qwen38_modules_outer_bertscore \
  --batch-size "$batch_size" \
  --device "$device"
