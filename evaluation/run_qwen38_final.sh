#!/usr/bin/env bash
set -eu

# Run from the repository root. The benchmark loads GROQ_API_KEY from the
# ignored .env file and never writes it to result artifacts.
python3 -m evaluation.benchmark \
  --suite controller \
  --provider groq \
  --model qwen/qwen3.8-27b \
  --intent-interface native_tools \
  --structured-interface native_tools \
  --max-completion-tokens 384 \
  --max-attempts 3 \
  --max-retry-wait 60 \
  --request-delay 7.5 \
  --output-dir evaluation/results/qwen38_controller_native_final

python3 -m evaluation.benchmark \
  --suite multidomain \
  --provider groq \
  --model qwen/qwen3.8-27b \
  --structured-interface native_tools \
  --max-completion-tokens 384 \
  --max-attempts 3 \
  --max-retry-wait 60 \
  --request-delay 7.5 \
  --output-dir evaluation/results/qwen38_multidomain_final
