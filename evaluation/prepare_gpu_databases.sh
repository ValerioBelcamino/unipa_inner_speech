#!/usr/bin/env bash
set -eu

# These are dedicated, tmpfs-backed benchmark databases. Recreating them is
# intentional: it prevents stale state from invalidating the frozen references.
docker compose -f evaluation/docker-compose.query.yml down
docker compose -f evaluation/docker-compose.intent.yml down
docker compose -f evaluation/docker-compose.query.yml up -d --wait
docker compose -f evaluation/docker-compose.intent.yml up -d --wait

NEO4J_URI=neo4j://localhost:19687 \
NEO4J_PASSWORD=password \
POPULATE_RANDOM_SEED=42 \
  python3 query_generation/query_generation/populate_database.py

BENCHMARK_NEO4J_PASSWORD=password \
  python3 -m evaluation.seed_intent_database \
    --neo4j-password-env BENCHMARK_NEO4J_PASSWORD

python3 -m evaluation.grounding \
  --dataset evaluation/datasets/grounding_v1.json \
  --neo4j-uri bolt://localhost:19687 \
  --neo4j-password password

python3 -m evaluation.module_benchmark --validate-only --module all
python3 -m evaluation.benchmark --suite readiness --validate-only
python3 -m evaluation.benchmark --suite controller --validate-only

echo "Dedicated Query and Intent benchmark databases are ready."
