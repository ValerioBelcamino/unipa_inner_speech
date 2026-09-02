"""Seed a dedicated Neo4j graph with the external facts in the Intent suite.

The graph used by Query Generation is intentionally not reused: its current
population script gives every user a seven-day plan, while the Intent reference
set contains both users with and without complete plans.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

from dotenv import load_dotenv
from neo4j import GraphDatabase


REPO_ROOT = Path(__file__).resolve().parents[1]
DATASET = REPO_ROOT / "intent_recognition" / "test" / "examples_ADVISOR.json"
DAYS = [
    "lunedi",
    "martedi",
    "mercoledi",
    "giovedi",
    "venerdi",
    "sabato",
    "domenica",
]


def _plan_facts() -> dict[str, bool]:
    entries = json.loads(DATASET.read_text(encoding="utf-8"))
    facts: dict[str, bool] = {}
    for entry in entries:
        if entry["action_name"] != "SubstituteDish":
            continue
        parameters = entry["parameters"]
        name = str(parameters.get("nome_utente", "")).strip().lower()
        if not name:
            continue
        has_plan = bool(parameters["ha_piano_settimanale"])
        if name in facts and facts[name] != has_plan:
            raise RuntimeError(f"conflicting weekly-plan labels for {name}")
        facts[name] = has_plan
    return facts


def main() -> int:
    load_dotenv(REPO_ROOT / ".env", override=True)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--neo4j-uri", default="bolt://localhost:18687")
    parser.add_argument("--neo4j-user", default="neo4j")
    parser.add_argument("--neo4j-password-env", default="NEO4J_PASSWORD")
    args = parser.parse_args()
    password = os.getenv(args.neo4j_password_env, "password")

    rows = [
        {"name": name, "days": DAYS if has_plan else DAYS[:1]}
        for name, has_plan in sorted(_plan_facts().items())
    ]
    driver = GraphDatabase.driver(args.neo4j_uri, auth=(args.neo4j_user, password))
    try:
        driver.verify_connectivity()
        with driver.session() as session:
            count = session.run("MATCH (n) RETURN count(n) AS count").single()["count"]
            if count:
                raise SystemExit(
                    "Intent fixture graph is not empty; recreate its dedicated "
                    "container instead of overwriting data"
                )
            session.run(
                """
                UNWIND $rows AS row
                CREATE (p:Person {name: row.name, benchmark_fixture: true})
                WITH p, row
                UNWIND row.days AS day
                MERGE (d:Dish {name: 'intent_benchmark_placeholder'})
                CREATE (p)-[:SHOULD_EAT {day: day, meal: 'cena'}]->(d)
                """,
                rows=rows,
            ).consume()
    finally:
        driver.close()

    print(f"Seeded {len(rows)} user plan facts at {args.neo4j_uri}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
