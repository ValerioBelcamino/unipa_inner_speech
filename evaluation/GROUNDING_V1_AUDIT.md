# Grounding v1 pre-run audit

This audit was completed on 2026-09-03 **before any model was evaluated on
`grounding_v1`**.

- Dataset: `evaluation/datasets/grounding_v1.json`
- Dataset SHA-256: `bd35e2c47044fa9c0c7491bacb3591430428996d8c15ed1e3a2d68cf0151fd5c`
- Population seed: `42`
- Full Neo4j snapshot SHA-256:
  `e6945f94f060aef7e04ce17b0bafe0df372abb60ae8c79c284ddd202251b8436`
- Cases: `30`
- Reference evidence rows: `29`
- Deliberately empty-evidence cases: `2`
- Reference execution mismatches: `0`
- Existing JSON prompt strings audited: `657` (`494` normalized unique)
- Normalized exact prompt overlaps: `0`

Category counts:

| Category | Cases |
|---|---:|
| Dish properties | 6 |
| Dish composition | 6 |
| Personalized compatibility | 8 |
| Substitution | 6 |
| Missing evidence / abstention | 2 |
| Stale-memory conflict | 2 |

The gold evidence is produced by fixed, parameterized, read-only Cypher in
`evaluation/grounding.py`; it is not generated or judged by an LLM. The
validator also checks task/action alignment, parameters against reference
arguments, substitution-constraint consistency, answer-target coverage, and
the full database fingerprint. Re-run the audit with:

```bash
python3 -m evaluation.grounding \
  --dataset evaluation/datasets/grounding_v1.json \
  --neo4j-uri bolt://localhost:19687
```
