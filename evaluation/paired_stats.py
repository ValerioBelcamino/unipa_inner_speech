"""Exact paired significance and Wilson intervals for frozen benchmark records."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

from .metrics import score_controller_record, score_readiness_record


def exact_mcnemar(left_only: int, right_only: int) -> float:
    """Two-sided exact McNemar/binomial p-value for discordant pairs."""
    discordant = left_only + right_only
    if discordant == 0:
        return 1.0
    lower_tail = sum(
        math.comb(discordant, index)
        for index in range(min(left_only, right_only) + 1)
    ) / (2**discordant)
    return min(1.0, 2 * lower_tail)


def wilson_interval(successes: int, total: int) -> tuple[float, float]:
    """95% Wilson score interval for one binomial proportion."""
    z = 1.959963984540054
    proportion = successes / total
    denominator = 1 + z * z / total
    centre = (proportion + z * z / (2 * total)) / denominator
    half_width = (
        z
        * math.sqrt(proportion * (1 - proportion) / total + z * z / (4 * total * total))
        / denominator
    )
    return centre - half_width, centre + half_width


def _records(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line]


def compare(
    records: list[dict[str, Any]], suite: str, left: str, right: str, metric: str
) -> dict[str, Any]:
    """Compare two architectures on matched (case, repeat) observations."""
    paired: dict[str, dict[tuple[str, int], bool]] = {left: {}, right: {}}
    for record in records:
        architecture = record["architecture"]
        if architecture not in paired:
            continue
        score = (
            score_controller_record(record)
            if suite == "controller"
            else score_readiness_record(record)
        )
        paired[architecture][(record["case_id"], int(record["repeat"]))] = bool(score[metric])

    left_keys = set(paired[left])
    right_keys = set(paired[right])
    if left_keys != right_keys or not left_keys:
        raise ValueError("architectures must have the same non-empty set of case/repeat pairs")

    def architecture_summary(name: str) -> dict[str, Any]:
        successes = sum(paired[name].values())
        lower, upper = wilson_interval(successes, len(left_keys))
        return {
            "successes": successes,
            "n": len(left_keys),
            "rate": successes / len(left_keys),
            "wilson_95": [lower, upper],
        }

    left_only = sum(paired[left][key] and not paired[right][key] for key in left_keys)
    right_only = sum(not paired[left][key] and paired[right][key] for key in left_keys)
    return {
        "suite": suite,
        "metric": metric,
        left: architecture_summary(left),
        right: architecture_summary(right),
        "discordant": {"left_only": left_only, "right_only": right_only},
        "mcnemar_exact_two_sided_p": exact_mcnemar(left_only, right_only),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw", required=True, type=Path)
    parser.add_argument("--suite", required=True, choices=("controller", "readiness"))
    parser.add_argument("--left", required=True)
    parser.add_argument("--right", required=True)
    parser.add_argument("--metric")
    args = parser.parse_args()
    metric = args.metric or (
        "operational_success" if args.suite == "controller" else "readiness_correct"
    )
    result = compare(_records(args.raw), args.suite, args.left, args.right, metric)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
