from __future__ import annotations

import json
from pathlib import Path

from evaluation.grounding import (
    audit_prompt_overlap,
    canonicalize,
    load_dataset,
    prior_prompt_paths,
    validate_case_alignment,
    validate_dataset,
)


class FrozenReferenceDatabase:
    def __init__(self, dataset):
        self.dataset = dataset
        self.references = {
            json.dumps(case["reference"]["arguments"], sort_keys=True): case[
                "reference"
            ]["expected_evidence"]
            for case in dataset["cases"]
        }

    def fingerprint(self):
        return self.dataset["database"]["fingerprint_sha256"]

    def execute_reference(self, reference):
        key = json.dumps(reference["arguments"], sort_keys=True)
        return canonicalize(self.references[key])


def test_frozen_grounding_dataset_is_internally_aligned():
    dataset = load_dataset()
    report = validate_dataset(dataset, FrozenReferenceDatabase(dataset))
    assert report["valid"] is True
    assert report["cases"] == 30
    assert report["empty_evidence_cases"] == 2
    assert report["category_counts"] == {
        "dish_composition": 6,
        "dish_properties": 6,
        "memory_conflict": 2,
        "missing_evidence": 2,
        "personalized_compatibility": 8,
        "substitution": 6,
    }


def test_grounding_prompts_are_not_exact_copies_of_existing_material():
    dataset = load_dataset()
    report = audit_prompt_overlap(dataset, prior_prompt_paths())
    assert report["valid"] is True
    assert report["normalized_exact_overlaps"] == []


def test_prompt_overlap_audit_detects_normalized_copy(tmp_path: Path):
    dataset = load_dataset()
    source = tmp_path / "prior.json"
    source.write_text(
        json.dumps({"question": dataset["cases"][0]["user_input"].upper()}),
        encoding="utf-8",
    )
    report = audit_prompt_overlap(dataset, [source])
    assert report["valid"] is False
    assert report["normalized_exact_overlaps"][0]["case_id"] == "g-dp-01"


def test_alignment_rejects_wrong_reference_entity():
    case = json.loads(json.dumps(load_dataset()["cases"][0]))
    case["reference"]["arguments"]["dish"] = "lasagna"
    errors = validate_case_alignment(case)
    assert "nome_piatto does not match reference dish" in errors


def test_canonicalization_is_order_invariant():
    left = [{"allergens": ["uova", "glutine"], "dish": "x"}]
    right = [{"dish": "x", "allergens": ["glutine", "uova"]}]
    assert canonicalize(left) == canonicalize(right)
