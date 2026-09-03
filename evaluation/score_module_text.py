"""Add the submitted BERTScore metric to saved module responses offline."""

from __future__ import annotations

import argparse
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from evaluation.module_metrics import write_module_summary


PROFILES = {
    # The old test requested lang="it". Pin its historical multilingual
    # checkpoint explicitly because newer bert-score releases dropped the
    # automatic Italian mapping.
    "inner": {
        "field": "inner_speech",
        "model_type": "bert-base-multilingual-cased",
        "num_layers": 9,
        "lang": "it",
    },
    # The submitted Explainability test requested lang="en" even though both
    # strings are Italian; this resolves to roberta-large in bert-score.
    "outer": {
        "field": "explanation",
        "model_type": "roberta-large",
        "num_layers": 17,
        "lang": "en",
    },
    "scope": {
        "field": "reason",
        "model_type": "bert-base-multilingual-cased",
        "num_layers": 9,
        "lang": "it",
    },
}


def _records(path: Path) -> list[dict[str, Any]]:
    return [
        json.loads(line)
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument(
        "--device",
        default="auto",
        help="PyTorch device such as cuda, cuda:1, or cpu (default: auto)",
    )
    args = parser.parse_args()

    source_raw = args.source / "raw.jsonl" if args.source.is_dir() else args.source
    if not source_raw.exists():
        raise SystemExit(f"source does not exist: {source_raw}")
    records = _records(source_raw)
    modules = {str(record.get("module")) for record in records}
    if len(modules) != 1 or next(iter(modules)) not in PROFILES:
        raise SystemExit("source must contain one of: inner, outer, scope")
    module = next(iter(modules))
    profile = PROFILES[module]
    field = str(profile["field"])

    try:
        from bert_score import score
        import torch
    except ImportError as exc:
        raise SystemExit("install the optional BERTScore dependencies first") from exc

    device = args.device
    if device == "auto":
        device = "cuda" if torch.cuda.is_available() else "cpu"

    predictions = [str(record["prediction"].get(field, "")) for record in records]
    references = [str(record["expected"].get(field, "")) for record in records]
    (_, _, f1), metric_hash = score(
        predictions,
        references,
        model_type=str(profile["model_type"]),
        num_layers=int(profile["num_layers"]),
        lang=str(profile["lang"]),
        batch_size=args.batch_size,
        device=device,
        return_hash=True,
        verbose=True,
    )
    for record, value in zip(records, f1.tolist()):
        record["scores"]["bert_f1"] = float(value)

    source_dir = source_raw.parent
    output_dir = args.output_dir or source_dir.with_name(source_dir.name + "_bertscore")
    if output_dir.resolve() == source_dir.resolve():
        raise SystemExit("BERTScore output must differ from its source directory")
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "raw.jsonl").write_text(
        "".join(
            json.dumps(record, ensure_ascii=False, sort_keys=True) + "\n"
            for record in records
        ),
        encoding="utf-8",
    )
    rows = write_module_summary(records, output_dir)

    source_metadata = source_dir / "metadata.json"
    metadata = (
        json.loads(source_metadata.read_text(encoding="utf-8"))
        if source_metadata.exists()
        else {}
    )
    metadata.update(
        {
            "semantic_metric": "BERTScore F1",
            "semantic_metric_hash": metric_hash,
            "semantic_metric_profile": profile,
            "semantic_batch_size": args.batch_size,
            "semantic_device": device,
            "torch_version": torch.__version__,
            "torch_cuda_version": torch.version.cuda,
            "gpu_name": (
                torch.cuda.get_device_name(torch.device(device))
                if device.startswith("cuda")
                else None
            ),
            "semantic_scored_at_utc": datetime.now(timezone.utc).strftime(
                "%Y%m%dT%H%M%SZ"
            ),
            "semantic_source": str(source_dir),
            "semantic_source_raw_sha256": hashlib.sha256(
                source_raw.read_bytes()
            ).hexdigest(),
        }
    )
    (output_dir / "metadata.json").write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(rows, ensure_ascii=False, indent=2, sort_keys=True))
    print(f"BERTScore results: {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
