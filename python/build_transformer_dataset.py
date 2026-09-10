"""Build scenario-separated Transformer training windows from simulation logs.

The module intentionally uses only the Python standard library so that its
contract can be checked without installing a training framework.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
from pathlib import Path
from typing import Iterable, Mapping, Sequence

FEATURE_ORDER = (
    "vx", "vy", "yaw_rate", "ay", "beta", "delta_meas",
    "delta_rate_meas", "ax_meas", "e_y", "e_psi", "e_y_rate",
    "e_psi_rate", "kappa_ref", "v_ref_base",
)
TARGET_ORDER = ("r_low", "r_ey", "r_stab", "k_v")


def _number(value: object, label: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be numeric") from exc
    if not math.isfinite(number):
        raise ValueError(f"{label} must be finite")
    return number


def build_windows(
    records: Iterable[Mapping[str, object]], history_len: int = 16
) -> dict[str, list]:
    """Create chronological windows grouped by ``scenario_id``.

    Only complete windows are emitted.  Labels are taken from the final row
    when all four target columns are present; otherwise the label is ``None``.
    """
    if not isinstance(history_len, int) or history_len <= 0:
        raise ValueError("history_len must be a positive integer")

    grouped: dict[str, list[tuple[list[float], list[float] | None]]] = {}
    for position, record in enumerate(records):
        if not isinstance(record, Mapping):
            raise ValueError(f"record {position} must be a mapping")
        scenario = str(record.get("scenario_id", "default"))
        features: list[float] = []
        for name in FEATURE_ORDER:
            if name not in record:
                raise ValueError(f"record {position} missing feature {name}")
            features.append(_number(record[name], f"feature {name}"))
        target = None
        if all(name in record for name in TARGET_ORDER):
            target = [_number(record[name], f"target {name}") for name in TARGET_ORDER]
        grouped.setdefault(scenario, []).append((features, target))

    windows: list[list[list[float]]] = []
    targets: list[list[float] | None] = []
    scenario_ids: list[str] = []
    for scenario in sorted(grouped):
        rows = grouped[scenario]
        for end in range(history_len, len(rows) + 1):
            start = end - history_len
            windows.append([row[0] for row in rows[start:end]])
            targets.append(rows[end - 1][1])
            scenario_ids.append(scenario)
    return {"windows": windows, "targets": targets, "scenario_ids": scenario_ids}


def _split_scenarios(scenarios: Sequence[str], seed: int) -> dict[str, list[str]]:
    unique = sorted(set(scenarios))
    if not unique:
        return {"train": [], "val": [], "test": []}
    shuffled = list(unique)
    random.Random(seed).shuffle(shuffled)
    if len(shuffled) == 1:
        return {"train": shuffled, "val": [], "test": []}
    n_train = max(1, int(round(0.6 * len(shuffled))))
    n_val = max(0, int(round(0.2 * len(shuffled))))
    if n_train + n_val >= len(shuffled):
        n_val = max(0, len(shuffled) - n_train - 1)
    return {
        "train": sorted(shuffled[:n_train]),
        "val": sorted(shuffled[n_train:n_train + n_val]),
        "test": sorted(shuffled[n_train + n_val:]),
    }


def _training_stats(windows: Sequence[list[list[float]]], scenarios: Sequence[str], train_scenarios: set[str]) -> tuple[list[float], list[float]]:
    rows = [row for window, scenario in zip(windows, scenarios) if scenario in train_scenarios for row in window]
    if not rows:
        rows = [row for window in windows for row in window]
    if not rows:
        return [0.0] * len(FEATURE_ORDER), [1.0] * len(FEATURE_ORDER)
    mean = [sum(row[index] for row in rows) / len(rows) for index in range(len(FEATURE_ORDER))]
    std = []
    for index, average in enumerate(mean):
        variance = sum((row[index] - average) ** 2 for row in rows) / len(rows)
        value = math.sqrt(variance)
        std.append(value if value > 0.0 and math.isfinite(value) else 1.0)
    return mean, std


def build_dataset(records: Iterable[Mapping[str, object]], history_len: int = 16, seed: int = 0) -> dict[str, object]:
    dataset = build_windows(records, history_len=history_len)
    splits = _split_scenarios(dataset["scenario_ids"], seed)
    mean_train, std_train = _training_stats(
        dataset["windows"], dataset["scenario_ids"], set(splits["train"])
    )
    dataset.update({
        "feature_order": list(FEATURE_ORDER),
        "target_order": list(TARGET_ORDER),
        "history_len": history_len,
        "splits": splits,
        "mean_train": mean_train,
        "std_train": std_train,
    })
    return dataset


def load_records(paths: Sequence[str | Path]) -> list[dict[str, object]]:
    records: list[dict[str, object]] = []
    for path_value in paths:
        path = Path(path_value)
        if path.suffix.lower() == ".csv":
            with path.open("r", newline="", encoding="utf-8") as handle:
                records.extend(dict(row) for row in csv.DictReader(handle))
        elif path.suffix.lower() == ".json":
            payload = json.loads(path.read_text(encoding="utf-8"))
            records.extend(payload if isinstance(payload, list) else payload["records"])
        else:
            raise ValueError(f"unsupported input format: {path.suffix}")
    return records


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", nargs="+", required=True, help="CSV or JSON simulation logs")
    parser.add_argument("--output", required=True, help="JSON dataset output path")
    parser.add_argument("--history-len", type=int, default=16)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args(argv)
    dataset = build_dataset(load_records(args.input), args.history_len, args.seed)
    Path(args.output).write_text(json.dumps(dataset, indent=2), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
