"""Export Day 7 Transformer artifacts for the Day 9 online inference stage."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Sequence

from build_transformer_dataset import FEATURE_ORDER, TARGET_ORDER


def export_artifacts(
    dataset_path: str | Path, checkpoint_path: str | Path, output_dir: str | Path
) -> dict[str, Path]:
    try:
        import numpy as np
        import torch
        from scipy.io import savemat
    except ImportError as exc:
        raise RuntimeError("NumPy, SciPy, and PyTorch are required to export .mat artifacts") from exc

    dataset = json.loads(Path(dataset_path).read_text(encoding="utf-8"))
    if dataset.get("feature_order") != list(FEATURE_ORDER) or dataset.get("history_len") != 16:
        raise ValueError("dataset does not satisfy the frozen [16,14] input contract")
    if dataset.get("target_order") != list(TARGET_ORDER):
        raise ValueError("dataset target order is incompatible with online risk inference")
    mean = dataset.get("mean_train")
    std = dataset.get("std_train")
    if not isinstance(mean, list) or not isinstance(std, list) or len(mean) != 14 or len(std) != 14:
        raise ValueError("dataset mean_train and std_train must each contain 14 values")
    checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=False)
    required = {"state_dict", "model_version", "input_size", "output_names", "model_dim"}
    if not isinstance(checkpoint, dict) or not required.issubset(checkpoint):
        raise ValueError("checkpoint does not contain the required Transformer metadata")
    if checkpoint["input_size"] != [16, 14] or checkpoint["output_names"] != list(TARGET_ORDER):
        raise ValueError("checkpoint is incompatible with the frozen Transformer contract")

    directory = Path(output_dir)
    directory.mkdir(parents=True, exist_ok=True)
    weights_path = directory / "transformer_weights.mat"
    stats_path = directory / "feature_stats.mat"
    manifest_path = directory / "transformer_manifest.mat"
    weights = {
        name.replace(".", "_"): tensor.detach().cpu().numpy()
        for name, tensor in checkpoint["state_dict"].items()
    }
    savemat(weights_path, {"state_dict": weights, "model_dim": checkpoint["model_dim"]},
            long_field_names=True)
    savemat(stats_path, {
        "mean_train": np.asarray(mean, dtype=float).reshape(1, 14),
        "std_train": np.asarray(std, dtype=float).reshape(1, 14),
        "feature_order": np.asarray(list(FEATURE_ORDER), dtype=object).reshape(1, 14),
    })
    savemat(manifest_path, {
        "model_version": checkpoint["model_version"],
        "input_size": np.asarray([16, 14], dtype=int).reshape(1, 2),
        "output_names": np.asarray(list(TARGET_ORDER), dtype=object).reshape(1, 4),
        "output_bounds": np.asarray([[0, 1], [0, 1], [0, 1], [0.35, 1]], dtype=float),
        "weights_file": "transformer_weights.mat",
        "model_dim": checkpoint["model_dim"],
    })
    return {"weights": weights_path, "feature_stats": stats_path, "manifest": manifest_path}


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args(argv)
    export_artifacts(args.dataset, args.checkpoint, args.output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
