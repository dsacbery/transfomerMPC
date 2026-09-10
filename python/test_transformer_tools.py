import math
import json

import pytest

from build_transformer_dataset import FEATURE_ORDER, build_dataset, build_windows
from export_transformer_artifacts import export_artifacts
from train_transformer import RiskTransformer, train


def row(scenario, index):
    values = {name: float(index + offset) for offset, name in enumerate(FEATURE_ORDER)}
    values["scenario_id"] = scenario
    values["r_low"] = 0.1
    values["r_ey"] = 0.2
    values["r_stab"] = 0.3
    values["k_v"] = 0.9
    return values


def test_build_windows_preserves_frozen_order_and_labels():
    records = [row("A", index) for index in range(16)]

    dataset = build_windows(records, history_len=16)

    assert FEATURE_ORDER == (
        "vx", "vy", "yaw_rate", "ay", "beta", "delta_meas",
        "delta_rate_meas", "ax_meas", "e_y", "e_psi", "e_y_rate",
        "e_psi_rate", "kappa_ref", "v_ref_base",
    )
    assert dataset["windows"] == [[[float(index + offset)
        for offset in range(14)] for index in range(16)]]
    assert dataset["targets"] == [[0.1, 0.2, 0.3, 0.9]]
    assert dataset["scenario_ids"] == ["A"]


def test_build_dataset_splits_by_scenario_without_window_leakage():
    records = [row("A", index) for index in range(20)]
    records += [row("B", index) for index in range(20)]
    records += [row("C", index) for index in range(20)]

    dataset = build_dataset(records, history_len=16, seed=7)

    split_sets = [set(dataset["splits"][name]) for name in ("train", "val", "test")]
    assert not (split_sets[0] & split_sets[1])
    assert not (split_sets[0] & split_sets[2])
    assert not (split_sets[1] & split_sets[2])
    assert set().union(*split_sets) == {"A", "B", "C"}
    assert len(dataset["mean_train"]) == 14
    assert len(dataset["std_train"]) == 14
    assert all(math.isfinite(value) and value > 0.0 for value in dataset["std_train"])


def test_build_windows_rejects_nonfinite_or_missing_features():
    invalid = row("A", 0)
    invalid["ay"] = float("nan")
    with pytest.raises(ValueError, match="finite"):
        build_windows([invalid], history_len=1)

    missing = row("A", 0)
    del missing["beta"]
    with pytest.raises(ValueError, match="feature"):
        build_windows([missing], history_len=1)


def test_training_uses_full_window_transformer_and_exports_mat_artifacts(tmp_path):
    torch = pytest.importorskip("torch")
    records = [row("A", index) for index in range(16)]
    dataset = build_dataset(records, history_len=16, seed=3)

    artifact = train(dataset, epochs=1, learning_rate=1e-3, seed=3, model_dim=16)
    assert isinstance(artifact["model"], RiskTransformer)
    assert artifact["input_size"] == [16, 14]
    assert artifact["output_names"] == ["r_low", "r_ey", "r_stab", "k_v"]

    dataset_path = tmp_path / "dataset.json"
    checkpoint_path = tmp_path / "checkpoint.pt"
    dataset_path.write_text(json.dumps(dataset), encoding="utf-8")
    torch.save(artifact, checkpoint_path)
    paths = export_artifacts(dataset_path, checkpoint_path, tmp_path / "artifacts")

    assert {path.name for path in paths.values()} == {
        "transformer_weights.mat", "feature_stats.mat", "transformer_manifest.mat"
    }
    scipy_io = pytest.importorskip("scipy.io")
    stats = scipy_io.loadmat(paths["feature_stats"])
    manifest = scipy_io.loadmat(paths["manifest"])
    assert stats["mean_train"].shape == (1, 14)
    assert stats["std_train"].shape == (1, 14)
    assert manifest["input_size"].tolist() == [[16, 14]]
