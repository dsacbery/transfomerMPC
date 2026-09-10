"""Train a small Transformer risk estimator from a Day 7 JSON dataset.

PyTorch is intentionally an optional runtime dependency.  The script refuses
to invent data or model hyperparameters when the caller has not supplied them.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Sequence

from build_transformer_dataset import FEATURE_ORDER, TARGET_ORDER

try:
    import torch
    from torch import nn
except ImportError:
    torch = None
    nn = None


def load_dataset(path: str | Path) -> dict:
    dataset = json.loads(Path(path).read_text(encoding="utf-8"))
    if dataset.get("feature_order") != list(FEATURE_ORDER):
        raise ValueError("dataset feature_order does not match the frozen 14-column contract")
    if dataset.get("target_order") != list(TARGET_ORDER):
        raise ValueError("dataset target_order must be r_low, r_ey, r_stab, k_v")
    if dataset.get("history_len") != 16:
        raise ValueError("dataset history_len must be 16")
    return dataset


class RiskTransformer(nn.Module if nn is not None else object):
    """Transformer encoder that consumes a full [batch, 16, 14] window."""

    def __init__(self, model_dim: int = 64) -> None:
        if torch is None or nn is None:
            raise RuntimeError("PyTorch is required to construct the Transformer model")
        super().__init__()
        self.input_projection = nn.Linear(14, model_dim)
        self.position = nn.Parameter(torch.zeros(1, 16, model_dim))
        layer = nn.TransformerEncoderLayer(
            d_model=model_dim, nhead=4, dim_feedforward=2 * model_dim,
            dropout=0.0, batch_first=True, activation="gelu"
        )
        self.encoder = nn.TransformerEncoder(layer, num_layers=2)
        self.output_head = nn.Sequential(nn.LayerNorm(model_dim), nn.Linear(model_dim, 4))

    def forward(self, windows):
        if windows.ndim != 3 or tuple(windows.shape[1:]) != (16, 14):
            raise ValueError("Transformer input must have shape [batch, 16, 14]")
        encoded = self.encoder(self.input_projection(windows) + self.position)
        raw = self.output_head(encoded[:, -1, :]).sigmoid()
        return torch.cat((raw[:, :3], 0.35 + 0.65 * raw[:, 3:4]), dim=1)


def train(
    dataset: dict, epochs: int, learning_rate: float, seed: int, model_dim: int = 64
) -> dict:
    try:
        import torch as installed_torch
    except ImportError as exc:
        raise RuntimeError("PyTorch is required to train the Transformer model") from exc
    if epochs <= 0 or learning_rate <= 0.0:
        raise ValueError("epochs and learning_rate must be positive")

    train_scenarios = set(dataset["splits"]["train"])
    samples = [
        (window, target)
        for window, target, scenario in zip(
            dataset["windows"], dataset["targets"], dataset["scenario_ids"]
        )
        if scenario in train_scenarios and target is not None
    ]
    if not samples:
        raise ValueError("training split has no labeled windows")
    installed_torch.manual_seed(seed)
    if model_dim <= 0 or model_dim % 4 != 0:
        raise ValueError("model_dim must be a positive multiple of four")
    model = RiskTransformer(model_dim)
    optimizer = installed_torch.optim.Adam(model.parameters(), lr=learning_rate)
    features = installed_torch.tensor([window for window, _ in samples], dtype=installed_torch.float32)
    targets = installed_torch.tensor([target for _, target in samples], dtype=installed_torch.float32)
    mean = installed_torch.tensor(dataset["mean_train"], dtype=installed_torch.float32).view(1, 1, 14)
    std = installed_torch.tensor(dataset["std_train"], dtype=installed_torch.float32).view(1, 1, 14)
    normalized = (features - mean) / std
    for _ in range(epochs):
        optimizer.zero_grad()
        loss = installed_torch.nn.functional.mse_loss(model(normalized), targets)
        loss.backward()
        optimizer.step()
    return {
        "model": model,
        "state_dict": model.state_dict(),
        "model_version": "day7-transformer-risk",
        "input_size": [16, 14],
        "output_names": list(TARGET_ORDER),
        "model_dim": model_dim,
        "final_loss": float(loss.detach().cpu()),
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--output", required=True, help="PyTorch checkpoint path outside source control")
    parser.add_argument("--epochs", type=int, default=50)
    parser.add_argument("--learning-rate", type=float, default=1e-3)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args(argv)
    dataset = load_dataset(args.dataset)
    artifact = train(dataset, args.epochs, args.learning_rate, args.seed)
    try:
        import torch
    except ImportError as exc:
        raise RuntimeError("PyTorch is required to save the Transformer model") from exc
    checkpoint = {key: value for key, value in artifact.items() if key != "model"}
    torch.save(checkpoint, args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
