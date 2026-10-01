"""Per-modality observation encoders (spec §27.2).

One encoder per observation key, chosen by the key's shape:

- ``(C, H, W)`` images (forward depth, egocentric map crops) -> a 3-layer
  CNN (32-64-64 channels, 3x3 kernels, stride 2, ReLU) -> Linear -> 256.
  Images arrive already scaled to fixed physical ranges by the builders in
  :mod:`aeris.learning.spaces` (depth / max_range), never by running stats.
- ``(D,)`` low-dimensional vectors -> a one-layer MLP -> 64 (tanh).

The per-key features are concatenated (keys in sorted order, so the layout
never depends on dict insertion order) and passed through the trunk MLP
(default one layer of 256, tanh). Initialization is orthogonal with gain
sqrt(2) and zero bias (Huang et al. 2022, "The 37 Implementation Details of
PPO", detail #2); heads override their own output gains.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence

import torch
from torch import nn

HIDDEN_GAIN = math.sqrt(2.0)


def orthogonal_(layer: nn.Module, gain: float = HIDDEN_GAIN) -> nn.Module:
    if isinstance(layer, (nn.Linear, nn.Conv2d)):
        nn.init.orthogonal_(layer.weight, gain)
        if layer.bias is not None:
            nn.init.zeros_(layer.bias)
    return layer


class ImageCNN(nn.Module):
    def __init__(
        self, shape: tuple[int, int, int], out_dim: int, channels: Sequence[int] = (32, 64, 64)
    ) -> None:
        super().__init__()
        layers: list[nn.Module] = []
        c_in = shape[0]
        for c in channels:
            layers += [
                orthogonal_(nn.Conv2d(c_in, c, kernel_size=3, stride=2, padding=1)),
                nn.ReLU(),
            ]
            c_in = c
        self.conv = nn.Sequential(*layers, nn.Flatten())
        with torch.no_grad():
            n_flat = int(self.conv(torch.zeros(1, *shape)).shape[1])
        self.fc = nn.Sequential(orthogonal_(nn.Linear(n_flat, out_dim)), nn.ReLU())
        self.out_dim = out_dim

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        out: torch.Tensor = self.fc(self.conv(x))
        return out


class VectorMLP(nn.Module):
    def __init__(self, dim: int, out_dim: int) -> None:
        super().__init__()
        self.net = nn.Sequential(orthogonal_(nn.Linear(dim, out_dim)), nn.Tanh())
        self.out_dim = out_dim

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        out: torch.Tensor = self.net(x)
        return out


class MultiModalEncoder(nn.Module):
    """Encodes a dict observation ``{key: [B, *shape]}`` to ``[B, trunk_dim]``."""

    def __init__(
        self,
        obs_shapes: Mapping[str, tuple[int, ...]],
        *,
        image_dim: int = 256,
        vector_dim: int = 64,
        trunk_sizes: Sequence[int] = (256,),
    ) -> None:
        super().__init__()
        self.keys = sorted(obs_shapes)
        encoders: dict[str, nn.Module] = {}
        total = 0
        for k in self.keys:
            shape = tuple(obs_shapes[k])
            if len(shape) == 3:
                enc: ImageCNN | VectorMLP = ImageCNN((shape[0], shape[1], shape[2]), image_dim)
            elif len(shape) == 1:
                enc = VectorMLP(shape[0], vector_dim)
            else:
                raise ValueError(f"observation {k!r}: unsupported shape {shape}")
            encoders[k] = enc
            total += enc.out_dim
        self.encoders = nn.ModuleDict(encoders)
        trunk: list[nn.Module] = []
        for size in trunk_sizes:
            trunk += [orthogonal_(nn.Linear(total, size)), nn.Tanh()]
            total = size
        self.trunk = nn.Sequential(*trunk)
        self.out_dim = total

    def forward(self, obs: Mapping[str, torch.Tensor]) -> torch.Tensor:
        feats = [self.encoders[k](obs[k]) for k in self.keys]
        out: torch.Tensor = self.trunk(torch.cat(feats, dim=-1))
        return out
