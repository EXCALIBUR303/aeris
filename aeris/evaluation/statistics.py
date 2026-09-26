"""Bootstrap statistics (spec §51 Phase 7 task 3): paired bootstrap CI +
Holm correction, unit-tested against known distributions (spec's own
requirement: "bootstrap coverage sanity (~95% on synthetic data)")."""

from __future__ import annotations

import random
import statistics
from collections.abc import Callable, Sequence


def bootstrap_ci(
    samples: Sequence[float],
    *,
    statistic: Callable[[Sequence[float]], float] = statistics.mean,
    n_resamples: int = 10_000,
    alpha: float = 0.05,
    seed: int | None = None,
) -> tuple[float, float]:
    """A percentile bootstrap confidence interval for ``statistic(samples)``."""
    if len(samples) < 2:
        raise ValueError("need at least 2 samples for a bootstrap CI")
    rng = random.Random(seed)
    n = len(samples)
    resampled = sorted(
        statistic([samples[rng.randrange(n)] for _ in range(n)]) for _ in range(n_resamples)
    )
    lo_idx = max(0, int((alpha / 2) * n_resamples))
    hi_idx = min(n_resamples - 1, int((1 - alpha / 2) * n_resamples) - 1)
    return resampled[lo_idx], resampled[hi_idx]


def paired_bootstrap_ci(
    samples_a: Sequence[float],
    samples_b: Sequence[float],
    *,
    n_resamples: int = 10_000,
    alpha: float = 0.05,
    seed: int | None = None,
) -> tuple[float, float]:
    """A bootstrap CI on the paired difference ``mean(a) - mean(b)``."""
    if len(samples_a) != len(samples_b):
        raise ValueError("paired samples must be the same length")
    diffs = [a - b for a, b in zip(samples_a, samples_b, strict=True)]
    return bootstrap_ci(diffs, n_resamples=n_resamples, alpha=alpha, seed=seed)


def holm_correction(p_values: Sequence[float], *, alpha: float = 0.05) -> list[bool]:
    """Holm-Bonferroni step-down correction. Returns, per input p-value in
    its *original* order, whether it's rejected (significant) at ``alpha``
    after correction for ``len(p_values)`` simultaneous comparisons."""
    m = len(p_values)
    if m == 0:
        return []
    order = sorted(range(m), key=lambda i: p_values[i])
    reject = [False] * m
    for rank, idx in enumerate(order):
        threshold = alpha / (m - rank)
        if p_values[idx] <= threshold:
            reject[idx] = True
        else:
            break  # step-down: stop at the first non-rejection
    return reject
