from __future__ import annotations

import random
import statistics

import pytest

from aeris.evaluation.statistics import bootstrap_ci, holm_correction, paired_bootstrap_ci


def test_bootstrap_ci_requires_at_least_two_samples() -> None:
    with pytest.raises(ValueError):
        bootstrap_ci([1.0])


def test_bootstrap_ci_is_deterministic_given_a_seed() -> None:
    samples = [1.0, 2.0, 3.0, 4.0, 5.0]
    ci_a = bootstrap_ci(samples, n_resamples=500, seed=42)
    ci_b = bootstrap_ci(samples, n_resamples=500, seed=42)
    assert ci_a == ci_b


def test_bootstrap_ci_contains_the_sample_mean_for_low_variance_data() -> None:
    samples = [10.0, 10.1, 9.9, 10.05, 9.95, 10.0, 10.0]
    lo, hi = bootstrap_ci(samples, n_resamples=2000, seed=7)
    assert lo <= statistics.mean(samples) <= hi


def test_bootstrap_ci_coverage_sanity_on_synthetic_normal_data() -> None:
    """spec's own requirement: "bootstrap coverage sanity (~=95% on
    synthetic data)". Draws many independent samples from a KNOWN normal
    distribution, builds a 95% bootstrap CI for each, and checks that the
    true mean falls inside the CI close to 95% of the time."""
    true_mean = 5.0
    rng = random.Random(1234)
    n_trials = 200
    n_per_sample = 30
    hits = 0
    for trial in range(n_trials):
        sample = [rng.gauss(true_mean, 1.0) for _ in range(n_per_sample)]
        lo, hi = bootstrap_ci(sample, n_resamples=1000, alpha=0.05, seed=trial)
        if lo <= true_mean <= hi:
            hits += 1
    coverage = hits / n_trials
    # A generous band around 95% -- this is a sanity check on the
    # *mechanism*, not a precise coverage-rate certification.
    assert 0.85 <= coverage <= 1.0, f"observed coverage {coverage:.2f}, expected ~0.95"


def test_paired_bootstrap_ci_rejects_mismatched_lengths() -> None:
    with pytest.raises(ValueError):
        paired_bootstrap_ci([1.0, 2.0], [1.0])


def test_paired_bootstrap_ci_zero_when_identical_samples() -> None:
    samples = [1.0, 2.0, 3.0, 4.0, 5.0]
    lo, hi = paired_bootstrap_ci(samples, samples, n_resamples=500, seed=1)
    assert lo == pytest.approx(0.0, abs=1e-9)
    assert hi == pytest.approx(0.0, abs=1e-9)


def test_paired_bootstrap_ci_detects_a_clear_shift() -> None:
    a = [10.0, 10.1, 9.9, 10.05, 9.95, 10.0, 10.0, 10.02]
    b = [5.0, 5.1, 4.9, 5.05, 4.95, 5.0, 5.0, 5.02]
    lo, _hi = paired_bootstrap_ci(a, b, n_resamples=2000, seed=3)
    assert lo > 0  # a - b is clearly positive, CI should exclude 0


def test_holm_correction_empty_input() -> None:
    assert holm_correction([]) == []


def test_holm_correction_hand_computed_all_significant() -> None:
    # 3 tests, alpha=0.05: thresholds are 0.05/3, 0.05/2, 0.05/1 in rank
    # order. p-values all well below their thresholds -> all rejected.
    p_values = [0.001, 0.002, 0.003]
    assert holm_correction(p_values, alpha=0.05) == [True, True, True]


def test_holm_correction_hand_computed_step_down_stops_early() -> None:
    # Ranked: 0.01 (thresh 0.05/3=0.0167, rejected), 0.02 (thresh
    # 0.05/2=0.025, rejected), 0.5 (thresh 0.05/1=0.05, NOT rejected).
    p_values = [0.5, 0.01, 0.02]
    result = holm_correction(p_values, alpha=0.05)
    assert result == [False, True, True]


def test_holm_correction_is_never_more_lenient_than_uncorrected() -> None:
    # Every Holm-rejected p-value must also be <= alpha uncorrected.
    p_values = [0.001, 0.03, 0.049, 0.2]
    rejected = holm_correction(p_values, alpha=0.05)
    for p, r in zip(p_values, rejected, strict=True):
        if r:
            assert p <= 0.05
