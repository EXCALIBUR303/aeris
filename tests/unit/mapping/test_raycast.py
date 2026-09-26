"""Unit tests for :mod:`aeris.mapping.raycast` -- the Numba ray-integration core."""

from __future__ import annotations

import numpy as np
import pytest

from aeris.mapping.raycast import decode_key, encode_key, integrate_frame

_RES = 0.2
_L_OCC = 0.85
_L_FREE = -0.4
_MAX_RANGE = 15.0


def test_encode_decode_roundtrip_positive_and_negative() -> None:
    for ix, iy, iz in [(0, 0, 0), (5, -3, 12), (-100, 200, -7), (-1, -1, -1)]:
        assert decode_key(encode_key(ix, iy, iz)) == (ix, iy, iz)


def test_single_ray_marks_free_along_path_and_occupied_at_hit() -> None:
    endpoints = np.array([[2.0, 0.0, 0.0]])
    is_hit = np.array([True])
    keys, deltas = integrate_frame(
        (0.0, 0.0, 0.0),
        endpoints,
        is_hit,
        resolution_m=_RES,
        max_range_m=_MAX_RANGE,
        l_occ=_L_OCC,
        l_free=_L_FREE,
    )
    by_key = dict(zip(keys.tolist(), deltas.tolist(), strict=True))

    hit_voxel = (round(2.0 / _RES), 0, 0)  # endpoint (2.0, 0, 0) at 0.2m res -> index 10
    hit_key = encode_key(*hit_voxel)
    assert by_key[hit_key] == pytest.approx(_L_OCC)

    near_origin_voxel = (0, 0, 0)
    assert by_key[encode_key(*near_origin_voxel)] == pytest.approx(_L_FREE)

    # Every touched voxel except the last must be a free delta.
    assert all(d == pytest.approx(_L_FREE) for k, d in by_key.items() if k != hit_key)


def test_no_return_ray_marks_free_all_the_way_never_occupied() -> None:
    endpoints = np.array([[5.0, 0.0, 0.0]])  # caller already placed this at some finite range
    is_hit = np.array([False])
    keys, deltas = integrate_frame(
        (0.0, 0.0, 0.0),
        endpoints,
        is_hit,
        resolution_m=_RES,
        max_range_m=_MAX_RANGE,
        l_occ=_L_OCC,
        l_free=_L_FREE,
    )
    assert len(keys) > 0
    assert all(d == pytest.approx(_L_FREE) for d in deltas.tolist())


def test_ray_beyond_max_range_is_clipped_and_never_marked_occupied() -> None:
    endpoints = np.array([[100.0, 0.0, 0.0]])
    is_hit = np.array([True])  # even a claimed hit beyond max_range must not become "occupied"
    keys, deltas = integrate_frame(
        (0.0, 0.0, 0.0),
        endpoints,
        is_hit,
        resolution_m=_RES,
        max_range_m=_MAX_RANGE,
        l_occ=_L_OCC,
        l_free=_L_FREE,
    )
    assert all(d == pytest.approx(_L_FREE) for d in deltas.tolist())
    # No voxel beyond max_range_m along the ray was touched at all.
    max_ix = int(_MAX_RANGE / _RES) + 1
    assert all(decode_key(int(k))[0] <= max_ix for k in keys.tolist())


def test_overlapping_rays_accumulate_net_delta_at_shared_voxels() -> None:
    """Spec's "corner" case: two rays sharing an early voxel near the
    origin but hitting different endpoints -- the shared voxel's delta is
    the *sum* of both rays' free contributions there, not just one."""
    endpoints = np.array([[3.0, 0.0, 0.0], [0.0, 3.0, 0.0]])
    is_hit = np.array([True, True])
    keys, deltas = integrate_frame(
        (0.0, 0.0, 0.0),
        endpoints,
        is_hit,
        resolution_m=_RES,
        max_range_m=_MAX_RANGE,
        l_occ=_L_OCC,
        l_free=_L_FREE,
    )
    by_key = dict(zip(keys.tolist(), deltas.tolist(), strict=True))
    origin_key = encode_key(0, 0, 0)
    assert by_key[origin_key] == pytest.approx(2 * _L_FREE)


def test_zero_length_ray_is_skipped() -> None:
    endpoints = np.array([[0.0, 0.0, 0.0]])
    is_hit = np.array([True])
    keys, _deltas = integrate_frame(
        (0.0, 0.0, 0.0),
        endpoints,
        is_hit,
        resolution_m=_RES,
        max_range_m=_MAX_RANGE,
        l_occ=_L_OCC,
        l_free=_L_FREE,
    )
    assert len(keys) == 0
