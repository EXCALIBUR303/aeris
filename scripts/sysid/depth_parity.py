#!/usr/bin/env python3
"""Depth-statistics parity (spec §51 Phase 13 validation gate item 3):
median absolute depth error <= 5% on matched frames.

For every stationary Tier H frame recorded by ``record_tier_h.py --session
depth``, FastSim renders the same full-resolution image from the same GT
camera pose (GT body pose x the committed camera extrinsics), in the same
``WorldSpec``, through the same intrinsics. Pixels valid in *both* images
are compared; the fraction of pixels where only one side has a return is
reported separately (a disagreement about *whether* something is there is
a different failure mode from a disagreement about *how far*).

Usage::

    uv run python scripts/sysid/depth_parity.py [--resolution 0.1]
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

from aeris.simulation.fastsim.batch import GZ_MODEL_TO_BASE_LINK_M, CameraRig, _quat_matrix
from aeris.simulation.fastsim.sensors import cast_rays_batch
from aeris.simulation.fastsim.world import WorldBank, build_world
from aeris.simulation.worlds.batch import load_world_spec

_REPO_ROOT = Path(__file__).resolve().parents[2]
_DATA = _REPO_ROOT / "results" / "sysid" / "depth"
_OUT = _REPO_ROOT / "results" / "sysid" / "parity_depth.json"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--resolution", type=float, default=0.1)
    parser.add_argument("--max-range", type=float, default=20.0)
    parser.add_argument("--out", type=Path, default=_OUT)
    args = parser.parse_args()

    meta = json.loads((_DATA / "meta.json").read_text())
    frames = np.load(_DATA / "frames.npz")
    spec = load_world_spec(
        _REPO_ROOT / "results" / "worlds" / "f2_office" / "val" / f"{meta['world']}.json"
    )
    bank = WorldBank.from_worlds([build_world(spec, resolution_m=args.resolution)])
    rig = CameraRig.load(stride=1)

    per_frame = []
    all_rel = []
    for i, fm in enumerate(meta["frames"]):
        tier_h = frames[f"arr_{i}"].astype(np.float64)
        r_wb = _quat_matrix(*fm["gt_quat_wxyz"])
        # GT is the gz *model* pose; extrinsics are relative to base_link.
        base_link = np.array(fm["gt_pos_world"]) + r_wb @ GZ_MODEL_TO_BASE_LINK_M
        cam_pos = base_link + r_wb @ rig.t_body_optical
        r_wc = r_wb @ rig.r_body_optical
        ranges = cast_rays_batch(
            bank.occ,
            bank.origins,
            bank.dims,
            bank.ground_z,
            bank.resolution_m,
            np.array([0], dtype=np.int64),
            cam_pos[None, :],
            r_wc[None, :, :],
            rig.dirs_optical,
            args.max_range,
        )[0]
        fast = (ranges * rig.z_factor).reshape(rig.height, rig.width)
        h_ok, f_ok = np.isfinite(tier_h) & (tier_h > 0), np.isfinite(fast)
        both = h_ok & f_ok
        rel = np.abs(fast[both] - tier_h[both]) / tier_h[both]
        all_rel.append(rel)
        per_frame.append(
            {
                "frame": i,
                "n_both_valid": int(both.sum()),
                "median_abs_rel_error": round(float(np.median(rel)), 4),
                "p90_abs_rel_error": round(float(np.percentile(rel, 90)), 4),
                "return_disagreement_frac": round(float((h_ok ^ f_ok).mean()), 4),
                "tier_h_median_depth_m": round(float(np.median(tier_h[h_ok])), 3),
            }
        )
        print(per_frame[-1], flush=True)

    pooled = np.concatenate(all_rel)
    pooled_median = float(np.median(pooled))
    worst = max(p["median_abs_rel_error"] for p in per_frame)
    summary = {
        "gate": "median absolute depth error <= 5% on matched frames",
        # Two readings, both reported. "pooled" is the spec's literal wording
        # (the median over the matched frames' pixels). "per_frame_worst" is a
        # stricter reading this script applied from its first run, before any
        # data was seen: every individual frame's median <= 5%.
        "world": meta["world"],
        "voxel_resolution_m": args.resolution,
        "n_frames": len(per_frame),
        "pooled_median_abs_rel_error": round(pooled_median, 4),
        "per_frame_worst_median_abs_rel_error": worst,
        "passed_pooled_literal": bool(pooled_median <= 0.05),
        "passed_per_frame_strict": bool(worst <= 0.05),
        "frames": per_frame,
    }
    args.out.write_text(json.dumps(summary, indent=2))
    print(
        f"pooled median {pooled_median:.4f} ({'PASS' if pooled_median <= 0.05 else 'FAIL'}, spec literal); "
        f"per-frame worst {worst:.4f} ({'PASS' if worst <= 0.05 else 'FAIL'}, strict)"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
