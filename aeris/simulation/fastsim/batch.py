"""The batched FastSim core (spec §17.3): N vehicles, each in its own world
from a :class:`~aeris.simulation.fastsim.world.WorldBank`, with identified
dynamics, a raycast depth camera, the S3 shield in the loop, an EKF-like
pose-error model, and collision checking. Task envs
(:mod:`aeris.learning.envs`) are thin layers on top of this.

**Truth vs estimate.** Sensors render from the vehicle's *true* pose (a
camera is bolted to the real airframe); the agent's pose *estimate* is
true pose + a random-walk drift + white noise, with both magnitudes
fitted to Tier H GT-vs-EKF data (``configs/fastsim/pose_noise.yaml``).
The estimate is what the observation builders see; the truth is only for
physics, sensing, collision, and (reward-only) geodesic progress.

**Control step (10 Hz).** Order matches Tier H's own loop: the depth image
rendered at the start of the step both feeds the observation *and* is the
point cloud the shield checks the command against; then the shielded
command is integrated for ``control_dt / dt_sim`` dynamics substeps; then
collision is checked at the new pose.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml
from numba import njit

from aeris.core.frames.conventions import camera_optical_to_body_rotation
from aeris.perception.depth.projection import CameraIntrinsics
from aeris.safety.shield import N_SECTORS, ShieldConfig
from aeris.simulation.fastsim.dynamics import DT_SIM_S, BatchDynamics, DynamicsParams
from aeris.simulation.fastsim.sensors import camera_ray_dirs, cast_rays_batch
from aeris.simulation.fastsim.shield import fov_mask, shield_batch
from aeris.simulation.fastsim.world import WorldBank

_REPO_ROOT = Path(__file__).resolve().parents[3]
SENSORS_YAML = _REPO_ROOT / "configs" / "vehicle" / "sensors.yaml"
VEHICLE_RADIUS_M = 0.3  # == Phase 10's experiment r_v and Phase 12's A* inflation
# GroundTruthService reports the Gazebo *model* pose; the x500 family places
# base_link 0.24 m above the model origin (x500_base/model.sdf: model-level
# <pose>0 0 .24 0 0 0</pose>), while every sensor extrinsic in
# configs/vehicle/sensors.yaml is relative to base_link. Found live in
# Phase 13's depth parity (ground returns 30% too close until corrected).
GZ_MODEL_TO_BASE_LINK_M = np.array([0.0, 0.0, 0.24])


def _quat_matrix(w: float, x: float, y: float, z: float) -> np.ndarray:
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


@dataclass(frozen=True, slots=True)
class CameraRig:
    """The real depth camera's intrinsics + mount, from the committed
    ``configs/vehicle/sensors.yaml`` (generated from the pinned PX4 model
    SDF in Phase 8), so FastSim needs no PX4 checkout to run."""

    intrinsics: CameraIntrinsics
    stride: int
    dirs_optical: np.ndarray  # [K, 3]
    z_factor: np.ndarray  # [K]
    r_body_optical: np.ndarray  # [3, 3]
    t_body_optical: np.ndarray  # [3]
    height: int
    width: int

    @staticmethod
    def load(*, stride: int, path: Path = SENSORS_YAML) -> CameraRig:
        doc = yaml.safe_load(path.read_text())["sensors"]["front_depth"]
        i = doc["intrinsics"]
        intr = CameraIntrinsics(i["width"], i["height"], i["fx"], i["fy"], i["cx"], i["cy"])
        ext = doc["extrinsics_T_base_sensor"]
        r_mount = _quat_matrix(*ext["rotation_wxyz"])
        q = camera_optical_to_body_rotation()
        r_opt = _quat_matrix(q.w, q.x, q.y, q.z)
        dirs, zf = camera_ray_dirs(intr, stride=stride)
        return CameraRig(
            intrinsics=intr,
            stride=stride,
            dirs_optical=dirs,
            z_factor=zf,
            r_body_optical=r_mount @ r_opt,
            t_body_optical=np.array(ext["translation"], dtype=np.float64),
            height=len(range(0, intr.height, stride)),
            width=len(range(0, intr.width, stride)),
        )


@dataclass(frozen=True, slots=True)
class PoseNoiseParams:
    """Agent pose estimate = truth + drift + white noise (per horizontal axis)."""

    walk_step_m: float = 0.005  # drift increment std per control step
    noise_std_m: float = 0.05
    yaw_noise_std_rad: float = 0.01

    @staticmethod
    def from_yaml(path: Path) -> PoseNoiseParams:
        d = yaml.safe_load(path.read_text())["identified"]
        return PoseNoiseParams(
            walk_step_m=float(d["walk_step_m"]),
            noise_std_m=float(d["noise_std_m"]),
            yaw_noise_std_rad=float(d["yaw_noise_std_rad"]),
        )


@njit(cache=True)
def collides(
    occ: np.ndarray,
    origins: np.ndarray,
    dims: np.ndarray,
    ground_z: np.ndarray,
    res: float,
    world_idx: np.ndarray,
    pos: np.ndarray,
    radius: float,
) -> np.ndarray:
    """Sphere-vs-occupied-voxel (exact closest-point test) or ground contact."""
    n = pos.shape[0]
    out = np.zeros(n, dtype=np.bool_)
    for e in range(n):
        w = world_idx[e]
        px, py, pz = pos[e, 0], pos[e, 1], pos[e, 2]
        if pz - radius <= ground_z[w]:
            out[e] = True
            continue
        ox, oy, oz = origins[w, 0], origins[w, 1], origins[w, 2]
        i0 = max(0, int(math.floor((px - radius - ox) / res)))  # noqa: RUF046
        i1 = min(dims[w, 0] - 1, int(math.floor((px + radius - ox) / res)))  # noqa: RUF046
        j0 = max(0, int(math.floor((py - radius - oy) / res)))  # noqa: RUF046
        j1 = min(dims[w, 1] - 1, int(math.floor((py + radius - oy) / res)))  # noqa: RUF046
        k0 = max(0, int(math.floor((pz - radius - oz) / res)))  # noqa: RUF046
        k1 = min(dims[w, 2] - 1, int(math.floor((pz + radius - oz) / res)))  # noqa: RUF046
        r2 = radius * radius
        hit = False
        for i in range(i0, i1 + 1):
            if hit:
                break
            cx = min(max(px, ox + i * res), ox + (i + 1) * res)
            for j in range(j0, j1 + 1):
                if hit:
                    break
                cy = min(max(py, oy + j * res), oy + (j + 1) * res)
                for k in range(k0, k1 + 1):
                    if occ[w, i, j, k] == 0:
                        continue
                    cz = min(max(pz, oz + k * res), oz + (k + 1) * res)
                    if (cx - px) ** 2 + (cy - py) ** 2 + (cz - pz) ** 2 <= r2:
                        hit = True
                        break
        out[e] = hit
    return out


@njit(cache=True)
def geodesic_field(blocked: np.ndarray, goal_i: int, goal_j: int, res: float) -> np.ndarray:
    """8-connected Dijkstra distance (m) from the goal cell over a 2D grid;
    ``inf`` where unreachable. Reward-only privileged information (spec
    §27.5): computed on the true map, never observed by the agent."""
    nx, ny = blocked.shape
    dist = np.full((nx, ny), np.inf)
    if blocked[goal_i, goal_j]:
        return dist
    # Bucket-free Dijkstra via a binary heap of (d, i, j) in parallel arrays.
    cap = nx * ny * 8 + 1
    hd = np.empty(cap)
    hi = np.empty(cap, dtype=np.int64)
    hj = np.empty(cap, dtype=np.int64)
    size = 0
    dist[goal_i, goal_j] = 0.0
    hd[0], hi[0], hj[0] = 0.0, goal_i, goal_j
    size = 1
    diag = math.sqrt(2.0) * res
    while size > 0:
        d, ci, cj = hd[0], hi[0], hj[0]
        size -= 1
        hd[0], hi[0], hj[0] = hd[size], hi[size], hj[size]
        p = 0
        while True:  # sift down
            lft, rgt, m = 2 * p + 1, 2 * p + 2, p
            if lft < size and hd[lft] < hd[m]:
                m = lft
            if rgt < size and hd[rgt] < hd[m]:
                m = rgt
            if m == p:
                break
            hd[p], hd[m] = hd[m], hd[p]
            hi[p], hi[m] = hi[m], hi[p]
            hj[p], hj[m] = hj[m], hj[p]
            p = m
        if d > dist[ci, cj]:
            continue
        for di in range(-1, 2):
            for dj in range(-1, 2):
                if di == 0 and dj == 0:
                    continue
                ni, nj = ci + di, cj + dj
                if ni < 0 or nj < 0 or ni >= nx or nj >= ny or blocked[ni, nj]:
                    continue
                if di != 0 and dj != 0 and (blocked[ci + di, cj] or blocked[ci, cj + dj]):
                    continue  # no corner-cutting through a diagonal gap
                nd = d + (diag if di != 0 and dj != 0 else res)
                if nd < dist[ni, nj]:
                    dist[ni, nj] = nd
                    c = size
                    hd[c], hi[c], hj[c] = nd, ni, nj
                    size += 1
                    while c > 0:  # sift up
                        par = (c - 1) // 2
                        if hd[par] <= hd[c]:
                            break
                        hd[par], hd[c] = hd[c], hd[par]
                        hi[par], hi[c] = hi[c], hi[par]
                        hj[par], hj[c] = hj[c], hj[par]
                        c = par
    return dist


def slab_blocked(bank: WorldBank, w: int, z: float, radius: float) -> np.ndarray:
    """2D ``[nx, ny]`` "can a sphere of ``radius`` at altitude ``z`` occupy
    this column's cell center" grid: any occupied voxel in [z-r, z+r],
    dilated by ``radius`` horizontally (disk). Used for start/goal
    sampling and the reward-only geodesic field."""
    world = bank.worlds[w]
    occ = world.occ
    res = world.resolution_m
    k0 = max(0, math.floor((z - radius - world.origin[2]) / res))
    k1 = min(occ.shape[2] - 1, math.floor((z + radius - world.origin[2]) / res))
    col = occ[:, :, k0 : k1 + 1].any(axis=2)
    rc = math.ceil(radius / res)
    out = col.copy()
    for di in range(-rc, rc + 1):
        for dj in range(-rc, rc + 1):
            if (di * res) ** 2 + (dj * res) ** 2 > (radius + res / 2) ** 2:
                continue
            shifted = np.zeros_like(col)
            si = slice(max(0, di), col.shape[0] + min(0, di))
            sj = slice(max(0, dj), col.shape[1] + min(0, dj))
            ti = slice(max(0, -di), col.shape[0] + min(0, -di))
            tj = slice(max(0, -dj), col.shape[1] + min(0, -dj))
            shifted[si, sj] = col[ti, tj]
            out |= shifted
    return out


@dataclass(slots=True)
class FastSimBatch:
    bank: WorldBank
    rig: CameraRig
    dyn_params: list[DynamicsParams]
    pose_noise: PoseNoiseParams = field(default_factory=PoseNoiseParams)
    shield: ShieldConfig = field(default_factory=ShieldConfig)
    control_dt_s: float = 0.1
    depth_max_range_m: float = 10.0
    depth_noise_std_frac: float = 0.0
    seed: int = 0
    n: int = field(init=False)
    dyn: BatchDynamics = field(init=False)
    world_idx: np.ndarray = field(init=False)
    last_seen: np.ndarray = field(init=False)
    drift: np.ndarray = field(init=False)
    rng: np.random.Generator = field(init=False)
    _fov: np.ndarray = field(init=False)
    _substeps: int = field(init=False)
    depth: np.ndarray = field(init=False)  # [N, H, W] z-depth of the latest render
    points_body: np.ndarray = field(init=False)  # [N, K, 3]

    def __post_init__(self) -> None:
        self.n = len(self.dyn_params)
        self.dyn = BatchDynamics(self.dyn_params, dt_sim_s=DT_SIM_S)
        self.world_idx = np.zeros(self.n, dtype=np.int64)
        self.last_seen = np.zeros((self.n, N_SECTORS))
        self.drift = np.zeros((self.n, 2))
        self.rng = np.random.default_rng(self.seed)
        self._fov = fov_mask(self.shield)
        self._substeps = round(self.control_dt_s / DT_SIM_S)
        k = self.rig.dirs_optical.shape[0]
        self.depth = np.full((self.n, self.rig.height, self.rig.width), np.inf)
        self.points_body = np.full((self.n, k, 3), np.inf)

    def reset_envs(
        self, idx: np.ndarray, *, world: np.ndarray, pos: np.ndarray, yaw: np.ndarray
    ) -> None:
        self.world_idx[idx] = world
        self.dyn.reset(idx, pos=pos, yaw=yaw)
        self.last_seen[idx] = (
            0.0  # fresh shield memory: out-of-FOV sectors start blocked (Tier H parity)
        )
        self.drift[idx] = 0.0

    def render(self) -> None:
        """Raycast the depth camera from every vehicle's true pose."""
        s = self.dyn.state
        c, sn = np.cos(s.yaw), np.sin(s.yaw)
        r_wb = np.zeros((self.n, 3, 3))
        r_wb[:, 0, 0], r_wb[:, 0, 1], r_wb[:, 1, 0], r_wb[:, 1, 1], r_wb[:, 2, 2] = (
            c,
            -sn,
            sn,
            c,
            1.0,
        )
        cam_pos = s.pos + r_wb @ self.rig.t_body_optical
        r_wc = r_wb @ self.rig.r_body_optical
        ranges = cast_rays_batch(
            self.bank.occ,
            self.bank.origins,
            self.bank.dims,
            self.bank.ground_z,
            self.bank.resolution_m,
            self.world_idx,
            cam_pos,
            r_wc,
            self.rig.dirs_optical,
            self.depth_max_range_m,
        )
        if self.depth_noise_std_frac > 0.0:
            ranges = ranges * (1.0 + self.rng.normal(0.0, self.depth_noise_std_frac, ranges.shape))
        z = ranges * self.rig.z_factor
        self.depth = z.reshape(self.n, self.rig.height, self.rig.width)
        finite = np.isfinite(ranges)
        pts_opt = self.rig.dirs_optical[None, :, :] * np.where(finite, ranges, 0.0)[:, :, None]
        pts_opt[~finite] = np.inf  # no-return rays stay non-finite (the shield skips them)
        self.points_body = pts_opt @ self.rig.r_body_optical.T + self.rig.t_body_optical

    def estimated_pose(self) -> tuple[np.ndarray, np.ndarray]:
        """(pose_xy [N, 2], yaw [N]) as the agent's estimator would report it."""
        s = self.dyn.state
        xy = (
            s.pos[:, :2]
            + self.drift
            + self.rng.normal(0.0, self.pose_noise.noise_std_m, (self.n, 2))
        )
        yaw = s.yaw + self.rng.normal(0.0, self.pose_noise.yaw_noise_std_rad, self.n)
        return xy, yaw

    def step_control(self, cmd_odom: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Shield ``cmd_odom`` [N, 3] (vx, vy, yaw rate) against the current
        depth points, integrate one control period, return
        ``(collided [N], shield_intervened [N])``. Call :meth:`render`
        first (each step's observation and shield share one render)."""
        s = self.dyn.state
        v_sh, intervened = shield_batch(
            np.ascontiguousarray(cmd_odom[:, :2]),
            s.yaw,
            self.points_body,
            self.last_seen,
            self._fov,
            self.shield.max_range_m,
            self.shield.a_brake_mps2,
            self.shield.d_safe_m,
            self.shield.tau_s,
        )
        cmd = np.column_stack([v_sh, cmd_odom[:, 2]])
        for _ in range(self._substeps):
            self.dyn.step(cmd)
        self.drift += self.rng.normal(0.0, self.pose_noise.walk_step_m, (self.n, 2))
        hit = collides(
            self.bank.occ,
            self.bank.origins,
            self.bank.dims,
            self.bank.ground_z,
            self.bank.resolution_m,
            self.world_idx,
            s.pos,
            VEHICLE_RADIUS_M,
        )
        return hit, intervened
