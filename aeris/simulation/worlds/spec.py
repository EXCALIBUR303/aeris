"""``WorldSpec`` — the single source of world truth for both tiers (spec §9.3, ADR-007).

A typed, seed-reproducible description of one world: bounds, primitive
geometry (boxes, cylinders — walls/slabs are just oriented boxes), targets,
spawn poses, an altitude band, a family, and a split. Rendered by
``SdfRenderer`` (sdf.py) into a Gazebo world and by ``occupancy.py`` into a
ground-truth voxel grid — never authored twice (spec: "no world geometry
is authored twice").

Field style matches ``aeris.autonomy.mission.spec.WaypointSpec`` (Phase 6):
plain floats in the model, with a ``Vec3``-returning property for callers
that want frame-math types — this keeps the model JSON/hash-stable and
free of any ``aeris.core.frames`` import inside the model itself.
"""

from __future__ import annotations

import hashlib
import json
from enum import StrEnum, unique

from pydantic import BaseModel, Field, field_validator, model_validator

from aeris.core.frames.vector import Vec3


@unique
class WorldFamily(StrEnum):
    """Procedural world families (spec §33.1)."""

    RUBBLE = "f1_rubble"
    OFFICE = "f2_office"
    WAREHOUSE = "f3_warehouse"
    COLLAPSED = "f4_collapsed"


@unique
class WorldSplit(StrEnum):
    """Train/val/test splits (spec §33.2, ADR-010)."""

    TRAIN = "train"
    VAL = "val"
    TEST_ID = "test_id"
    TEST_OOD = "test_ood"


class Bounds(BaseModel, frozen=True):
    """An axis-aligned world volume, in the ``M``/world ENU frame (spec §19.1)."""

    min_x: float
    min_y: float
    min_z: float = 0.0
    max_x: float
    max_y: float
    max_z: float = 5.0

    @model_validator(mode="after")
    def _min_less_than_max(self) -> Bounds:
        if not (self.min_x < self.max_x and self.min_y < self.max_y and self.min_z < self.max_z):
            raise ValueError(f"Bounds requires min < max on every axis, got {self!r}")
        return self

    def contains(self, p: Vec3) -> bool:
        return (
            self.min_x <= p.x <= self.max_x
            and self.min_y <= p.y <= self.max_y
            and self.min_z <= p.z <= self.max_z
        )


class Box(BaseModel, frozen=True):
    """A box primitive — also used for walls and slabs (a wall is an elongated,
    thin box; a tilted slab is a box with nonzero roll/pitch, spec §33.1's F4).
    """

    x: float
    y: float
    z: float
    size_x: float = Field(gt=0.0)
    size_y: float = Field(gt=0.0)
    size_z: float = Field(gt=0.0)
    roll_rad: float = 0.0
    pitch_rad: float = 0.0
    yaw_rad: float = 0.0

    @property
    def center(self) -> Vec3:
        return Vec3(self.x, self.y, self.z)

    @property
    def half_extent(self) -> Vec3:
        return Vec3(self.size_x / 2.0, self.size_y / 2.0, self.size_z / 2.0)


class Cylinder(BaseModel, frozen=True):
    """An upright cylinder primitive (radius + height, centered at ``z``)."""

    x: float
    y: float
    z: float
    radius_m: float = Field(gt=0.0)
    height_m: float = Field(gt=0.0)

    @property
    def center(self) -> Vec3:
        return Vec3(self.x, self.y, self.z)


class Target(BaseModel, frozen=True):
    """A search-mission target placement slot (spec §Phase-21: "N in [3, 8] targets of defined classes")."""

    x: float
    y: float
    z: float
    target_class: str = "generic"
    radius_m: float = Field(default=0.3, gt=0.0)

    @property
    def position(self) -> Vec3:
        return Vec3(self.x, self.y, self.z)


class SpawnPose(BaseModel, frozen=True):
    x: float
    y: float
    z: float
    yaw_rad: float = 0.0

    @property
    def position(self) -> Vec3:
        return Vec3(self.x, self.y, self.z)


class WorldSpec(BaseModel, frozen=True):
    """A complete, seed-reproducible world description (spec §9.3)."""

    name: str
    family: WorldFamily
    split: WorldSplit
    seed: int
    bounds: Bounds
    altitude_band_m: tuple[float, float] = (0.3, 3.0)
    boxes: tuple[Box, ...] = ()
    cylinders: tuple[Cylinder, ...] = ()
    targets: tuple[Target, ...] = ()
    spawn_poses: tuple[SpawnPose, ...] = Field(default_factory=tuple, min_length=1)
    ambient_intensity: float = Field(default=1.0, ge=0.0, le=1.0)

    @field_validator("altitude_band_m")
    @classmethod
    def _altitude_band_ordered(cls, v: tuple[float, float]) -> tuple[float, float]:
        low, high = v
        if not low < high:
            raise ValueError(f"altitude_band_m must be (low, high) with low < high, got {v!r}")
        return v

    @model_validator(mode="after")
    def _family_split_pairing(self) -> WorldSpec:
        # Spec §33.1: F4 is "OOD test only, never used for training, tuning
        # or model selection" -- enforced as a hard pairing rather than a
        # convention, so a generator bug can't accidentally produce a
        # trainable F4 world or a non-F4 test_ood world.
        is_collapsed = self.family is WorldFamily.COLLAPSED
        is_ood = self.split is WorldSplit.TEST_OOD
        if is_collapsed != is_ood:
            raise ValueError(
                f"family={self.family!r}/split={self.split!r}: F4 (collapsed) must always use "
                f"split=test_ood, and test_ood must always be family=f4_collapsed (spec §33.1)"
            )
        return self

    def canonical_json(self) -> str:
        """A stable JSON serialization (sorted keys) for hashing (spec: "same seed -> same hash")."""
        return json.dumps(self.model_dump(mode="json"), sort_keys=True, separators=(",", ":"))

    def content_hash(self) -> str:
        return hashlib.sha256(self.canonical_json().encode()).hexdigest()
