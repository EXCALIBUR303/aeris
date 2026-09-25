"""Named simulation launch profiles (spec §17.2, §51 Phase 3).

A profile is a complete, named description of one PX4 SITL + Gazebo launch:
which world, which vehicle model, where to spawn it, how fast, headless or
not. Profiles live in ``configs/simulation/*.yaml`` and compose via the
``extends:`` mechanism in :mod:`aeris.core.config`, so e.g. a
``depth_camera`` profile can extend a shared ``_base.yaml`` and override
just ``model:``.
"""

from __future__ import annotations

from pathlib import Path
from typing import Annotated

from pydantic import BaseModel, Field, field_validator, model_validator

from aeris.core.config import compose_config
from aeris.core.errors import ConfigCompositionError

Pose6 = Annotated[tuple[float, float, float, float, float, float], Field()]


class SimulationProfile(BaseModel):
    """One named PX4 SITL + Gazebo launch configuration.

    Field semantics map directly to the PX4/Gazebo environment variables
    documented in spec §9.1 and verified against the pinned PX4 checkout in
    Phase 3 (``ROMFS/px4fmu_common/init.d-posix/px4-rc.gzsim``):

    - ``world`` -> the ``.sdf`` file under ``Tools/simulation/gz/worlds/``.
    - ``model`` -> ``PX4_SIM_MODEL`` (must start with ``gz_``; PX4 matches
      this against an ``airframes/<id>_<model>`` file to pick the autostart
      airframe — AERIS never sets ``PX4_SYS_AUTOSTART`` directly).
    - ``spawn_pose`` -> ``PX4_GZ_MODEL_POSE`` as ``"x,y,z,roll,pitch,yaw"``.
    - ``speed_factor`` -> ``PX4_SIM_SPEED_FACTOR``.
    - ``standalone`` -> whether AERIS launches ``gz sim`` itself (spec
      §17.2's designed architecture: AERIS owns the Gazebo server process
      for tracked-PID shutdown) rather than letting PX4 launch it.
      Always ``True`` for AERIS-managed runs; ``False`` exists only for
      debugging against an already-running, hand-launched Gazebo instance.
    - ``headless`` -> when ``standalone``, whether AERIS passes ``-g`` to
      also start the Gazebo GUI.
    - ``render_engine`` -> Gazebo's ``--render-engine`` override (e.g.
      ``"ogre"`` for the legacy Ogre1 renderer). ``None`` uses Gazebo's
      default (Ogre2), confirmed working headless on this Mac in Phase 1 —
      set this only if a specific machine needs the fallback.
    """

    name: str
    world: str = "default"
    model: str
    instance: int = Field(default=0, ge=0)
    spawn_pose: Pose6 | None = None
    speed_factor: float = Field(default=1.0, gt=0.0)
    standalone: bool = True
    headless: bool = True
    render_engine: str | None = None
    gz_verbose: int = Field(default=1, ge=0, le=4)

    # How long to wait for each readiness stage before giving up (spec
    # §51 Phase 3 validation gate implies these must be generous enough for
    # 10/10 clean cycles, but not so generous that a genuine hang wastes
    # minutes before AERIS reports it as an error).
    world_ready_timeout_s: float = Field(default=30.0, gt=0.0)
    heartbeat_timeout_s: float = Field(default=30.0, gt=0.0)

    @field_validator("model")
    @classmethod
    def _model_has_gz_prefix(cls, v: str) -> str:
        if not v.startswith("gz_"):
            raise ValueError(f"model must start with 'gz_' (PX4_SIM_MODEL convention), got: {v!r}")
        return v

    @model_validator(mode="after")
    def _gui_requires_non_headless(self) -> SimulationProfile:
        # Purely documents intent: nothing to cross-validate today, but this
        # is the right place to add a rule if headless+render_engine
        # combinations ever turn out to be mutually exclusive on some
        # machine (spec §9.4's fallback table).
        return self


def load_profile(path: Path | str) -> SimulationProfile:
    """Load and validate one :class:`SimulationProfile` from a YAML file.

    Composes ``extends:`` chains via :func:`aeris.core.config.compose_config`
    before validating, so shared defaults (e.g. ``_base.yaml``) apply.
    """
    resolved = compose_config(path)
    if "name" not in resolved:
        # Profiles conventionally take their name from the filename, so a
        # bare "model: gz_x500" file doesn't have to repeat it.
        resolved = {**resolved, "name": Path(path).stem}
    try:
        return SimulationProfile.model_validate(resolved)
    except Exception as exc:  # pydantic.ValidationError, but keep this typed
        raise ConfigCompositionError(f"invalid simulation profile at {path}: {exc}") from exc
