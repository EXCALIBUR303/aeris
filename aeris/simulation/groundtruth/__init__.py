"""``GroundTruthService`` (spec §17.4) -- privileged. Constructed only by
``aeris.evaluation``, dataset-labeling scripts, and the world generator.
Nothing under ``aeris.learning``/``aeris.autonomy``/``aeris.perception``/
``aeris.mapping``/``aeris.localization`` may import this package (spec
§14.3 contract 1, import-linter enforced).
"""

from __future__ import annotations

from aeris.simulation.groundtruth.service import GroundTruthService

__all__ = ["GroundTruthService"]
