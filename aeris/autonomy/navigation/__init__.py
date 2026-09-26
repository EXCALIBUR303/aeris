"""aeris.autonomy.navigation — path following and reactive avoidance (spec §51 Phase 10).

Both :mod:`follower` and :mod:`reactive` compute a *desired* velocity in
the ODOM/ENU frame; :mod:`aeris.safety.shield` is the sole authority for
turning that desire into a safe command (spec §16.1: "the safety layer
... never learned", and §14.3 contract 2: this package never imports a
vehicle adapter).
"""
