"""aeris.simulation.worlds — WorldSpec, procedural generators, and renderers (spec §9.3, §33, Phase 9).

``WorldSpec`` (spec.py) is the single source of world truth (ADR-007): no
world geometry is authored twice. Both ``SdfRenderer`` (sdf.py, this
package) and the future ``FastSimRenderer`` (Phase 13) render the exact
same ``WorldSpec``.
"""
