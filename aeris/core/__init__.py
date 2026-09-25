"""aeris.core — foundational types, errors, units, clock, config, logging.

Nothing in this package may import from any other ``aeris.*`` package (spec
§14.3 contract 1 applies transitively: everything depends on core, core
depends on nothing in AERIS). Only ``aeris.core.errors``, ``.types`` and
``.units`` are required to be dependency-free of each other too — see the
import-linter contracts in ``pyproject.toml``.
"""
