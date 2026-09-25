"""Deliberately violates the fixture's 'independence' contract by importing
`broken.errors` — the exact pattern the real aeris.core.errors/types
contract forbids (see pyproject.toml at the repo root).
"""

from broken import errors  # noqa: F401  (the import itself IS the violation)
