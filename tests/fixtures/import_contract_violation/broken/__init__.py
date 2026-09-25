"""Fixture package: deliberately violates one import-linter contract.

Used only by tests/unit/core/test_import_contracts.py to prove that
`lint-imports` actually catches a violation (spec §51 Phase 2 validation
gate: "a deliberate violation in a test fixture is detected"). This package
is never imported by AERIS code.
"""
