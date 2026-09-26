"""Metric library v1 (spec §41). Every metric here is hand-computable and
unit-tested against a hand-computed case. Ground-truth-based metrics
(collision, clearance) are computed by the evaluator only, never exposed
to agent-facing code.
"""

from __future__ import annotations
