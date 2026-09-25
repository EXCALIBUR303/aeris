"""Structured logging for AERIS.

Spec §14.2: "structured logging (``structlog`` or stdlib JSON logging,
decided in P2) carries ``run_id``, ``t_sim``." This module picks
``structlog``, configured to emit one JSON object per line so log lines are
directly greppable/parseable in the run directories described in spec
§38.1 (``results/runs/<...>/logs/``).

May depend on ``aeris.core.errors``/``.types`` (import-linter contract:
"core.clock may depend on types/units/errors, not vice versa" — logging is
in the same layer as clock/config).
"""

from __future__ import annotations

import logging
import sys
from collections.abc import Iterator
from contextlib import contextmanager

import structlog

_CONFIGURED = False


def configure_logging(*, level: int = logging.INFO, json: bool = True) -> None:
    """Configure structlog + stdlib logging for the current process.

    Idempotent — safe to call more than once (e.g. once from a CLI entry
    point and once from a test fixture); only the first call takes effect
    unless ``force`` semantics are added later.

    Args:
        level: stdlib logging level for the root logger.
        json: emit newline-delimited JSON (the default; for log files and
            CI). Set ``False`` for a human-readable console renderer during
            interactive development.
    """
    global _CONFIGURED
    if _CONFIGURED:
        return

    shared_processors: list[structlog.types.Processor] = [
        structlog.contextvars.merge_contextvars,
        structlog.processors.add_log_level,
        structlog.processors.TimeStamper(fmt="iso", utc=True, key="t_wall"),
        structlog.processors.StackInfoRenderer(),
        structlog.processors.format_exc_info,
    ]

    renderer: structlog.types.Processor = (
        structlog.processors.JSONRenderer() if json else structlog.dev.ConsoleRenderer()
    )

    structlog.configure(
        processors=[*shared_processors, renderer],
        wrapper_class=structlog.make_filtering_bound_logger(level),
        context_class=dict,
        logger_factory=structlog.PrintLoggerFactory(file=sys.stdout),
        cache_logger_on_first_use=True,
    )
    logging.basicConfig(level=level, stream=sys.stdout, format="%(message)s")
    _CONFIGURED = True


def get_logger(**initial_context: object) -> structlog.stdlib.BoundLogger:
    """Get a structlog logger, optionally pre-bound with context fields.

    Ensures logging is configured (with defaults) if nothing has done so
    yet, so importing this module and calling :func:`get_logger` always
    works standalone (useful in tests and small scripts).
    """
    if not _CONFIGURED:
        configure_logging()
    return structlog.get_logger(**initial_context)  # type: ignore[no-any-return]


@contextmanager
def bind_run_context(*, run_id: str, t_sim_s: float | None = None) -> Iterator[None]:
    """Bind ``run_id`` (and optionally ``t_sim_s``) to every log line in this context.

    Spec §14.2 requires every log line to carry ``run_id`` and ``t_sim``.
    Use as a context manager around a run's execution::

        with bind_run_context(run_id=run_id):
            ...  # every get_logger().info(...) call in here carries run_id
    """
    tokens = structlog.contextvars.bind_contextvars(
        run_id=run_id, **({"t_sim_s": t_sim_s} if t_sim_s is not None else {})
    )
    try:
        yield
    finally:
        structlog.contextvars.reset_contextvars(**tokens)
