"""Config loading: YAML + ``extends:`` composition + CLI overrides + hashing.

Spec §38.1: "Config: Pydantic-validated YAML. Tiers: ``smoke`` (CI, < 5
min), ``dev`` (≤ 1 h), ``full`` (hours). Config composition is an explicit
``extends:`` list plus CLI overrides (``key=value``). The resolved config
is frozen, hashed (SHA-256 of canonical JSON) and saved."

This module provides the composition/hashing mechanics; each package that
needs a config defines its own Pydantic model and calls
:func:`compose_config` to get a plain, resolved ``dict`` to validate against
it. Pydantic itself is not imported here on purpose — this module has no
opinion on any config's *shape*, only on how raw YAML files combine.

May depend on ``aeris.core.errors``/``.types`` (import-linter contract:
"core.clock may depend on types/units/errors, not vice versa" — config is
in the same layer as clock).
"""

from __future__ import annotations

import copy
import hashlib
import json
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

import yaml

from aeris.core.errors import ConfigCompositionError, ConfigNotFoundError

_EXTENDS_KEY = "extends"


def load_yaml(path: Path | str) -> dict[str, Any]:
    """Load one YAML file as a dict. Raises :class:`ConfigNotFoundError` if missing."""
    p = Path(path)
    if not p.is_file():
        raise ConfigNotFoundError(f"config file not found: {p}")
    with p.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ConfigCompositionError(
            f"config file must contain a YAML mapping at the top level, "
            f"got {type(data).__name__}: {p}"
        )
    return data


def deep_merge(base: Mapping[str, Any], override: Mapping[str, Any]) -> dict[str, Any]:
    """Recursively merge ``override`` onto ``base``.

    Dicts merge key-by-key (recursively). Any other type — including lists —
    is replaced wholesale by ``override``'s value: config lists are typically
    complete specifications (e.g. a sensor list), not something you'd want
    silently concatenated across an ``extends:`` chain.
    """
    result: dict[str, Any] = dict(base)
    for key, ov in override.items():
        bv = result.get(key)
        if isinstance(bv, Mapping) and isinstance(ov, Mapping):
            result[key] = deep_merge(bv, ov)
        else:
            result[key] = ov
    return result


def compose_config(path: Path | str, *, _seen: tuple[Path, ...] = ()) -> dict[str, Any]:
    """Load ``path`` and recursively resolve its ``extends:`` chain.

    ``extends:`` may be a single path (str) or a list of paths, resolved
    relative to the file that declares them. Bases are merged in the order
    listed (later entries win), then the current file's own keys are merged
    on top (so a file always wins over what it extends). The ``extends:``
    key itself is stripped from the result.

    Raises :class:`ConfigCompositionError` on a cyclic ``extends:`` chain.
    """
    p = Path(path).resolve()
    if p in _seen:
        chain = " -> ".join(str(s) for s in (*_seen, p))
        raise ConfigCompositionError(f"cyclic config extends chain: {chain}")

    own = load_yaml(p)
    extends = own.pop(_EXTENDS_KEY, None)

    if extends is None:
        bases: list[str | Path] = []
    elif isinstance(extends, str):
        bases = [extends]
    elif isinstance(extends, Sequence):
        bases = list(extends)
    else:
        raise ConfigCompositionError(
            f"'extends' must be a string or list of strings in {p}, got {type(extends).__name__}"
        )

    merged: dict[str, Any] = {}
    for base in bases:
        base_path = (p.parent / base).resolve()
        merged = deep_merge(merged, compose_config(base_path, _seen=(*_seen, p)))

    return deep_merge(merged, own)


def parse_override(spec: str) -> tuple[list[str], Any]:
    """Parse one ``a.b.c=value`` CLI override into a (key-path, value) pair.

    The value is parsed with ``yaml.safe_load`` so ``42``, ``true``,
    ``null`` and quoted strings behave as a config author would expect,
    matching the values that would appear in the YAML file itself.
    """
    if "=" not in spec:
        raise ConfigCompositionError(f"override must be 'key.path=value', got: {spec!r}")
    key_path, _, raw_value = spec.partition("=")
    key_path = key_path.strip()
    if not key_path:
        raise ConfigCompositionError(f"override has an empty key: {spec!r}")
    value = yaml.safe_load(raw_value)
    return key_path.split("."), value


def apply_overrides(config: Mapping[str, Any], overrides: Sequence[str]) -> dict[str, Any]:
    """Apply a sequence of ``a.b.c=value`` CLI overrides on top of ``config``."""
    result: dict[str, Any] = copy.deepcopy(dict(config))
    for spec in overrides:
        keys, value = parse_override(spec)
        node = result
        for key in keys[:-1]:
            existing = node.get(key)
            if not isinstance(existing, dict):
                existing = {}
                node[key] = existing
            node = existing
        node[keys[-1]] = value
    return result


def canonical_json(config: Mapping[str, Any]) -> str:
    """Deterministic JSON serialization used for hashing and storage.

    Sorted keys, no insignificant whitespace, so the same logical config
    always hashes the same way regardless of dict insertion order.
    """
    return json.dumps(config, sort_keys=True, separators=(",", ":"), default=str)


def config_hash(config: Mapping[str, Any]) -> str:
    """SHA-256 hex digest of a config's canonical JSON (spec §38.1/§38.2)."""
    return hashlib.sha256(canonical_json(config).encode("utf-8")).hexdigest()


def config_hash8(config: Mapping[str, Any]) -> str:
    """First 8 hex chars of :func:`config_hash` — used in run-directory names (spec §38.1)."""
    return config_hash(config)[:8]


def load_resolved_config(
    path: Path | str, *, overrides: Sequence[str] = ()
) -> tuple[dict[str, Any], str]:
    """Load, compose (``extends:``), and override a config in one call.

    Returns ``(resolved_config, config_hash)``. This is the entry point
    most callers want; :func:`compose_config`/:func:`apply_overrides` exist
    separately mainly for testability.
    """
    resolved = compose_config(path)
    resolved = apply_overrides(resolved, overrides)
    return resolved, config_hash(resolved)
