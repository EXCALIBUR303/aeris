.PHONY: setup test test-unit lint typecheck imports format fmt-check sim-smoke sim-up clean

# Thin wrappers only — see AERIS_TECHNICAL_SPEC.md §14 for the reasoning.
# Each target does exactly one obvious thing; nothing here is a build system
# in disguise.

setup:
	uv sync --extra dev --extra sim

test: test-unit

test-unit:
	uv run pytest tests/unit -v

lint: fmt-check
	uv run ruff check .
	uv run mypy aeris
	uv run lint-imports

fmt-check:
	uv run ruff format --check .

format:
	uv run ruff format .
	uv run ruff check --fix .

typecheck:
	uv run mypy aeris

imports:
	uv run lint-imports

# Needs a built PX4 (docs/mac-setup.md) — not run in hosted CI (spec §44.2).
sim-smoke:
	uv run pytest tests/sim -v -m sim

# Also needs a built PX4. Ctrl-C to stop.
sim-up:
	uv run aeris sim up --profile headless_x500

clean:
	find . -type d -name "__pycache__" -not -path "./aeris-deps/*" -exec rm -rf {} +
	rm -rf .mypy_cache .ruff_cache .pytest_cache htmlcov .coverage
