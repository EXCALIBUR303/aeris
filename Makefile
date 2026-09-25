.PHONY: setup test test-unit lint typecheck imports format fmt-check sim-smoke clean

# Thin wrappers only — see AERIS_TECHNICAL_SPEC.md §14 for the reasoning.
# Each target does exactly one obvious thing; nothing here is a build system
# in disguise.

setup:
	uv sync --extra dev

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

# Placeholder until Phase 3 creates aeris.simulation.launcher — see
# docs/phase_reports/phase-1.md for the manual equivalent in the meantime.
sim-smoke:
	@echo "sim-smoke is not implemented yet — see docs/mac-setup.md for the" \
	      "manual PX4 SITL smoke test until Phase 3 adds a launcher CLI."
	@exit 1

clean:
	find . -type d -name "__pycache__" -not -path "./aeris-deps/*" -exec rm -rf {} +
	rm -rf .mypy_cache .ruff_cache .pytest_cache htmlcov .coverage
