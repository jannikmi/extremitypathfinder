# NOTE: install the package itselt to make the CLI commands available (required for the tests)
install:
	@echo "installing the development dependencies..."
	@uv sync --all-extras --all-groups


update:
	@echo "updating the dependencies pinned in 'uv.lock':"
	@uv lock --upgrade

lock:
	@echo "pinning the dependencies in 'uv.lock':"
	@uv lock


test:
	uv run pytest

# quick check before committing: all pre-commit hooks and the tests in the current environment
check:
	@uv run pre-commit run --all-files
	@uv run pytest -x -q

tox:
	@uv run tox

hook:
	@uv run pre-commit install
	@uv run pre-commit run --all-files

hookup:
	@uv run pre-commit autoupdate
	@uv run pre-commit install

clean:
	rm -rf .pytest_cache .coverage coverage.xml tests/__pycache__ .mypy_cache/ .tox


build:
	uv build --clear

release-check:
	uv build --clear
	uv run python scripts/validate_release.py dist --version "$$(uv version --short)"
	uv run python scripts/release.py check

# usage: make release-prepare BUMP=patch (or minor, major, an explicit version)
release-prepare:
	uv run python scripts/release.py prepare $(BUMP)

# comparison with pyvisgraph (docs/7_performance.rst): quick smoke test (seconds).
# Full run (hours, on an idle machine): uv run --all-extras --group benchmark python -m benchmarks.run
benchmark:
	uv run --all-extras --group benchmark python -m benchmarks.run --quick $(ARGS)

# documentation generation:
# https://docs.readthedocs.io/en/stable/intro/getting-started-with-sphinx.html
docs:
	(cd docs && uv run make html)


.PHONY: clean test check benchmark build release-check release-prepare docs
