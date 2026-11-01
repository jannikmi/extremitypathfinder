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

tox:
	@uv run tox

hook:
	@pre-commit install
	@pre-commit run --all-files

hookup:
	@pre-commit autoupdate
	@pre-commit install

clean:
	rm -rf .pytest_cache .coverage coverage.xml tests/__pycache__ .mypyp_cache/ .tox


build:
	uv build --clear

release-check:
	uv build --clear
	uv run python scripts/validate_release.py dist --version "$$(uv version --short)"
	uv run python scripts/release.py check

# usage: make release-prepare BUMP=patch (or minor, major, an explicit version)
release-prepare:
	uv run python scripts/release.py prepare $(BUMP)

# comparison with pyvisgraph (docs/7_performance.rst). Takes hours: run on an idle machine.
# options: make benchmark ARGS="--quick" (smoke test), ARGS="--sizes 250,1k,5k --budget 600"
benchmark:
	uv run --all-extras --group benchmark python -m benchmarks.run $(ARGS)

# documentation generation:
# https://docs.readthedocs.io/en/stable/intro/getting-started-with-sphinx.html
docs:
	(cd docs && make html)


.PHONY: clean test benchmark build release-check release-prepare docs
