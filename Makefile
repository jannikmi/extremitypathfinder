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

# documentation generation:
# https://docs.readthedocs.io/en/stable/intro/getting-started-with-sphinx.html
docs:
	(cd docs && make html)


.PHONY: clean test build release-check release-prepare docs
