install:
	@echo "installing the development dependencies..."
	@poetry install
	@#poetry install --no-dev


update:
	@echo "updating the dependencies pinned in 'pyproject.toml':"
	@poetry update -vvv
	#poetry export -f requirements.txt --output docs/requirements.txt --without-hashes

lock:
	@echo "pinning the dependencies in 'pyproject.toml':"
	@poetry lock -vvv


test:
	#pytest
	@tox

hook:
	@pre-commit install
	@pre-commit run --all-files

hook2:
	@pre-commit autoupdate

clean:
	rm -rf .pytest_cache .coverage coverage.xml tests/__pycache__ .mypyp_cache/ .tox


build:
	poetry build

# documentation generation:
# https://docs.readthedocs.io/en/stable/intro/getting-started-with-sphinx.html
docs:
	(cd docs && make html)


.PHONY: clean test build docs
