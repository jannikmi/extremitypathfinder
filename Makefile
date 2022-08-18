pin:
	@echo "pinning the dependencies specified in 'pyproject.toml':"
	@poetry update -vv
	#poetry export -f requirements.txt --output docs/requirements.txt --without-hashes

req:
	@echo "installing the development dependencies..."
	@poetry install
	@#poetry install --no-dev


update: pin req

test:
	@tox
	#pytest

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
