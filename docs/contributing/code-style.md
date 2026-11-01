# Code style

The pre-commit hooks enforce most of the style. Run `make check` (or `uv run pre-commit run --all-files`).

## Linting and formatting

- **ruff** lints (default rules, fixes applied automatically) and formats the code.
- **flake8** with `flake8-bugbear`, `flake8-comprehensions` and `flake8-tidy-imports` checks the package
  (not `docs/`, `scripts/` and `tests/`) with a maximum line length of 120.
- **pyupgrade** keeps the syntax modern, **blacken-docs** formats code examples in the documentation.

The configuration lives in `.pre-commit-config.yaml`. Moving it to `pyproject.toml` and replacing flake8 with ruff rules
is tracked in [#173](https://github.com/jannikmi/extremitypathfinder/issues/173).

## Conventions

- Type hints for function signatures. Shared aliases live in `types.py`, constants in `configs.py`.
- Docstrings for public functions and for non-obvious internal ones. Explain *why*, not what the next line does.
- Keep functions in `utils_numba.py` compilable by Numba: numpy arrays and scalars, explicit signatures,
  no Python objects (cf. [Invariants](invariants.md)).
- Prefer vectorised numpy code over Python loops outside of the Numba kernels.
- No new runtime dependencies without an issue that decides it (the package depends on numpy and networkx only).
- Do not reformat or restructure code unrelated to your change.
