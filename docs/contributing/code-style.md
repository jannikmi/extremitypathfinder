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

- **Strict typing:** annotate every parameter, return value and variable whose type is not inferred, as precisely
  as possible: e.g. `list[int]` instead of `list`, `npt.NDArray[np.float64]` (`types.FloatArray`) instead of `np.ndarray`,
  `X | None` for optional values, `Self` for alternative constructors. Use `Any` only for untyped data at the boundaries
  (JSON, GeoJSON mappings). Optional dependencies (e.g. Shapely) are imported for annotations under `TYPE_CHECKING`.
  New code must pass `mypy --strict` (until #226 is done, check that the reported lines are not yours:
  `uv run --with mypy --with types-networkx --with types-shapely mypy --strict extremitypathfinder`).
  Enforcing it for the whole package is tracked in [#226](https://github.com/jannikmi/extremitypathfinder/issues/226).
- Shared type aliases live in `types.py`, constants in `configs.py`.
- **Modular code:** algorithms are functions in the module of their layer, not methods of `PolygonEnvironment`,
  which only checks the input and delegates (cf. [Codebase map](codebase-map.md#package-modules)).
  Split long functions into helpers with one purpose each.
- Docstrings for public functions and for non-obvious internal ones. Explain *why*, not what the next line does.
- Keep functions in `utils_numba.py` compilable by Numba: numpy arrays and scalars, explicit signatures,
  no Python objects (cf. [Invariants](invariants.md)).
- Prefer vectorised numpy code over Python loops outside of the Numba kernels.
- New dependencies must be justified and accepted by a maintainer (cf. [Invariants](invariants.md)).
- Do not reformat or restructure code unrelated to your change.
