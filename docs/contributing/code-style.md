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
  The package must pass `mypy --strict` (pre-commit hook `mypy`, so also `make check` and CI;
  configured in `pyproject.toml`, every exception is justified there): `uv run pre-commit run mypy --all-files`.
  Numba and Matplotlib are untyped imports, so `@njit` kernels are untyped for mypy.
  The package ships `py.typed` (PEP 561): its annotations are part of the public interface.
- **No assertions in the package:** no `assert` statements in `extremitypathfinder/` (tests are exempt, ruff `S101`). They vanish
  with `python -O`, so they neither validate nor narrow types reliably. Handle the case (an explicit exception or result)
  or restructure the types. Raise an exception for invalid input, never use an assertion for it.
- **No casts for the type checker:** do not wrap values in `float()`, `bool()`, `int()` or `typing.cast()` only to
  satisfy mypy: they run on every call, in the innermost loops too. Annotate a local variable instead
  (`rep: float = _compute_repr_n_dist(v)[0]`). Convert only to change the value (e.g. NumPy scalars to Python floats
  in results), and cast only untyped data at the boundaries (JSON).
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
