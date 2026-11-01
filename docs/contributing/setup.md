# Setup

The project is managed with [uv](https://docs.astral.sh/uv/). Python 3.12 or newer is required.

## Install

Fork the repository on GitHub and clone your fork. Then install the package with all extras
(Numba) and all development dependency groups (tests, plotting, docs) into a local virtual environment:

```sh
make install  # uv sync --all-extras --all-groups
```

Run commands in this environment with `uv run`, e.g. `uv run pytest`.
`uv.lock` pins the development dependencies. Never edit it by hand:
`make lock` updates it after a change of the dependencies in `pyproject.toml`, `make update` upgrades all pins.

## Pre-commit hooks

The hooks in `.pre-commit-config.yaml` lint and format the code (ruff, flake8, pyupgrade, …)
and check the packaging (check-manifest). CI runs them on every pull request. Install them once:

```sh
make hook  # pre-commit install, then a run on all files
```

## Numba

Numba is optional. The package produces the same results with and without it (cf. [Invariants](invariants.md)).
`make install` installs it. To work without it, create the environment without extras: `uv sync --all-groups`.
The tests check which mode they run in when the environment variable `EXPECT_NUMBA` is set (cf. [Testing](testing.md)).

## Checks

`make check` runs all pre-commit hooks and the tests in the current environment.
It is the quick check before committing. The full test matrix is described in [Testing](testing.md).
