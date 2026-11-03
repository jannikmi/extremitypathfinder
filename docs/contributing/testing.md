# Testing

## Quick checks

```sh
make check                                       # pre-commit hooks + tests in the current environment
uv run pytest tests/test_contacts.py             # one module
uv run pytest tests/test_contacts.py -k touch    # tests matching a name
```

The tests use [pytest](https://docs.pytest.org/). A bare `pytest` only collects `tests/`
(`scripts/` and `prototypes/` are not part of the test suite).

## Full matrix

The tox configuration in `pyproject.toml` tests all supported Python versions with the minimum and the latest
dependencies, with and without Numba. CI runs all of these environments.

```sh
make tox                  # all environments (needs all Python versions)
uv run tox -e py312-numba # a single environment
uv run tox -e py312       # without Numba
uv run tox -e docs        # strict documentation build
uv run tox -e release     # tests of the release tooling (scripts/release.py)
```

## CI and external packages

CI must not fail because of errors in pyvisgraph or other external packages and services,
only because of errors in extremitypathfinder:

- The benchmarks against pyvisgraph (`benchmarks/`) do not run in CI. Run the smoke test locally
  with `make benchmark` (`--quick`) when you change them.
- Tests depending on such packages skip when they are not installed (`pytest.importorskip`)
  and do not run in the tox environments of CI.

Keep it this way when adding CI jobs or tests that use external packages or services.

The tox environments set `EXPECT_NUMBA` to `0` or `1`, and `tests/test_acceleration.py` fails
when the environment does not run in the expected mode. Run the affected tests with and without Numba
when you change `utils_numba.py` or code calling it.

## Writing tests

- **Every bugfix comes with a regression test** that fails before the fix and passes after it.
- Put tests next to related ones:

  | Module | Content |
  | --- | --- |
  | `test_cases.py` | shared test environments with expected paths and lengths |
  | `main_test.py` | the test environments of `test_cases.py`, run end to end |
  | `test_contacts.py` | touching obstacles block paths (#24) |
  | `test_orientation.py` | both vertex orderings are accepted (#161) |
  | `test_core_regressions.py` | regressions in visibility, free wedges, graph and query |
  | `test_geometry_regressions.py` | documented polygon edge cases |
  | `test_find_visible.py` | visibility detection against a reference implementation |
  | `helper_fcts_test.py` | helper functions in `utils.py` |
  | `test_acceleration.py` | the execution backend (Numba or not) |
  | `test_pickle.py`, `test_plotting.py`, `cli_test.py` | pickle export, plotting, command line interface |
  | `test_release.py`, `test_validate_release.py` | release tooling (`scripts/`) |

- Expected paths must be exact shortest paths. Derive them by hand or from geometry, not from the output of the code
  under test.
- Keep tests independent of Numba: they must pass in both modes.
- Plotting tests run headless (tox sets `MPLBACKEND=Agg`).
