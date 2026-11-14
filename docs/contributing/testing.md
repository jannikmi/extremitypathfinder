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
dependencies, with and without Numba, and free-threaded CPython 3.14 (`py314t`, `py314t-numba`) with the latest
dependencies. There, `tests/test_threads.py` fails if an imported extension module re-enables the GIL.
CI runs all of these environments.

```sh
make tox                  # all environments (needs all Python versions)
uv run tox -e py312-numba # a single environment
uv run tox -e py312       # without Numba
uv run tox -e docs        # strict documentation build
uv run tox -e release     # tests of the release tooling (scripts/release.py)
```

## Real-world-like geometry

`prototypes/real_world_geometry.py` generates triangulated obstacle maps like those of users who merge triangles
with Shapely (touching triangles, float32 seams, near-degenerate vertices), queries random paths and checks them
independently, failures exactly with rational arithmetic. It also runs with older releases.
Run it (Shapely needed, not in CI) when you change decisions on vertices, contacts or angles:

```sh
uv run python prototypes/real_world_geometry.py --seeds 40 --queries 15
```

## Floating point decisions

The core floating point decisions (angle representations, edge, point in polygon and orientation tests,
the `RAY_WINDOW` selection, `_lies_behind_inner()`) are exact on integer coordinates (#3).
`tests/test_exact_decisions.py` asserts this against the exact references of `tests/exact.py` (`fractions.Fraction`).
Use these references for new exact checks instead of writing new ones.
`prototypes/float_audit.py` uses them too and reports the misdecisions per function on all kinds of inputs
(decimals, near-degenerate cases), which lie within about one unit in the last place and are not asserted.
Run it in both modes when you change one of these functions:

```sh
uv run python prototypes/float_audit.py -n 100000
uv run python prototypes/float_audit.py -n 100000 --no-numba
```

## Coverage

The core algorithm (all modules of the package except `command_line.py`, which the tests only run in a subprocess)
is tested completely: every line and every branch. Check it after a change:

```sh
make coverage    # the tox environments cov-jit-off, cov-no-numba and coverage
```

Numba compiled code is invisible to coverage, so the code is measured as plain Python in two runs
(`NUMBA_DISABLE_JIT=1` with Numba installed, and without Numba, which covers `numba_replacements.py`),
and the results are combined. The report lists the missing lines and fails below 100%.
New code needs tests that assert its behaviour (not only run it). Exclude a line with `# pragma: no cover`
or `# pragma: no branch` only if it cannot be reached, and say why in a comment.
CI does not check the coverage yet (a job edits `.github/workflows/`, which needs an explicit request).

## CI and external packages

CI must not fail because of errors in pyvisgraph or other external packages and services,
only because of errors in extremitypathfinder:

- The benchmarks against pyvisgraph (`benchmarks/`) do not run in CI. Run the smoke test locally
  with `make benchmark` (`--quick`) when you change them.
- Tests depending on such packages skip when they are not installed (`pytest.importorskip`)
  and do not run in the tox environments of CI.
- Exception: the optional extras of the package itself (Shapely for `PolygonEnvironment.from_shapely()`) are
  part of its features. The tox environments install them, also their declared minimum versions,
  so their tests run in CI. Their tests still skip when the extra is not installed.

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
  | `test_random_queries.py` | random query points: visibility, paths, graph reduction, transforms |
  | `test_distance_matrix.py` | `distance_matrix()`: identical to a loop over `find_shortest_path()` |
  | `helper_fcts_test.py` | helper functions in `utils.py` |
  | `test_input_and_output.py` | malformed input is always rejected, results are Python floats |
  | `test_acceleration.py` | the execution backend (Numba or not) |
  | `test_exact_decisions.py`, `exact.py` | floating point decisions exact on integer coordinates, exact references |
  | `test_pickle.py`, `test_plotting.py`, `cli_test.py` | pickle export, plotting, command line interface |
  | `test_release.py`, `test_validate_release.py` | release tooling (`scripts/`) |

- Expected paths must be exact shortest paths. Derive them by hand or from geometry, not from the output of the code
  under test.
- Keep tests independent of Numba: they must pass in both modes.
- Plotting tests run headless (tox sets `MPLBACKEND=Agg`).
