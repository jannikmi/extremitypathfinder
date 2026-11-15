# Invariants

Rules every change to the package must keep. The tests cover most of them, but not all.
Changing one of them needs an issue that decides it first.
The [Engineering Design](../8_design.rst) page gives the full reasoning.

## Results

- **Exact shortest paths.** A returned path is a shortest Euclidean path in the free space (up to floating point
  precision), not an approximation. Optimisations such as graph reductions must not lose optimality.
- **Identical results with and without Numba.** Numba is an optional accelerator.
  Every function in `utils_numba.py` must also run as plain Python via `numba_replacements.py`,
  and keep explicit signatures that Numba can compile. Run the affected tests in both modes (cf. [Testing](testing.md)).

## Geometry

- **Orientation convention.** Internally the boundary polygon is counter-clockwise and the holes are clockwise,
  so the free space is on the left of every edge. The input may use either ordering:
  polygons are reversed when they are stored (#161). The free wedges and the point in polygon test rely on it.
- **Numerical policy.** All coordinates are `float64`. Contacts are exact: polygons touch only when coordinates are
  exactly equal or a vertex lies exactly on an edge. Near-contacts do not block. There is no tolerance parameter.
- **Touching obstacles form a wall.** A path never passes between two obstacles that touch, and never through
  a point where an obstacle touches itself (#24). Cf. the touching section in the About page and `test_contacts.py`.
- **Free wedges decide everything at vertices.** The four rules in the module docstring of `wedges.py` are the single
  source of truth for which points are graph nodes and where a line of sight may start, pass through and arrive.
  Do not add special cases for vertices elsewhere.

## Input

- **Malformed input is never accepted silently.** Input that violates the requirements raises an error that names
  the problem and its location (`TypeError` or `ValueError`). It never gives a wrong result, a default or a silently
  repaired value, and an unexpected internal state is no reason to return a plausible answer: fail loudly.
  Cheap structural checks (polygon format, finite coordinates, point format) always run; only the geometric
  checks can be skipped, with the explicit `validate=False`. A regression test covers every new check.

- **No quiet fallbacks.** Never replace an unexpected state by a default, `None`, `False`, an empty result or a skipped
  step: raise an error that names the problem. A documented, tested outcome is a result, not a fallback
  (e.g. no path: `None` and an infinite distance). `except ...: pass` is only for an exception that is the expected
  outcome (comment why). Ruff enforces the mechanical part (`assert`, blind and swallowed `except`, cf. `pyproject.toml`).

## Architecture

- **Module layering.** A module only imports from the layers below it (cf. [Codebase map](codebase-map.md)).
- **Queries do not change the environment.** A query works on a copy of the coordinates and query local data,
  and never writes to the environment or its visibility graph (thread safety, #136).
  `utils.astar_path()` follows the temporary edges of start and goal for this reason.
- **The file format of `save()` changes only with the graph semantics.** The file stores the polygons and the visibility
  graph, everything else is recomputed (cf. the engineering design). A change that keeps other edges needs a new
  `configs.FILE_FORMAT_VERSION`: `test_file_format.py` fails for it.
- **Few dependencies.** Runtime dependencies are numpy and networkx, Numba is an optional extra.
  A new dependency must be justified (what it saves compared to the code it replaces) and accepted by a maintainer.

## Public API and licensing

- The public API is what `__init__.py` exports, the documented methods and attributes of `PolygonEnvironment` and
  `SearchResult`, the plotting functions and the console command. Mark breaking changes with `BREAKING` in the changelog
  and describe the migration (cf. [Changelog](changelog.md)).
- The package is licensed under MIT only: do not copy or adapt code from other projects (#157).
