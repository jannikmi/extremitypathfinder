# Performance

Measure before and after optimising, and include the numbers in the pull request.

## Measuring

- `benchmarks/` compares the package with pyvisgraph on world maps of increasing size,
  validates all paths and renders the tables and figures of the [performance section](../7_performance.rst)
  of the documentation. Requires the `benchmark` dependency group (`uv sync --all-extras --group benchmark`).
  - `make benchmark` runs it with `--quick`: a smoke test of a few seconds (not run in CI).
    Its outputs go to the gitignored `benchmarks/results/raw/quick/`.
  - The full run takes hours: `uv run --all-extras --group benchmark python -m benchmarks.run`.
    Run it on an otherwise idle machine and state the machine. Check the machine first with `--preflight`,
    continue an interrupted run with `--resume`
    (cf. {ref}`running-benchmarks`).
- `scripts/speed_benchmarks.py` measures environment preparation and queries on the small test environments
  with the same harness (`uv run --all-extras --group benchmark python scripts/speed_benchmarks.py`).
- `prototypes/large_env_query_profile.py` profiles queries on a large, configurable environment
  and compares the query time with the time a copy of the visibility graph would take (queries do not copy it):

  ```sh
  uv run python prototypes/large_env_query_profile.py --holes-per-side 10 --profile
  ```

`prototypes/` contains such development scripts. They are not part of the package and not run by the tests.

Compare the numbers on the same machine, measured on `master` and on your branch.
Check whether Numba was installed: it changes the times.

## Where optimising pays off

Most of the query time is spent on the visibility of start and goal (`utils.find_visible()`).
The A* search takes only a small part of the query time.

## Shortest path search

`utils.astar_path()` is a port of `networkx.astar_path()` that also follows the temporary edges of start and goal,
so queries neither copy nor change the visibility graph (#136).
It is under the 3-clause BSD license of networkx (`LICENSES/networkx.txt`): keep its copyright notice above the function.
Alternatives compared in #152 (with measurements) and rejected, mainly for maintainability:

- `networkx.astar_path()` on the visibility graph with the temporary edges added and removed again:
  changes shared state during a query (not thread-safe, edges of extremities merged with start or goal must be kept)
  and was considerably slower than the port.
- other graph libraries (scipy `csgraph`, igraph, rustworkx): an additional dependency
  and a second graph representation (or `environment.graph` is no networkx graph anymore).
  Their Dijkstra searches were slower than the port (no heuristic, adding the temporary edges is expensive),
  the A* of rustworkx saved only little time per query.
- precomputed shortest paths between all extremities: memory grows quadratically
  with the number of extremities.
