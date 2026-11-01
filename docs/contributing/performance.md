# Performance

Measure before and after optimising, and include the numbers in the pull request.

## Measuring

- `scripts/speed_benchmarks.py` benchmarks environment preparation and queries on the small test environments
  (`uv run pytest -s scripts/speed_benchmarks.py`). The results are shown in the
  [performance section](../7_performance.rst) of the documentation.
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
The A* search takes about 5% of the query time (900 and 2,500 extremities).

## Shortest path search

`utils.astar_path()` is a port of `networkx.astar_path()` that also follows the temporary edges of start and goal,
so queries neither copy nor change the visibility graph (#136).
It is under the 3-clause BSD license of networkx (`LICENSES/networkx.txt`): keep its copyright notice above the function.
Alternatives compared in #152 and rejected, mainly for maintainability:

- `networkx.astar_path()` on the visibility graph with the temporary edges added and removed again:
  changes shared state during a query (not thread-safe, edges of extremities merged with start or goal must be kept)
  and was about twice as slow as the port.
- other graph libraries (scipy `csgraph`, igraph, rustworkx): an additional dependency
  and a second graph representation (or `environment.graph` is no networkx graph anymore).
  Their Dijkstra searches were slower than the port (no heuristic, adding the temporary edges is expensive),
  the A* of rustworkx saved less than 1 ms per query.
- precomputed shortest paths between all extremities: memory grows quadratically
  with the number of extremities (75 MB for 2,500 extremities).
