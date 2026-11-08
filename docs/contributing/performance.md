# Performance

Measure before and after optimising, and include the numbers in the pull request.

## Measuring

- `benchmarks/` compares the package with pyvisgraph on world maps of increasing size,
  validates all paths and renders the tables and figures of the [performance section](../7_performance.rst)
  of the documentation. Requires the `benchmark` dependency group (`uv sync --all-extras --group benchmark`).
  `make benchmark` is a smoke test of a few seconds (not run in CI). The full run takes hours:
  run it on an otherwise idle machine and state the machine (cf. {ref}`running-benchmarks`).
  A release requires up-to-date results (cf. [Releasing](releasing.md#benchmark-results)).
- `scripts/speed_benchmarks.py` measures environment preparation and queries on the small test environments
  with the same harness (`uv run --all-extras --group benchmark python scripts/speed_benchmarks.py`).
- `prototypes/large_env_query_profile.py` profiles queries on a large, configurable environment
  and compares the query time with the time a copy of the visibility graph would take (queries do not copy it):

  ```sh
  uv run python prototypes/large_env_query_profile.py --holes-per-side 10 --profile
  ```

- `prototypes/complexity_profile.py` reports per environment size the time of the wedges and of the visibility graph
  and how often AREA checks a candidate against an edge (constant in its typical case):
  `uv run --all-extras --group benchmark python prototypes/complexity_profile.py world 1k 2k`.

`prototypes/` contains such development scripts. They are not part of the package and not run by the tests.

Compare the numbers on the same machine, measured on `master` and on your branch.
Check whether Numba was installed: it changes the times.

(running-benchmarks)=

## Running the benchmarks

```sh
# full run: hours, on an otherwise idle machine
uv run --all-extras --group benchmark python -m benchmarks.run
# smoke test of the harness (seconds): python -m benchmarks.run --quick
make benchmark
```

`--quick` uses two small maps and 20 queries, writes its tables and figures to the gitignored
`benchmarks/results/raw/quick/` and treats a failed download of the map data as a warning.
Other options: `--sizes 250,1k,5k` (the query set is generated for the maps of the run:
compare only runs with the same sizes), `--budget` (seconds per build), `--query-limit` (seconds per package and size),
`--queries`, `--seed` and `--packages`.
The Natural Earth data and the generated maps are cached in `benchmarks/data/`.
The raw results (JSON, with the hardware, the exact versions, the git commit, the command line, the budgets,
the seeds, the query set and every timing) are written to `benchmarks/results/benchmark.json`
and committed with the tables and figures, so the published numbers can be checked and rendered again.
The paths of the measurements, needed only to resume a run, stay in the gitignored `benchmarks/results/raw/`.
The tables are written to `benchmarks/results/` and the figures to `docs/_static/`.
`--render-only benchmarks/results/benchmark.json` renders them again from the raw results.
`--baselines benchmarks/results/benchmark.json` measures the baselines without input data
(cf. {ref}`benchmark-measurements`) and adds them to the raw results of a run on the same machine,
without repeating the run.

### After a full run

The tables and figures are rendered from the results, the text derived from them is not
(the take-aways of the performance page, the comparison with pyvisgraph, the claims about Numba).
It is marked with `.. benchmark-derived:` comments, and a full run lists the files with one
(or `git grep -n benchmark-derived`). Update every marked passage in the pull request of the results.
Mark new derived text the same way, and keep it qualitative (cf. [Documentation](documentation.md#content)).

### Full run on a laptop

1. Connect the power adapter, close other applications and keep the lid open
   (a closed MacBook sleeps without an external display). On battery power the run warns
   (laptops may throttle on battery).
2. Check the machine: `uv run --all-extras --group benchmark python -m benchmarks.run --preflight`.
   It checks that Numba is active, that the peak memory measurement is correct on this machine
   and the power source, and it downloads the data and generates the maps (minutes, cached).
3. Start the full run (command above). On macOS it keeps the machine awake with `caffeinate`
   while it runs. A measurement during which the machine slept anyway is repeated
   (the monotonic clocks of the timings stop during sleep, the wall clock does not).
4. After an interruption (Ctrl-C, crash, restart), continue with the same command and `--resume`.
   The raw results are written after every measurement: the finished measurements are kept,
   the interrupted one is repeated. Resuming refuses changed settings, versions, code of the package or the benchmark,
   or a different machine. `--fresh` starts over. An existing run is never overwritten without one of the two options.

While developing the benchmark, use `--quick`: shared or virtual machines give noisy timings.
To develop the validation on maps where pyvisgraph builds for minutes, cache its graphs
(`VisGraph.save`/`load`), but never use a cached graph for the timings.

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
