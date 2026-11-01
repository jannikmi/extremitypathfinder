# Plan: reproducible benchmarks against pyvisgraph (#111)

Status: **accepted** (2026-11-01). Implemented: steps 1–4 and the documentation (step 6) with placeholder results generated from a `--quick` run. Pending: step 5, the full run on a stated machine, then the results text of the documentation.

## Decisions taken

| Topic | Decision |
|---|---|
| Competitor | `pyvisgraph==0.2.1`, pinned exactly (unmaintained, last release 2018). `tqdm` is listed explicitly: pyvisgraph imports it without declaring it. |
| Path validation | Shapely (benchmark-only dependency), independent of both packages. |
| Committed outputs | Tables (RST), figures (SVG) and, since 2026-11-01, the raw results of the full run (`benchmarks/results/benchmark.json`): the published numbers can be checked and rendered again without a rerun of hours. Downloaded source data, quick runs and the paths of the measurements (needed only to resume) stay gitignored. |
| Numba | Only the best case is compared: extremitypathfinder with Numba, fully warmed up. Compilation and the import time are excluded and not reported. There is no benchmark run without Numba. |
| Polygon orientation | No preprocessing needed: both packages accept either orientation (extremitypathfinder since #161). |
| Existing benchmark | `scripts/speed_benchmarks.py` becomes a thin wrapper around the new harness. |
| Budget | 10 min per build. Queries are not limited unless the runs take too long (see below). The sweep stops once an extremitypathfinder build takes longer. pyvisgraph builds get the same limit. Data points beyond the budget are left out entirely. There is no extrapolation. |
| Repetitions | One build per package and size. Only builds faster than 1 s are repeated (see Measurements). |
| Queries | One query set, shared by all sizes and both packages. |

## Common input semantics

The packages solve the same problem only on a subset of inputs. All benchmark maps stay inside this subset:

- **Unbounded maps, obstacles only.** pyvisgraph has no boundary polygon. extremitypathfinder is used with `boundary_coordinates=None`.
- **Simple, pairwise disjoint polygons.** Touching obstacles form a wall in extremitypathfinder. pyvisgraph merges vertices with identical coordinates into one graph node, and its behaviour at contacts is undefined.
- **No nearly identical vertices.** Vertices 1e-6 apart crash the pyvisgraph build: in `angle2` a cosine of -1.0000000014 survives the `int(cos·1e10)/1e10` rounding and `acos` raises `math domain error`.
- **Query points strictly in free space**, at least ε away from every polygon. On an edge, the two packages define "inside" differently.

Maps with a boundary, grids and touching obstacles are extremitypathfinder-only features. They are listed as unsupported by pyvisgraph in the documentation, not benchmarked.

## Workloads

### 1. World map: realistic geometry and a size sweep (main workload)

pyvisgraph's own documentation computes ocean routes on world shorelines: GSHHS crude L1, 4,335 points, a 555 s build with 4 workers. The benchmark uses a comparable, freely redistributable dataset.

- **Source:** Natural Earth land polygons, public domain, release `v5.1.2` (pinned), SHA-256 verified. Resolutions:

  | Resolution | Raw size |
  |---|---|
  | 110m | 127 polygons, 4,964 vertices |
  | 50m | 1,421 polygons, 58,857 vertices |
  | 10m | about 4,000 polygons, more than 500k vertices |

  GSHHG was considered and rejected: it is LGPL-licensed and its download host is less accessible.
- **Cleaning** (`benchmarks/world_map.py`, committed):
  1. Split multipolygons and drop interior rings (lakes are unreachable from the sea).
  2. Snap to a 1e-4° grid (about 11 m) and remove consecutive duplicates. This fixes the pyvisgraph crash described above.
  3. Repair self-touching polygons with a 1e-4° mitre buffer. One polygon in 110m needs this: Haida Gwaii.
  4. Drop the smaller polygon of any pair that intersects or touches. After downsampling this rarely happens, and the number dropped is reported.
  5. Check that every polygon is simple, that the polygons are pairwise disjoint, and that `utils.check_hole_data_requirements` passes.
- **Downsampling for the size sweep:** Douglas-Peucker with `preserve_topology=True`. The tolerance is found by bisection (log space) to reach a target vertex count within 2 %, starting from the coarsest resolution with enough vertices. Polygons with an area below tolerance² are dropped (slivers, tiny islands). This keeps realistic geometry, a realistic distribution of obstacle sizes and realistic narrow passages at every size.
- **Downsampling results** (measured with `python -m benchmarks.world_map --target-vertices N`):

  | Target | Source | Polygons | Vertices | Tolerance (°) | Dropped (intersecting) | Generation time |
  |---:|---|---:|---:|---:|---:|---:|
  | 250 | 110m | 24 | 249 | 3.0 | 1 | 1 s |
  | 1k | 110m | 89 | 994 | 0.78 | 3 | 2 s |
  | 2k | 110m | 120 | 1,985 | 0.36 | 3 | 2 s |
  | (raw) | 110m | 127 | 4,954 | 0 | 0 | 1 s |
  | 10k | 50m | 730 | 9,952 | 0.12 | 14 | 3 s |
  | 20k | 50m | 1,305 | 19,956 | 0.050 | 8 | 3 s |
  | 50k | 50m | 1,415 | 49,744 | 0.0079 | 5 | 3 s |
  | 100k | 10m | 4,366 | 99,619 | 0.017 | 61 | 54 s |
  | 200k | 10m | 5,790 | 198,388 | 0.0062 | 27 | 73 s |
  | 400k | 10m | 6,689 | 393,774 | 0.00096 | 20 | 87 s |

  The raw 10m data has 436,950 vertices, so the sweep can reach about 400k vertices.

  **Downsampling with Shapely is fast.** The simplification is Shapely's (GEOS) Douglas-Peucker, `simplify(tolerance, preserve_topology=True)`. Vectorised over all 1,421 polygons of the 50m data it takes 0.07 s. A complete cleaning pass (simplify, snap, repair, separate) takes 0.25 s, so the about 20 bisection steps cost a few seconds. Most of the time for the 10m maps goes into loading the 10 MB GeoJSON and the per-vertex snapping in Python. `shapely.set_precision` (0.07 s on the 50m data, output stays valid) could replace the snapping if needed.

  The earlier generation times (614 s at 50k) came from extremitypathfinder's own input validation (`utils.check_hole_data_requirements`), which took 94 s at 20k vertices and grows superlinearly. The Shapely checks already cover its requirements (simple polygons, pairwise disjoint), so it now only runs with `--validate-epf`. Its scaling on large inputs may deserve a separate issue.
- **Sweep sizes:** 250, 500, 1k, 2k, 5k (raw 110m), 10k, 20k, 50k vertices, then continued from the 10m data (100k, 200k, ...) until an extremitypathfinder build exceeds the 10 min budget (see below). pyvisgraph will exceed its budget much earlier.
- **Queries: one shared set** for all sizes and both packages, so query latencies are directly comparable across the sweep.
  - Proposed size: 200 seeded random point pairs.
  - Points are drawn uniformly over longitude [-180, 180] and latitude [-60, 75].
  - A point is rejected if it lies within ε = 0.1° of land in **any** map of the sweep, checked against the union of the land of all sweep maps. Coarse maps move the coastline by up to the simplification tolerance (3° at 250 vertices), so this guarantees that every point is in free space at every size.
  - Points therefore keep some distance from the coasts, which is less representative of near-coast queries. The query set is stored with the raw results.
  - Because the free space is unbounded and the obstacles are disjoint, a path always exists.
- **Limitations (to be documented):** longitude/latitude are used as planar coordinates. Distances are not geodesic and paths do not wrap around the antimeridian. pyvisgraph's own example does the same. Both packages compute the same planar problem, so the comparison is fair, but the routes are not real shipping routes.

### 2. Small examples (smoke test)

The existing test environments that fit the common semantics (`SEPARATED_ENV` and others converted to unbounded maps). These are used for the `--quick` smoke run and the thin wrapper, not for the documented comparison.

## Measurements

All extremitypathfinder measurements use Numba, after a warm-up: one environment is built and queried in the same process before timing starts. pyvisgraph is pure Python and needs no warm-up beyond the first call. Both run single-threaded (`workers=1`).

| Metric | Method |
|---|---|
| Preprocessing time | Wall time of `PolygonEnvironment(None, holes)` and of `VisGraph().build(polys, workers=1, status=False)`. **One build per package and size.** Builds under 1 s are repeated until 5 s or 5 runs, whichever comes first, and the minimum is reported. That costs nothing and removes the timer and scheduling noise that matters at millisecond scale. |
| Query latency | `time.perf_counter_ns` per query on the prepared environment, the shared query set with each pair queried once. Report p50, p95, p99, mean and queries/s. At about 0.7 s per pyvisgraph query on the 5k map, the 200 queries take about 2.5 min. |
| Peak memory | Peak resident memory (Linux: `VmHWM`, reset after the warm-up; `ru_maxrss` survives `execve` and would include the peak of the parent process. Elsewhere `ru_maxrss`) of the subprocess that does the build and the queries, minus its baseline after the imports and the warm-up. It is measured in the same run as the timings, without `tracemalloc`, whose overhead would distort the timings and would otherwise need a second build: about 10 min more per large pyvisgraph size. The allocator keeps freed memory, so this is a coarse figure, which is adequate for a comparison expected to differ by large factors. |
| Graph size | Nodes and edges (extremitypathfinder's reduced graph vs pyvisgraph's full visibility graph). |

### Budget and sizes the competitor cannot handle

- **Budget: 10 min per build.** It applies to builds only.
- **Query time limit, only if needed.** By default the query phase is not limited. A limit is introduced if the benchmark runs take too long in practice, e.g. pyvisgraph's query phase on the largest maps it can build. Proposed form: 10 min per package and size. The shared query set is stored in random order, so the queries completed within the limit are an unbiased prefix of it. Statistics are reported over that prefix, with the number of completed queries stated, instead of dropping the data point.
- Each build runs in its own subprocess, which is killed when the budget is exceeded.
- The sizes run in ascending order.
- **End of the sweep:** the sweep stops at the first size where the extremitypathfinder build exceeds the budget.
- **pyvisgraph:** the same 10 min build limit applies, so a single build cannot run for hours. Once pyvisgraph exceeds it at one size, it is not run at any larger size.
- **Data points beyond the budget are left out** of all tables and figures: no markers, no extrapolation. The raw results only record that the budget was exceeded.
- **The largest size each package handles within the budget is a documented result.** The documentation states it in the text and the workload table, e.g. "pyvisgraph exceeded the 10 min build budget at N vertices; larger maps were not run".

Results per package and size are one of:

- `ok`: measured and validated
- `invalid`: completed, but validation failed for at least one query (see below)
- `over budget`: the budget was exceeded. Left out of the results, only the size where it happened is documented.
- `crash`: exception, for example the `math domain error`, with the message recorded

## Validation (before any timing is accepted)

For every query and both packages:

1. **Feasibility.** The path starts at the start point and ends at the goal. No path segment intersects the interior of any obstacle: `segment.relate_pattern(polygon, "T********")` against an `STRtree`. Running along edges and through vertices is allowed.
2. **Optimality agreement.** The lengths satisfy `|L_epf - L_pvg| <= 1e-9 * max(1, L_epf)`. Different routes of equal length are accepted.

Any failing query marks the package and size as `invalid`. The failures, with the path and the segments through land, go into the raw results, and the count goes into the report. Invalid and crashed runs are left out of the speed figures. They are listed in a separate table with the size and the number of failed queries, because they are results about correctness, not speed.

**Finding from the feasibility probe** (110m map, North Sea (3, 56) → South China Sea (105, 5)): extremitypathfinder returned a valid path of length 211.14 around the Cape of Good Hope. pyvisgraph returned length 150.26 with two segments crossing land: Lisbon → Valencia across Iberia (9.4° on land) and Algeria → Bab-el-Mandeb across Africa (44.2° on land). **Root cause (investigated):** all 14 of the 277,886 visibility graph edges through land connect two vertices of the same polygon whose midpoint lies level with another vertex of that polygon. pyvisgraph's `edge_in_polygon` tests the midpoint with `polygon_crossing`, which skips edges with exact y comparisons but treats a vertex as on the ray when `ccw` rounds the cross product to zero (absolute tolerance 1e-10, ray length 1e4). The floating point midpoint misses the level vertex by a rounding error (`(0.1 + 0.7) / 2 == 0.39999999999999997`), one edge at that vertex is skipped, the other waits for a partner, the crossing is lost and the midpoint counts as outside. Grid coordinates make such coincidences frequent. An exact crossing count rejects all 14 edges. Minimal reproduction (a six-vertex polygon): `python -m benchmarks.pyvisgraph_issues`. Documented in `docs/7_performance.rst`.

Timings from the same probe, informal and single run: extremitypathfinder built in about 50 s and answered a query in about 60 ms. pyvisgraph built in 335 s and answered a query in 0.6–0.8 s.

## Visualisation (matplotlib, SVG, committed to `docs/_static/`)

Generated from the raw results by `benchmarks/plot.py`, in the style of the existing documentation figures:

1. **`benchmark_build_time.svg`**: build time over the number of vertices, log-log, one line per package (single builds, so no error bars). Only `ok` data points are plotted, so the pyvisgraph line ends at the last size within the budget. The 10 min budget is a horizontal dashed line.
2. **`benchmark_query_time.svg`**: query latency (p50 line, p5–p95 band) over the number of vertices, log-log.
3. **`benchmark_memory.svg`**: peak memory of build and queries over the number of vertices.
4. **`benchmark_ratio.svg`** (optional): the ratio pyvisgraph / extremitypathfinder for build and query, only for sizes where both are `ok`.
5. **`benchmark_world_example.svg`** (optional): the 110m world map with the paths of both packages for one query, illustrating an invalid path when one occurs.

## Reproducibility

- One command: `uv run --all-extras --group benchmark python -m benchmarks.run`. Options: `--quick` (small maps, few queries, smoke test), `--sizes`, `--budget`, `--seed`. `make benchmark` runs `--quick` (decided 2026-11-01); quick outputs go to the gitignored `benchmarks/results/raw/quick/`.
- Recorded per run in the raw JSON: CPU model, cores, RAM, OS, Python, the exact versions of all dependencies (numpy, networkx, numba, scipy, pyvisgraph, shapely), the git SHA and dirty flag, the seeds, the full command line, the budgets, and the Natural Earth release with its checksums.
- The rendered tables and figures carry the hardware and version summary.

## Layout

```
benchmarks/
├── PLAN.md          this file (replaced by the documentation once implemented)
├── world_map.py     download, clean, downsample the Natural Earth data (done)
├── workloads.py     maps (world sweep, small examples) and seeded query generation
├── adapters.py      extremitypathfinder / pyvisgraph: build(), query() -> (path, length)
├── validate.py      feasibility (Shapely) and length agreement
├── measure.py       timing, memory, budgets via subprocesses, warm-up
├── run.py           command line entry point -> raw JSON + RST tables
├── plot.py          SVG figures from raw results
└── data/            downloaded source data (gitignored)
scripts/speed_benchmarks.py   thin wrapper: the small-example benchmark of extremitypathfinder via benchmarks.measure
```

## Documentation

- Rewrite `docs/7_performance.rst`: method, common semantics, the workload table (sizes, polygons, simplification tolerance), the result tables and figures, hardware and versions, and the sizes each package handles within the budget. It then describes the strengths, weaknesses and limitations based on the data, assuming no speedup in advance. One expected trade-off to check: extremitypathfinder's graph reduction may pay off at query time.
- `docs/9_alternatives.rst`: link the measured comparison and the correctness finding.
- `CONTRIBUTING.rst`: how to run the benchmarks. Changelog entry.

## CI

Decision (2026-11-01): the benchmarks do not run in CI, not even `--quick`; the smoke test is `make benchmark` (`--quick`), run locally. CI must not fail because of errors in pyvisgraph or other external packages (dependency installation, data download), only because of extremitypathfinder. CI timings are too noisy to publish, so the full run is a manual step. The documented results come from a stated machine.

## Order of work

1. ~~World map cleaning and downsampling~~ (done)
2. ~~Workloads, adapters and validation; investigate the pyvisgraph invalid paths~~ (done)
3. ~~Measurement harness with budgets and subprocesses~~ (done)
4. ~~`run.py`, `make benchmark`, the thin wrapper, the smoke run (`make benchmark`, not in CI)~~ (done)
5. Full run on a stated machine, figures and tables: `uv run --all-extras --group benchmark python -m benchmarks.run` (pending)
6. Documentation (done, with placeholder results; the strengths and weaknesses follow the full run)

## Notes for the implementation

Facts gathered while planning that are not obvious from the code:

- **Environment:** `uv sync --all-extras --group benchmark`. In the cloud sandbox, naciscdn.org and ngdc.noaa.gov were blocked, but raw.githubusercontent.com (the pinned Natural Earth source in `world_map.py`) was reachable.
- **Reproducing the pyvisgraph crash:** use the raw, unsnapped 110m data, e.g. `world_map.clean(polygons, grid=1e-12)`. The trigger is two consecutive vertices about 1e-6° apart near (-122.85, 76.12) in the Canadian Arctic: `angle2` gets cos = -1.0000000014. Useful as a documented example. **Minimal reproduction:** two triangles of that data (found by removing vertices) crash the build within milliseconds: `CRASH_POLYGONS` in `benchmarks/pyvisgraph_issues.py`. Clamping the cosine to [-1, 1] (upstream PR TaipanRex/pyvisgraph#33) fixes it.
- **Reproducing the invalid path:** cleaned 110m map (`--resolution 110m`), query (3.0, 56.0) → (105.0, 5.0). The expected values are in the Validation section. The pyvisgraph build takes about 5.5 min on a 4-core Xeon @ 2.1 GHz, so cache built graphs (`VisGraph.save`/`load`) while developing the validation. Never use a cached graph for the timings.
- **Path format:** extremitypathfinder returns numpy coordinates and a numpy float length. pyvisgraph returns `vg.Point` objects and no length, so the adapters compute the length from the path in both cases, with the same code.
- **Where to run the published results:** the cloud sandbox is a shared VM (4 vCPUs) with noisy timings and an ephemeral disk. Run the full benchmark for the documentation on a stated, otherwise idle machine. In the sandbox, only develop and run `--quick`. Decision (2026-11-01): the full run happens on the maintainer's MacBook. Prepared for it: `--preflight` (Numba, memory measurement incl. a macOS method, power source, maps), `caffeinate` while running, measurements during sleep repeated, checkpoints after every measurement and `--resume`.
- **Generated maps:** deterministic for a given release, grid and target. Cache them in `benchmarks/data/` (gitignored) and keep the parameters in their JSON header.

## Open questions

None blocking. Decide during the implementation:

- Size of the shared query set: 200 pairs proposed. At about 0.7 s per pyvisgraph query on the 5k map, 200 queries take about 2.5 min. If that grows too long at larger sizes, the query time limit above applies.
- Sweep steps above 50k: 100k, 200k and 400k are available (at most 437k from the 10m data). The sweep ends earlier if an extremitypathfinder build exceeds 10 min.
