# Plan: reproducible benchmarks against pyvisgraph (#111)

Status: **draft for review**. Implemented so far: the world map cleaning script `benchmarks/world_map.py`.

## Decisions taken

| Topic | Decision |
|---|---|
| Competitor | `pyvisgraph==0.2.1`, pinned exactly (unmaintained, last release 2018). `tqdm` is listed explicitly: pyvisgraph imports it without declaring it. |
| Path validation | Shapely (benchmark-only dependency), independent of both packages. |
| Committed outputs | Rendered results only: tables (RST) and figures (SVG). Raw JSON results and downloaded source data are gitignored. |
| Numba | Only the best case is compared: extremitypathfinder with Numba, fully warmed up. Compilation and the import time are excluded and not reported. There is no benchmark run without Numba. |
| Polygon orientation | No preprocessing needed: both packages accept either orientation (extremitypathfinder since #161). |
| Existing benchmark | `scripts/speed_benchmarks.py` becomes a thin wrapper around the new harness. |

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
  | (raw) | 110m | 127 | 4,954 | 0 | 0 | 11 s |
  | 10k | 50m | 730 | 9,952 | 0.12 | 14 | 28 s |
  | 20k | 50m | 1,305 | 19,956 | 0.050 | 8 | 97 s |
  | 50k | 50m | 1,415 | 49,744 | 0.0079 | 5 | 614 s |

  Generation time is dominated by the bisection, which runs the full cleaning at each step. The maps can be cached between benchmark runs, since they are deterministic for a given release, grid and target.
- **Sweep sizes (proposed):** 250, 500, 1k, 2k, 5k (about raw 110m), 10k, 20k, 50k, 100k vertices. The top end is limited by the extremitypathfinder build time and is decided after a first measurement.
- **Queries:** seeded random point pairs drawn uniformly over the map's bounding box, rejected if within ε = 0.1° of land. A separate query set is drawn for each size, all from the same seed. Because the free space is unbounded and the obstacles are disjoint, a path always exists.
- **Limitations (to be documented):** longitude/latitude are used as planar coordinates. Distances are not geodesic and paths do not wrap around the antimeridian. pyvisgraph's own example does the same. Both packages compute the same planar problem, so the comparison is fair, but the routes are not real shipping routes.

### 2. Small examples (smoke test, CI)

The existing test environments that fit the common semantics (`SEPARATED_ENV` and others converted to unbounded maps). These are used for the CI smoke run and the thin wrapper, not for the documented comparison.

## Measurements

All extremitypathfinder measurements use Numba, after a warm-up: one environment is built and queried in the same process before timing starts. pyvisgraph is pure Python and needs no warm-up beyond the first call. Both run single-threaded (`workers=1`).

| Metric | Method |
|---|---|
| Preprocessing time | Wall time of `PolygonEnvironment(None, holes)` and of `VisGraph().build(polys, workers=1, status=False)`. Repeated R times (R=5 for small maps, fewer at large sizes, the count recorded). Report median, min and max. |
| Query latency | `time.perf_counter_ns` per query on the prepared environment. Report p50, p95, p99, mean and queries/s. Count: 1000 per size, reduced for pyvisgraph at large sizes, where it takes about 0.7 s per query on the 5k map. |
| Peak memory | `tracemalloc` peak, measured separately for the build and the query phase, in separate runs so the tracing overhead does not affect the timings. `ru_maxrss` of the process is also recorded. |
| Graph size | Nodes and edges (extremitypathfinder's reduced graph vs pyvisgraph's full visibility graph). |

### Budgets and sizes the competitor cannot handle

Each package and size gets a build budget (proposed: 30 min) and a query budget. Each build runs in its own subprocess, which is killed when the budget is exceeded. Results per package and size are one of:

- `ok`: measured and validated
- `invalid`: completed, but validation failed for at least one query (see below)
- `timeout`: build or query budget exceeded
- `crash`: exception, for example the `math domain error`, with the message recorded

The largest size each package handles within the budget is a documented result. For sizes beyond the budget, an extrapolation from a fitted model (`a·n²·log n` for pyvisgraph) may be shown, clearly marked as an extrapolation and never as a measurement.

## Validation (before any timing is accepted)

For every query and both packages:

1. **Feasibility.** The path starts at the start point and ends at the goal. No path segment intersects the interior of any obstacle: `segment.relate_pattern(polygon, "T********")` against an `STRtree`. Running along edges and through vertices is allowed.
2. **Optimality agreement.** The lengths satisfy `|L_epf - L_pvg| <= 1e-9 * max(1, L_epf)`. Different routes of equal length are accepted.

Any failing query marks the package and size as `invalid`. The failures, with the path and the segments through land, go into the raw results, and the count goes into the report. Invalid runs are excluded from the speed tables and figures (shown as distinct markers).

**Finding from the feasibility probe** (110m map, North Sea (3, 56) → South China Sea (105, 5)): extremitypathfinder returned a valid path of length 211.14 around the Cape of Good Hope. pyvisgraph returned length 150.26 with two segments crossing land: Lisbon → Valencia across Iberia (9.4° on land) and Algeria → Bab-el-Mandeb across Africa (44.2° on land). The root cause in pyvisgraph will be investigated (with which geometry does it fail?) before the documentation is written, so that the limitation can be described precisely.

Timings from the same probe, informal and single run: extremitypathfinder built in about 50 s and answered a query in about 60 ms. pyvisgraph built in 335 s and answered a query in 0.6–0.8 s.

## Visualisation (matplotlib, SVG, committed to `docs/_static/`)

Generated from the raw results by `benchmarks/plot.py`, in the style of the existing documentation figures:

1. **`benchmark_build_time.svg`**: build time over the number of vertices, log-log, one line per package, min/max as error bars. `timeout`, `crash` and `invalid` get distinct markers at the size where they occurred. The budget is a horizontal dashed line. Any extrapolation is a dotted line labelled as such.
2. **`benchmark_query_time.svg`**: query latency (p50 line, p5–p95 band) over the number of vertices, log-log.
3. **`benchmark_memory.svg`**: peak memory of build and queries over the number of vertices.
4. **`benchmark_ratio.svg`** (optional): the ratio pyvisgraph / extremitypathfinder for build and query, only for sizes where both are `ok`.
5. **`benchmark_world_example.svg`** (optional): the 110m world map with the paths of both packages for one query, illustrating an invalid path when one occurs.

## Reproducibility

- One command: `make benchmark` → `uv run --all-extras --group benchmark python -m benchmarks.run`. Options: `--quick` (small maps, few queries, for CI), `--sizes`, `--budget`, `--seed`.
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

Run `--quick` only, as a smoke test of the harness and the validation. CI timings are too noisy to publish, so the full run is a manual step. The documented results come from a stated machine.

## Order of work

1. ~~World map cleaning and downsampling~~ (done)
2. Workloads, adapters and validation; investigate the pyvisgraph invalid paths
3. Measurement harness with budgets and subprocesses
4. `run.py`, `make benchmark`, the thin wrapper, the CI smoke run
5. Full run on a stated machine, figures and tables
6. Documentation

## Open questions

- Build budget per package and size: 30 min proposed.
- Upper end of the sweep: depends on the extremitypathfinder build time at 50k–100k vertices.
- One query set across all sizes (points at least 2° offshore, valid at every simplification level) instead of one set per size. This compares better across sizes but is less representative near coasts.
