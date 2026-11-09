# Decision log

Decided trade-offs of the package, its benchmarks and releases, newest first, including the rejected options
(cf. [Trade-offs and decisions](workflow.md#trade-offs-and-decisions)). Entry format:

```markdown
## <decision> (#<issue or pull request>)

- **Decided by:** a maintainer (link) | the contributor | an agent
- **Rejected:** <options, each with the main reason>
- **Details:** <link to the full reasoning, if any>
```

Record only decisions that outlive their pull request and whose rejected options are likely to be proposed again,
in the final state of the pull request. Leave out details fully explained on the page they affect.

Keep the entries correct when the referenced content moves or changes.
A maintainer confirms an agent decision by changing **Decided by**.
Amend the entry of a revised decision instead of adding a new one:
the log states the current decisions (git keeps the history).
Decisions about how we work on the project are in the [process decision log](decisions-process.md).
Earlier decisions are described in the {ref}`Design Decisions <design_decisions>` and the issues linked there.

## AREA stays the visibility algorithm, others are added next to it (#224)

- **Decided by:** a maintainer
- **Rejected:** a rotational sweep (#216), triangular expansion, Polyanya or continuous Dijkstra instead.
- **Details:** {ref}`Design Decisions <design_decisions>`

## No angular sweep for the worst case of AREA (#216)

- **Decided by:** a maintainer
- **Rejected:** an angular sweep (slower on the measured maps, no fitting package, none works with Numba);
  a sweep only above a threshold of checks per candidate (a second code path without a reported need).
- **Details:** [#216](https://github.com/jannikmi/extremitypathfinder/issues/216), the findings in
  `prototypes/sweep_tree_overhead.py`, which reproduces the measurements

## A release requires up-to-date benchmark results, checked when it is prepared, on its pull request and when it is tagged (#191)

- **Decided by:** a maintainer (the rule and closing the gap of manual version changes); an agent (how it is checked)
- **Rejected:** running the benchmarks in CI (cf. #165 below; shared runners also give noisy timings
  on changing hardware); checking every pull request (code still changes until the release);
  only checking when the release is prepared (misses manual version changes and later merges);
  comparing `pyproject.toml` and `uv.lock` as files (they change with every release, the measured dependency
  versions are compared instead).
- **Details:** [Releasing](releasing.md#benchmark-results)

## The 2×2 system in `_lies_behind_inner()` is solved in closed form, SciPy is dropped from the numba extra (#185)

- **Decided by:** a maintainer ([#185](https://github.com/jannikmi/extremitypathfinder/issues/185))
- **Rejected:** keeping `numpy.linalg.solve` (LAPACK) and SciPy: Numba compiles it only with SciPy
  (about 110 MB, more memory in every process with Numba), different LAPACK builds with and without Numba,
  slower without Numba, not exact on integer coordinates.
- **Details:** the measurements in #185 and its pull request, the comment in `utils_numba._lies_behind_inner()`

## Benchmarks against pyvisgraph on Natural Earth world maps of increasing size (#165)

- **Decided by:** a maintainer
- **Rejected:** GSHHG data (LGPL and less accessible); a separate query set per size (latencies not comparable
  across sizes); measuring memory with `tracemalloc` (distorts the timings or needs a second, much slower build);
  a separate run without Numba (the results would mix two runs).
- **Details:** [Benchmark method](../7_benchmark_method.rst)

## The benchmarks do not run in CI, and CI never fails because of external packages (#165)

- **Decided by:** a maintainer
- **Rejected:** a `--quick` smoke test in CI: it fails on errors of pyvisgraph, installations or downloads,
  not of this package. `make benchmark` runs it locally.
- **Details:** [Testing](testing.md#ci-and-external-packages)

## Published benchmark results come from one full run, whose raw results are committed (#165)

- **Decided by:** a maintainer
- **Rejected:** rerunning only the changed package (mixes two runs: conditions, commit);
  committing only the tables and figures (the numbers cannot be checked or rendered again without a rerun
  of hours on that machine); a release asset (separated from the commit that produced the docs).
- **Details:** [Running the benchmarks](performance.md#running-the-benchmarks)
