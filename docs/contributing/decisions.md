# Decision log

Decided trade-offs, newest first, including the rejected options
(cf. [Trade-offs and decisions](workflow.md#trade-offs-and-decisions)). Entry format:

```markdown
## <decision> (#<issue or pull request>)

- **Decided by:** a maintainer (link) | the contributor | an agent
- **Rejected:** <options, each with the main reason>
- **Details:** <link to the full reasoning, if any>
```

Keep the entries correct: update links and wording when the referenced content moves or changes.
A maintainer confirms an agent decision by changing **Decided by**.
A revised decision gets a new entry that marks the old one **superseded**.
Earlier decisions are described in the {ref}`Design Decisions <design_decisions>` and the issues linked there.

## Open decisions are cleared one by one, by a guide page and the `/decisions` skill (#188)

- **Decided by:** a maintainer
- **Rejected:** leaving it as it is (open decisions stay spread over issues, pull requests and the log);
  a section of the workflow page (the procedure gets its own page); the procedure only in the Claude Code skill
  (other tools and humans would not find it); a `decision needed` label (needs triage by a maintainer,
  and the agent entries in the log stay unlisted); asking in batches or up to a limit (the maintainer decides when to stop);
  ordering blocking decisions before all others (they inherit the priority of the work they block instead);
  agents recording decisions without showing the text first (comments notify people and cannot be taken back).
- **Details:** [Open decisions](open-decisions.md)

## The benchmark documentation is split into results, method and running them (#165)

- **Decided by:** a maintainer
- **Rejected:** only moving the conclusions to the top (the page stays as long, three kinds of readers on one page);
  separate pages for speed and memory (they are read side by side); a page of its own for the pyvisgraph defects
  (they belong next to the validation that found them).

## Benchmarks also measure the retained memory and extremitypathfinder without Numba (#165)

- **Decided by:** a maintainer. Supersedes "a benchmark run without Numba" as a rejected option below.
- **Rejected:** retained memory from the footprint or resident memory after the build (allocators keep freed memory,
  so it stays close to the peak on macOS); from `tracemalloc` (a second, much slower build);
  a separate run without Numba (the results would mix two runs).
- **Details:** [Measurements](../7_benchmark_method.rst) of the benchmark method

## Benchmark memory: peak above the baseline, the baseline and the total (#165)

- **Decided by:** a maintainer
- **Rejected:** only the peak above the baseline (hides the cost of the imports, much larger with Numba
  than for pyvisgraph, which a process pays on small maps); only the total (the fixed baseline hides the part that grows
  with the map).
- **Details:** [Measurements](../7_benchmark_method.rst) of the benchmark method

## Invalid benchmark runs: queries left out, build measured like any other (#165)

- **Decided by:** a maintainer
- **Rejected:** leaving out all results of an invalid run (its build time and memory are measured correctly,
  only its paths are wrong); marking build time and memory as invalid (suggests a wrong measurement).
  The graph size stays marked, because the graph contains wrong edges.
- **Details:** [Measurements](../7_benchmark_method.rst) of the benchmark method

## Benchmarks against pyvisgraph on Natural Earth world maps of increasing size (#165)

- **Decided by:** a maintainer
- **Rejected:** GSHHG data (LGPL and less accessible); extrapolating the sizes beyond the 10 min build budget;
  a benchmark run without Numba (**superseded**, see above); repeating builds slower than 1 s (minutes per build);
  a separate query set per size (latencies not comparable across sizes);
  measuring memory with `tracemalloc` (distorts the timings or needs a second build).
- **Details:** [Benchmark method](../7_benchmark_method.rst)

## The benchmarks do not run in CI, and CI never fails because of external packages (#165)

- **Decided by:** a maintainer
- **Rejected:** a `--quick` smoke test in CI: it fails on errors of pyvisgraph, dependency installations or downloads,
  which are not errors of this package. `make benchmark` runs the smoke test locally.
- **Details:** [Testing](testing.md#ci-and-external-packages)

## The raw results of the full benchmark run are committed (#165)

- **Decided by:** a maintainer
- **Rejected:** only the tables and figures (the published numbers cannot be checked or rendered again without a rerun
  of hours on that machine); a release asset (separated from the commit that produced the docs).
  The file is written compactly (one list per line) and marked as generated in `.gitattributes`.
- **Details:** [Running the benchmarks](performance.md#running-the-benchmarks)

## The benchmark is rerun in full after a change of the package (#165)

- **Decided by:** a maintainer
- **Rejected:** rerunning only the changed package: the results would mix two runs (conditions, commit).
  Resuming an interrupted run is refused when the package or the benchmark code changed.
- **Details:** [Full run on a laptop](performance.md#full-run-on-a-laptop)

## Trade-offs are decided before larger changes and recorded in this log (#179)

- **Decided by:** a maintainer
- **Rejected:** no explicit rule: decisions, in particular by agents, stay invisible to reviewers.

## A short log linking to the reasoning, which stays where it is read (#179)

- **Decided by:** an agent
- **Rejected:** only the existing pages: no place for decisions about the development, "who decided" not recorded,
  past decisions spread over several pages. Full records in the log: duplicate the design page.
