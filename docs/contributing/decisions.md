# Decision log

Decided trade-offs, newest first, including the rejected options
(cf. [Trade-offs and decisions](workflow.md#trade-offs-and-decisions)). Entry format:

```markdown
## <decision> (#<issue or pull request>)

- **Decided by:** a maintainer (link) | the contributor | an agent
- **Rejected:** <options, each with the main reason>
- **Details:** <link to the full reasoning, if any>
```

Entries are not edited, except that a maintainer confirms an agent decision by changing **Decided by**,
and a revised decision gets a new entry that marks the old one **superseded**.
Earlier decisions are described in the {ref}`Design Decisions <design_decisions>` and the issues linked there.

## Benchmarks against pyvisgraph as specified in `benchmarks/PLAN.md` (#165)

- **Decided by:** a maintainer
- **Rejected:** the options listed in the plan's decisions table (e.g. GSHHG data: LGPL and less accessible;
  extrapolating the sizes beyond the build budget; a benchmark run without Numba).
- **Details:** `benchmarks/PLAN.md`

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
- **Details:** `benchmarks/PLAN.md` (Committed outputs)

## The benchmark is rerun in full after a change of the package (#165)

- **Decided by:** a maintainer
- **Rejected:** rerunning only the changed package: the results would mix two runs (conditions, commit).
  Resuming an interrupted run is refused when the package or the benchmark code changed.
- **Details:** [Running the benchmarks](../7_performance.rst) (Full run on a laptop)

## Trade-offs are decided before larger changes and recorded in this log (#179)

- **Decided by:** a maintainer
- **Rejected:** no explicit rule: decisions, in particular by agents, stay invisible to reviewers.

## A short log linking to the reasoning, which stays where it is read (#179)

- **Decided by:** an agent
- **Rejected:** only the existing pages: no place for decisions about the development, "who decided" not recorded,
  past decisions spread over several pages. Full records in the log: duplicate the design page.
