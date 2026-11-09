# Decision log

Decided trade-offs, newest first, including the rejected options
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
Earlier decisions are described in the {ref}`Design Decisions <design_decisions>` and the issues linked there.

## AREA stays the visibility algorithm, others are added next to it (#224)

- **Decided by:** a maintainer
- **Rejected:** a rotational sweep (#216), triangular expansion, Polyanya or continuous Dijkstra instead.
- **Details:** {ref}`Design Decisions <design_decisions>`

## The next issue is implemented by a guide page and the `/implement-next` skill, claimed by a comment (#195)

- **Decided by:** a maintainer (bundling related issues, claims for concurrent work); an agent (comment claims
  where the oldest active one wins, a new branch per run as the claim id, the stale-claim limit)
- **Rejected:** one concern per pull request (more pull requests, reviews and conflicts for related changes);
  claiming by assignment (agents share the maintainer's account); by a label or an issue edit (the last writer wins;
  a label goes stale when a session dies); by pushing a lock branch (invisible in the issue, needs cleanup);
  a `ready` label (needs triage; readiness follows from the issue and its links).
- **Details:** [Implement the next issue](implement-next.md)

## A release requires up-to-date benchmark results, checked when it is prepared, on its pull request and when it is tagged (#191)

- **Decided by:** a maintainer (the rule and closing the gap of manual version changes); an agent (how it is checked)
- **Rejected:** running the benchmarks in CI (cf. #165 below; shared runners also give noisy timings
  on changing hardware); checking every pull request (code still changes until the release);
  only checking when the release is prepared (misses manual version changes and later merges);
  comparing `pyproject.toml` and `uv.lock` as files (they change with every release, the measured dependency
  versions are compared instead).
- **Details:** [Releasing](releasing.md#benchmark-results)

## Contributing pages have a configurable word limit and are reachable from `CONTRIBUTING.md`

- **Decided by:** a maintainer (the limit, justified exceptions, amending, condensing before splitting,
  reachability); an agent (counting like `wc -w`, `CONTRIBUTING.md` as the only entry point,
  the configuration in `pyproject.toml`)
- **Rejected:** counting lines or characters (depend on line wrapping and language); excluding code blocks
  (they cost reading time and agent context too); requiring only one reference from any file
  (pages linking to each other, the toctree or a skill would pass without being findable from the guide).
- **Details:** [Size and structure of this guide](documentation.md#size-and-structure-of-this-guide)

## Blocked issues are unblocked and re-ranked by a guide page and the `/unblock` skill, which performs the audits and amends the issues and files (#188, #194)

- **Decided by:** a maintainer
- **Rejected:** leaving it as it is (open decisions stay spread out);
  the procedure only in the Claude Code skill (other tools and humans would not find it);
  a `decision needed` label (needs triage, finds no missing information);
  asking in batches or up to a limit (the maintainer decides when to stop);
  blocking decisions before all others (they inherit the priority of the work they block instead);
  approving each change separately (slow, showing them suffices);
  changes without showing them;
  a comment per decision (scope hidden in the thread, reaches the repository only with the
  implementation); rescoping into a later audit (defers the needed information);
  asking before longer audits (stalls the run, a heads-up suffices).
- **Details:** [Unblock and re-rank](unblock-and-rerank.md)

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

## Trade-offs are decided before larger changes and recorded in a short log (#179)

- **Decided by:** a maintainer (the rule); an agent (the form of the log)
- **Rejected:** no explicit rule (decisions, in particular by agents, stay invisible to reviewers);
  only the existing pages (no place for development decisions, "who decided" not recorded,
  past decisions spread out); full records in the log (duplicate the design page).
